//
// Created by lenovo on 26-7-26.
//

#include "pump_drive.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"

/*
 * ISC1000 的发送方向是 ASCII 命令，接收方向是二进制反馈帧。
 * 本驱动固定注册 USART3：PB10=TX，PB11=RX，对应 Core/Src/usart.c 里的 huart3。
 */
#define PUMP_DRIVE_TX_BUFFER_SIZE 96U

static PumpDrive_s *s_active_pump = NULL;

/**
 * @brief USART3 接收完成回调入口。
 *
 * bsp_usart 在 USART3 收到一帧 DMA + IDLE 数据后，会调用此函数。
 * 该函数不直接处理串口寄存器，只把接收缓冲区交给 PumpDrive_ParseStream()。
 */
static void PumpDrive_UsartRxCallback(void);

/**
 * @brief 使用 printf 风格格式化命令，然后发送到 ISC1000。
 * @param pump 驱动实例指针，必须已经初始化。
 * @param format 不带结尾换行的命令格式字符串，例如 `"set spd=%lu"`。
 * @param ... format 对应的参数。
 * @return 1 表示格式化并发送成功；0 表示参数错误、命令过长或发送失败。
 */
static uint8_t PumpDrive_SendFormatted(PumpDrive_s *pump, const char *format, ...);

/**
 * @brief 从输入缓冲区当前位置尝试解析一帧 ISC1000 反馈。
 * @param pump 驱动实例指针，用于保存解析结果和错误码。
 * @param data 指向疑似帧头 0xFF 的数据地址。
 * @param len 从 data 开始可用的数据长度。
 * @param used_len 成功解析后返回本帧消耗的字节数。
 * @return 1 表示解析成功；0 表示当前数据不足或帧格式/校验错误。
 */
static uint8_t PumpDrive_ParseOneFrame(PumpDrive_s *pump,
                                       const uint8_t *data,
                                       uint16_t len,
                                       uint16_t *used_len);

/**
 * @brief 判断离线时序步骤序号是否合法。
 * @param step_index 步骤序号，手册允许范围为 1~60。
 * @return 1 表示合法；0 表示超出手册范围。
 */
static uint8_t PumpDrive_IsValidStep(uint8_t step_index)
{
    return ((step_index >= 1U) && (step_index <= 60U)) ? 1U : 0U;
}

/**
 * @brief 将 `sta` 返回的 32 位状态值拆解到 PumpDriveStatus_s。
 * @param pump 驱动实例指针，status 字段会被更新。
 * @param raw_value 由 5 字节 7-bit 编码解码得到的 32 位状态值。
 *
 * raw_value 的低 16 位是状态位，高 16 位为手册预留字段。
 * 这里既保存原始位图，也拆出 sensor/busy/enable/valve 等常用布尔字段。
 */
static void PumpDrive_UpdateStatus(PumpDrive_s *pump, uint32_t raw_value)
{
    uint16_t flags = (uint16_t)(raw_value & 0xFFFFU);

    pump->status.raw_flags = flags;
    pump->status.reserved = (int16_t)((raw_value >> 16) & 0xFFFFU);
    pump->status.sensor1 = ((flags & PUMP_DRIVE_STATUS_SENSOR1) != 0U) ? 1U : 0U;
    pump->status.busy = ((flags & PUMP_DRIVE_STATUS_BUSY) != 0U) ? 1U : 0U;
    pump->status.sensor2 = ((flags & PUMP_DRIVE_STATUS_SENSOR2) != 0U) ? 1U : 0U;
    pump->status.autorun = ((flags & PUMP_DRIVE_STATUS_AUTORUN) != 0U) ? 1U : 0U;
    pump->status.enabled = ((flags & PUMP_DRIVE_STATUS_ENABLE) != 0U) ? 1U : 0U;
    pump->status.valve1 = ((flags & PUMP_DRIVE_STATUS_VALVE1) != 0U) ? 1U : 0U;
    pump->status.valve2 = ((flags & PUMP_DRIVE_STATUS_VALVE2) != 0U) ? 1U : 0U;
}

/**
 * @brief 初始化 ISC1000 驱动实例并注册 USART3 接收服务。
 * @param pump 驱动实例指针，函数会先清空整个结构体。
 * @param mode 通讯模式，决定发送命令时是否自动添加设备 ID。
 * @param device_id RS485 设备地址；RS232 模式下只用于记录反馈 ID。
 * @return 1 表示初始化成功；0 表示参数为空或 USART3 注册失败。
 */
uint8_t PumpDrive_Init(PumpDrive_s *pump, PumpDriveBusMode_e mode, uint8_t device_id)
{
    USART_Init_Config_s usart_config;

    if (pump == NULL)
    {
        return 0U;
    }

    memset(pump, 0, sizeof(PumpDrive_s));

    pump->bus_mode = mode;
    pump->device_id = device_id;

    /*
     * 这里故意硬编码为 huart3：
     * 1. 满足“只用 USART3 与模块交互”的要求；
     * 2. 防止后续调用者误把 USART2 等其它串口传进来。
     */
    usart_config.recv_buff_size = PUMP_DRIVE_RX_BUFFER_SIZE;
    usart_config.usart_handle = &huart3;
    usart_config.module_callback = PumpDrive_UsartRxCallback;

    pump->usart = USARTRegister(&usart_config);
    if (pump->usart == NULL)
    {
        pump->last_error = PUMP_DRIVE_ERR_SHORT_FRAME;
        return 0U;
    }

    s_active_pump = pump;
    return 1U;
}

/**
 * @brief 发送不带换行符的 ISC1000 原始命令。
 * @param pump 驱动实例指针。
 * @param command 命令正文，例如 `"hsk"`、`"in 2000"`、`"set spd=2400"`。
 * @return 1 表示命令已通过 USART3 发送；0 表示参数错误或命令超出发送缓冲区。
 *
 * 函数会根据 bus_mode 自动生成最终串口文本：
 * - RS232：`command\n`
 * - RS485：`device_id command\n`
 */
uint8_t PumpDrive_SendCommand(PumpDrive_s *pump, const char *command)
{
    char tx_buffer[PUMP_DRIVE_TX_BUFFER_SIZE];
    int tx_len;

    if ((pump == NULL) || (pump->usart == NULL) || (command == NULL))
    {
        return 0U;
    }

    /*
     * 手册规定：
     * - RS232：命令直接以 '\n' 结尾；
     * - RS485：命令前增加设备 ID 和空格。
     */
    if (pump->bus_mode == PUMP_DRIVE_BUS_RS485)
    {
        tx_len = snprintf(tx_buffer, sizeof(tx_buffer), "%u %s\n", pump->device_id, command);
    }
    else
    {
        tx_len = snprintf(tx_buffer, sizeof(tx_buffer), "%s\n", command);
    }

    if ((tx_len <= 0) || ((uint32_t)tx_len >= sizeof(tx_buffer)))
    {
        return 0U;
    }

    USARTSend(pump->usart, (uint8_t *)tx_buffer, (uint16_t)tx_len, USART_TRANSFER_BLOCKING);
    return 1U;
}

/**
 * @brief 格式化并发送 ISC1000 命令。
 * @param pump 驱动实例指针。
 * @param format printf 风格格式字符串，不需要包含结尾 `\n`。
 * @param ... 格式参数。
 * @return 1 表示格式化和发送成功；0 表示格式化失败或命令过长。
 */
static uint8_t PumpDrive_SendFormatted(PumpDrive_s *pump, const char *format, ...)
{
    char command[PUMP_DRIVE_TX_BUFFER_SIZE];
    int len;
    va_list args;

    va_start(args, format);
    len = vsnprintf(command, sizeof(command), format, args);
    va_end(args);

    if ((len <= 0) || ((uint32_t)len >= sizeof(command)))
    {
        return 0U;
    }

    return PumpDrive_SendCommand(pump, command);
}

/**
 * @brief 发送握手命令，读取 ISC1000 型号、版本和发布日期。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Handshake(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "hsk");
}

/**
 * @brief 发送运行状态查询命令。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryStatus(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "sta");
}

/**
 * @brief 发送参数配置查询命令。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryConfig(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "cfg");
}

/**
 * @brief 发送参数保存命令，将当前设置写入 ISC1000 Flash。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SaveConfig(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "sav");
}

/**
 * @brief 发送电机使能命令。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Enable(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "on");
}

/**
 * @brief 发送电机脱机/失能命令。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Disable(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "off");
}

/**
 * @brief 发送停止命令。
 * @param pump 驱动实例指针。
 * @param emergency 0 表示减速停止；非 0 表示立即急停。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Stop(PumpDrive_s *pump, uint8_t emergency)
{
    return PumpDrive_SendFormatted(pump, "stp %u", emergency ? 1U : 0U);
}

/**
 * @brief 发送复位归零命令。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ResetHome(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "rst");
}

/**
 * @brief 发送吸入/反向运动命令。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveIn(PumpDrive_s *pump, uint32_t steps)
{
    return PumpDrive_SendFormatted(pump, "in %lu", (unsigned long)steps);
}

/**
 * @brief 发送排出/正向运动命令。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveOut(PumpDrive_s *pump, uint32_t steps)
{
    return PumpDrive_SendFormatted(pump, "out %lu", (unsigned long)steps);
}

/**
 * @brief 控制输出通道 1 / 阀 1。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve1(PumpDrive_s *pump, uint8_t on)
{
    return PumpDrive_SendFormatted(pump, "v1 %u", on ? 1U : 0U);
}

/**
 * @brief 控制输出通道 2 / 阀 2。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve2(PumpDrive_s *pump, uint8_t on)
{
    return PumpDrive_SendFormatted(pump, "v2 %u", on ? 1U : 0U);
}

/**
 * @brief 设置一个整数参数。
 * @param pump 驱动实例指针。
 * @param key ISC1000 参数键名，例如 `"spd"`、`"acc"`、`"dec"`、`"id"`。
 * @param value 参数值，调用者应按手册范围传入。
 * @return 1 表示发送成功；0 表示 key 为空或发送失败。
 *
 * 本函数生成 `set key=value`，适合 `spd/acc/dec/ccw/sdr` 等整数参数。
 */
uint8_t PumpDrive_SetParamInt(PumpDrive_s *pump, const char *key, int32_t value)
{
    if (key == NULL)
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump, "set %s=%ld", key, (long)value);
}

/**
 * @brief 设置 ISC1000 目标速度参数 `spd`。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetSpeed(PumpDrive_s *pump, uint32_t speed_pps)
{
    return PumpDrive_SendFormatted(pump, "set spd=%lu", (unsigned long)speed_pps);
}

/**
 * @brief 同时设置目标速度、加速度和减速度。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @param acc_pps2 加速度，单位 PPS^2，手册范围 1~80000。
 * @param dec_pps2 减速度，单位 PPS^2，手册范围 1~80000。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * 一条命令同时设置多个键值，可以减少串口交互次数。
 */
uint8_t PumpDrive_SetMotion(PumpDrive_s *pump, uint32_t speed_pps, uint32_t acc_pps2, uint32_t dec_pps2)
{
    return PumpDrive_SendFormatted(pump,
                                   "set spd=%lu acc=%lu dec=%lu",
                                   (unsigned long)speed_pps,
                                   (unsigned long)acc_pps2,
                                   (unsigned long)dec_pps2);
}

/**
 * @brief 设置运行电流 `crn` 和保持电流 `crh`。
 * @param pump 驱动实例指针。
 * @param run_current_ca 运行电流，单位厘安培；150 表示 1.50A。
 * @param hold_current_ca 保持电流，单位厘安培；20 表示 0.20A。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * 使用“厘安培”是为了避免在嵌入式 printf 中开启浮点格式化。
 */
uint8_t PumpDrive_SetCurrentCentiAmp(PumpDrive_s *pump, uint16_t run_current_ca, uint16_t hold_current_ca)
{
    /*
     * 手册中 crn/crh 单位为 A，范围 0.01~2.00。
     * 这里使用“厘安培”避免启用 printf 浮点格式：
     * 150 表示 1.50A，20 表示 0.20A。
     */
    return PumpDrive_SendFormatted(pump,
                                   "set crn=%u.%02u crh=%u.%02u",
                                   run_current_ca / 100U,
                                   run_current_ca % 100U,
                                   hold_current_ca / 100U,
                                   hold_current_ca % 100U);
}

/**
 * @brief 启动驱动器内部离线时序。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActStart(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "act 0 start");
}

/**
 * @brief 设置离线时序循环运行次数。
 * @param pump 驱动实例指针。
 * @param times 循环次数，手册范围 1~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActTimes(PumpDrive_s *pump, uint32_t times)
{
    return PumpDrive_SendFormatted(pump, "act 0 times %lu", (unsigned long)times);
}

/**
 * @brief 清空驱动器内部离线时序脚本。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActClear(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "act 0 clear");
}

/**
 * @brief 读取驱动器内部离线时序脚本。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * 返回反馈号为 0x04，数据区是 ASCII 文本，并且该反馈类型没有 BCC 校验字节。
 */
uint8_t PumpDrive_ActGet(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "act 0 get");
}

/**
 * @brief 读取离线时序已运行次数。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * 返回反馈号为 0x05，数据区是 5 字节编码的 32 位运行次数。
 */
uint8_t PumpDrive_ActReadCount(PumpDrive_s *pump)
{
    return PumpDrive_SendCommand(pump, "act 0 rdcnt");
}

/**
 * @brief 写入离线时序运行计数起始值。
 * @param pump 驱动实例指针。
 * @param count 起始计数值，手册范围 0~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActWriteCount(PumpDrive_s *pump, uint32_t count)
{
    return PumpDrive_SendFormatted(pump, "act 0 wrcnt %lu", (unsigned long)count);
}

/**
 * @brief 添加或修改离线时序中的复位步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepReset(PumpDrive_s *pump, uint8_t step_index)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump, "act %u reset", step_index);
}

/**
 * @brief 添加或修改离线时序中的延时步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param delay_ms 延时时间，单位 ms，手册范围 1~60000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepDelay(PumpDrive_s *pump, uint8_t step_index, uint32_t delay_ms)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump, "act %u delay %lu", step_index, (unsigned long)delay_ms);
}

/**
 * @brief 添加或修改离线时序中的吸入步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 吸入整步数，手册范围 1~60000。
 * @param speed_pps 吸入速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveIn(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump,
                                   "act %u in step=%lu speed=%lu",
                                   step_index,
                                   (unsigned long)steps,
                                   (unsigned long)speed_pps);
}

/**
 * @brief 添加或修改离线时序中的排出步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 排出整步数，手册范围 1~60000。
 * @param speed_pps 排出速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveOut(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump,
                                   "act %u out step=%lu speed=%lu",
                                   step_index,
                                   (unsigned long)steps,
                                   (unsigned long)speed_pps);
}

/**
 * @brief 添加或修改离线时序中的阀 1 控制步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭阀 1；非 0 表示打开阀 1。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve1(PumpDrive_s *pump, uint8_t step_index, uint8_t on)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump, "act %u v1 %u", step_index, on ? 1U : 0U);
}

/**
 * @brief 添加或修改离线时序中的阀 2 控制步骤。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭阀 2；非 0 表示打开阀 2。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve2(PumpDrive_s *pump, uint8_t step_index, uint8_t on)
{
    if (!PumpDrive_IsValidStep(step_index))
    {
        return 0U;
    }

    return PumpDrive_SendFormatted(pump, "act %u v2 %u", step_index, on ? 1U : 0U);
}

/**
 * @brief 将 5 字节 7-bit 编码还原为 32 位整数。
 * @param raw_5bytes 指向连续 5 字节编码数据的指针。
 * @return 解码后的 32 位无符号整数。
 *
 * ISC1000 为了避免数据区出现 0x80~0xFF，将 32 位整数拆成 5 个 7-bit 字节发送。
 */
uint32_t PumpDrive_Decode5ByteToUint32(const uint8_t raw_5bytes[5])
{
    uint32_t result = 0U;

    result |= ((uint32_t)(raw_5bytes[0] & 0x7FU)) << 28;
    result |= ((uint32_t)(raw_5bytes[1] & 0x7FU)) << 21;
    result |= ((uint32_t)(raw_5bytes[2] & 0x7FU)) << 14;
    result |= ((uint32_t)(raw_5bytes[3] & 0x7FU)) << 7;
    result |= ((uint32_t)(raw_5bytes[4] & 0x7FU));

    return result;
}

/**
 * @brief 计算 BCC 异或校验原始值。
 * @param data 参与校验的数据，不包含帧头 0xFF 和帧尾 0xFE。
 * @param len 参与校验的数据长度。
 * @return 1 字节 BCC 原始值。
 */
uint8_t PumpDrive_CalcBcc(const uint8_t *data, uint16_t len)
{
    uint8_t bcc = 0U;

    for (uint16_t i = 0U; i < len; i++)
    {
        bcc ^= data[i];
    }

    return bcc;
}

/**
 * @brief 检查反馈帧中的两个 BCC 传输字节是否正确。
 * @param payload 参与校验的数据，通常从设备 ID 字节开始。
 * @param payload_len payload 长度。
 * @param bcc_high 反馈帧中的 BCC 高 4 位传输字节。
 * @param bcc_low 反馈帧中的 BCC 低 4 位传输字节。
 * @return 1 表示校验通过；0 表示校验失败。
 */
static uint8_t PumpDrive_CheckBcc(const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint8_t bcc_high,
                                  uint8_t bcc_low)
{
    uint8_t bcc = PumpDrive_CalcBcc(payload, payload_len);

    /*
     * 手册写法为：
     * Byte1 = BCC 高 4 位，Byte0 = BCC 低 4 位。
     * 反馈帧中两个校验传输字节按“高 nibble、低 nibble”顺序保存。
     */
    return (((bcc >> 4) & 0x0FU) == (bcc_high & 0x0FU)) &&
           ((bcc & 0x0FU) == (bcc_low & 0x0FU));
}

/**
 * @brief 在一段接收数据中搜索并解析 ISC1000 反馈帧。
 * @param pump 驱动实例指针，用于保存解析结果。
 * @param data 接收缓冲区，可以包含噪声、半帧或多帧数据。
 * @param len 接收缓冲区长度。
 * @return 成功解析出的完整反馈帧数量。
 *
 * 函数会从缓冲区中搜索 0xFF 帧头，每找到一个帧头就尝试解析一帧。
 */
uint8_t PumpDrive_ParseStream(PumpDrive_s *pump, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0U;
    uint16_t used_len = 0U;
    uint8_t parsed_count = 0U;

    if ((pump == NULL) || (data == NULL))
    {
        return 0U;
    }

    while (offset < len)
    {
        if (data[offset] != PUMP_DRIVE_FRAME_HEADER)
        {
            offset++;
            continue;
        }

        if (PumpDrive_ParseOneFrame(pump, &data[offset], (uint16_t)(len - offset), &used_len) != 0U)
        {
            parsed_count++;
            offset = (uint16_t)(offset + used_len);
        }
        else
        {
            offset++;
        }
    }

    return parsed_count;
}

/**
 * @brief 从当前位置解析单帧反馈。
 * @param pump 驱动实例指针，用于写入 last_type、last_error、status、text_feedback 等结果。
 * @param data 指向帧头 0xFF 的数据地址。
 * @param len 从 data 开始剩余的字节数。
 * @param used_len 解析成功时返回该帧总长度。
 * @return 1 表示解析成功；0 表示帧不完整、尾部错误、BCC 错误或反馈号不支持。
 *
 * 反馈号 0x02/0x05 使用固定 5 字节数值区；
 * 反馈号 0x01/0x03 使用 ASCII 文本 + BCC；
 * 反馈号 0x04 使用 ASCII 文本且没有 BCC。
 */
static uint8_t PumpDrive_ParseOneFrame(PumpDrive_s *pump,
                                       const uint8_t *data,
                                       uint16_t len,
                                       uint16_t *used_len)
{
    PumpDriveFeedbackType_e type;
    uint16_t tail_index;
    uint16_t data_start = 3U;
    uint16_t data_len;
    uint8_t bcc_high;
    uint8_t bcc_low;

    if (used_len != NULL)
    {
        *used_len = 0U;
    }

    if (len < 4U)
    {
        pump->last_error = PUMP_DRIVE_ERR_SHORT_FRAME;
        return 0U;
    }

    type = (PumpDriveFeedbackType_e)data[2];

    if ((type == PUMP_DRIVE_FB_STATUS) || (type == PUMP_DRIVE_FB_SEQUENCE_COUNT))
    {
        uint32_t decoded_value;

        if (len < 11U)
        {
            pump->last_error = PUMP_DRIVE_ERR_SHORT_FRAME;
            return 0U;
        }

        if (data[10] != PUMP_DRIVE_FRAME_TAIL)
        {
            pump->last_error = PUMP_DRIVE_ERR_BAD_TAIL;
            return 0U;
        }

        bcc_high = data[8];
        bcc_low = data[9];
        if (PumpDrive_CheckBcc(&data[1], 7U, bcc_high, bcc_low) == 0U)
        {
            pump->last_error = PUMP_DRIVE_ERR_BAD_BCC;
            return 0U;
        }

        decoded_value = PumpDrive_Decode5ByteToUint32(&data[3]);
        pump->last_value = decoded_value;

        if (type == PUMP_DRIVE_FB_STATUS)
        {
            PumpDrive_UpdateStatus(pump, decoded_value);
        }
        else
        {
            pump->sequence_count = decoded_value;
        }

        pump->last_rx_id = data[1];
        pump->last_type = type;
        pump->last_error = PUMP_DRIVE_ERR_NONE;
        pump->frame_ready = 1U;

        if (used_len != NULL)
        {
            *used_len = 11U;
        }

        return 1U;
    }

    for (tail_index = data_start; tail_index < len; tail_index++)
    {
        if (data[tail_index] == PUMP_DRIVE_FRAME_TAIL)
        {
            break;
        }
    }

    if (tail_index >= len)
    {
        pump->last_error = PUMP_DRIVE_ERR_BAD_TAIL;
        return 0U;
    }

    if (type == PUMP_DRIVE_FB_SEQUENCE_DATA)
    {
        /*
         * 手册中特别说明：0x04 离线时序步骤反馈没有 BCC 校验字节。
         * 因此 data_start 到 tail_index 前一个字节全部都是 ASCII 数据。
         */
        data_len = (uint16_t)(tail_index - data_start);
    }
    else if ((type == PUMP_DRIVE_FB_HANDSHAKE) || (type == PUMP_DRIVE_FB_PARAM_CONFIG))
    {
        if (tail_index < (data_start + 2U))
        {
            pump->last_error = PUMP_DRIVE_ERR_SHORT_FRAME;
            return 0U;
        }

        data_len = (uint16_t)(tail_index - data_start - 2U);
        bcc_high = data[tail_index - 2U];
        bcc_low = data[tail_index - 1U];

        if (PumpDrive_CheckBcc(&data[1], (uint16_t)(2U + data_len), bcc_high, bcc_low) == 0U)
        {
            pump->last_error = PUMP_DRIVE_ERR_BAD_BCC;
            return 0U;
        }
    }
    else
    {
        pump->last_error = PUMP_DRIVE_ERR_UNSUPPORTED_TYPE;
        return 0U;
    }

    if (data_len >= PUMP_DRIVE_TEXT_MAX)
    {
        data_len = PUMP_DRIVE_TEXT_MAX - 1U;
    }

    memcpy(pump->text_feedback, &data[data_start], data_len);
    pump->text_feedback[data_len] = '\0';
    pump->text_len = data_len;
    pump->last_rx_id = data[1];
    pump->last_type = type;
    pump->last_error = PUMP_DRIVE_ERR_NONE;
    pump->frame_ready = 1U;

    if (used_len != NULL)
    {
        *used_len = (uint16_t)(tail_index + 1U);
    }

    return 1U;
}

/**
 * @brief USART3 收到一帧数据后的回调处理。
 *
 * bsp_usart 会在 DMA + IDLE 接收完成后调用该函数。
 * 这里取出 USART3 的完成缓冲区和长度，再交给 PumpDrive_ParseStream() 做协议解析。
 */
static void PumpDrive_UsartRxCallback(void)
{
    if ((s_active_pump == NULL) ||
        (s_active_pump->usart == NULL) ||
        (s_active_pump->usart->rx_buffer_finished == NULL) ||
        (s_active_pump->usart->rx_len == 0U))
    {
        return;
    }

    (void)PumpDrive_ParseStream(s_active_pump,
                                s_active_pump->usart->rx_buffer_finished,
                                s_active_pump->usart->rx_len);
}
