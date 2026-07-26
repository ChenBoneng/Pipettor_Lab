#ifndef PUMP_DRIVE_H
#define PUMP_DRIVE_H

#include <stdint.h>
#include "bsp_usart.h"

/**
 * @file pump_drive.h
 * @brief ISC1000 微型一体化步进电机控制驱动器协议层。
 *
 * 本文件封装 ISC1000 的上位机控制协议：
 * - 发送方向：按照手册生成小写 ASCII 命令，并自动追加换行符 `\n`；
 * - 接收方向：解析驱动器返回的二进制反馈帧；
 * - 串口资源：固定使用 USART3，不占用 USART2 或其它串口。
 *
 * @note 底层 USART3 的 DMA + IDLE 接收由 bsp_usart 管理，pump_drive 只注册回调并解析数据。
 */

/** ISC1000 默认接收反馈的最大长度，不能超过 bsp_usart 的 USART_RXBUFF_LIMIT。 */
#define PUMP_DRIVE_RX_BUFFER_SIZE      256U

/** 保存握手、参数配置、离线时序脚本等文本反馈的最大长度。 */
#define PUMP_DRIVE_TEXT_MAX            220U

/** ISC1000 二进制反馈帧固定帧头，所有反馈帧都以 0xFF 开始。 */
#define PUMP_DRIVE_FRAME_HEADER        0xFFU

/** ISC1000 二进制反馈帧固定帧尾，除不完整数据外，所有反馈帧都以 0xFE 结束。 */
#define PUMP_DRIVE_FRAME_TAIL          0xFEU

/** 状态位 Bit0：传感器 1 是否触发，1 表示触发。 */
#define PUMP_DRIVE_STATUS_SENSOR1      (1U << 0)

/** 状态位 Bit1：电机是否忙碌运行，1 表示 Busy/运行中。 */
#define PUMP_DRIVE_STATUS_BUSY         (1U << 1)

/** 状态位 Bit2：传感器 2 是否触发，1 表示触发。 */
#define PUMP_DRIVE_STATUS_SENSOR2      (1U << 2)

/** 状态位 Bit3：离线时序是否正在运行，1 表示正在执行离线动作。 */
#define PUMP_DRIVE_STATUS_AUTORUN      (1U << 3)

/** 状态位 Bit4：电机是否使能，1 表示 Enable 已置位。 */
#define PUMP_DRIVE_STATUS_ENABLE       (1U << 4)

/** 状态位 Bit5：输出通道 1 / 阀 1 是否打开，1 表示打开。 */
#define PUMP_DRIVE_STATUS_VALVE1       (1U << 5)

/** 状态位 Bit6：输出通道 2 / 阀 2 是否打开，1 表示打开。 */
#define PUMP_DRIVE_STATUS_VALVE2       (1U << 6)

/**
 * @brief ISC1000 通讯总线模式。
 *
 * ISC1000 同一套 ASCII 命令可工作在 RS232 或 RS485 上：
 * - RS232：直接发送命令，例如 `hsk\n`；
 * - RS485：命令前追加设备 ID，例如 `1 hsk\n`。
 */
typedef enum
{
    PUMP_DRIVE_BUS_RS232 = 0, /**< RS232 点对点模式：发送命令时不添加设备 ID。 */
    PUMP_DRIVE_BUS_RS485,     /**< RS485 总线模式：发送命令时在最前面添加设备 ID 和空格。 */
} PumpDriveBusMode_e;

/**
 * @brief ISC1000 下位机反馈号。
 *
 * 反馈号位于二进制反馈帧的第 3 个字节，用于判断数据区的含义和解析方式。
 */
typedef enum
{
    PUMP_DRIVE_FB_NONE           = 0x00U, /**< 无有效反馈；驱动初始化或尚未解析成功时使用。 */
    PUMP_DRIVE_FB_HANDSHAKE      = 0x01U, /**< 握手反馈；数据区为型号、软件版本、发布日期等 ASCII 文本。 */
    PUMP_DRIVE_FB_STATUS         = 0x02U, /**< 运行状态反馈；数据区为 5 字节 7-bit 编码的 32 位状态值。 */
    PUMP_DRIVE_FB_PARAM_CONFIG   = 0x03U, /**< 参数配置反馈；数据区为 `id=... spd=...` 形式的 ASCII 文本。 */
    PUMP_DRIVE_FB_SEQUENCE_DATA  = 0x04U, /**< 离线时序步骤反馈；数据区为 `|` 分隔的 ASCII 文本，且此帧无 BCC。 */
    PUMP_DRIVE_FB_SEQUENCE_COUNT = 0x05U, /**< 时序运行次数反馈；数据区为 5 字节 7-bit 编码的 32 位计数值。 */
} PumpDriveFeedbackType_e;

/**
 * @brief ISC1000 反馈帧解析错误码。
 *
 * last_error 用于记录最近一次解析失败原因，便于调试线缆、校验、帧格式问题。
 */
typedef enum
{
    PUMP_DRIVE_ERR_NONE = 0,          /**< 无错误，最近一次反馈帧解析成功。 */
    PUMP_DRIVE_ERR_SHORT_FRAME,       /**< 数据长度不足，无法组成完整反馈帧。 */
    PUMP_DRIVE_ERR_BAD_TAIL,          /**< 未找到帧尾 0xFE，或帧尾位置不符合当前反馈类型。 */
    PUMP_DRIVE_ERR_BAD_BCC,           /**< BCC 异或校验失败，说明数据可能损坏或校验字节顺序不匹配。 */
    PUMP_DRIVE_ERR_UNSUPPORTED_TYPE,  /**< 反馈号不是当前驱动支持的 0x01~0x05。 */
} PumpDriveError_e;

/**
 * @brief `sta` 运行状态反馈解析结果。
 *
 * ISC1000 的 `sta` 指令返回反馈号 0x02，数据区为 5 字节编码的 32 位整数。
 * 低 16 位是状态位，高 16 位按手册说明预留。
 */
typedef struct
{
    uint16_t raw_flags;  /**< 原始低 16 位状态位，保留完整位图供上层做额外判断。 */
    int16_t reserved;    /**< 高 16 位预留字段，按手册说明保留为有符号值。 */
    uint8_t sensor1;     /**< 传感器 1 触发状态：0 未触发，1 已触发。 */
    uint8_t busy;        /**< 电机运行状态：0 空闲，1 忙碌/运行中。 */
    uint8_t sensor2;     /**< 传感器 2 触发状态：0 未触发，1 已触发。 */
    uint8_t autorun;     /**< 离线时序状态：0 未运行，1 正在运行。 */
    uint8_t enabled;     /**< 电机使能状态：0 脱机/失能，1 使能。 */
    uint8_t valve1;      /**< 输出通道 1 / 阀 1 状态：0 关闭，1 打开。 */
    uint8_t valve2;      /**< 输出通道 2 / 阀 2 状态：0 关闭，1 打开。 */
} PumpDriveStatus_s;

/**
 * @brief ISC1000 驱动实例。
 *
 * 当前工程只使用 USART3 和一个 ISC1000 模块，因此驱动内部按单实例设计；
 * 仍然保留结构体，是为了集中保存串口实例、通讯模式、最近反馈和解析状态。
 */
typedef struct
{
    USARTInstance *usart;                     /**< bsp_usart 注册得到的 USART3 实例。 */
    uint8_t device_id;                        /**< RS485 设备地址；RS232 模式下仅用于记录反馈 ID。 */
    PumpDriveBusMode_e bus_mode;              /**< 当前通讯模式：RS232 或 RS485。 */
    PumpDriveFeedbackType_e last_type;        /**< 最近一次成功解析到的反馈号。 */
    PumpDriveError_e last_error;              /**< 最近一次解析错误码，成功时为 PUMP_DRIVE_ERR_NONE。 */
    PumpDriveStatus_s status;                 /**< 最近一次 `sta` 指令反馈解析出的运行状态。 */
    uint32_t last_value;                      /**< 最近一次 5 字节数值反馈的原始 32 位值。 */
    uint32_t sequence_count;                  /**< `act 0 rdcnt` 返回的离线时序运行次数。 */
    uint8_t frame_ready;                      /**< 成功解析到一帧后置 1，上层处理完可手动清 0。 */
    uint8_t last_rx_id;                       /**< 最近一次反馈帧中的设备 ID。 */
    uint16_t text_len;                        /**< text_feedback 中的有效字符数量，不含结尾 `\0`。 */
    char text_feedback[PUMP_DRIVE_TEXT_MAX];  /**< 握手、配置、离线时序脚本等 ASCII 文本反馈。 */
} PumpDrive_s;

/**
 * @brief 初始化 ISC1000 驱动，并固定注册 USART3。
 *
 * @param pump 驱动实例指针，函数内部会清零并初始化该结构体。
 * @param mode 通讯模式：RS232 不添加 ID，RS485 会在命令前添加 ID。
 * @param device_id RS485 设备 ID，范围 1~99；广播发送可使用 0，RS232 模式下只记录该值。
 * @return 1 表示初始化成功；0 表示参数为空、USART3 已被注册或底层 USART 注册失败。
 *
 * @note 本函数只注册 USART3，不会占用 USART2 或其它串口。
 */
uint8_t PumpDrive_Init(PumpDrive_s *pump, PumpDriveBusMode_e mode, uint8_t device_id);

/**
 * @brief 发送一条 ISC1000 原始 ASCII 命令。
 *
 * @param pump 驱动实例指针，必须先调用 PumpDrive_Init()。
 * @param command 不包含结尾换行符的命令正文，例如 `"hsk"`、`"set spd=2000"`。
 * @return 1 表示命令已交给 USART3 阻塞发送；0 表示参数错误或命令过长。
 *
 * @note 函数会自动追加 `\n`。RS485 模式下还会自动添加 `device_id` 前缀。
 */
uint8_t PumpDrive_SendCommand(PumpDrive_s *pump, const char *command);

/**
 * @brief 发送握手命令 `hsk`，用于读取型号、版本和发布日期。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Handshake(PumpDrive_s *pump);

/**
 * @brief 发送状态查询命令 `sta`，用于读取传感器、电机忙碌、使能和阀状态。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryStatus(PumpDrive_s *pump);

/**
 * @brief 发送参数查询命令 `cfg`，用于读取当前全部参数配置文本。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryConfig(PumpDrive_s *pump);

/**
 * @brief 发送保存命令 `sav`，将当前参数写入驱动器 Flash。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 手册说明保存耗时约 10ms；id、bdr 等参数保存后通常需要重启生效。
 */
uint8_t PumpDrive_SaveConfig(PumpDrive_s *pump);

/**
 * @brief 发送电机使能命令 `on`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Enable(PumpDrive_s *pump);

/**
 * @brief 发送电机脱机命令 `off`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Disable(PumpDrive_s *pump);

/**
 * @brief 发送停止命令 `stp`。
 * @param pump 驱动实例指针。
 * @param emergency 停止方式：0 表示减速停止；非 0 表示立即急停。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Stop(PumpDrive_s *pump, uint8_t emergency);

/**
 * @brief 发送复位归零命令 `rst`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 驱动器内部使用 Sensor1 执行原点搜索、脱离原点区和安全偏置动作。
 */
uint8_t PumpDrive_ResetHome(PumpDrive_s *pump);

/**
 * @brief 发送吸入/反向运动命令 `in N`。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveIn(PumpDrive_s *pump, uint32_t steps);

/**
 * @brief 发送排出/正向运动命令 `out N`。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveOut(PumpDrive_s *pump, uint32_t steps);

/**
 * @brief 控制输出通道 1 / 阀 1。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve1(PumpDrive_s *pump, uint8_t on);

/**
 * @brief 控制输出通道 2 / 阀 2。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve2(PumpDrive_s *pump, uint8_t on);

/**
 * @brief 设置一个整数型参数，发送格式为 `set key=value`。
 * @param pump 驱动实例指针。
 * @param key 参数键名，例如 `"spd"`、`"acc"`、`"dec"`、`"id"`。
 * @param value 参数值，按手册范围传入。
 * @return 1 表示发送成功；0 表示参数错误或发送失败。
 */
uint8_t PumpDrive_SetParamInt(PumpDrive_s *pump, const char *key, int32_t value);

/**
 * @brief 设置目标运行速度 `spd`。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetSpeed(PumpDrive_s *pump, uint32_t speed_pps);

/**
 * @brief 同时设置目标速度、加速度和减速度。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @param acc_pps2 加速度，单位 PPS^2，手册范围 1~80000。
 * @param dec_pps2 减速度，单位 PPS^2，手册范围 1~80000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetMotion(PumpDrive_s *pump, uint32_t speed_pps, uint32_t acc_pps2, uint32_t dec_pps2);

/**
 * @brief 设置运行电流 `crn` 和保持电流 `crh`。
 * @param pump 驱动实例指针。
 * @param run_current_ca 运行电流，单位厘安培；150 表示 1.50A。
 * @param hold_current_ca 保持电流，单位厘安培；20 表示 0.20A。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 使用厘安培是为了避免在嵌入式 printf 中启用浮点格式化。
 */
uint8_t PumpDrive_SetCurrentCentiAmp(PumpDrive_s *pump, uint16_t run_current_ca, uint16_t hold_current_ca);

/**
 * @brief 启动离线时序运行，发送 `act 0 start`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActStart(PumpDrive_s *pump);

/**
 * @brief 设置离线时序循环运行次数，发送 `act 0 times N`。
 * @param pump 驱动实例指针。
 * @param times 循环次数，手册范围 1~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActTimes(PumpDrive_s *pump, uint32_t times);

/**
 * @brief 清空离线时序脚本，发送 `act 0 clear`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActClear(PumpDrive_s *pump);

/**
 * @brief 读取离线时序脚本，发送 `act 0 get`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 成功反馈为 0x04，文本中多个步骤用 `|` 分隔，且该反馈帧无 BCC 校验字节。
 */
uint8_t PumpDrive_ActGet(PumpDrive_s *pump);

/**
 * @brief 读取离线时序已运行次数，发送 `act 0 rdcnt`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActReadCount(PumpDrive_s *pump);

/**
 * @brief 设置离线时序运行计数起始值，发送 `act 0 wrcnt A`。
 * @param pump 驱动实例指针。
 * @param count 起始计数值，手册范围 0~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActWriteCount(PumpDrive_s *pump, uint32_t count);

/**
 * @brief 添加/修改离线时序中的复位步骤，发送 `act X reset`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepReset(PumpDrive_s *pump, uint8_t step_index);

/**
 * @brief 添加/修改离线时序中的延时步骤，发送 `act X delay A`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param delay_ms 延时时间，单位 ms，手册范围 1~60000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepDelay(PumpDrive_s *pump, uint8_t step_index, uint32_t delay_ms);

/**
 * @brief 添加/修改离线时序中的吸入步骤，发送 `act X in step=A speed=B`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 吸入整步数，手册范围 1~60000。
 * @param speed_pps 运动速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveIn(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps);

/**
 * @brief 添加/修改离线时序中的排出步骤，发送 `act X out step=A speed=B`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 排出整步数，手册范围 1~60000。
 * @param speed_pps 运动速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveOut(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps);

/**
 * @brief 添加/修改离线时序中的阀 1 控制步骤，发送 `act X v1 Switch`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve1(PumpDrive_s *pump, uint8_t step_index, uint8_t on);

/**
 * @brief 添加/修改离线时序中的阀 2 控制步骤，发送 `act X v2 Switch`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve2(PumpDrive_s *pump, uint8_t step_index, uint8_t on);

/**
 * @brief 解析一段来自 ISC1000 的二进制反馈流。
 * @param pump 驱动实例指针。
 * @param data 接收到的数据缓冲区，可以包含噪声、半帧或多帧数据。
 * @param len 数据长度，单位 byte。
 * @return 成功解析出的反馈帧数量。
 *
 * @note 解析成功后会更新 pump->last_type、pump->status、pump->text_feedback 等字段。
 */
uint8_t PumpDrive_ParseStream(PumpDrive_s *pump, const uint8_t *data, uint16_t len);

/**
 * @brief 将 ISC1000 的 5 字节 7-bit 数据流还原为 32 位无符号整数。
 * @param raw_5bytes 连续 5 字节编码流，每个字节只使用低 7 位。
 * @return 解码后的 32 位数值。
 */
uint32_t PumpDrive_Decode5ByteToUint32(const uint8_t raw_5bytes[5]);

/**
 * @brief 计算 ISC1000 反馈帧 BCC 异或校验。
 * @param data 参与校验的数据起始地址，不包含帧头 0xFF 和帧尾 0xFE。
 * @param len 参与校验的数据长度，单位 byte。
 * @return 1 字节 BCC 原始值；发送/接收时会被拆成两个 4-bit 字节。
 */
uint8_t PumpDrive_CalcBcc(const uint8_t *data, uint16_t len);

#endif //PUMP_DRIVE_H
