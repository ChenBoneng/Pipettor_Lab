#ifndef ACTIVITY_METER_H
#define ACTIVITY_METER_H

#include <stdint.h>

/**
 * @file activity_meter.h
 * @brief RAM-100 放射性活度计 Modbus RTU 驱动模块。
 *
 * 本模块只负责活度计通信和数据解析，不直接控制配药流程、LCD 页面或 CAN 协议。
 *
 * 当前硬件连接：
 * - USART2_TX = PA2，对应原理图 UART_TX_DEVICE；
 * - USART2_RX = PA3，对应原理图 UART_RX_DEVICE；
 * - 外部接口为 RS485_DEVICE_A / RS485_DEVICE_B；
 * - 串口参数固定为 9600, 8N1。
 *
 * @note 活度计是 Modbus RTU 从站，下位机需要主动发送 0x04 读输入寄存器命令。
 */

/** 活度计 USART2 接收缓冲区长度，响应帧固定 23 字节，这里留出少量冗余。 */
#define ACTIVITY_METER_RX_BUFFER_SIZE       64U

/** RAM-100 默认 Modbus 从站地址。 */
#define ACTIVITY_METER_SLAVE_ID             1U

/** 活度计轮询周期，单位 ms。 */
#define ACTIVITY_METER_POLL_PERIOD_MS       1000U

/** 活度计响应超时时间，单位 ms。 */
#define ACTIVITY_METER_TIMEOUT_MS           500U

/** RAM-100 活度数据起始输入寄存器。 */
#define ACTIVITY_METER_REGISTER_START       0x0401U

/** 连续读取 9 个输入寄存器，共 18 字节有效数据。 */
#define ACTIVITY_METER_REGISTER_COUNT       9U

/**
 * @brief 活度计数据单位。
 */
typedef enum
{
    ACTIVITY_METER_UNIT_UCI = 0, /**< uCi，微居里。 */
    ACTIVITY_METER_UNIT_MCI,     /**< mCi，毫居里。 */
    ACTIVITY_METER_UNIT_CI,      /**< Ci，居里。 */
    ACTIVITY_METER_UNIT_BQ,      /**< Bq，贝克勒尔。 */
    ACTIVITY_METER_UNIT_KBQ,     /**< kBq，千贝克勒尔。 */
    ACTIVITY_METER_UNIT_MBQ,     /**< MBq，兆贝克勒尔。 */
    ACTIVITY_METER_UNIT_GBQ      /**< GBq，吉贝克勒尔。 */
} ActivityMeterUnit_e;

/**
 * @brief 活度计通信状态。
 *
 * 数值 0~4 与 CommunicationActivityState_e 保持一致，便于后续直接上报 CAN 状态帧。
 */
typedef enum
{
    ACTIVITY_METER_STATE_NOT_READ = 0,     /**< 尚未读到有效数据。 */
    ACTIVITY_METER_STATE_OK = 1,           /**< 最近一次读取成功。 */
    ACTIVITY_METER_STATE_TIMEOUT = 2,      /**< 等待响应超时。 */
    ACTIVITY_METER_STATE_CRC_ERROR = 3,    /**< 响应帧 CRC 校验失败。 */
    ACTIVITY_METER_STATE_BAD_RESPONSE = 4, /**< 响应帧格式错误。 */
    ACTIVITY_METER_STATE_WAITING = 5       /**< 已发送读命令，正在等待响应。 */
} ActivityMeterState_e;

/**
 * @brief 活度计最近一次解析结果。
 */
typedef struct
{
    float activity;                         /**< 当前活度值。 */
    float background;                       /**< 当前本底值。 */
    ActivityMeterUnit_e activity_unit;      /**< 活度值单位。 */
    ActivityMeterUnit_e background_unit;    /**< 本底值单位。 */
    uint16_t nuclide_id;                    /**< 核素序号，取值见 RAM-100 内置核素表。 */
    uint8_t background_subtracted;          /**< 0 未扣除本底，1 已扣除本底。 */
    uint8_t channel;                        /**< 通道号原始值，0 表示通道 1，1 表示通道 2。 */
    ActivityMeterState_e state;             /**< 最近一次通信状态。 */
    uint32_t update_count;                  /**< 成功解析次数。 */
    uint32_t last_update_ms;                /**< 最近一次成功解析时间，单位 ms。 */
} ActivityMeterData_s;

/**
 * @brief 初始化活度计驱动，并注册 USART2 DMA + IDLE 接收。
 *
 * @return 1 表示初始化成功或已经初始化；0 表示 USART2 注册失败。
 */
uint8_t ActivityMeter_Init(void);

/**
 * @brief 活度计周期维护函数。
 *
 * @note 建议放在 ModuleTask() 中周期调用。本函数负责自动轮询和超时判断。
 */
void ActivityMeter_Process(void);

/**
 * @brief 立即发送一次活度计读取命令。
 *
 * @return 1 表示读命令已经发出；0 表示模块未初始化、串口忙或正在等待上一帧响应。
 */
uint8_t ActivityMeter_RequestRead(void);

/**
 * @brief 判断活度计驱动是否已经初始化。
 *
 * @return 1 表示已经初始化；0 表示未初始化。
 */
uint8_t ActivityMeter_IsReady(void);

/**
 * @brief 读取最近一次活度计数据。
 *
 * @param data 输出参数，用于保存最近一次解析结果。
 * @return 1 表示参数合法；0 表示 data 为空。
 */
uint8_t ActivityMeter_GetData(ActivityMeterData_s *data);

/**
 * @brief 获取最近一次活度计通信状态。
 *
 * @return 当前 ActivityMeterState_e 状态值。
 */
ActivityMeterState_e ActivityMeter_GetState(void);

/**
 * @brief 把单位枚举转换成 ASCII 字符串。
 *
 * @param unit 单位枚举。
 * @return 单位字符串；未知单位返回 "--"。
 */
const char *ActivityMeter_GetUnitString(ActivityMeterUnit_e unit);

/**
 * @brief 把核素 ID 转换成 ASCII 字符串。
 *
 * @param nuclide_id RAM-100 内置核素 ID。
 * @return 核素字符串；未知 ID 返回 "Unknown"。
 */
const char *ActivityMeter_GetNuclideString(uint16_t nuclide_id);

/**
 * @brief 把 RAM-100 内部核素 ID 转换成上位机协议使用的核素质量数。
 *
 * @param nuclide_id RAM-100 内置核素表下标。
 * @return 协议 isotope 字段使用的质量数，例如 I131 返回 131；未知 ID 返回 0。
 *
 * @note RAM-100 返回的 nuclide_id 是设备内部核素表序号，不是协议里直接显示的核素值。
 *       如果把内部序号原样发给上位机，上位机只能看到 1、2、3 这类索引，无法按
 *       I-131、Tc-99m 等业务含义显示。本函数把这个转换固定在活度计驱动层，
 *       CAN 协议层只需要拿到已经符合协议定义的 isotope 数值。
 */
uint16_t ActivityMeter_GetNuclideMassNumber(uint16_t nuclide_id);

/**
 * @brief 计算标准 Modbus CRC16。
 *
 * @param data 待校验数据。
 * @param len 数据长度，单位 byte。
 * @return CRC16 原始值。发送时低字节在前，高字节在后。
 */
uint16_t ActivityMeter_CalcModbusCrc(const uint8_t *data, uint16_t len);

/**
 * @brief 解析一帧 RAM-100 响应数据。
 *
 * @param frame 响应帧起始地址。
 * @param len 响应帧长度，标准响应为 23 字节。
 * @param data 输出解析结果。
 * @return 1 表示解析成功；0 表示帧格式或 CRC 错误。
 *
 * @note 该函数主要用于调试协议解析，正式接收由 USART2 回调自动完成。
 */
uint8_t ActivityMeter_ParseFrame(const uint8_t *frame, uint16_t len, ActivityMeterData_s *data);

#endif //ACTIVITY_METER_H
