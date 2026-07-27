#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

/**
 * @file Communication.h
 * @brief 上位机 CAN 通信协议层。
 *
 * 本模块只负责和上位机之间的 CAN 协议解析，不直接控制电机或泵。
 * 当前协议为 1 帧标准 CAN 数据帧，接收 ID 为 0x3FE，发送 ID 预留为 0x3FF。
 *
 * 接收帧格式：
 * - Byte0：电机状态，由上位机定义具体含义，本模块只原样保存；
 * - Byte1：水量整数部分，单位 mL；
 * - Byte2：水量小数部分，精度 2 位，例如 25 表示 0.25 mL；
 * - Byte3~Byte7：保留位，当前只保存，不参与业务判断。
 */

/** 下位机发送给上位机的标准 CAN ID，当前仅保留，暂不主动发送业务内容。 */
#define COMMUNICATION_CAN_TX_ID      0x3FFU

/** 上位机发送给下位机的标准 CAN ID。 */
#define COMMUNICATION_CAN_RX_ID      0x3FEU

/** 上位机通信协议固定使用 8 字节 CAN 数据帧。 */
#define COMMUNICATION_CAN_FRAME_LEN  8U

/** 当前协议中保留位数量：Byte3~Byte7，共 5 字节。 */
#define COMMUNICATION_RESERVED_LEN   5U

/**
 * @brief 上位机通过 CAN 下发的最新命令。
 *
 * 水量同时保留拆分后的整数/小数部分，以及放大 100 倍后的整数值。
 * 这样后续业务层既可以按协议字段显示，也可以直接用 water_ml_x100 做计算，
 * 避免在 STM32F103 上引入不必要的浮点运算。
 */
typedef struct
{
    uint8_t motor_state;                          /**< Byte0，电机状态，具体状态码由上位机协议定义。 */
    uint8_t water_integer_ml;                     /**< Byte1，水量整数部分，单位 mL。 */
    uint8_t water_decimal;                        /**< Byte2，水量小数部分，范围建议 0~99，表示两位小数。 */
    uint16_t water_ml_x100;                       /**< 水量放大 100 倍后的整数值，例如 12.34mL 保存为 1234。 */
    uint8_t reserved[COMMUNICATION_RESERVED_LEN]; /**< Byte3~Byte7，保留位，当前不解析，只保存原始值。 */
    uint8_t updated;                              /**< 收到新命令后置 1，业务层读取处理后可清零。 */
    uint32_t frame_count;                         /**< 成功解析到的上位机命令帧计数，便于调试通信是否正常。 */
} CommunicationHostCommand_s;

/**
 * @brief 初始化上位机 CAN 通信模块。
 *
 * @return 1 表示初始化成功；0 表示 CAN 注册失败。
 *
 * @note 本函数会注册 CAN 接收 ID 0x3FE，并保留发送 ID 0x3FF。
 *       当前协议没有主动发送给上位机的内容，因此初始化后主要依赖 CAN 接收回调更新命令。
 */
uint8_t Communication_Init(void);

/**
 * @brief 判断 communication 模块是否已经成功注册 CAN。
 *
 * @return 1 表示已初始化；0 表示尚未初始化或初始化失败。
 */
uint8_t Communication_IsReady(void);

/**
 * @brief 判断是否收到过尚未清除的新命令。
 *
 * @return 1 表示有新命令；0 表示没有新命令。
 *
 * @note 本函数只看 updated 标志，不会自动清除标志。
 */
uint8_t Communication_HasNewCommand(void);

/**
 * @brief 读取最近一次上位机 CAN 命令。
 *
 * @param command 输出参数，用于保存最新命令内容，不能为 NULL。
 * @return 1 表示读取成功；0 表示参数为空。
 *
 * @note 读取不会自动清除 updated 标志，业务层处理完成后可调用
 *       Communication_ClearNewCommandFlag() 清除。
 */
uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command);

/**
 * @brief 清除新命令标志。
 *
 * @note 只清除 updated，不会清空最近一次命令内容。
 */
void Communication_ClearNewCommandFlag(void);

/**
 * @brief 使用预留的 0x3FF ID 发送一帧原始 CAN 数据。
 *
 * @param data 8 字节发送数据，不能为 NULL。
 * @return 1 表示发送成功；0 表示模块未初始化、参数错误或 CAN 发送失败。
 *
 * @note 当前上位机协议暂时没有定义下位机上报内容，本函数只是保留发送通道，
 *       不会被模块内部主动调用。
 */
uint8_t Communication_SendFrame(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);

#endif //COMMUNICATION_H
