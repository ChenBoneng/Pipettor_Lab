#include "Communication.h"
#include "bsp_can.h"
#include "can.h"

/*
 * 上位机命令帧字节位置。
 * 这里把位置集中定义，后续如果上位机协议调整，只需要改这一处。
 */
#define COMMUNICATION_BYTE_MOTOR_STATE      0U
#define COMMUNICATION_BYTE_WATER_INTEGER    1U
#define COMMUNICATION_BYTE_WATER_DECIMAL    2U
#define COMMUNICATION_BYTE_RESERVED_START   3U
#define COMMUNICATION_CAN_MIN_RX_LEN        3U
#define COMMUNICATION_CAN_TX_TIMEOUT_MS     2.0f

static CANInstance *communication_can = NULL;
static volatile CommunicationHostCommand_s communication_host_command = {0};

static void Communication_CANCallback(CANInstance *instance);
static void Communication_SaveReservedBytes(CANInstance *instance);

uint8_t Communication_Init(void)
{
    CAN_Init_Config_s can_config = {0};

    /*
     * communication_can 不为空，说明之前已经注册过 CAN。
     * 重复初始化直接返回成功，避免重复占用 CAN 过滤器组。
     */
    if (communication_can != NULL)
    {
        return 1U;
    }

    can_config.can_handle = &hcan;
    can_config.tx_id = COMMUNICATION_CAN_TX_ID;
    can_config.rx_id = COMMUNICATION_CAN_RX_ID;
    can_config.can_module_callback = Communication_CANCallback;
    can_config.id = NULL;

    communication_can = CANRegister(&can_config);
    if (communication_can == NULL)
    {
        return 0U;
    }

    /*
     * 当前协议使用固定 8 字节帧：
     * Byte0~Byte2 是有效业务数据，Byte3~Byte7 是保留位。
     */
    CANSetDLC(communication_can, COMMUNICATION_CAN_FRAME_LEN);

    return 1U;
}

uint8_t Communication_IsReady(void)
{
    return (communication_can != NULL) ? 1U : 0U;
}

uint8_t Communication_HasNewCommand(void)
{
    return (communication_host_command.updated != 0U) ? 1U : 0U;
}

uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command)
{
    uint8_t i;

    if (command == NULL)
    {
        return 0U;
    }

    /*
     * CAN 接收回调可能在中断中更新 communication_host_command。
     * 这里按字段复制，保持代码简单清楚；业务层通常只需要最近一帧命令。
     */
    command->motor_state = communication_host_command.motor_state;
    command->water_integer_ml = communication_host_command.water_integer_ml;
    command->water_decimal = communication_host_command.water_decimal;
    command->water_ml_x100 = communication_host_command.water_ml_x100;
    command->updated = communication_host_command.updated;
    command->frame_count = communication_host_command.frame_count;

    for (i = 0U; i < COMMUNICATION_RESERVED_LEN; i++)
    {
        command->reserved[i] = communication_host_command.reserved[i];
    }

    return 1U;
}

void Communication_ClearNewCommandFlag(void)
{
    communication_host_command.updated = 0U;
}

uint8_t Communication_SendFrame(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    uint8_t i;

    if ((communication_can == NULL) || (data == NULL))
    {
        return 0U;
    }

    for (i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        communication_can->tx_buff[i] = data[i];
    }

    CANSetDLC(communication_can, COMMUNICATION_CAN_FRAME_LEN);
    return CANTransmit(communication_can, COMMUNICATION_CAN_TX_TIMEOUT_MS);
}

static void Communication_CANCallback(CANInstance *instance)
{
    uint8_t water_integer;
    uint8_t water_decimal;

    if ((instance == NULL) || (instance->rx_len < COMMUNICATION_CAN_MIN_RX_LEN))
    {
        return;
    }

    /*
     * 协议解析：
     * Byte0：电机状态码，本模块不解释具体业务含义，只保存原始值；
     * Byte1：水量整数部分，单位 mL；
     * Byte2：水量小数部分，两位小数，例如 5 表示 0.05mL，25 表示 0.25mL。
     */
    water_integer = instance->rx_buff[COMMUNICATION_BYTE_WATER_INTEGER];
    water_decimal = instance->rx_buff[COMMUNICATION_BYTE_WATER_DECIMAL];

    communication_host_command.motor_state = instance->rx_buff[COMMUNICATION_BYTE_MOTOR_STATE];
    communication_host_command.water_integer_ml = water_integer;
    communication_host_command.water_decimal = water_decimal;
    communication_host_command.water_ml_x100 = (uint16_t)water_integer * 100U + water_decimal;

    Communication_SaveReservedBytes(instance);

    /*
     * updated 表示“有一帧新的上位机命令等待业务层处理”。
     * frame_count 用来调试 CAN 是否持续收到有效帧。
     */
    communication_host_command.updated = 1U;
    communication_host_command.frame_count++;
}

static void Communication_SaveReservedBytes(CANInstance *instance)
{
    uint8_t i;
    uint8_t source_index;

    for (i = 0U; i < COMMUNICATION_RESERVED_LEN; i++)
    {
        source_index = COMMUNICATION_BYTE_RESERVED_START + i;

        if (source_index < instance->rx_len)
        {
            communication_host_command.reserved[i] = instance->rx_buff[source_index];
        }
        else
        {
            /*
             * 正常情况下上位机会发送 8 字节帧。
             * 如果实际 DLC 不足 8 字节，缺失的保留位按 0 保存，避免留下旧数据。
             */
            communication_host_command.reserved[i] = 0U;
        }
    }
}
