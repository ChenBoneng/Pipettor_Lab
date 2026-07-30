#include "Communication.h"
#include <string.h>
#include "bsp_can.h"
#include "can.h"

/*
 * V1.0 协议中，上位机会向多个 CAN ID 发送命令。
 * bsp_can 当前是“一个 CANInstance 对应一个接收 ID”的模型，所以这里把所有
 * 上位机输入 ID 注册成多个实例，收到任意一个 ID 都进入同一个解析回调。
 */
#define COMMUNICATION_RX_ID_COUNT       5U
#define COMMUNICATION_SEQ_DEFAULT_INDEX 6U
#define COMMUNICATION_SEQ_TAIL_INDEX    7U

static const uint16_t communication_rx_ids[COMMUNICATION_RX_ID_COUNT] = {
    COMMUNICATION_CAN_ID_AUTH_REQUEST,
    COMMUNICATION_CAN_ID_AUTH_RESPONSE,
    COMMUNICATION_CAN_ID_CONTROL,
    COMMUNICATION_CAN_ID_PARAM,
    COMMUNICATION_CAN_ID_QUERY,
};

static CANInstance *communication_can[COMMUNICATION_RX_ID_COUNT] = {0};
static volatile CommunicationHostCommand_s communication_host_command = {0};
static uint8_t communication_inited = 0U;

static void Communication_CANCallback(CANInstance *instance);
static uint8_t Communication_RegisterRxId(uint8_t index, uint16_t rx_id);
static uint8_t Communication_GetSeqFromFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);

uint8_t Communication_Init(void)
{
    if (communication_inited != 0U)
    {
        return 1U;
    }

    memset((void *)&communication_host_command, 0, sizeof(communication_host_command));

    for (uint8_t i = 0U; i < COMMUNICATION_RX_ID_COUNT; i++)
    {
        if (Communication_RegisterRxId(i, communication_rx_ids[i]) == 0U)
        {
            return 0U;
        }
    }

    communication_inited = 1U;
    return 1U;
}

uint8_t Communication_IsReady(void)
{
    return (communication_inited != 0U) ? 1U : 0U;
}

uint8_t Communication_HasNewCommand(void)
{
    return (communication_host_command.updated != 0U) ? 1U : 0U;
}

uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    /*
     * CAN 接收回调可能在中断中更新 communication_host_command。
     * 这里按字段复制，保持逻辑简单；业务层只处理最近一帧命令。
     */
    command->id = communication_host_command.id;
    command->cmd = communication_host_command.cmd;
    command->obj = communication_host_command.obj;
    command->seq = communication_host_command.seq;
    command->updated = communication_host_command.updated;
    command->frame_count = communication_host_command.frame_count;

    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        command->data[i] = communication_host_command.data[i];
    }

    return 1U;
}

void Communication_ClearNewCommandFlag(void)
{
    communication_host_command.updated = 0U;
}

uint8_t Communication_SendFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    if ((communication_inited == 0U) || (std_id > 0x7FFU) || (data == NULL))
    {
        return 0U;
    }

    return canx_send_data(&hcan, std_id, (uint8_t *)data, COMMUNICATION_CAN_FRAME_LEN);
}

uint8_t Communication_SendAck(uint8_t cmd,
                              uint8_t obj,
                              uint8_t result,
                              uint16_t error,
                              uint8_t exec_state,
                              uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = cmd;
    data[1] = obj;
    data[2] = result;
    Communication_WriteU16LE(&data[3], error);
    data[5] = exec_state;
    data[6] = seq;
    data[7] = 0U;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_ACK, data);
}

uint8_t Communication_SendStatus(const CommunicationStatus_s *status)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    if (status == NULL)
    {
        return 0U;
    }

    data[0] = status->sys_state;
    data[1] = status->step;
    data[2] = status->motor_state;
    data[3] = status->output_state;
    data[4] = status->sensor_state;
    data[5] = status->activity_state;
    Communication_WriteU16LE(&data[6], status->alarm);

    return Communication_SendFrame(COMMUNICATION_CAN_ID_STATUS, data);
}

uint8_t Communication_SendAlarm(const CommunicationAlarm_s *alarm)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    if (alarm == NULL)
    {
        return 0U;
    }

    Communication_WriteU16LE(&data[0], alarm->alarm);
    data[2] = alarm->source;
    data[3] = alarm->level;
    data[4] = alarm->detail0;
    data[5] = alarm->detail1;
    data[6] = alarm->seq;
    data[7] = 0U;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_ALARM, data);
}

uint8_t Communication_SendActivity(float activity, uint8_t unit, uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_READ_ACTIVITY;
    data[1] = 0x01U;
    /*
     * STM32F103 是小端架构，memcpy 后 Byte2 是 float 最低地址字节。
     * CAN 协议约定下位机返回给上位机的 float 使用小端 IEEE754。
     */
    (void)memcpy(&data[2], &activity, sizeof(activity));
    data[6] = unit;
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_DATA, data);
}

uint8_t Communication_SendActivityInfo(uint16_t isotope,
                                       uint8_t background_subtracted,
                                       uint8_t channel,
                                       uint8_t status,
                                       uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_READ_ACTIVITY;
    data[1] = 0x02U;
    Communication_WriteU16LE(&data[2], isotope);
    data[4] = background_subtracted;
    data[5] = channel;
    data[6] = status;
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_DATA, data);
}

uint8_t Communication_SendVersion(uint8_t hw_major,
                                  uint8_t hw_minor,
                                  uint8_t fw_major,
                                  uint8_t fw_minor,
                                  uint8_t proto_major,
                                  uint8_t proto_minor,
                                  uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_QUERY_VERSION;
    data[1] = hw_major;
    data[2] = hw_minor;
    data[3] = fw_major;
    data[4] = fw_minor;
    data[5] = proto_major;
    data[6] = proto_minor;
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_DATA, data);
}

uint8_t Communication_SendAuthChallenge(uint32_t board_id, uint16_t nonce_low, uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_AUTH_CHALLENGE;
    Communication_WriteU32LE(&data[1], board_id);
    Communication_WriteU16LE(&data[5], nonce_low);
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_AUTH_CHALLENGE, data);
}

uint8_t Communication_SendAuthChallengeExt(uint16_t nonce_high, uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_AUTH_CHALLENGE_EXT;
    Communication_WriteU16LE(&data[1], nonce_high);
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_AUTH_CHALLENGE, data);
}

uint8_t Communication_SendAuthResult(uint8_t success,
                                     uint8_t lock_state,
                                     uint8_t fail_count,
                                     uint16_t session_timeout_sec,
                                     uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_AUTH_RESULT;
    data[1] = success;
    data[2] = lock_state;
    data[3] = fail_count;
    Communication_WriteU16LE(&data[4], session_timeout_sec);
    data[6] = 0U;
    data[7] = seq;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_AUTH_RESULT, data);
}

uint8_t Communication_IsHostCommandId(uint16_t std_id)
{
    for (uint8_t i = 0U; i < COMMUNICATION_RX_ID_COUNT; i++)
    {
        if (std_id == communication_rx_ids[i])
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t Communication_CommandRequiresAuth(uint8_t cmd, uint16_t std_id)
{
    if ((std_id == COMMUNICATION_CAN_ID_AUTH_REQUEST) ||
        (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE))
    {
        return 0U;
    }

    switch (cmd)
    {
    case COMMUNICATION_CMD_STOP_PROCESS:
    case COMMUNICATION_CMD_RESET_ERROR:
    case COMMUNICATION_CMD_QUERY_STATUS:
    case COMMUNICATION_CMD_HEARTBEAT:
    case COMMUNICATION_CMD_QUERY_VERSION:
    case COMMUNICATION_CMD_AUTH_REQUEST:
        /*
         * 这些命令允许在未认证状态下执行，用于安全停止、查询、
         * 会话维持和认证本身。
         */
        return 0U;

    default:
        return 1U;
    }
}

uint16_t Communication_ReadU16LE(const uint8_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

uint32_t Communication_ReadU32LE(const uint8_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

void Communication_WriteU16LE(uint8_t *data, uint16_t value)
{
    if (data == NULL)
    {
        return;
    }

    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

void Communication_WriteU32LE(uint8_t *data, uint32_t value)
{
    if (data == NULL)
    {
        return;
    }

    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint8_t Communication_RegisterRxId(uint8_t index, uint16_t rx_id)
{
    CAN_Init_Config_s can_config = {0};

    if (index >= COMMUNICATION_RX_ID_COUNT)
    {
        return 0U;
    }

    can_config.can_handle = &hcan;
    can_config.tx_id = COMMUNICATION_CAN_ID_ACK;
    can_config.rx_id = rx_id;
    can_config.can_module_callback = Communication_CANCallback;
    can_config.id = NULL;

    communication_can[index] = CANRegister(&can_config);
    if (communication_can[index] == NULL)
    {
        return 0U;
    }

    CANSetDLC(communication_can[index], COMMUNICATION_CAN_FRAME_LEN);
    return 1U;
}

static uint8_t Communication_GetSeqFromFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    uint8_t cmd;

    if (data == NULL)
    {
        return 0U;
    }

    cmd = data[0];

    /*
     * 大多数命令 SEQ 在 Byte6。
     * 步进运动、A/B 同步运动和认证响应为了放下 16/32 位数据，把 SEQ 放在 Byte7。
     */
    if ((cmd == COMMUNICATION_CMD_MOVE_STEPPER) ||
        (cmd == COMMUNICATION_CMD_MOVE_STEPPER_BOTH) ||
        (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE))
    {
        return data[COMMUNICATION_SEQ_TAIL_INDEX];
    }

    return data[COMMUNICATION_SEQ_DEFAULT_INDEX];
}

static void Communication_CANCallback(CANInstance *instance)
{
    if ((instance == NULL) ||
        (instance->rx_len != COMMUNICATION_CAN_FRAME_LEN) ||
        (Communication_IsHostCommandId((uint16_t)instance->rx_id) == 0U))
    {
        return;
    }

    communication_host_command.id = (uint16_t)instance->rx_id;
    communication_host_command.cmd = instance->rx_buff[0];
    communication_host_command.obj = instance->rx_buff[1];
    communication_host_command.seq = Communication_GetSeqFromFrame((uint16_t)instance->rx_id,
                                                                    instance->rx_buff);

    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        communication_host_command.data[i] = instance->rx_buff[i];
    }

    communication_host_command.updated = 1U;
    communication_host_command.frame_count++;
}
