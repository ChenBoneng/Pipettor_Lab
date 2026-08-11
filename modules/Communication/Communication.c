#include "Communication.h"
#include <string.h>
#include "bsp_can.h"
#include "can.h"

/*
 * V1.0 协议中，上位机会向多个 CAN ID 发送命令。
 * bsp_can 支持“一个 CANInstance 绑定多个接收 ID”，所以这里仍按协议 ID
 * 逐个注册，上层不需要关心这些 ID 最终是否复用同一个底层实例。
 */
#define COMMUNICATION_RX_ID_COUNT              6U
#define COMMUNICATION_COMMAND_QUEUE_CAPACITY   8U
#define COMMUNICATION_FRAME_QUEUE_CAPACITY     16U
#define COMMUNICATION_SEQ_DEFAULT_INDEX        6U
#define COMMUNICATION_SEQ_TAIL_INDEX           7U

/*
 * 产品盐值，用于参与 BoardID 计算。
 *
 * 当前字节内容是 ASCII "PIPETTOR_LAB_2026"。
 * 这里的目的不是加密，而是避免直接把 STM32 UID 的 CRC32 暴露为 BoardID。
 */
static const uint8_t communication_product_salt[] =
{
    0x50U, 0x49U, 0x50U, 0x45U, 0x54U, 0x54U, 0x4FU, 0x52U,
    0x5FU, 0x4CU, 0x41U, 0x42U, 0x5FU, 0x32U, 0x30U, 0x32U,
    0x36U
};

/*
 * 厂家私有授权密钥。
 *
 * 当前轻量方案需要下位机知道该密钥，方便离线校验 LicenseKey。
 * 量产前需要替换默认值，并开启 STM32 读保护。
 */
static const uint8_t communication_factory_secret[] =
{
    0x31U, 0x39U, 0xC7U, 0x52U, 0xA6U, 0x0DU, 0x44U, 0xE8U,
    0x92U, 0x16U, 0x5AU, 0xB3U, 0xCFU, 0x27U, 0x68U, 0x10U
};

/*
 * 协议级混淆密钥，用于参与 AuthCode 计算。
 *
 * LicenseKey 用于绑定板卡，ProtocolSecret 用于避免直接重放 LicenseKey。
 */
static const uint8_t communication_protocol_secret[] =
{
    0xA4U, 0x2DU, 0x7EU, 0x19U, 0xC0U, 0x83U, 0x55U, 0x6BU,
    0xE1U, 0x37U, 0x90U, 0x4CU, 0x2AU, 0xF5U, 0x08U, 0xD6U
};

static const uint16_t communication_rx_ids[COMMUNICATION_RX_ID_COUNT] =
{
    COMMUNICATION_CAN_ID_AUTH_REQUEST,
    COMMUNICATION_CAN_ID_AUTH_RESPONSE,
    COMMUNICATION_CAN_ID_CONTROL,
    COMMUNICATION_CAN_ID_PARAM,
    COMMUNICATION_CAN_ID_QUERY,
    COMMUNICATION_CAN_ID_CONTROL_RESPONSE,
};

typedef struct
{
    uint16_t id;
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN];
} CommunicationFrame_s;

static CANInstance *communication_can[COMMUNICATION_RX_ID_COUNT] = {0};
static CommunicationFrame_s communication_rx_queue[COMMUNICATION_FRAME_QUEUE_CAPACITY];
static CommunicationFrame_s communication_tx_queue[COMMUNICATION_FRAME_QUEUE_CAPACITY];
static volatile uint8_t communication_rx_head = 0U;
static volatile uint8_t communication_rx_tail = 0U;
static volatile uint8_t communication_tx_head = 0U;
static volatile uint8_t communication_tx_tail = 0U;
static volatile uint32_t communication_rx_frame_count = 0U;
static volatile uint32_t communication_rx_drop_count = 0U;
static volatile uint32_t communication_tx_frame_count = 0U;
static volatile uint32_t communication_tx_drop_count = 0U;
static volatile CommunicationHostCommand_s communication_command_queue[COMMUNICATION_COMMAND_QUEUE_CAPACITY];
static volatile uint8_t communication_command_head = 0U;
static volatile uint8_t communication_command_tail = 0U;
static volatile uint32_t communication_command_frame_count = 0U;
static volatile uint32_t communication_command_drop_count = 0U;
static CommunicationHostCommand_s communication_last_command = {0};
static CommunicationStatus_s communication_status_snapshot = {0};
static CommunicationAuthContext_s communication_auth = {0};
static CommunicationControlContext_s communication_control = {0};
static uint8_t communication_last_auth_state = COMMUNICATION_AUTH_LOCKED;
static uint8_t communication_inited = 0U;

static void Communication_CANCallback(CANInstance *instance);
static uint8_t Communication_StoreRxFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static uint8_t Communication_TryGetRxFrame(CommunicationFrame_s *frame);
static void Communication_ProcessTxQueue(void);
static void Communication_HandleRxFrame(const CommunicationFrame_s *frame);
static uint8_t Communication_RegisterRxId(uint8_t index, uint16_t rx_id);
static uint8_t Communication_GetSeqFromFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static uint8_t Communication_StoreHostCommand(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static uint8_t Communication_NextCommandIndex(uint8_t index);
static uint8_t Communication_NextFrameIndex(uint8_t index);
static uint32_t Communication_EnterCritical(void);
static void Communication_ExitCritical(uint32_t primask);
static void Communication_HandleAuthRequest(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleAuthResponse(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleAuthHeartbeat(void);
static uint8_t Communication_HandleAuthFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleControlResponse(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_OnPcFrame(uint16_t std_id, uint8_t cmd);
static void Communication_SyncAuthState(void);
static void Communication_SetControlMode(uint8_t control_mode, uint8_t reason);
static uint8_t Communication_CanAutoEnterRemote(void);
static uint8_t Communication_CanLocalRequestRemote(void);
static uint8_t Communication_IsQueryOrSafeCommand(uint16_t std_id, uint8_t cmd);
static uint8_t Communication_IsRemoteCommandAllowed(uint16_t std_id,
                                                    uint8_t cmd,
                                                    uint8_t *result,
                                                    uint16_t *error);
static uint8_t Communication_IsValidBusinessFrame(uint16_t std_id, uint8_t cmd);
static uint8_t Communication_NextControlEventSeq(void);
static void Communication_RequestSafetyAction(uint8_t action);
static uint32_t Communication_MakeNonce(void);
static uint32_t Communication_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t length);
static uint32_t Communication_Crc32Finish(uint32_t crc);
static void Communication_Crc32AddU32(uint32_t *crc, uint32_t value);
static void Communication_Crc32AddU8(uint32_t *crc, uint8_t value);

uint8_t Communication_Init(void)
{
    if (communication_inited != 0U)
    {
        return 1U;
    }

    memset(communication_rx_queue, 0, sizeof(communication_rx_queue));
    memset(communication_tx_queue, 0, sizeof(communication_tx_queue));
    memset((void *)communication_command_queue, 0, sizeof(communication_command_queue));
    memset(&communication_last_command, 0, sizeof(communication_last_command));
    memset(&communication_status_snapshot, 0, sizeof(communication_status_snapshot));
    memset(&communication_auth, 0, sizeof(communication_auth));
    memset(&communication_control, 0, sizeof(communication_control));
    communication_rx_head = 0U;
    communication_rx_tail = 0U;
    communication_tx_head = 0U;
    communication_tx_tail = 0U;
    communication_rx_frame_count = 0U;
    communication_rx_drop_count = 0U;
    communication_tx_frame_count = 0U;
    communication_tx_drop_count = 0U;
    communication_command_head = 0U;
    communication_command_tail = 0U;
    communication_command_frame_count = 0U;
    communication_command_drop_count = 0U;

    /*
     * 板卡上电后立即计算 BoardID。
     * BoardID 固定由 STM32 UID 和产品盐值计算得到，除非 MCU 或盐值改变。
     */
    communication_auth.board_id = Communication_CalculateBoardIdFromUid();
    communication_auth.state = COMMUNICATION_AUTH_LOCKED;
    communication_auth.license_type = COMMUNICATION_LICENSE_TYPE_NORMAL;
    communication_auth.flags = COMMUNICATION_AUTH_FLAGS_NONE;
    communication_last_auth_state = COMMUNICATION_AUTH_LOCKED;

    /*
     * 设备默认面向上位机使用，但真正的动作控制权仍按新协议状态机判定：
     * 上电先处于 LOCAL，完成授权且设备空闲后自动进入 REMOTE。
     */
    communication_control.sys_state = COMMUNICATION_SYS_IDLE;
    communication_control.control_mode = COMMUNICATION_CONTROL_LOCAL;
    communication_status_snapshot.sys_state = COMMUNICATION_SYS_IDLE;
    communication_status_snapshot.step = COMMUNICATION_STEP_IDLE;
    communication_status_snapshot.activity_state = COMMUNICATION_ACTIVITY_NOT_READ;

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

void Communication_Process(void)
{
    CommunicationFrame_s frame;
    uint32_t now;

    if (communication_inited == 0U)
    {
        return;
    }

    /*
     * 先冲一次发送队列，保证上一次循环里生成的 ACK/状态帧尽快发出；
     * 再集中处理 RX 队列，所有协议解析、授权判断和业务命令缓存都在任务上下文完成。
     * 这样 CAN 中断回调只负责拷贝入队，不会在中断里调用发送、校验密钥或驱动业务状态机。
     */
    Communication_ProcessTxQueue();

    while (Communication_TryGetRxFrame(&frame) != 0U)
    {
        Communication_HandleRxFrame(&frame);
    }

    /*
     * RX 处理完成后再做超时状态机，是为了让本周期刚收到的授权心跳、控制帧或状态查询
     * 先刷新 last_heartbeat / last_remote_frame_tick，避免“刚收到帧又被同一轮判超时”。
     */
    now = HAL_GetTick();
    if ((communication_auth.state == COMMUNICATION_AUTH_UNLOCKED) &&
        ((uint32_t)(now - communication_auth.last_heartbeat) > COMMUNICATION_SESSION_TIMEOUT_MS))
    {
        /* 授权失效后拒绝新命令；已经开始的完整流程仍按断线策略执行完。 */
        communication_auth.state = COMMUNICATION_AUTH_LOCKED;
        Communication_SyncAuthState();
    }

    if ((communication_control.pc_connected != 0U) &&
        ((uint32_t)(now - communication_control.last_remote_frame_tick) >
         COMMUNICATION_PC_ONLINE_TIMEOUT_MS))
    {
        /*
         * 掉线关闭新命令入口但不改变 REMOTE 控制权。
         * 已经开始的完整流程继续运行到自身结束；没有完整流程时仍关闭可能锁存的直控输出。
         */
        communication_control.pc_connected = 0U;
        if ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE) &&
            (communication_control.remote_flow_running == 0U))
        {
            Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE);
        }
    }

    if ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING) &&
        ((uint32_t)(now - communication_control.remote_request_tick) >
         COMMUNICATION_REMOTE_REQUEST_TIMEOUT_MS))
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                     COMMUNICATION_CONTROL_REASON_HANDSHAKE_TIMEOUT);
    }

    if ((communication_control.control_mode == COMMUNICATION_CONTROL_LOCAL) &&
        (Communication_CanAutoEnterRemote() != 0U))
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE,
                                     COMMUNICATION_CONTROL_REASON_PC_AUTH_SUCCESS);
    }

    /*
     * 本轮 RX 可能又生成了 ACK、0x181、0x184 或授权响应，末尾再冲一次发送队列。
     * 保留这个顺序可以降低上位机一问一答等待时间，同时不改变原有任务调度模型。
     */
    Communication_ProcessTxQueue();
}

uint8_t Communication_IsReady(void)
{
    return (communication_inited != 0U) ? 1U : 0U;
}

uint8_t Communication_HasNewCommand(void)
{
    return (communication_command_tail != communication_command_head) ? 1U : 0U;
}

uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    if (communication_command_tail == communication_command_head)
    {
        return 0U;
    }

    /*
     * CAN 回调按顺序写入环形队列，业务层每次读取 tail 对应的一帧。
     * ClearNewCommandFlag() 再推进 tail，保持原有 Has/Get/Clear 调用习惯。
     */
    command->id = communication_command_queue[communication_command_tail].id;
    command->cmd = communication_command_queue[communication_command_tail].cmd;
    command->obj = communication_command_queue[communication_command_tail].obj;
    command->seq = communication_command_queue[communication_command_tail].seq;
    command->updated = communication_command_queue[communication_command_tail].updated;
    command->frame_count = communication_command_queue[communication_command_tail].frame_count;

    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        command->data[i] = communication_command_queue[communication_command_tail].data[i];
    }

    return 1U;
}

void Communication_ClearNewCommandFlag(void)
{
    if (communication_command_tail != communication_command_head)
    {
        communication_command_tail = Communication_NextCommandIndex(communication_command_tail);
    }
}

uint8_t Communication_SendFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    uint8_t next;
    uint32_t primask;

    if ((communication_inited == 0U) || (std_id > 0x7FFU) || (data == NULL))
    {
        return 0U;
    }

    primask = Communication_EnterCritical();

    if ((communication_tx_head == communication_tx_tail) &&
        (canx_send_data(&hcan, std_id, (uint8_t *)data, COMMUNICATION_CAN_FRAME_LEN) != 0U))
    {
        communication_tx_frame_count++;
        Communication_ExitCritical(primask);
        return 1U;
    }

    next = Communication_NextFrameIndex(communication_tx_head);
    if (next == communication_tx_tail)
    {
        communication_tx_drop_count++;
        Communication_ExitCritical(primask);
        return 0U;
    }

    communication_tx_queue[communication_tx_head].id = std_id;
    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        communication_tx_queue[communication_tx_head].data[i] = data[i];
    }
    communication_tx_head = next;
    Communication_ExitCritical(primask);
    return 1U;
}

static uint8_t Communication_StoreRxFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    uint8_t next;

    if ((std_id > 0x7FFU) || (data == NULL))
    {
        return 0U;
    }

    next = Communication_NextFrameIndex(communication_rx_head);
    if (next == communication_rx_tail)
    {
        communication_rx_drop_count++;
        return 0U;
    }

    communication_rx_queue[communication_rx_head].id = std_id;
    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        communication_rx_queue[communication_rx_head].data[i] = data[i];
    }
    __DMB();
    communication_rx_head = next;
    communication_rx_frame_count++;
    return 1U;
}

static uint8_t Communication_TryGetRxFrame(CommunicationFrame_s *frame)
{
    uint32_t primask;

    if (frame == NULL)
    {
        return 0U;
    }

    primask = Communication_EnterCritical();
    if (communication_rx_tail == communication_rx_head)
    {
        Communication_ExitCritical(primask);
        return 0U;
    }

    *frame = communication_rx_queue[communication_rx_tail];
    communication_rx_tail = Communication_NextFrameIndex(communication_rx_tail);
    Communication_ExitCritical(primask);
    return 1U;
}

static void Communication_ProcessTxQueue(void)
{
    CommunicationFrame_s *frame;
    uint32_t primask;

    if (communication_inited == 0U)
    {
        return;
    }

    for (;;)
    {
        primask = Communication_EnterCritical();
        if (communication_tx_tail == communication_tx_head)
        {
            Communication_ExitCritical(primask);
            return;
        }

        frame = &communication_tx_queue[communication_tx_tail];
        if (canx_send_data(&hcan,
                           frame->id,
                           frame->data,
                           COMMUNICATION_CAN_FRAME_LEN) == 0U)
        {
            Communication_ExitCritical(primask);
            return;
        }

        communication_tx_tail = Communication_NextFrameIndex(communication_tx_tail);
        communication_tx_frame_count++;
        Communication_ExitCritical(primask);
    }
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
    data[5] = (uint8_t)((status->activity_state & 0x3FU) |
                        ((communication_control.control_mode & 0x03U) << 6U));
    Communication_WriteU16LE(&data[6], status->alarm);
    communication_status_snapshot = *status;
    /*
     * 0x181 Byte0 是对外显示的系统状态，communication_control.sys_state 是控制权
     * 状态机内部使用的运行状态。大多数状态可以同步，但 AUTH_LOCKED 只是周期帧里
     * 给上位机看的授权提示，不能反写到控制权状态机，否则授权未完成时会把本地/远控
     * 模式判断污染成一个业务状态，后续远控申请和自动进入远控都会被误判。
     */
    if (status->sys_state != COMMUNICATION_SYS_AUTH_LOCKED)
    {
        communication_control.sys_state = status->sys_state;
    }

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

uint8_t Communication_SendPrepareResult(uint16_t actual_total_volume_x100,
                                        uint16_t measured_activity_x100,
                                        uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_SET_PARAM;
    data[1] = COMMUNICATION_OBJ_PREPARE_RESULT;
    Communication_WriteU16LE(&data[2], actual_total_volume_x100);
    Communication_WriteU16LE(&data[4], measured_activity_x100);
    data[6] = seq;
    data[7] = 0U;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_DATA, data);
}

uint8_t Communication_SendRemainResult(uint16_t remain_volume_x100,
                                       uint16_t remain_activity_x100,
                                       uint8_t flags,
                                       uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_SET_PARAM;
    data[1] = COMMUNICATION_OBJ_REMAIN_RESULT;
    Communication_WriteU16LE(&data[2], remain_volume_x100);
    Communication_WriteU16LE(&data[4], remain_activity_x100);
    data[6] = seq;
    data[7] = flags;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_DATA, data);
}

uint8_t Communication_SendProcessResult(uint8_t process_id,
                                        uint8_t result,
                                        uint16_t detail,
                                        uint8_t final_step,
                                        uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = COMMUNICATION_CMD_START_PROCESS;
    data[1] = COMMUNICATION_OBJ_PROCESS_RESULT;
    data[2] = process_id;
    data[3] = result;
    Communication_WriteU16LE(&data[4], detail);
    data[6] = seq;
    data[7] = final_step;

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

uint8_t Communication_SendAuthChallenge(uint32_t board_id, uint32_t nonce)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    /*
     * 新版授权协议中，0x091 的 8 字节刚好放满 BoardID 和 nonce。
     * 该帧不携带 CMD，也不回传 SEQ。
     */
    Communication_WriteU32LE(&data[0], board_id);
    Communication_WriteU32LE(&data[4], nonce);

    return Communication_SendFrame(COMMUNICATION_CAN_ID_AUTH_CHALLENGE, data);
}

uint8_t Communication_SendAuthResult(uint8_t auth_state,
                                     uint8_t result,
                                     uint8_t fail_count,
                                     uint8_t license_type,
                                     uint16_t session_timeout_sec,
                                     uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = auth_state;
    data[1] = result;
    data[2] = fail_count;
    data[3] = license_type;
    Communication_WriteU16LE(&data[4], session_timeout_sec);
    data[6] = seq;
    data[7] = 0U;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_AUTH_RESULT, data);
}

uint8_t Communication_SendControlEvent(uint8_t event,
                                       uint8_t control_mode,
                                       uint8_t sys_state,
                                       uint8_t reason,
                                       uint8_t seq)
{
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN] = {0};

    data[0] = event;
    data[1] = control_mode;
    data[2] = sys_state;
    data[3] = reason;
    data[6] = seq;
    data[7] = 0U;

    return Communication_SendFrame(COMMUNICATION_CAN_ID_CONTROL_EVENT, data);
}

uint8_t Communication_GetAuthState(void)
{
    return communication_auth.state;
}

uint8_t Communication_IsUnlocked(void)
{
    return (communication_auth.state == COMMUNICATION_AUTH_UNLOCKED) ? 1U : 0U;
}

uint32_t Communication_GetBoardId(void)
{
    return communication_auth.board_id;
}

uint32_t Communication_GetNonce(void)
{
    return communication_auth.nonce;
}

const CommunicationAuthContext_s *Communication_GetAuthContext(void)
{
    return &communication_auth;
}

uint8_t Communication_GetControlMode(void)
{
    return communication_control.control_mode;
}

uint8_t Communication_IsRemoteControlActive(void)
{
    return ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE) ||
            (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_PAUSED) ||
            (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING)) ? 1U : 0U;
}

const CommunicationControlContext_s *Communication_GetControlContext(void)
{
    return &communication_control;
}

void Communication_SetSystemState(uint8_t sys_state)
{
    communication_control.sys_state = sys_state;
}

void Communication_OnLocalFlowStarted(void)
{
    communication_control.local_flow_running = 1U;
    communication_control.sys_state = COMMUNICATION_SYS_RUNNING;
}

void Communication_OnLocalFlowStopped(void)
{
    communication_control.local_flow_running = 0U;
    if ((communication_control.alarm_active == 0U) &&
        (communication_control.estop_active == 0U))
    {
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
    }
}

void Communication_OnRemoteFlowStarted(void)
{
    communication_control.remote_flow_running = 1U;
    communication_control.sys_state = COMMUNICATION_SYS_RUNNING;
}

void Communication_OnRemoteFlowStopped(void)
{
    communication_control.remote_flow_running = 0U;
    if ((communication_control.alarm_active == 0U) &&
        (communication_control.estop_active == 0U))
    {
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
    }
}

void Communication_OnAlarmChanged(uint8_t active)
{
    communication_control.alarm_active = (active != 0U) ? 1U : 0U;
    if (communication_control.alarm_active != 0U)
    {
        communication_control.remote_resume_allowed = 0U;
        communication_control.sys_state = COMMUNICATION_SYS_ALARM;
        if (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE)
        {
            Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE);
            Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE_PAUSED,
                                         COMMUNICATION_CONTROL_REASON_ALARM);
        }
    }
}

void Communication_OnEStopChanged(uint8_t active)
{
    communication_control.estop_active = (active != 0U) ? 1U : 0U;
    if (communication_control.estop_active != 0U)
    {
        communication_control.remote_resume_allowed = 0U;
        communication_control.sys_state = COMMUNICATION_SYS_ESTOP;
        if (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE)
        {
            Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE);
            Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE_PAUSED,
                                         COMMUNICATION_CONTROL_REASON_ESTOP);
        }
    }
}

void Communication_OnRemoteResetError(void)
{
    uint32_t now = HAL_GetTick();

    /*
     * 上位机急停后的复位不是“本地启动键恢复远控”。
     * 它需要明确清掉急停/报警锁存，并让 0x181 重新回到 IDLE + REMOTE + alarm=0，
     * 否则后续动作帧会继续被 IsRemoteCommandAllowed() 当作非远控或报警状态拒绝。
     */
    communication_control.pc_connected = 1U;
    communication_control.last_remote_frame_tick = now;
    communication_control.pc_authorized =
        (communication_auth.state == COMMUNICATION_AUTH_UNLOCKED) ? 1U : 0U;
    communication_control.alarm_active = 0U;
    communication_control.estop_active = 0U;
    communication_control.remote_resume_allowed = 0U;
    communication_control.local_flow_running = 0U;
    communication_control.local_takeover_latched = 0U;
    communication_control.remote_request_seq = 0U;
    communication_control.remote_request_tick = 0U;
    communication_control.pending_safety_action = COMMUNICATION_SAFETY_ACTION_NONE;
    communication_control.sys_state = COMMUNICATION_SYS_IDLE;

    if (communication_control.pc_authorized != 0U)
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE,
                                     COMMUNICATION_CONTROL_REASON_KEY_RESET);
    }
    else
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                     COMMUNICATION_CONTROL_REASON_KEY_RESET);
    }
}

void Communication_OnLocalPauseKey(void)
{
    if (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE)
    {
        communication_control.sys_state = COMMUNICATION_SYS_PAUSED;
        communication_control.remote_resume_allowed = 1U;
        Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE);
        Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE_PAUSED,
                                     COMMUNICATION_CONTROL_REASON_KEY_PAUSE);
        (void)Communication_SendControlEvent(COMMUNICATION_CONTROL_EVENT_LOCAL_PAUSE,
                                             communication_control.control_mode,
                                             communication_control.sys_state,
                                             COMMUNICATION_CONTROL_REASON_KEY_PAUSE,
                                             Communication_NextControlEventSeq());
    }
}

void Communication_OnLocalStartKey(void)
{
    if ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_PAUSED) &&
        (communication_control.remote_resume_allowed != 0U) &&
        (communication_control.pc_connected != 0U) &&
        (communication_control.pc_authorized != 0U) &&
        (communication_control.alarm_active == 0U) &&
        (communication_control.estop_active == 0U))
    {
        communication_control.remote_resume_allowed = 0U;
        communication_control.local_takeover_latched = 0U;
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
        Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE,
                                     COMMUNICATION_CONTROL_REASON_KEY_START);
    }
}

void Communication_OnLocalResetKey(void)
{
    if (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_PAUSED)
    {
        communication_control.remote_resume_allowed = 0U;
        communication_control.local_flow_running = 0U;
        communication_control.local_takeover_latched = 1U;
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
        Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_TAKEOVER_LOCAL);
        Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                     COMMUNICATION_CONTROL_REASON_KEY_RESET);
        (void)Communication_SendControlEvent(COMMUNICATION_CONTROL_EVENT_LOCAL_TAKEOVER,
                                             communication_control.control_mode,
                                             communication_control.sys_state,
                                             COMMUNICATION_CONTROL_REASON_KEY_RESET,
                                             Communication_NextControlEventSeq());
    }
    else if (communication_control.control_mode == COMMUNICATION_CONTROL_LOCAL)
    {
        /*
         * 复位键在本机侧含义是“本地收回/保持控制权”，不再承担解除接管锁存的职责。
         * 如果在 LOCAL 下按复位就清锁存，下一轮收到上位机心跳可能马上自动回 REMOTE，
         * 用户就会看到上位机显示远控、LCD 却还停在本地页面的错觉。
         * 解除锁存统一交给远控键处理，语义更单一。
         */
        communication_control.remote_resume_allowed = 0U;
        communication_control.local_flow_running = 0U;
        communication_control.local_takeover_latched = 1U;
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
    }
}

void Communication_OnLocalRemoteKey(void)
{
    uint8_t seq;

    /*
     * 用户主动按远控键，含义就是解除本地接管并重新申请上位机控制权。
     * 先清锁存再判断条件，避免“本地已接管”把本次远控申请挡在门外，
     * 也避免必须先按一次复位、再按远控才能恢复的绕路操作。
     */
    communication_control.local_takeover_latched = 0U;
    if ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_PAUSED) ||
        (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING))
    {
        if ((communication_control.local_flow_running != 0U) ||
            (communication_control.alarm_active != 0U) ||
            (communication_control.estop_active != 0U))
        {
            return;
        }

        communication_control.remote_resume_allowed = 0U;
        communication_control.sys_state = COMMUNICATION_SYS_IDLE;
        Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                     COMMUNICATION_CONTROL_REASON_KEY_REMOTE);
    }

    if (Communication_CanLocalRequestRemote() == 0U)
    {
        return;
    }

    seq = Communication_NextControlEventSeq();
    communication_control.remote_request_seq = seq;
    communication_control.remote_request_tick = HAL_GetTick();
    Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE_SWITCHING,
                                 COMMUNICATION_CONTROL_REASON_KEY_REMOTE);
    (void)Communication_SendControlEvent(COMMUNICATION_CONTROL_EVENT_REMOTE_REQUEST,
                                         communication_control.control_mode,
                                         communication_control.sys_state,
                                         COMMUNICATION_CONTROL_REASON_KEY_REMOTE,
                                         seq);
}

uint8_t Communication_ConsumeSafetyAction(void)
{
    uint8_t action = communication_control.pending_safety_action;

    communication_control.pending_safety_action = COMMUNICATION_SAFETY_ACTION_NONE;
    return action;
}

uint8_t Communication_IsCommandAllowed(uint8_t cmd, uint16_t std_id)
{
    if (Communication_CommandRequiresAuth(cmd, std_id) == 0U)
    {
        return 1U;
    }

    return Communication_IsUnlocked();
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
        (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE) ||
        (std_id == COMMUNICATION_CAN_ID_CONTROL_RESPONSE))
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
         * 这些命令允许在未授权状态下执行，用于安全停止、查询和认证本身。
         * READ_ACTIVITY 属于关键业务数据读取，按授权文档要求仍需要授权。
         */
        return 0U;

    default:
        return 1U;
    }
}

static uint8_t Communication_IsValidBusinessFrame(uint16_t std_id, uint8_t cmd)
{
    switch (std_id)
    {
    case COMMUNICATION_CAN_ID_CONTROL:
        return ((cmd == COMMUNICATION_CMD_START_PROCESS) ||
                (cmd == COMMUNICATION_CMD_STOP_PROCESS) ||
                (cmd == COMMUNICATION_CMD_RESET_ERROR) ||
                (cmd == COMMUNICATION_CMD_MOVE_STEPPER) ||
                (cmd == COMMUNICATION_CMD_VALVE_CONTROL) ||
                (cmd == COMMUNICATION_CMD_PUMP_CONTROL) ||
                (cmd == COMMUNICATION_CMD_MOVE_STEPPER_BOTH) ||
                (cmd == COMMUNICATION_CMD_STOP_OBJECT)) ? 1U : 0U;

    case COMMUNICATION_CAN_ID_PARAM:
        return (cmd == COMMUNICATION_CMD_SET_PARAM) ? 1U : 0U;

    case COMMUNICATION_CAN_ID_QUERY:
        return ((cmd == COMMUNICATION_CMD_READ_ACTIVITY) ||
                (cmd == COMMUNICATION_CMD_QUERY_STATUS) ||
                (cmd == COMMUNICATION_CMD_HEARTBEAT) ||
                (cmd == COMMUNICATION_CMD_QUERY_VERSION)) ? 1U : 0U;

    default:
        return 0U;
    }
}

uint32_t Communication_CalculateLicenseKey(uint32_t board_id, uint8_t license_type)
{
    uint32_t crc = 0xFFFFFFFFUL;

    /*
     * LicenseKey 绑定 BoardID 和授权类型。
     *
     * 上位机/厂家授权工具必须按完全相同顺序输入：
     * 1. board_id 小端 4 字节；
     * 2. license_type 1 字节；
     * 3. communication_factory_secret 原始字节；
     * 4. final xor。
     */
    Communication_Crc32AddU32(&crc, board_id);
    Communication_Crc32AddU8(&crc, license_type);
    crc = Communication_Crc32Update(crc,
                                    communication_factory_secret,
                                    (uint32_t)sizeof(communication_factory_secret));

    return Communication_Crc32Finish(crc);
}

uint32_t Communication_CalculateAuthCode(uint32_t board_id,
                                         uint32_t nonce,
                                         uint32_t license_key,
                                         uint8_t license_type,
                                         uint8_t flags)
{
    uint32_t crc = 0xFFFFFFFFUL;

    /*
     * AuthCode 是本次会话的授权响应。
     * 加入 nonce 后，旧的 0x092 授权响应不能长期重放。
     */
    Communication_Crc32AddU32(&crc, board_id);
    Communication_Crc32AddU32(&crc, nonce);
    Communication_Crc32AddU32(&crc, license_key);
    Communication_Crc32AddU8(&crc, license_type);
    Communication_Crc32AddU8(&crc, flags);
    crc = Communication_Crc32Update(crc,
                                    communication_protocol_secret,
                                    (uint32_t)sizeof(communication_protocol_secret));

    return Communication_Crc32Finish(crc);
}

uint32_t Communication_CalculateBoardIdFromUid(void)
{
    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t *uid = (const uint8_t *)COMMUNICATION_STM32_UID_BASE;

    /*
     * 注意这里读取的是 UID 内存原始字节顺序，不把 UID 当作 uint32 数组重排。
     * 上位机通常不需要重新计算 BoardID，只需要显示 0x091 返回的 BoardID。
     */
    crc = Communication_Crc32Update(crc, uid, COMMUNICATION_STM32_UID_SIZE);
    crc = Communication_Crc32Update(crc,
                                    communication_product_salt,
                                    (uint32_t)sizeof(communication_product_salt));

    return Communication_Crc32Finish(crc);
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

    (void)std_id;

    if (data == NULL)
    {
        return 0U;
    }

    cmd = data[0];

    /*
     * 新版协议只有单轴 MOVE_STEPPER 的 SEQ 在 Byte7。
     * 其它控制、参数、查询和授权响应都使用 Byte6。
     *
     * 这里统一封装 SEQ 读取，避免业务层在 ACK 时忘记 MOVE_STEPPER 的特殊格式。
     * 如果后续 MOVE_STEPPER_BOTH 的帧格式正式启用，再按新文档在这里补充即可。
     */
    if (cmd == COMMUNICATION_CMD_MOVE_STEPPER)
    {
        return data[COMMUNICATION_SEQ_TAIL_INDEX];
    }

    return data[COMMUNICATION_SEQ_DEFAULT_INDEX];
}

static uint8_t Communication_StoreHostCommand(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    CommunicationHostCommand_s *command;
    uint8_t next;

    /*
     * 上位机通常会连续发送 PREPARE_PARAM、PREPARE_VOLUME_PARAM、
     * START_PROCESS。这里用小环形队列保序，避免旧的“最近一帧”缓存覆盖参数。
     */
    next = Communication_NextCommandIndex(communication_command_head);
    if (next == communication_command_tail)
    {
        communication_command_drop_count++;
        return 0U;
    }

    command = (CommunicationHostCommand_s *)&communication_command_queue[communication_command_head];
    command->id = std_id;
    command->cmd = data[0];
    command->obj = data[1];
    command->seq = Communication_GetSeqFromFrame(std_id, data);
    command->updated = 1U;
    command->frame_count = ++communication_command_frame_count;

    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        command->data[i] = data[i];
    }

    communication_last_command = *command;
    communication_command_head = next;
    return 1U;
}

static uint8_t Communication_NextCommandIndex(uint8_t index)
{
    return (uint8_t)((index + 1U) % COMMUNICATION_COMMAND_QUEUE_CAPACITY);
}

static uint8_t Communication_NextFrameIndex(uint8_t index)
{
    return (uint8_t)((index + 1U) % COMMUNICATION_FRAME_QUEUE_CAPACITY);
}

static uint32_t Communication_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

static void Communication_ExitCritical(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Communication_HandleAuthRequest(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    /*
     * 0x090 Byte0=0x01：上位机请求授权挑战。
     *
     * 收到新的授权请求时：
     * 1. 生成新的 nonce；
     * 2. 重新回到 LOCKED；
     * 3. 返回 0x091，内容为 BoardID + nonce。
     *
     * 0x091 八字节刚好放满 BoardID 和 nonce，因此不回传请求 SEQ。
     */
    (void)data;
    communication_auth.nonce = Communication_MakeNonce();
    if (communication_auth.nonce == 0U)
    {
        communication_auth.nonce = 1U;
    }

    communication_auth.state = COMMUNICATION_AUTH_LOCKED;
    Communication_SyncAuthState();
    (void)Communication_SendAuthChallenge(communication_auth.board_id, communication_auth.nonce);
}

static void Communication_HandleAuthResponse(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    uint32_t rx_auth_code;
    uint32_t license_key;
    uint32_t expected_auth_code;
    uint8_t license_type;
    uint8_t flags;
    uint8_t seq;

    /*
     * 0x092 授权响应帧格式：
     * Byte0~Byte3 = AuthCode，小端 uint32；
     * Byte4       = license_type；
     * Byte5       = flags；
     * Byte6       = SEQ；
     * Byte7       = 0。
     *
     * 下位机不直接接收 LicenseKey，而是根据当前 BoardID 和 licenseType 自己算出
     * LicenseKey，再和 nonce、flags 一起计算期望 AuthCode。
     * 这样上位机只需要发送一次会话相关的 AuthCode，不能简单重放固定密钥帧。
     */
    rx_auth_code = Communication_ReadU32LE(&data[0]);
    license_type = data[4];
    flags = data[5];
    seq = data[6];

    if (communication_auth.nonce == 0U)
    {
        /*
         * 还没发过 0x091 challenge，就收到 0x092 response。
         * 这种 response 没有基于有效 nonce，必须拒绝。
         */
        communication_auth.state = COMMUNICATION_AUTH_LOCKED;
        communication_auth.fail_count++;
        Communication_SyncAuthState();
        (void)Communication_SendAuthResult(communication_auth.state,
                                           COMMUNICATION_AUTH_RESULT_BAD_NONCE,
                                           communication_auth.fail_count,
                                           communication_auth.license_type,
                                           (uint16_t)(COMMUNICATION_SESSION_TIMEOUT_MS / 1000U),
                                           seq);
        return;
    }

    license_key = Communication_CalculateLicenseKey(communication_auth.board_id, license_type);
    expected_auth_code = Communication_CalculateAuthCode(communication_auth.board_id,
                                                         communication_auth.nonce,
                                                         license_key,
                                                         license_type,
                                                         flags);

    if (rx_auth_code == expected_auth_code)
    {
        /*
         * AuthCode 正确：
         * - 解锁动作命令权限；
         * - 记录当前心跳时间；
         * - 清零失败次数；
         * - 保存本次授权类型和标志位。
         */
        communication_auth.state = COMMUNICATION_AUTH_UNLOCKED;
        communication_auth.last_heartbeat = HAL_GetTick();
        communication_auth.fail_count = 0U;
        communication_auth.license_type = license_type;
        communication_auth.flags = flags;
        Communication_SyncAuthState();

        (void)Communication_SendAuthResult(communication_auth.state,
                                           COMMUNICATION_AUTH_RESULT_OK,
                                           communication_auth.fail_count,
                                           communication_auth.license_type,
                                           (uint16_t)(COMMUNICATION_SESSION_TIMEOUT_MS / 1000U),
                                           seq);
    }
    else
    {
        /*
         * AuthCode 不一致：
         * - 保持锁定；
         * - 增加失败次数；
         * - 返回 BAD_KEY，方便上位机给出明确提示。
         */
        communication_auth.state = COMMUNICATION_AUTH_LOCKED;
        communication_auth.fail_count++;
        Communication_SyncAuthState();

        (void)Communication_SendAuthResult(communication_auth.state,
                                           COMMUNICATION_AUTH_RESULT_BAD_KEY,
                                           communication_auth.fail_count,
                                           communication_auth.license_type,
                                           (uint16_t)(COMMUNICATION_SESSION_TIMEOUT_MS / 1000U),
                                           seq);
    }
}

static void Communication_HandleAuthHeartbeat(void)
{
    /*
     * 0x090 Byte0=0x02：授权心跳。
     *
     * 心跳只在已授权状态下有效。
     * 未授权状态下收到心跳不会自动解锁，必须重新走完整授权握手。
     * 心跳不写入 communication_host_command，避免占用 LCD 联调显示。
     */
    if (communication_auth.state == COMMUNICATION_AUTH_UNLOCKED)
    {
        communication_auth.last_heartbeat = HAL_GetTick();
    }
}

static uint8_t Communication_HandleAuthFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    /*
     * 授权帧在通信层内部消化，不再交给业务层。
     *
     * 这样 MachineCMDTask 后续只需要面对真正的业务命令：
     * - 控制命令 0x100；
     * - 参数命令 0x101；
     * - 查询命令 0x102。
     *
     * 授权请求、授权响应、授权心跳都不会刷新“最近命令”缓存。
     */
    if (std_id == COMMUNICATION_CAN_ID_AUTH_REQUEST)
    {
        if (data[0] == COMMUNICATION_AUTH_MSG_REQUEST)
        {
            Communication_HandleAuthRequest(data);
            return 1U;
        }

        if (data[0] == COMMUNICATION_AUTH_MSG_HEARTBEAT)
        {
            Communication_HandleAuthHeartbeat();
            return 1U;
        }

        /*
         * 未识别的 0x090 子命令仍视为已处理，避免误进入普通控制命令缓存。
         */
        return 1U;
    }

    if (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE)
    {
        Communication_HandleAuthResponse(data);
        return 1U;
    }

    return 0U;
}

static void Communication_HandleControlResponse(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    if (data == NULL)
    {
        return;
    }

    /*
     * 0x103 只用于回应本地远控键发出的 REMOTE_REQUEST。
     * 必须同时匹配模式、SEQ、上位机结果和当前安全条件，避免过期响应抢控制权。
     */
    if ((communication_control.control_mode != COMMUNICATION_CONTROL_REMOTE_SWITCHING) ||
        (data[6] != communication_control.remote_request_seq) ||
        (data[0] != COMMUNICATION_CONTROL_RESPONSE_ACCEPT_REMOTE) ||
        (data[1] != COMMUNICATION_CONTROL_RESPONSE_OK) ||
        (communication_control.sys_state != COMMUNICATION_SYS_IDLE) ||
        (communication_control.pc_connected == 0U) ||
        (communication_control.pc_authorized == 0U) ||
        (communication_control.alarm_active != 0U) ||
        (communication_control.estop_active != 0U))
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                     COMMUNICATION_CONTROL_REASON_KEY_REMOTE);
        return;
    }

    communication_control.local_takeover_latched = 0U;
    communication_control.remote_resume_allowed = 0U;
    Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE,
                                 COMMUNICATION_CONTROL_REASON_KEY_REMOTE);
}

static void Communication_OnPcFrame(uint16_t std_id, uint8_t cmd)
{
    (void)cmd;

    if ((std_id == COMMUNICATION_CAN_ID_AUTH_REQUEST) ||
        (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE) ||
        (std_id == COMMUNICATION_CAN_ID_CONTROL) ||
        (std_id == COMMUNICATION_CAN_ID_PARAM) ||
        (std_id == COMMUNICATION_CAN_ID_QUERY) ||
        (std_id == COMMUNICATION_CAN_ID_CONTROL_RESPONSE))
    {
        communication_control.pc_connected = 1U;
        communication_control.last_remote_frame_tick = HAL_GetTick();
    }
}

static void Communication_SyncAuthState(void)
{
    uint8_t was_authorized = communication_control.pc_authorized;

    communication_control.pc_authorized =
        (communication_auth.state == COMMUNICATION_AUTH_UNLOCKED) ? 1U : 0U;
    communication_last_auth_state = communication_auth.state;

    if ((was_authorized == 0U) && (communication_control.pc_authorized != 0U))
    {
        communication_control.pc_connected = 1U;
        communication_control.last_remote_frame_tick = HAL_GetTick();
    }

    if (communication_control.pc_authorized == 0U)
    {
        communication_control.remote_resume_allowed = 0U;
        /*
         * REMOTE 下授权失效只拒绝后续动作命令，不改变控制权或打断已开始流程。
         * 上位机重新完成授权后可直接继续发送下一条命令。
         */
        if ((communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE) &&
            (communication_control.remote_flow_running == 0U))
        {
            Communication_RequestSafetyAction(COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE);
        }
        else if (communication_control.control_mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING)
        {
            Communication_SetControlMode(COMMUNICATION_CONTROL_LOCAL,
                                         COMMUNICATION_CONTROL_REASON_HEARTBEAT_TIMEOUT);
        }
    }
}

static void Communication_SetControlMode(uint8_t control_mode, uint8_t reason)
{
    if (communication_control.control_mode == control_mode)
    {
        return;
    }

    communication_control.control_mode = control_mode;
    (void)Communication_SendControlEvent(COMMUNICATION_CONTROL_EVENT_MODE_CHANGED,
                                         communication_control.control_mode,
                                         communication_control.sys_state,
                                         reason,
                                         Communication_NextControlEventSeq());
}

static uint8_t Communication_CanAutoEnterRemote(void)
{
    return ((communication_control.pc_connected != 0U) &&
            (communication_control.pc_authorized != 0U) &&
            (communication_control.local_takeover_latched == 0U) &&
            (communication_control.local_flow_running == 0U) &&
            (communication_control.alarm_active == 0U) &&
            (communication_control.estop_active == 0U) &&
            (communication_control.sys_state == COMMUNICATION_SYS_IDLE)) ? 1U : 0U;
}

static uint8_t Communication_CanLocalRequestRemote(void)
{
    return ((communication_control.control_mode == COMMUNICATION_CONTROL_LOCAL) &&
            (communication_control.pc_authorized != 0U) &&
            (communication_control.local_flow_running == 0U) &&
            (communication_control.alarm_active == 0U) &&
            (communication_control.estop_active == 0U) &&
            (communication_control.sys_state == COMMUNICATION_SYS_IDLE)) ? 1U : 0U;
}

static uint8_t Communication_IsQueryOrSafeCommand(uint16_t std_id, uint8_t cmd)
{
    if ((std_id == COMMUNICATION_CAN_ID_AUTH_REQUEST) ||
        (std_id == COMMUNICATION_CAN_ID_AUTH_RESPONSE) ||
        (std_id == COMMUNICATION_CAN_ID_CONTROL_RESPONSE) ||
        (std_id == COMMUNICATION_CAN_ID_QUERY))
    {
        return 1U;
    }

    return ((std_id == COMMUNICATION_CAN_ID_CONTROL) &&
            ((cmd == COMMUNICATION_CMD_STOP_PROCESS) ||
             (cmd == COMMUNICATION_CMD_RESET_ERROR))) ? 1U : 0U;
}

static uint8_t Communication_IsRemoteCommandAllowed(uint16_t std_id,
                                                    uint8_t cmd,
                                                    uint8_t *result,
                                                    uint16_t *error)
{
    if (result != NULL)
    {
        *result = COMMUNICATION_RESULT_OK;
    }
    if (error != NULL)
    {
        *error = COMMUNICATION_ERROR_NONE;
    }

    if (Communication_IsQueryOrSafeCommand(std_id, cmd) != 0U)
    {
        return 1U;
    }

    if ((communication_control.control_mode != COMMUNICATION_CONTROL_REMOTE) ||
        (communication_control.pc_connected == 0U))
    {
        if (result != NULL)
        {
            *result = COMMUNICATION_RESULT_BUSY;
        }
        if (error != NULL)
        {
            *error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
        return 0U;
    }

    if ((communication_control.sys_state == COMMUNICATION_SYS_ALARM) ||
        (communication_control.sys_state == COMMUNICATION_SYS_ESTOP) ||
        (communication_control.alarm_active != 0U) ||
        (communication_control.estop_active != 0U))
    {
        if (result != NULL)
        {
            *result = COMMUNICATION_RESULT_ALARM;
        }
        if (error != NULL)
        {
            *error = (communication_control.estop_active != 0U) ?
                     COMMUNICATION_ERROR_ESTOP : COMMUNICATION_ERROR_PROCESS_FAILED;
        }
        return 0U;
    }

    return 1U;
}

static uint8_t Communication_NextControlEventSeq(void)
{
    communication_control.next_event_seq++;
    return communication_control.next_event_seq;
}

static void Communication_RequestSafetyAction(uint8_t action)
{
    /*
     * 本地接管优先级高于远控暂停，避免尚未消费的接管动作被暂停覆盖。
     */
    if ((action == COMMUNICATION_SAFETY_ACTION_TAKEOVER_LOCAL) ||
        (communication_control.pending_safety_action == COMMUNICATION_SAFETY_ACTION_NONE))
    {
        communication_control.pending_safety_action = action;
    }
}

static uint32_t Communication_MakeNonce(void)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t tick = HAL_GetTick();

    /*
     * STM32F103 没有硬件随机数，这里用 tick、BoardID、上一次 nonce 混合出伪随机数。
     * 该 nonce 主要用于防止简单录制重放，不用于强密码学安全。
     */
    Communication_Crc32AddU32(&crc, tick);
    Communication_Crc32AddU32(&crc, communication_auth.board_id);
    Communication_Crc32AddU32(&crc, communication_auth.nonce);

    return Communication_Crc32Finish(crc);
}

static uint32_t Communication_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return crc;
    }

    for (i = 0U; i < length; i++)
    {
        /*
         * 使用反射 CRC32 的逐 bit 算法：
         * - 每次先把输入字节异或到 crc 低 8 位；
         * - 低位为 1 时右移并异或 0xEDB88320；
         * - 低位为 0 时只右移。
         */
        crc ^= (uint32_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint32_t Communication_Crc32Finish(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

static void Communication_Crc32AddU32(uint32_t *crc, uint32_t value)
{
    uint8_t data[4];

    if (crc == NULL)
    {
        return;
    }

    Communication_WriteU32LE(data, value);
    *crc = Communication_Crc32Update(*crc, data, sizeof(data));
}

static void Communication_Crc32AddU8(uint32_t *crc, uint8_t value)
{
    if (crc == NULL)
    {
        return;
    }

    *crc = Communication_Crc32Update(*crc, &value, 1U);
}

static void Communication_HandleRxFrame(const CommunicationFrame_s *frame)
{
    uint16_t std_id;
    uint8_t cmd;
    uint8_t obj;
    uint8_t seq;
    uint8_t result;
    uint16_t error;

    if ((frame == NULL) ||
        (Communication_IsHostCommandId(frame->id) == 0U))
    {
        /*
         * CAN 中断只负责收帧入队；协议解析统一在任务上下文执行。
         * 这里再检查一次 ID，避免后续扩展时把非协议帧误送进准入逻辑。
         */
        return;
    }

    std_id = frame->id;
    cmd = frame->data[0];
    obj = frame->data[1];
    seq = Communication_GetSeqFromFrame(std_id, frame->data);
    Communication_OnPcFrame(std_id, cmd);

    /*
     * 第一步先处理授权相关帧。
     * 授权帧不是业务命令，不需要写入 command cache，也不需要普通 ACK。
     */
    if (Communication_HandleAuthFrame(std_id, frame->data) != 0U)
    {
        return;
    }

    if (std_id == COMMUNICATION_CAN_ID_CONTROL_RESPONSE)
    {
        Communication_HandleControlResponse(frame->data);
        return;
    }

    if (Communication_IsValidBusinessFrame(std_id, cmd) == 0U)
    {
        if ((std_id == COMMUNICATION_CAN_ID_CONTROL) ||
            (std_id == COMMUNICATION_CAN_ID_PARAM))
        {
            (void)Communication_SendAck(cmd,
                                        obj,
                                        COMMUNICATION_RESULT_UNSUPPORTED,
                                        COMMUNICATION_ERROR_NONE,
                                        0U,
                                        seq);
        }
        return;
    }

    if (Communication_IsCommandAllowed(cmd, std_id) == 0U)
    {
        /*
         * 未授权动作类命令不写入业务命令缓存，直接返回未授权 ACK。
         * 这样业务层不会误执行已经被通信层拒绝的命令。
         *
         * 文档要求未授权 ACK 固定为：
         * RESULT = 0x06；
         * ERROR  = 0x0008。
         */
        (void)Communication_SendAck(cmd,
                                    obj,
                                    COMMUNICATION_RESULT_UNAUTHORIZED,
                                    COMMUNICATION_ERROR_UNAUTHORIZED,
                                    0U,
                                    seq);
        return;
    }

    /*
     * 上位机断开重连后，第一帧业务命令本身就证明 PC 已在线。
     * 如果仍等到 Communication_Process() 末尾才自动进入 REMOTE，
     * 同一批连续发送的参数帧和 START_PROCESS 会先被 LOCAL 状态拒掉。
     */
    if ((communication_control.control_mode == COMMUNICATION_CONTROL_LOCAL) &&
        (Communication_CanAutoEnterRemote() != 0U))
    {
        Communication_SetControlMode(COMMUNICATION_CONTROL_REMOTE,
                                     COMMUNICATION_CONTROL_REASON_PC_AUTH_SUCCESS);
    }

    if (Communication_IsRemoteCommandAllowed(std_id, cmd, &result, &error) == 0U)
    {
        (void)Communication_SendAck(cmd, obj, result, error, 0U, seq);
        return;
    }

    /*
     * QUERY_STATUS 和普通心跳由通信层即时处理，不占用业务命令队列。
     * 状态查询只需要 communication_status_snapshot + control_mode，通信层本来就有完整数据；
     * 如果再转给 MachineCMD，会多等一个任务周期，而且可能和远控动作命令抢同一个业务队列。
     *
     * READ_ACTIVITY、QUERY_VERSION 仍入队给 MachineCMD 回复 0x183 数据帧，因为它们需要业务层
     * 读取活度计缓存或版本信息，不能只靠通信层快照完成。
     */
    if ((std_id == COMMUNICATION_CAN_ID_QUERY) &&
        (cmd == COMMUNICATION_CMD_HEARTBEAT))
    {
        return;
    }
    if ((std_id == COMMUNICATION_CAN_ID_QUERY) &&
        (cmd == COMMUNICATION_CMD_QUERY_STATUS))
    {
        CommunicationStatus_s status = communication_status_snapshot;

        status.sys_state = communication_control.sys_state;
        (void)Communication_SendStatus(&status);
        return;
    }

    if (Communication_StoreHostCommand(std_id, frame->data) == 0U)
    {
        (void)Communication_SendAck(cmd,
                                    obj,
                                    COMMUNICATION_RESULT_BUSY,
                                    COMMUNICATION_ERROR_CAN_TIMEOUT,
                                    0U,
                                    seq);
        return;
    }

    if (((std_id == COMMUNICATION_CAN_ID_CONTROL) ||
         (std_id == COMMUNICATION_CAN_ID_PARAM)) &&
        (cmd != COMMUNICATION_CMD_START_PROCESS) &&
        (cmd != COMMUNICATION_CMD_SET_PARAM))
    {
        /*
         * 这里的 OK 只表示通信层已接收并缓存命令，不代表机械动作已经完成。
         * START_PROCESS/SET_PARAM 需要业务层按参数完整性和流程顺序返回 ACK，
         * 避免通信层先回 OK 后业务层又发现缺参数或状态不允许。
         *
         * 后续如果业务层需要返回更精细的 BUSY/FAILED，可在执行层补充状态帧或报警帧。
         *
         * 查询命令 0x102 不在这里统一 ACK：
         * - QUERY_STATUS 应返回 0x181；
         * - READ_ACTIVITY / QUERY_VERSION 应返回 0x183。
         */
        (void)Communication_SendAck(cmd,
                                    obj,
                                    COMMUNICATION_RESULT_OK,
                                    COMMUNICATION_ERROR_NONE,
                                    0U,
                                    seq);
    }
}

static void Communication_CANCallback(CANInstance *instance)
{
    if ((instance == NULL) ||
        (instance->rx_len != COMMUNICATION_CAN_FRAME_LEN) ||
        (Communication_IsHostCommandId((uint16_t)instance->rx_id) == 0U))
    {
        return;
    }

    /*
     * CAN 回调运行在 HAL 接收中断里，只做固定长度复制和入队。
     * 授权、ACK、控制权状态机和业务命令缓存都放到 Communication_Process()。
     * 这样做的核心原因是：ACK 发送、授权 CRC、远控状态切换都可能继续访问 CAN 队列或业务状态，
     * 放在中断里容易造成重入和响应时间抖动；回调只入队可以把中断时间压到固定长度。
     */
    (void)Communication_StoreRxFrame((uint16_t)instance->rx_id, instance->rx_buff);
}
