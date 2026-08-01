#include "Communication.h"
#include <string.h>
#include "bsp_can.h"
#include "can.h"

/*
 * V1.0 协议中，上位机会向多个 CAN ID 发送命令。
 * bsp_can 支持“一个 CANInstance 绑定多个接收 ID”，所以这里仍按协议 ID
 * 逐个注册，上层不需要关心这些 ID 最终是否复用同一个底层实例。
 */
#define COMMUNICATION_RX_ID_COUNT       5U
#define COMMUNICATION_SEQ_DEFAULT_INDEX 6U
#define COMMUNICATION_SEQ_TAIL_INDEX    7U

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
};

static CANInstance *communication_can[COMMUNICATION_RX_ID_COUNT] = {0};
static volatile CommunicationHostCommand_s communication_host_command = {0};
static CommunicationAuthContext_s communication_auth = {0};
static uint8_t communication_inited = 0U;

static void Communication_CANCallback(CANInstance *instance);
static uint8_t Communication_RegisterRxId(uint8_t index, uint16_t rx_id);
static uint8_t Communication_GetSeqFromFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_StoreHostCommand(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleAuthRequest(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleAuthResponse(const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
static void Communication_HandleAuthHeartbeat(void);
static uint8_t Communication_HandleAuthFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);
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

    memset((void *)&communication_host_command, 0, sizeof(communication_host_command));
    memset(&communication_auth, 0, sizeof(communication_auth));

    /*
     * 板卡上电后立即计算 BoardID。
     * BoardID 固定由 STM32 UID 和产品盐值计算得到，除非 MCU 或盐值改变。
     */
    communication_auth.board_id = Communication_CalculateBoardIdFromUid();
    communication_auth.state = COMMUNICATION_AUTH_LOCKED;
    communication_auth.license_type = COMMUNICATION_LICENSE_TYPE_NORMAL;
    communication_auth.flags = COMMUNICATION_AUTH_FLAGS_NONE;

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
    uint32_t now;

    if ((communication_inited == 0U) ||
        (communication_auth.state != COMMUNICATION_AUTH_UNLOCKED))
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - communication_auth.last_heartbeat) > COMMUNICATION_SESSION_TIMEOUT_MS)
    {
        /*
         * 心跳超时后只锁定通信权限。
         * 是否立即停止机械动作，应由机器主流程根据业务安全策略决定。
         */
        communication_auth.state = COMMUNICATION_AUTH_LOCKED;
    }
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

uint8_t Communication_GetAuthState(void)
{
    Communication_Process();
    return communication_auth.state;
}

uint8_t Communication_IsUnlocked(void)
{
    return (Communication_GetAuthState() == COMMUNICATION_AUTH_UNLOCKED) ? 1U : 0U;
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
         * 这些命令允许在未授权状态下执行，用于安全停止、查询和认证本身。
         * READ_ACTIVITY 属于关键业务数据读取，按授权文档要求仍需要授权。
         */
        return 0U;

    default:
        return 1U;
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

static void Communication_StoreHostCommand(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN])
{
    /*
     * 当前通信层只缓存“最近一帧已通过权限检查的命令”。
     *
     * 这样做有两个考虑：
     * 1. 现在 MachineCMDTask 还没有正式接管 CAN 命令队列，单帧缓存最简单；
     * 2. LCD 联调页面只需要显示最近一条上位机命令含义。
     *
     * 后续如果上位机连续高速下发动作命令，再把这里升级为环形队列。
     */
    communication_host_command.id = std_id;
    communication_host_command.cmd = data[0];
    communication_host_command.obj = data[1];
    communication_host_command.seq = Communication_GetSeqFromFrame(std_id, data);

    for (uint8_t i = 0U; i < COMMUNICATION_CAN_FRAME_LEN; i++)
    {
        communication_host_command.data[i] = data[i];
    }

    communication_host_command.updated = 1U;
    communication_host_command.frame_count++;
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

static void Communication_CANCallback(CANInstance *instance)
{
    uint16_t std_id;
    uint8_t cmd;
    uint8_t obj;
    uint8_t seq;

    if ((instance == NULL) ||
        (instance->rx_len != COMMUNICATION_CAN_FRAME_LEN) ||
        (Communication_IsHostCommandId((uint16_t)instance->rx_id) == 0U))
    {
        /*
         * 协议只接受上位机方向的标准 8 字节数据帧。
         * bsp_can 已经按标准 ID 过滤到对应实例，这里再检查长度和 ID，
         * 主要是为了防止误配置或后续扩展时把非协议帧送进来。
         */
        return;
    }

    std_id = (uint16_t)instance->rx_id;

    /*
     * 第一步先处理授权相关帧。
     * 授权帧不是业务命令，不需要写入 command cache，也不需要普通 ACK。
     */
    if (Communication_HandleAuthFrame(std_id, instance->rx_buff) != 0U)
    {
        return;
    }

    cmd = instance->rx_buff[0];
    obj = instance->rx_buff[1];
    seq = Communication_GetSeqFromFrame(std_id, instance->rx_buff);

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

    Communication_StoreHostCommand(std_id, instance->rx_buff);

    if ((std_id == COMMUNICATION_CAN_ID_CONTROL) || (std_id == COMMUNICATION_CAN_ID_PARAM))
    {
        /*
         * 这里的 OK 只表示通信层已接收并缓存命令，不代表机械动作已经完成。
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
