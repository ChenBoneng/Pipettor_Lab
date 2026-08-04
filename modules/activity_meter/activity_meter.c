#include "activity_meter.h"
#include <string.h>
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"

/*
 * RAM-100 使用 Modbus RTU 子集：
 * - 主站请求：01 04 04 01 00 09 CRC_L CRC_H；
 * - 从站响应：01 04 12 + 18 字节数据 + CRC_L CRC_H。
 *
 * 本模块固定使用 USART2，避免和 pump_drive 的 USART3 冲突。
 */
#define ACTIVITY_METER_FUNC_READ_INPUT      0x04U
#define ACTIVITY_METER_DATA_BYTES           18U
#define ACTIVITY_METER_REQUEST_LEN          8U
#define ACTIVITY_METER_RESPONSE_LEN         23U

static USARTInstance *activity_meter_usart = NULL;
static ActivityMeterData_s activity_meter_data = {0};
static uint8_t activity_meter_inited = 0U;
static uint8_t activity_meter_waiting = 0U;
static uint32_t activity_meter_last_request_ms = 0U;
static uint32_t activity_meter_last_poll_ms = 0U;

static void ActivityMeter_UsartRxCallback(void);
static uint32_t ActivityMeter_GetMs(void);
static void ActivityMeter_SetState(ActivityMeterState_e state);
static void ActivityMeter_StoreData(const ActivityMeterData_s *data);
static uint8_t ActivityMeter_ParseStream(const uint8_t *data,
                                         uint16_t len,
                                         ActivityMeterData_s *result,
                                         ActivityMeterState_e *error);
static uint8_t ActivityMeter_ParseCandidate(const uint8_t *frame, ActivityMeterData_s *result);
static uint16_t ActivityMeter_ReadU16BE(const uint8_t *data);
static float ActivityMeter_ReadFloatBE(const uint8_t *data);

static const char *const activity_meter_unit_name[] = {
    "uCi",
    "mCi",
    "Ci",
    "Bq",
    "kBq",
    "MBq",
    "GBq",
};

static const char *const activity_meter_nuclide_name[] = {
    "Unknown",
    "I131",
    "Tc99m",
    "F18",
    "I125",
    "Cs137",
    "Ba133",
    "Am241",
    "Co60",
    "Ra226",
    "Eu152",
    "Co57",
    "Ga67",
    "Ga68",
    "In111",
    "I123",
    "Xe133",
    "Tl201",
    "C14",
    "Cs131",
    "Cr51",
    "Lu177",
    "Ra223",
    "Y90",
    "O15",
    "C11",
    "N13",
    "Cu64",
    "Cu67",
    "Mo99",
    "Na22",
    "Na24",
    "Ir192",
};

/*
 * RAM-100 的 nuclide_id 是上面 activity_meter_nuclide_name[] 的数组下标，
 * 但 CAN 协议里的 isotope 字段要求的是核素质量数。两张表保持同一顺序，
 * 这样解析到内部 ID 后，可以用 O(1) 查表得到上位机需要显示的数值。
 *
 * 例如：
 * - nuclide_id = 1 表示 I131，协议 isotope 应发送 131；
 * - nuclide_id = 2 表示 Tc99m，协议 isotope 应发送 99；
 * - 未知或越界 ID 发送 0，让上位机按未知核素处理。
 */
static const uint16_t activity_meter_nuclide_mass_number[] = {
    0U,
    131U,
    99U,
    18U,
    125U,
    137U,
    133U,
    241U,
    60U,
    226U,
    152U,
    57U,
    67U,
    68U,
    111U,
    123U,
    133U,
    201U,
    14U,
    131U,
    51U,
    177U,
    223U,
    90U,
    15U,
    11U,
    13U,
    64U,
    67U,
    99U,
    22U,
    24U,
    192U,
};

/**
 * @brief 获取当前毫秒时间轴。
 *
 * @return HAL_GetTick() 返回的系统毫秒数。
 */
static uint32_t ActivityMeter_GetMs(void)
{
    return HAL_GetTick();
}

/**
 * @brief 只更新通信状态，不覆盖最近一次有效活度值。
 *
 * @param state 新通信状态。
 */
static void ActivityMeter_SetState(ActivityMeterState_e state)
{
    activity_meter_data.state = state;
}

/**
 * @brief 保存一帧成功解析出的活度计数据。
 *
 * @param data 解析结果。
 */
static void ActivityMeter_StoreData(const ActivityMeterData_s *data)
{
    if (data == NULL)
    {
        return;
    }

    activity_meter_data.activity = data->activity;
    activity_meter_data.background = data->background;
    activity_meter_data.activity_unit = data->activity_unit;
    activity_meter_data.background_unit = data->background_unit;
    activity_meter_data.nuclide_id = data->nuclide_id;
    activity_meter_data.background_subtracted = data->background_subtracted;
    activity_meter_data.channel = data->channel;
    activity_meter_data.state = ACTIVITY_METER_STATE_OK;
    activity_meter_data.update_count++;
    activity_meter_data.last_update_ms = ActivityMeter_GetMs();
}

/**
 * @brief 初始化活度计驱动并注册 USART2。
 */
uint8_t ActivityMeter_Init(void)
{
    USART_Init_Config_s usart_config;

    if (activity_meter_inited != 0U)
    {
        return 1U;
    }

    memset(&activity_meter_data, 0, sizeof(activity_meter_data));
    activity_meter_data.state = ACTIVITY_METER_STATE_NOT_READ;

    /*
     * 这里固定注册 huart2：
     * 1. 文档要求活度计 9600, 8N1；
     * 2. 原理图中 PA2/PA3 连接到 DEVICE 通信口；
     * 3. USART3 已留给 pump_drive，不在这里复用。
     */
    usart_config.recv_buff_size = ACTIVITY_METER_RX_BUFFER_SIZE;
    usart_config.usart_handle = &huart2;
    usart_config.module_callback = ActivityMeter_UsartRxCallback;

    activity_meter_usart = USARTRegister(&usart_config);
    if (activity_meter_usart == NULL)
    {
        ActivityMeter_SetState(ACTIVITY_METER_STATE_BAD_RESPONSE);
        return 0U;
    }

    activity_meter_last_poll_ms = ActivityMeter_GetMs() - ACTIVITY_METER_POLL_PERIOD_MS;
    activity_meter_inited = 1U;
    return 1U;
}

/**
 * @brief 周期轮询活度计并处理超时。
 */
void ActivityMeter_Process(void)
{
    uint32_t now_ms;

    if (activity_meter_inited == 0U)
    {
        return;
    }

    now_ms = ActivityMeter_GetMs();

    if (activity_meter_waiting != 0U)
    {
        if ((now_ms - activity_meter_last_request_ms) >= ACTIVITY_METER_TIMEOUT_MS)
        {
            activity_meter_waiting = 0U;
            ActivityMeter_SetState(ACTIVITY_METER_STATE_TIMEOUT);
        }

        return;
    }

    if ((now_ms - activity_meter_last_poll_ms) >= ACTIVITY_METER_POLL_PERIOD_MS)
    {
        (void)ActivityMeter_RequestRead();
    }
}

/**
 * @brief 发送一次 RAM-100 读输入寄存器命令。
 */
uint8_t ActivityMeter_RequestRead(void)
{
    uint8_t request[ACTIVITY_METER_REQUEST_LEN] = {0};
    uint16_t crc;

    if ((activity_meter_inited == 0U) || (activity_meter_usart == NULL))
    {
        return 0U;
    }

    if (activity_meter_waiting != 0U)
    {
        return 0U;
    }

    if (USARTIsReady(activity_meter_usart) == 0U)
    {
        return 0U;
    }

    request[0] = ACTIVITY_METER_SLAVE_ID;
    request[1] = ACTIVITY_METER_FUNC_READ_INPUT;
    request[2] = (uint8_t)((ACTIVITY_METER_REGISTER_START >> 8U) & 0xFFU);
    request[3] = (uint8_t)(ACTIVITY_METER_REGISTER_START & 0xFFU);
    request[4] = 0U;
    request[5] = ACTIVITY_METER_REGISTER_COUNT;

    crc = ActivityMeter_CalcModbusCrc(request, 6U);
    request[6] = (uint8_t)(crc & 0xFFU);
    request[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    USARTSend(activity_meter_usart, request, ACTIVITY_METER_REQUEST_LEN, USART_TRANSFER_BLOCKING);

    activity_meter_waiting = 1U;
    activity_meter_last_request_ms = ActivityMeter_GetMs();
    activity_meter_last_poll_ms = activity_meter_last_request_ms;
    ActivityMeter_SetState(ACTIVITY_METER_STATE_WAITING);

    return 1U;
}

/**
 * @brief 判断活度计模块是否就绪。
 */
uint8_t ActivityMeter_IsReady(void)
{
    return (activity_meter_inited != 0U) ? 1U : 0U;
}

/**
 * @brief 读取最近一次活度计数据。
 */
uint8_t ActivityMeter_GetData(ActivityMeterData_s *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    data->activity = activity_meter_data.activity;
    data->background = activity_meter_data.background;
    data->activity_unit = activity_meter_data.activity_unit;
    data->background_unit = activity_meter_data.background_unit;
    data->nuclide_id = activity_meter_data.nuclide_id;
    data->background_subtracted = activity_meter_data.background_subtracted;
    data->channel = activity_meter_data.channel;
    data->state = activity_meter_data.state;
    data->update_count = activity_meter_data.update_count;
    data->last_update_ms = activity_meter_data.last_update_ms;

    return 1U;
}

/**
 * @brief 获取最近一次通信状态。
 */
ActivityMeterState_e ActivityMeter_GetState(void)
{
    return activity_meter_data.state;
}

/**
 * @brief 单位枚举转 ASCII 字符串。
 */
const char *ActivityMeter_GetUnitString(ActivityMeterUnit_e unit)
{
    if ((uint32_t)unit >= (sizeof(activity_meter_unit_name) / sizeof(activity_meter_unit_name[0])))
    {
        return "--";
    }

    return activity_meter_unit_name[unit];
}

/**
 * @brief 核素 ID 转 ASCII 字符串。
 */
const char *ActivityMeter_GetNuclideString(uint16_t nuclide_id)
{
    if (nuclide_id >= (sizeof(activity_meter_nuclide_name) / sizeof(activity_meter_nuclide_name[0])))
    {
        return activity_meter_nuclide_name[0];
    }

    return activity_meter_nuclide_name[nuclide_id];
}

/**
 * @brief 内部核素 ID 转协议质量数。
 */
uint16_t ActivityMeter_GetNuclideMassNumber(uint16_t nuclide_id)
{
    if (nuclide_id >=
        (sizeof(activity_meter_nuclide_mass_number) / sizeof(activity_meter_nuclide_mass_number[0])))
    {
        return 0U;
    }

    return activity_meter_nuclide_mass_number[nuclide_id];
}

/**
 * @brief 计算 Modbus CRC16。
 */
uint16_t ActivityMeter_CalcModbusCrc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/**
 * @brief 解析一帧标准 RAM-100 响应帧。
 */
uint8_t ActivityMeter_ParseFrame(const uint8_t *frame, uint16_t len, ActivityMeterData_s *data)
{
    ActivityMeterState_e error = ACTIVITY_METER_STATE_BAD_RESPONSE;

    if ((frame == NULL) || (data == NULL) || (len < ACTIVITY_METER_RESPONSE_LEN))
    {
        return 0U;
    }

    return ActivityMeter_ParseStream(frame, len, data, &error);
}

/**
 * @brief 在接收流中寻找一帧有效 RAM-100 响应。
 *
 * @param data 接收缓冲区。
 * @param len 接收长度。
 * @param result 输出解析结果。
 * @param error 输出失败原因。
 * @return 1 表示找到并解析成功；0 表示没有有效响应帧。
 */
static uint8_t ActivityMeter_ParseStream(const uint8_t *data,
                                         uint16_t len,
                                         ActivityMeterData_s *result,
                                         ActivityMeterState_e *error)
{
    uint8_t crc_error_seen = 0U;

    if (error != NULL)
    {
        *error = ACTIVITY_METER_STATE_BAD_RESPONSE;
    }

    if ((data == NULL) || (result == NULL) || (len < ACTIVITY_METER_RESPONSE_LEN))
    {
        return 0U;
    }

    for (uint16_t offset = 0U; offset <= (uint16_t)(len - ACTIVITY_METER_RESPONSE_LEN); offset++)
    {
        const uint8_t *frame = &data[offset];
        uint16_t crc_calc;
        uint16_t crc_recv;

        if ((frame[0] != ACTIVITY_METER_SLAVE_ID) ||
            (frame[1] != ACTIVITY_METER_FUNC_READ_INPUT) ||
            (frame[2] != ACTIVITY_METER_DATA_BYTES))
        {
            continue;
        }

        crc_recv = (uint16_t)frame[ACTIVITY_METER_RESPONSE_LEN - 2U] |
                   ((uint16_t)frame[ACTIVITY_METER_RESPONSE_LEN - 1U] << 8U);
        crc_calc = ActivityMeter_CalcModbusCrc(frame, ACTIVITY_METER_RESPONSE_LEN - 2U);
        if (crc_recv != crc_calc)
        {
            crc_error_seen = 1U;
            continue;
        }

        return ActivityMeter_ParseCandidate(frame, result);
    }

    if ((error != NULL) && (crc_error_seen != 0U))
    {
        *error = ACTIVITY_METER_STATE_CRC_ERROR;
    }

    return 0U;
}

/**
 * @brief 解析已经通过帧头和 CRC 检查的 RAM-100 响应帧。
 *
 * @param frame 指向 23 字节响应帧。
 * @param result 输出解析结果。
 * @return 1 表示解析成功；0 表示参数为空。
 */
static uint8_t ActivityMeter_ParseCandidate(const uint8_t *frame, ActivityMeterData_s *result)
{
    const uint8_t *payload;

    if ((frame == NULL) || (result == NULL))
    {
        return 0U;
    }

    payload = &frame[3];

    memset(result, 0, sizeof(ActivityMeterData_s));
    result->activity = ActivityMeter_ReadFloatBE(&payload[0]);
    result->activity_unit = (ActivityMeterUnit_e)ActivityMeter_ReadU16BE(&payload[4]);
    result->background = ActivityMeter_ReadFloatBE(&payload[6]);
    result->background_unit = (ActivityMeterUnit_e)ActivityMeter_ReadU16BE(&payload[10]);
    result->nuclide_id = ActivityMeter_ReadU16BE(&payload[12]);
    result->background_subtracted = (ActivityMeter_ReadU16BE(&payload[14]) != 0U) ? 1U : 0U;
    result->channel = (uint8_t)ActivityMeter_ReadU16BE(&payload[16]);
    result->state = ACTIVITY_METER_STATE_OK;

    return 1U;
}

/**
 * @brief 读取大端 uint16。
 */
static uint16_t ActivityMeter_ReadU16BE(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8U) | (uint16_t)data[1];
}

/**
 * @brief 读取大端 IEEE754 float32。
 */
static float ActivityMeter_ReadFloatBE(const uint8_t *data)
{
    union
    {
        uint32_t u32;
        float f32;
    } value;

    value.u32 = ((uint32_t)data[0] << 24U) |
                ((uint32_t)data[1] << 16U) |
                ((uint32_t)data[2] << 8U) |
                (uint32_t)data[3];

    return value.f32;
}

/**
 * @brief USART2 接收到一段数据后的回调处理。
 */
static void ActivityMeter_UsartRxCallback(void)
{
    ActivityMeterData_s parsed_data;
    ActivityMeterState_e error_state = ACTIVITY_METER_STATE_BAD_RESPONSE;

    if ((activity_meter_usart == NULL) ||
        (activity_meter_usart->rx_buffer_finished == NULL) ||
        (activity_meter_usart->rx_len == 0U))
    {
        return;
    }

    if (ActivityMeter_ParseStream(activity_meter_usart->rx_buffer_finished,
                                  activity_meter_usart->rx_len,
                                  &parsed_data,
                                  &error_state) != 0U)
    {
        activity_meter_waiting = 0U;
        ActivityMeter_StoreData(&parsed_data);
        return;
    }

    /*
     * 如果只收到 8 字节左右，可能是半双工链路上的发送回显，不立即判错。
     * 收到长度已经足够组成标准响应但仍解析失败，才记录 CRC 或格式错误。
     */
    if (activity_meter_usart->rx_len >= ACTIVITY_METER_RESPONSE_LEN)
    {
        activity_meter_waiting = 0U;
        ActivityMeter_SetState(error_state);
    }
}
