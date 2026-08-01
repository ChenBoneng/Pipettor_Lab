//
// Created by lenovo on 26-7-26.
//

#include "bsp_can.h"
#include <string.h>
#include "bsp_dwt.h"

/*
 * F103RCT6 只有 CAN1。当前 CubeMX 工程只使能了 CAN1_RX1_IRQn，
 * 所以这里默认把过滤器分配到 FIFO1，保证接收中断能直接进入 HAL 回调。
 */
#define BSP_CAN_RX_FIFO CAN_RX_FIFO1
#define BSP_CAN_RX_IT   CAN_IT_RX_FIFO1_MSG_PENDING
#define BSP_CAN_STD_ID_SHIFT 5U
#define BSP_CAN_FILTER_ID_PER_BANK 4U

typedef struct
{
    CAN_HandleTypeDef *can_handle; // 所属 CAN 外设句柄，F103 当前通常只有 &hcan
    uint32_t rx_id;                // 需要接收的标准帧 ID
    CANInstance *instance;         // 该 ID 最终路由到的上层设备实例
} CANRxRoute_s;

/*
 * can_instance_pool 保存真正的设备实例。
 * can_rx_route 保存“接收 ID -> 设备实例”的映射关系。
 *
 * 例子：
 * Communication 模块注册 0x090/0x092/0x100/0x101/0x102 五个 ID，
 * 这里最终只创建一个 CANInstance，但会创建五条 CANRxRoute_s。
 */
static CANInstance can_instance_pool[CAN_MX_REGISTER_CNT];
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {0};
static CANRxRoute_s can_rx_route[CAN_MX_RX_ROUTE_CNT] = {0};
static uint8_t can_instance_count = 0U;
static uint8_t can_rx_route_count = 0U;
static uint8_t can_service_started = 0U;

static uint8_t CANIsSameDevice(CANInstance *instance, const CAN_Init_Config_s *config);
static CANInstance *CANFindDevice(CAN_Init_Config_s *config);
static CANRxRoute_s *CANFindRoute(CAN_HandleTypeDef *can_handle, uint32_t rx_id);
static HAL_StatusTypeDef CANConfigFilterBank(uint8_t filter_bank);
static uint8_t CANAddRxRoute(CANInstance *instance, uint32_t rx_id);

/*
 * 判断当前注册请求是否属于一个已经存在的设备。
 *
 * 注意这里刻意不把 rx_id 纳入判断：
 * 同一个设备可以拥有多个 rx_id，rx_id 只决定路由，不决定设备身份。
 */
static uint8_t CANIsSameDevice(CANInstance *instance, const CAN_Init_Config_s *config)
{
    if ((instance == NULL) || (config == NULL))
    {
        return 0U;
    }

    /*
     * 一个 CANInstance 表示一个“上层设备/模块”，而不是一个 CAN ID。
     * 这里用 can_handle、tx_id、callback 和 id 判断是不是同一个设备。
     *
     * 注意：
     * 如果两个不同模块都传同一个 callback 且 id=NULL，BSP 会把它们视为同一设备。
     * 后续新增 CAN 模块时，建议给 config.id 传入模块对象指针，避免歧义。
     */
    return ((instance->can_handle == config->can_handle) &&
            (instance->tx_id == config->tx_id) &&
            (instance->can_module_callback == config->can_module_callback) &&
            (instance->id == config->id)) ? 1U : 0U;
}

/*
 * 在已注册设备池中查找同一个上层设备。
 *
 * 找到后，新的 rx_id 会追加到该设备的路由表中；
 * 找不到时，CANRegister() 才会创建新的 CANInstance。
 */
static CANInstance *CANFindDevice(CAN_Init_Config_s *config)
{
    if (config == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < can_instance_count; i++)
    {
        if (CANIsSameDevice(can_instance[i], config) != 0U)
        {
            return can_instance[i];
        }
    }

    return NULL;
}

/*
 * 根据 CAN 句柄和实际收到的 StdId 查找路由。
 *
 * 接收中断里不再遍历 CANInstance，而是直接按路由表查找：
 * StdId -> CANRxRoute_s -> CANInstance。
 */
static CANRxRoute_s *CANFindRoute(CAN_HandleTypeDef *can_handle, uint32_t rx_id)
{
    if (can_handle == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < can_rx_route_count; i++)
    {
        if ((can_rx_route[i].can_handle == can_handle) &&
            (can_rx_route[i].rx_id == rx_id))
        {
            return &can_rx_route[i];
        }
    }

    return NULL;
}

/*
 * 根据路由表重新配置某一个 filter bank。
 *
 * 当前过滤器使用 16 位 IDLIST 模式，一个 filter bank 可以放 4 个标准 ID。
 * 每新增一条 rx_id 路由，只需要重新配置它所在的 filter bank。
 */
static HAL_StatusTypeDef CANConfigFilterBank(uint8_t filter_bank)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    uint32_t filter_id[BSP_CAN_FILTER_ID_PER_BANK] = {0};
    uint8_t route_index;
    uint8_t first_route_index;

    if (filter_bank >= MX_CAN_FILTER_CNT)
    {
        return HAL_ERROR;
    }

    first_route_index = (uint8_t)(filter_bank * BSP_CAN_FILTER_ID_PER_BANK);
    if (first_route_index >= can_rx_route_count)
    {
        return HAL_ERROR;
    }

    /*
     * 使用 16 位 IDLIST 模式。
     *
     * F103 一个 filter bank 在 16 位模式下能放 4 个标准 ID：
     * - FilterIdHigh
     * - FilterIdLow
     * - FilterMaskIdHigh
     * - FilterMaskIdLow
     *
     * 未用满的槽填本 bank 的第一个 ID，避免默认 0 误收 0x000。
     */
    for (uint8_t slot = 0U; slot < BSP_CAN_FILTER_ID_PER_BANK; slot++)
    {
        route_index = (uint8_t)(first_route_index + slot);
        if (route_index >= can_rx_route_count)
        {
            route_index = first_route_index;
        }

        filter_id[slot] = (can_rx_route[route_index].rx_id & 0x7FFU) << BSP_CAN_STD_ID_SHIFT;
    }

    can_filter_conf.FilterBank = filter_bank;
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_conf.FilterFIFOAssignment = BSP_CAN_RX_FIFO;
    can_filter_conf.FilterIdHigh = filter_id[0];
    can_filter_conf.FilterIdLow = filter_id[1];
    can_filter_conf.FilterMaskIdHigh = filter_id[2];
    can_filter_conf.FilterMaskIdLow = filter_id[3];
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_conf.SlaveStartFilterBank = MX_CAN_FILTER_CNT;

    return HAL_CAN_ConfigFilter(can_rx_route[first_route_index].can_handle, &can_filter_conf);
}

static uint8_t CANAddRxRoute(CANInstance *instance, uint32_t rx_id)
{
    uint8_t filter_bank;

    if ((instance == NULL) || (rx_id > 0x7FFU))
    {
        return 0U;
    }

    if (CANFindRoute(instance->can_handle, rx_id) != NULL)
    {
        /*
         * 同一个 CAN ID 已经注册过，不重复占用路由和过滤器。
         * 如果上层重复注册同一设备同一 ID，直接视为成功。
         */
        return 1U;
    }

    if ((can_rx_route_count >= CAN_MX_RX_ROUTE_CNT) ||
        ((can_rx_route_count / BSP_CAN_FILTER_ID_PER_BANK) >= MX_CAN_FILTER_CNT))
    {
        return 0U;
    }

    can_rx_route[can_rx_route_count].can_handle = instance->can_handle;
    can_rx_route[can_rx_route_count].rx_id = rx_id;
    can_rx_route[can_rx_route_count].instance = instance;

    /*
     * 路由表每 4 条共用一个 filter bank。
     * 第 0~3 条路由进入 bank0，第 4~7 条进入 bank1，以此类推。
     */
    filter_bank = (uint8_t)(can_rx_route_count / BSP_CAN_FILTER_ID_PER_BANK);
    can_rx_route_count++;

    if (CANConfigFilterBank(filter_bank) != HAL_OK)
    {
        can_rx_route_count--;
        memset(&can_rx_route[can_rx_route_count], 0, sizeof(CANRxRoute_s));
        return 0U;
    }

    if (instance->rx_id_count == 0U)
    {
        /*
         * rx_id 字段保留给上层读取“最近一次收到的真实 CAN ID”。
         * 初始化时先写入第一个绑定 ID，避免上层在收到第一帧前读到 0。
         */
        instance->rx_id = rx_id;
    }
    instance->rx_id_count++;

    return 1U;
}

/*
 * 启动 CAN 服务。
 *
 * CAN 只需要启动一次。后续新增 rx_id 只需要改过滤器，不需要重复 HAL_CAN_Start()。
 * 当前 CubeMX 工程只开了 RX1 中断，所以通知也只打开 BSP_CAN_RX_IT。
 */
static HAL_StatusTypeDef CANServiceInit(CAN_HandleTypeDef *can_handle)
{
    if (HAL_CAN_Start(can_handle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(can_handle, BSP_CAN_RX_IT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    can_service_started = 1U;
    return HAL_OK;
}

/*
 * 注册 CAN 设备或给已有设备追加接收 ID。
 *
 * 这个函数是 BSP 层和上层模块之间最重要的边界：
 * 上层仍然按“我要接收某个 rx_id”来调用；
 * BSP 内部负责判断是否复用设备实例、是否追加路由、是否更新过滤器。
 *
 * 这样上层不需要知道过滤器怎么分配，也不需要知道一个设备到底占了几个 ID。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;
    CANRxRoute_s *route;

    if ((config == NULL) ||
        (config->can_handle == NULL) ||
        (config->tx_id > 0x7FFU) ||
        (config->rx_id > 0x7FFU))
    {
        return NULL;
    }

    /*
     * 如果某个 rx_id 已经被注册过，不能再分配给另一个设备。
     * 同一设备重复注册同一个 rx_id 时，直接返回原实例，保持接口幂等。
     */
    route = CANFindRoute(config->can_handle, config->rx_id);
    if (route != NULL)
    {
        return (CANIsSameDevice(route->instance, config) != 0U) ? route->instance : NULL;
    }

    instance = CANFindDevice(config);
    if (instance != NULL)
    {
        /*
         * 同一个设备增加新的接收 ID。
         * 例如 Communication 模块会注册 0x090、0x092、0x100、0x101、0x102，
         * 这些 ID 最终都路由到同一个 CANInstance 和同一个回调。
         */
        return (CANAddRxRoute(instance, config->rx_id) != 0U) ? instance : NULL;
    }

    if (can_instance_count >= CAN_MX_REGISTER_CNT)
    {
        return NULL;
    }

    instance = &can_instance_pool[can_instance_count];
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->rx_id_count = 0U;

    instance->txconf.StdId = config->tx_id;
    instance->txconf.ExtId = 0U;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 8U;
    instance->txconf.TransmitGlobalTime = DISABLE;

    if (CANAddRxRoute(instance, config->rx_id) == 0U)
    {
        return NULL;
    }

    can_instance[can_instance_count] = instance;

    if ((can_service_started == 0U) && (CANServiceInit(config->can_handle) != HAL_OK))
    {
        can_instance[can_instance_count] = NULL;
        return NULL;
    }

    /*
     * 只有新设备真正创建成功后，设备计数才增加。
     * 如果只是给旧设备追加 rx_id，不会消耗 can_instance_pool。
     */
    can_instance_count++;
    return instance;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if ((_instance == NULL) || (length == 0U) || (length > 8U))
    {
        return;
    }

    _instance->txconf.DLC = length;
}

uint8_t CANTransmit(CANInstance *_instance, uint32_t timeout_us)
{
    uint64_t start_us;

    if (_instance == NULL)
    {
        return 0U;
    }

    start_us = DWT_GetTimeline_us();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0U)
    {
        /*
         * 发送邮箱等待时间使用 us。
         * HAL_GetTick() 只有 ms 精度，放在 CAN 这种短等待里会比较粗；
         * DWT_GetTimeline_us() 由 CYCCNT 维护，适合这里做微秒级超时判断。
         */
        if ((timeout_us == 0U) || ((DWT_GetTimeline_us() - start_us) >= (uint64_t)timeout_us))
        {
            return 0U;
        }
    }

    if (HAL_CAN_AddTxMessage(_instance->can_handle,
                             &_instance->txconf,
                             _instance->tx_buff,
                             &_instance->tx_mailbox) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    CANRxRoute_s *route;
    CANInstance *instance;

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox) > 0U)
    {
        if (HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK)
        {
            return;
        }

        /*
         * BSP 只把标准数据帧交给上层。
         * 扩展帧、远程帧和异常 DLC 帧直接丢弃，避免协议层重复判断。
         */
        if ((rxconf.IDE != CAN_ID_STD) ||
            (rxconf.RTR != CAN_RTR_DATA) ||
            (rxconf.DLC > 8U))
        {
            continue;
        }

        /*
         * 通过真实 StdId 查找路由。
         * 没有注册过的 ID 即使误进 FIFO，也不会调用任何上层回调。
         */
        route = CANFindRoute(_hcan, rxconf.StdId);
        if (route == NULL)
        {
            continue;
        }

        /*
         * 回调前写入本次真实 rx_id 和数据。
         * 上层通过 instance->rx_id 区分当前收到的是哪个协议 ID。
         */
        instance = route->instance;
        instance->rx_id = rxconf.StdId;
        instance->rx_len = (uint8_t)rxconf.DLC;
        memcpy(instance->rx_buff, can_rx_buff, instance->rx_len);

        if (instance->can_module_callback != NULL)
        {
            instance->can_module_callback(instance);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}

uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    if ((hcan == NULL) || (data == NULL) || (len > 8U))
    {
        return 0U;
    }

    tx_header.StdId = id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return 0U;
    }

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) == HAL_OK) ? 1U : 0U;
}
