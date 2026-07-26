//
// Created by lenovo on 26-7-26.
//

#include "bsp_can.h"
#include <string.h>

/*
 * F103RCT6 只有 CAN1。当前 CubeMX 工程只使能了 CAN1_RX1_IRQn，
 * 所以这里默认把过滤器分配到 FIFO1，保证接收中断能直接进入 HAL 回调。
 */
#define BSP_CAN_RX_FIFO CAN_RX_FIFO1
#define BSP_CAN_RX_IT   CAN_IT_RX_FIFO1_MSG_PENDING

static CANInstance can_instance_pool[CAN_MX_REGISTER_CNT];
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {0};
static uint8_t can_instance_count = 0U;
static uint8_t can_filter_index = 0U;
static uint8_t can_service_started = 0U;

static HAL_StatusTypeDef CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf = {0};
    const uint32_t filter_id = (_instance->rx_id & 0x7FFU) << 5U;

    if (can_filter_index >= MX_CAN_FILTER_CNT)
    {
        return HAL_ERROR;
    }

    /*
     * 使用 16 位 IDLIST 模式。一个过滤器组实际能放 4 个 16 位 ID，
     * 这里为了简单稳定，每个实例独占一个过滤器组，并把 4 个槽都写成同一个 ID。
     */
    can_filter_conf.FilterBank = can_filter_index++;
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_conf.FilterFIFOAssignment = BSP_CAN_RX_FIFO;
    can_filter_conf.FilterIdHigh = filter_id;
    can_filter_conf.FilterIdLow = filter_id;
    can_filter_conf.FilterMaskIdHigh = filter_id;
    can_filter_conf.FilterMaskIdLow = filter_id;
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_conf.SlaveStartFilterBank = MX_CAN_FILTER_CNT;

    return HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

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

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;

    if ((config == NULL) || (config->can_handle == NULL))
    {
        return NULL;
    }

    if (can_instance_count >= CAN_MX_REGISTER_CNT)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < can_instance_count; i++)
    {
        if ((can_instance[i]->can_handle == config->can_handle) &&
            (can_instance[i]->rx_id == config->rx_id))
        {
            return NULL;
        }
    }

    instance = &can_instance_pool[can_instance_count];
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    instance->txconf.StdId = config->tx_id;
    instance->txconf.ExtId = 0U;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 8U;
    instance->txconf.TransmitGlobalTime = DISABLE;

    if (CANAddFilter(instance) != HAL_OK)
    {
        return NULL;
    }

    can_instance[can_instance_count] = instance;

    if ((can_service_started == 0U) && (CANServiceInit(config->can_handle) != HAL_OK))
    {
        can_instance[can_instance_count] = NULL;
        return NULL;
    }

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

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    uint32_t start_tick;

    if (_instance == NULL)
    {
        return 0U;
    }

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0U)
    {
        if ((timeout <= 0.0f) || ((float)(HAL_GetTick() - start_tick) >= timeout))
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

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox) > 0U)
    {
        if (HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK)
        {
            return;
        }

        for (uint8_t i = 0U; i < can_instance_count; i++)
        {
            if ((can_instance[i]->can_handle == _hcan) &&
                (can_instance[i]->rx_id == rxconf.StdId))
            {
                can_instance[i]->rx_len = (uint8_t)rxconf.DLC;
                memcpy(can_instance[i]->rx_buff, can_rx_buff, can_instance[i]->rx_len);

                if (can_instance[i]->can_module_callback != NULL)
                {
                    can_instance[i]->can_module_callback(can_instance[i]);
                }
                return;
            }
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
