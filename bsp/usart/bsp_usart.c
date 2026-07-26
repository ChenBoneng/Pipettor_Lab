//
// Created by lenovo on 26-7-26.
//

#include "bsp_usart.h"
#include <string.h>

/*
 * USART BSP 层做三件事：
 * 1. 用 USARTRegister() 把具体串口注册成一个实例；
 * 2. 用 HAL_UARTEx_ReceiveToIdle_DMA() 接收变长数据；
 * 3. 在 HAL 的接收回调里找到对应实例，再转交给上层模块解析。
 *
 * 这里移植自 F407 工程，但针对当前 F103RCT6 做了简化：
 * - 去掉日志和动态内存分配等依赖；
 * - 当前只支持工程里已经启用的 USART2 / USART3；
 * - 实例使用静态池，避免单片机运行时动态分配内存。
 */
static USARTInstance usart_instance_pool[DEVICE_USART_CNT];
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {0};
static uint8_t usart_instance_count = 0U;

static uint16_t USARTLimitRxSize(uint16_t size)
{
    if ((size == 0U) || (size > USART_RXBUFF_LIMIT))
    {
        return USART_RXBUFF_LIMIT;
    }

    return size;
}

void USARTServiceInit(USARTInstance *_instance)
{
    if ((_instance == NULL) || (_instance->usart_handle == NULL))
    {
        return;
    }

    /*
     * 每次启动接收时，都从 buffer_0 开始。
     * 收到一帧后，回调会把当前 active buffer 标记为 finished，
     * 再切换到另一块 buffer 继续接收。
     */
    _instance->rx_buffer_active = _instance->recv_buffer_0;
    _instance->rx_buffer_finished = NULL;
    _instance->rx_len = 0U;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle,
                                       _instance->rx_buffer_active,
                                       _instance->recv_buff_size);

    /*
     * HAL 的 ReceiveToIdle DMA 会在三种情况下进 RxEventCallback：
     * 半传输、传输完成、空闲中断。
     * 大多数协议只希望处理“接收完成/空闲”两类事件，所以关闭 DMA 半传输中断，
     * 避免同一包数据在半包时提前触发一次解析。
     */
    if (_instance->usart_handle->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
    }
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    USARTInstance *instance;

    if ((init_config == NULL) || (init_config->usart_handle == NULL))
    {
        return NULL;
    }

    if (usart_instance_count >= DEVICE_USART_CNT)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < usart_instance_count; i++)
    {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
        {
            return NULL;
        }
    }

    instance = &usart_instance_pool[usart_instance_count];
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = USARTLimitRxSize(init_config->recv_buff_size);
    instance->module_callback = init_config->module_callback;

    usart_instance[usart_instance_count] = instance;
    usart_instance_count++;

    USARTServiceInit(instance);
    return instance;
}

void USARTSend(USARTInstance *_instance,
               uint8_t *send_buf,
               uint16_t send_size,
               USART_TRANSFER_MODE mode)
{
    if ((_instance == NULL) ||
        (_instance->usart_handle == NULL) ||
        (send_buf == NULL) ||
        (send_size == 0U))
    {
        return;
    }

    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        (void)HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100U);
        break;

    case USART_TRANSFER_IT:
        (void)HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;

    case USART_TRANSFER_DMA:
        (void)HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;

    case USART_TRANSFER_NONE:
    default:
        break;
    }
}

uint8_t USARTIsReady(USARTInstance *_instance)
{
    if ((_instance == NULL) || (_instance->usart_handle == NULL))
    {
        return 0U;
    }

    /*
     * gState 描述发送侧状态。原 F407 代码使用“状态 | BUSY_TX”判断，
     * 这会让结果几乎恒为 busy；这里直接判断发送状态是否 READY。
     */
    return (_instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    USARTInstance *instance;

    for (uint8_t i = 0U; i < usart_instance_count; i++)
    {
        instance = usart_instance[i];

        if (instance->usart_handle == huart)
        {
            /*
             * 当前 active buffer 已经接收到一帧数据，把它交给上层解析；
             * 然后马上切到另一块 buffer，重新打开 DMA 接收。
             */
            instance->rx_buffer_finished = instance->rx_buffer_active;
            instance->rx_len = (Size > instance->recv_buff_size) ? instance->recv_buff_size : Size;

            if (instance->rx_buffer_active == instance->recv_buffer_0)
            {
                instance->rx_buffer_active = instance->recv_buffer_1;
            }
            else
            {
                instance->rx_buffer_active = instance->recv_buffer_0;
            }

            if (instance->module_callback != NULL)
            {
                instance->module_callback();
            }

            /*
             * 上层回调已经执行完，此时可以清掉刚才完成的 buffer。
             * 这样变长协议下一次解析时，不会读到上一帧残留数据。
             */
            if (instance->rx_buffer_finished != NULL)
            {
                memset(instance->rx_buffer_finished, 0, instance->rx_len);
            }

            (void)HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                               instance->rx_buffer_active,
                                               instance->recv_buff_size);

            if (instance->usart_handle->hdmarx != NULL)
            {
                __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
            }

            return;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0U; i < usart_instance_count; i++)
    {
        if (usart_instance[i]->usart_handle == huart)
        {
            /*
             * 串口错误常见来源是噪声、帧错误、溢出等。
             * BSP 层不做协议判断，只负责尽快恢复 DMA + IDLE 接收。
             */
            USARTServiceInit(usart_instance[i]);
            return;
        }
    }
}
