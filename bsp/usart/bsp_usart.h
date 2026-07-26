//
// Created by lenovo on 26-7-26.
//

#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>
#include "usart.h"

/*
 * 当前 STM32F103RCT6 工程实际启用了 USART2 和 USART3。
 * 如果后续 CubeMX 再打开新的串口，只需要把数量加大，并确认 DMA/中断已经生成。
 */
#define DEVICE_USART_CNT 2U

/*
 * 单个 USART 实例的最大接收缓冲区长度。
 * ReceiveToIdle_DMA 常用于变长协议，缓冲区太小会导致一包数据被拆开。
 */
#define USART_RXBUFF_LIMIT 256U

// 上层模块的接收回调。回调里可以读取 USARTInstance 的 rx_buffer_finished 和 rx_len。
typedef void (*usart_module_callback)(void);

typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

typedef struct
{
    uint8_t recv_buffer_0[USART_RXBUFF_LIMIT]; // 双缓冲 0：一块给 DMA 接收，一块留给业务解析
    uint8_t recv_buffer_1[USART_RXBUFF_LIMIT]; // 双缓冲 1：和 buffer_0 交替使用，降低解析时覆盖数据的风险
    uint8_t *rx_buffer_finished;               // 最近一次接收完成的数据缓冲区
    uint8_t *rx_buffer_active;                 // 当前正在给 DMA 使用的接收缓冲区
    uint16_t recv_buff_size;                   // 本实例实际启用的接收长度，不能超过 USART_RXBUFF_LIMIT
    uint16_t rx_len;                           // 最近一次接收到的数据长度
    UART_HandleTypeDef *usart_handle;          // 串口句柄，例如 &huart2、&huart3
    usart_module_callback module_callback;     // 收到一帧数据后的业务解析回调，可为空
} USARTInstance;

typedef struct
{
    uint16_t recv_buff_size;                   // 接收缓冲长度，建议按协议最大帧长设置
    UART_HandleTypeDef *usart_handle;          // 串口句柄
    usart_module_callback module_callback;     // 接收回调，可为空
} USART_Init_Config_s;

// 注册一个 USART 实例，注册成功后会自动启动 ReceiveToIdle DMA 接收。
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

// 重新启动某个 USART 实例的 DMA + IDLE 接收，常用于错误恢复或手动重启。
void USARTServiceInit(USARTInstance *_instance);

// 发送一段数据，可选择阻塞、中断或 DMA 三种方式。
void USARTSend(USARTInstance *_instance,
               uint8_t *send_buf,
               uint16_t send_size,
               USART_TRANSFER_MODE mode);

// 判断串口发送侧是否空闲。空闲返回 1，忙碌或参数错误返回 0。
uint8_t USARTIsReady(USARTInstance *_instance);

#endif //BSP_USART_H
