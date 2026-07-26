//
// Created by lenovo on 26-7-26.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

// STM32F103RCT6 只有 CAN1，当前实现每个实例占用一个过滤器组。
#define CAN_MX_REGISTER_CNT 14U
#define MX_CAN_FILTER_CNT   14U
#define DEVICE_CAN_CNT      1U

typedef struct CANInstance CANInstance;

// 每个需要使用 CAN 的模块，都注册并持有一个 CANInstance。
struct CANInstance
{
    CAN_HandleTypeDef *can_handle;              // CAN 句柄，F103 工程里通常传 &hcan
    CAN_TxHeaderTypeDef txconf;                 // 发送帧头配置
    uint32_t tx_id;                             // 发送标准帧 ID
    uint32_t tx_mailbox;                        // HAL 返回的发送邮箱
    uint8_t tx_buff[8];                         // 发送缓存，发送前由业务模块写入
    uint8_t rx_buff[8];                         // 接收缓存，由中断回调写入
    uint32_t rx_id;                             // 接收标准帧 ID
    uint8_t rx_len;                             // 最近一次接收长度
    void (*can_module_callback)(CANInstance *); // 收到匹配报文后的模块回调
    void *id;                                   // 使用该 CAN 实例的上层模块指针，可选
};

typedef struct
{
    CAN_HandleTypeDef *can_handle;              // CAN 句柄
    uint32_t tx_id;                             // 发送标准帧 ID
    uint32_t rx_id;                             // 接收标准帧 ID
    void (*can_module_callback)(CANInstance *); // 接收回调，可为空
    void *id;                                   // 上层模块指针，可为空
} CAN_Init_Config_s;

// 注册一个 CAN 实例。使用前先填好 CAN_Init_Config_s。
CANInstance *CANRegister(CAN_Init_Config_s *config);

// 修改实例发送帧长度，合法范围 1-8。
void CANSetDLC(CANInstance *_instance, uint8_t length);

// 发送实例 tx_buff 中的数据，timeout 单位为 ms，成功返回 1，失败返回 0。
uint8_t CANTransmit(CANInstance *_instance, float timeout);

// 直接发送一帧标准 CAN 数据，成功返回 1，失败返回 0。
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

#endif //BSP_CAN_H
