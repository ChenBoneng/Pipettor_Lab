//
// Created by lenovo on 26-7-26.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

/*
 * STM32F103RCT6 只有 CAN1。一个 CANInstance 表示一个上层设备/模块，
 * 而不是一个单独的 CAN ID。
 *
 * 这样做的原因：
 * - 上位机通信这类模块天然会使用多个接收 ID；
 * - 如果一个 rx_id 创建一个实例，会浪费过滤器，也会让上层状态分散；
 * - BSP 层内部维护 rx_id 路由表，上层只关心“收到一帧属于我的 CAN 数据”。
 */
#define CAN_MX_REGISTER_CNT 14U
#define MX_CAN_FILTER_CNT   14U
#define CAN_MX_RX_ROUTE_CNT 32U
#define DEVICE_CAN_CNT      1U

typedef struct CANInstance CANInstance;

/*
 * 每个需要使用 CAN 的模块，都注册并持有一个 CANInstance。
 *
 * 重要约定：
 * - tx_id 是该模块默认发送 ID；
 * - rx_id 是最近一次收到的真实标准帧 ID，不再表示唯一绑定 ID；
 * - rx_buff/rx_len 保存最近一次接收数据；
 * - 同一个 CANInstance 可以通过 CANRegister() 兼容注册多个 rx_id；
 * - 回调触发前，BSP 会把 rx_id 更新成本次实际收到的 StdId。
 */
struct CANInstance
{
    CAN_HandleTypeDef *can_handle;              // CAN 句柄，F103 工程里通常传 &hcan
    CAN_TxHeaderTypeDef txconf;                 // 发送帧头配置
    uint32_t tx_id;                             // 发送标准帧 ID
    uint32_t tx_mailbox;                        // HAL 返回的发送邮箱
    uint8_t tx_buff[8];                         // 发送缓存，发送前由业务模块写入
    uint8_t rx_buff[8];                         // 接收缓存，由中断回调写入
    uint32_t rx_id;                             // 最近一次收到的真实标准帧 ID
    uint8_t rx_len;                             // 最近一次接收长度
    uint8_t rx_id_count;                        // 当前实例已经绑定的接收 ID 数量
    void (*can_module_callback)(CANInstance *); // 收到匹配报文后的模块回调
    void *id;                                   // 使用该 CAN 实例的上层模块指针，可选
};

typedef struct
{
    CAN_HandleTypeDef *can_handle;              // CAN 句柄
    uint32_t tx_id;                             // 发送标准帧 ID
    uint32_t rx_id;                             // 本次要绑定到设备实例的接收标准帧 ID
    void (*can_module_callback)(CANInstance *); // 接收回调，可为空
    void *id;                                   // 上层模块指针，可为空，建议不同设备传不同指针
} CAN_Init_Config_s;

/*
 * 注册一个 CAN 实例。
 *
 * 兼容旧用法：
 * - 第一次注册某个设备时创建 CANInstance；
 * - 同一设备再次注册不同 rx_id 时，复用原 CANInstance；
 * - 同一 rx_id 不能分配给两个不同设备。
 *
 * “同一设备”的判断依据：
 * can_handle + tx_id + callback + id 完全相同。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

// 修改实例发送帧长度，合法范围 1-8。
void CANSetDLC(CANInstance *_instance, uint8_t length);

// 发送实例 tx_buff 中的数据，timeout_us 单位为 us，成功返回 1，失败返回 0。
uint8_t CANTransmit(CANInstance *_instance, uint32_t timeout_us);

// 直接发送一帧标准 CAN 数据，成功返回 1，失败返回 0。
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

#endif //BSP_CAN_H
