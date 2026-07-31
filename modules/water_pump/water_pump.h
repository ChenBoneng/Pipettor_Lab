#ifndef WATER_PUMP_H
#define WATER_PUMP_H

#include <stdint.h>

/**
 * @file water_pump.h
 * @brief 抽水泵 GPIO 启停驱动模块。
 *
 * 本模块只负责抽水泵启停，不处理电磁阀流路切换。
 *
 * 当前硬件连接：
 * - WATER_PUMP_ID_MAIN = PC15，对应 GPIO_CONTROL_SHUIBENG。
 */

/**
 * @brief 抽水泵编号。
 */
typedef enum
{
    WATER_PUMP_ID_MAIN = 0, /**< 当前唯一一路抽水泵。 */
    WATER_PUMP_ID_MAX       /**< 水泵数量边界值，仅用于数组长度和参数检查。 */
} WaterPumpId_e;

/**
 * @brief 抽水泵运行状态。
 */
typedef enum
{
    WATER_PUMP_STATE_OFF = 0, /**< 停止输出。 */
    WATER_PUMP_STATE_ON       /**< 打开输出。 */
} WaterPumpState_e;

/**
 * @brief 初始化抽水泵模块。
 *
 * @note 本函数会关闭抽水泵输出。调用前应先完成 IoOutput_Init()。
 */
void WaterPump_Init(void);

/**
 * @brief 设置抽水泵启停状态。
 *
 * @param pump 抽水泵编号，取值见 WaterPumpId_e。
 * @param state 目标状态，取值见 WaterPumpState_e。
 * @return 1 表示设置成功；0 表示模块未初始化、编号非法或底层输出失败。
 *
 * @note 抽水泵只做启停控制，不带电磁阀那种 30ms 流路切换保护。
 */
uint8_t WaterPump_SetState(WaterPumpId_e pump, WaterPumpState_e state);

/**
 * @brief 打开抽水泵。
 *
 * @param pump 抽水泵编号，取值见 WaterPumpId_e。
 * @return 1 表示打开成功；0 表示打开失败。
 */
uint8_t WaterPump_Start(WaterPumpId_e pump);

/**
 * @brief 关闭抽水泵。
 *
 * @param pump 抽水泵编号，取值见 WaterPumpId_e。
 */
void WaterPump_Stop(WaterPumpId_e pump);

/**
 * @brief 关闭全部抽水泵。
 *
 * @note 当前只有一路抽水泵，保留 All 接口是为了和其它模块的急停调用风格一致。
 */
void WaterPump_StopAll(void);

/**
 * @brief 判断抽水泵是否处于打开状态。
 *
 * @param pump 抽水泵编号，取值见 WaterPumpId_e。
 * @return 1 表示当前软件记录为打开；0 表示关闭或参数非法。
 */
uint8_t WaterPump_IsOn(WaterPumpId_e pump);

#endif //WATER_PUMP_H
