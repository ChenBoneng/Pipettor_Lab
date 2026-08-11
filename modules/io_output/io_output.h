#ifndef IO_OUTPUT_H
#define IO_OUTPUT_H

#include <stdint.h>

/**
 * @file io_output.h
 * @brief 24V 功率输出 GPIO 封装层。
 *
 * 本模块只负责把 MCU GPIO 输出成高低电平，不理解阀门或泵的业务含义。
 *
 * 当前硬件连接：
 * - IO_OUTPUT_VALVE_1     = PC13，对应 GPIO_CONTROL_FAMEN_1；
 * - IO_OUTPUT_VALVE_2     = PC14，对应 GPIO_CONTROL_FAMEN_2；
 * - IO_OUTPUT_WATER_PUMP  = PC15，对应 GPIO_CONTROL_SHUIBENG。
 */

/**
 * @brief 24V 功率输出编号。
 */
typedef enum
{
    IO_OUTPUT_VALVE_1 = 0,    /**< 阀 1 输出，硬件网络 GPIO_CONTROL_FAMEN_1。 */
    IO_OUTPUT_VALVE_2,        /**< 阀 2 输出，硬件网络 GPIO_CONTROL_FAMEN_2。 */
    IO_OUTPUT_WATER_PUMP,     /**< 抽水泵输出，硬件网络 GPIO_CONTROL_SHUIBENG。 */
    IO_OUTPUT_MAX             /**< 输出数量边界值，仅用于数组长度和参数检查。 */
} IoOutputId_e;

/**
 * @brief 24V 功率输出状态。
 */
typedef enum
{
    IO_OUTPUT_STATE_OFF = 0,  /**< 输出低电平，外部功率管关断。 */
    IO_OUTPUT_STATE_ON        /**< 输出高电平，外部功率管导通。 */
} IoOutputState_e;

/**
 * @brief 初始化三路 24V 功率输出。
 *
 * @note 本函数不会重新配置 GPIO 模式，GPIO 模式仍然由 CubeMX 生成的
 *       MX_GPIO_Init() 负责。这里仅把三路输出全部拉低，避免上电后外部
 *       阀门或抽水泵处于不确定状态。
 */
void IoOutput_Init(void);

/**
 * @brief 设置一路 24V 功率输出状态。
 *
 * @param output 输出编号，取值见 IoOutputId_e。
 * @param state 目标输出状态，低电平关断，高电平导通。
 * @return 1 表示设置成功；0 表示模块未初始化或输出编号非法。
 *
 * @note 本函数只负责 GPIO 高低电平，不判断这个输出后面接的是阀门还是泵。
 */
uint8_t IoOutput_Set(IoOutputId_e output, IoOutputState_e state);

/**
 * @brief 关闭全部 24V 功率输出。
 *
 * @note 适合初始化、急停、退出设备调试等场景调用。
 */
void IoOutput_AllOff(void);

/**
 * @brief 读取软件记录的输出状态。
 *
 * @param output 输出编号，取值见 IoOutputId_e。
 * @return 最近一次写入的软件状态；参数非法时返回 IO_OUTPUT_STATE_OFF。
 */
IoOutputState_e IoOutput_GetState(IoOutputId_e output);

/**
 * @brief 判断指定输出是否处于打开状态。
 *
 * @param output 输出编号，取值见 IoOutputId_e。
 * @return 1 表示当前软件记录为打开；0 表示关闭或参数非法。
 */
uint8_t IoOutput_IsOn(IoOutputId_e output);

#endif //IO_OUTPUT_H
