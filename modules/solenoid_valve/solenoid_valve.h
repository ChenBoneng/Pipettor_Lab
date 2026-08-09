#ifndef SOLENOID_VALVE_H
#define SOLENOID_VALVE_H

#include <stdint.h>

/**
 * @file solenoid_valve.h
 * @brief 电磁三通阀驱动模块。
 *
 * 本模块只封装电磁三通阀的物理状态，不直接处理页面、按键或配药流程。
 *
 * 当前硬件连接：
 * - SOLENOID_VALVE_ID_WATER = 阀 1，PC13，水路阀；
 * - SOLENOID_VALVE_ID_MED   = 阀 2，PC14，药路阀。
 *
 * @note 文档约定 GPIO 高电平表示线圈上电，低电平表示线圈掉电。
 */

/** 电磁阀机械响应时间，切换后至少等待 30ms 再认为流路稳定。 */
#define SOLENOID_VALVE_RESPONSE_MS 30U

/**
 * @brief 电磁三通阀编号。
 */
typedef enum
{
    SOLENOID_VALVE_ID_WATER = 0, /**< 水路阀，对应硬件阀 1。 */
    SOLENOID_VALVE_ID_MED,       /**< 药路阀，对应硬件阀 2。 */
    SOLENOID_VALVE_ID_MAX        /**< 阀数量边界值，仅用于数组长度和参数检查。 */
} SolenoidValveId_e;

/**
 * @brief 电磁三通阀物理状态。
 */
typedef enum
{
    SOLENOID_VALVE_STATE_OFF_NO_OPEN = 0, /**< 掉电状态：COM 与 NO 连通，NC 关闭。 */
    SOLENOID_VALVE_STATE_ON_NC_OPEN       /**< 上电状态：COM 与 NC 连通，NO 关闭。 */
} SolenoidValveState_e;

/**
 * @brief 初始化电磁阀模块。
 *
 * @note 本函数会强制两只阀掉电，使阀门回到 COM 与 NO 连通状态。
 *       调用前应先完成 IoOutput_Init()，否则底层输出不会响应。
 */
void SolenoidValve_Init(void);

/**
 * @brief 设置单只电磁阀状态。
 *
 * @param valve 电磁阀编号，取值见 SolenoidValveId_e。
 * @param state 目标物理状态，取值见 SolenoidValveState_e。
 * @return 1 表示设置成功或状态本来一致；0 表示模块未初始化、编号非法或仍在响应等待期。
 *
 * @note 文档要求阀门切换后至少等待 30ms。本函数在 30ms 内拒绝反复切换；
 *       打开任一阀前会先关闭其它已打开的阀，保证阀1和阀2不会同时保持打开。
 */
uint8_t SolenoidValve_SetState(SolenoidValveId_e valve, SolenoidValveState_e state);

/**
 * @brief 强制关闭单只电磁阀。
 *
 * @param valve 电磁阀编号，取值见 SolenoidValveId_e。
 *
 * @note 本函数不受 30ms 响应等待限制，适合急停、退出手动调试和异常复位。
 */
void SolenoidValve_Off(SolenoidValveId_e valve);

/**
 * @brief 强制关闭全部电磁阀。
 *
 * @note 关闭后两只阀都会回到掉电 NO 连通状态。
 */
void SolenoidValve_AllOff(void);

/**
 * @brief 判断指定电磁阀是否还处于响应等待期。
 *
 * @param valve 电磁阀编号，取值见 SolenoidValveId_e。
 * @return 1 表示距离上次切换不足 SOLENOID_VALVE_RESPONSE_MS；0 表示已经稳定或参数非法。
 */
uint8_t SolenoidValve_IsBusy(SolenoidValveId_e valve);

/**
 * @brief 读取软件记录的电磁阀状态。
 *
 * @param valve 电磁阀编号，取值见 SolenoidValveId_e。
 * @return 最近一次写入的软件状态；参数非法时返回掉电状态。
 */
SolenoidValveState_e SolenoidValve_GetState(SolenoidValveId_e valve);

#endif //SOLENOID_VALVE_H
