#include "solenoid_valve.h"
#include "io_output.h"
#include "main.h"

/*
 * 电磁阀模块只描述“阀”的物理状态：
 * - 掉电：COM 与 NO 连通；
 * - 上电：COM 与 NC 连通。
 *
 * 具体 GPIO 引脚由 io_output 统一管理，避免应用层和阀门层重复写 PC13/PC14。
 */
static const IoOutputId_e solenoid_valve_output[SOLENOID_VALVE_ID_MAX] = {
    IO_OUTPUT_VALVE_1,
    IO_OUTPUT_VALVE_2,
};

/*
 * solenoid_valve_state 记录最近一次写入的阀状态。
 * solenoid_valve_last_switch_ms 用来实现文档要求的 30ms 响应保护。
 */
static volatile SolenoidValveState_e solenoid_valve_state[SOLENOID_VALVE_ID_MAX] = {
    SOLENOID_VALVE_STATE_OFF_NO_OPEN,
};
static uint32_t solenoid_valve_last_switch_ms[SOLENOID_VALVE_ID_MAX] = {0U};
static uint8_t solenoid_valve_inited = 0U;

/**
 * @brief 判断电磁阀编号是否合法。
 * @param valve 电磁阀编号。
 * @return 1 表示合法；0 表示越界。
 */
static uint8_t SolenoidValve_IsValidId(SolenoidValveId_e valve)
{
    return (valve < SOLENOID_VALVE_ID_MAX) ? 1U : 0U;
}

/**
 * @brief 获取当前毫秒时间轴。
 *
 * @return HAL_GetTick() 返回的系统毫秒数。
 */
static uint32_t SolenoidValve_GetMs(void)
{
    return HAL_GetTick();
}

/**
 * @brief 直接写入阀门输出并更新时间戳。
 *
 * @param valve 电磁阀编号。
 * @param state 目标物理状态。
 *
 * @note 本函数不判断 30ms 响应等待，外层接口根据场景决定是否允许强制断电。
 */
static void SolenoidValve_WriteOutput(SolenoidValveId_e valve, SolenoidValveState_e state)
{
    IoOutputState_e output_state = IO_OUTPUT_STATE_OFF;

    if (state == SOLENOID_VALVE_STATE_ON_NC_OPEN)
    {
        output_state = IO_OUTPUT_STATE_ON;
    }

    /* 当前硬件为正逻辑：输出高电平时阀线圈上电。 */
    (void)IoOutput_Set(solenoid_valve_output[valve], output_state);
    solenoid_valve_state[valve] = state;
    solenoid_valve_last_switch_ms[valve] = SolenoidValve_GetMs();
}

/**
 * @brief 初始化电磁阀状态并强制掉电。
 */
void SolenoidValve_Init(void)
{
    const uint32_t now_ms = SolenoidValve_GetMs();

    for (uint8_t i = 0U; i < SOLENOID_VALVE_ID_MAX; i++)
    {
        /*
         * 初始化时允许上层立刻切换阀门，所以 last_switch_ms 往前放 30ms。
         * 这样初始化完成后的第一次 SolenoidValve_SetState() 不会被误判 Busy。
         */
        (void)IoOutput_Set(solenoid_valve_output[i], IO_OUTPUT_STATE_OFF);
        solenoid_valve_state[i] = SOLENOID_VALVE_STATE_OFF_NO_OPEN;
        solenoid_valve_last_switch_ms[i] = now_ms - SOLENOID_VALVE_RESPONSE_MS;
    }

    solenoid_valve_inited = 1U;
}

/**
 * @brief 设置单只电磁阀状态。
 */
uint8_t SolenoidValve_SetState(SolenoidValveId_e valve, SolenoidValveState_e state)
{
    if ((solenoid_valve_inited == 0U) || (SolenoidValve_IsValidId(valve) == 0U))
    {
        return 0U;
    }

    if (solenoid_valve_state[valve] == state)
    {
        return 1U;
    }

    /*
     * 阀体机械响应约 30ms。
     * 太快反复翻转可能导致流路未到位，所以普通切换接口直接拒绝。
     */
    if (SolenoidValve_IsBusy(valve) != 0U)
    {
        return 0U;
    }

    SolenoidValve_WriteOutput(valve, state);
    return 1U;
}

/**
 * @brief 强制关闭单只电磁阀。
 */
void SolenoidValve_Off(SolenoidValveId_e valve)
{
    if ((solenoid_valve_inited == 0U) || (SolenoidValve_IsValidId(valve) == 0U))
    {
        return;
    }

    SolenoidValve_WriteOutput(valve, SOLENOID_VALVE_STATE_OFF_NO_OPEN);
}

/**
 * @brief 强制关闭全部电磁阀。
 */
void SolenoidValve_AllOff(void)
{
    for (uint8_t i = 0U; i < SOLENOID_VALVE_ID_MAX; i++)
    {
        SolenoidValve_Off((SolenoidValveId_e)i);
    }
}

/**
 * @brief 判断指定电磁阀是否还在 30ms 响应期。
 */
uint8_t SolenoidValve_IsBusy(SolenoidValveId_e valve)
{
    uint32_t elapsed_ms;

    if ((solenoid_valve_inited == 0U) || (SolenoidValve_IsValidId(valve) == 0U))
    {
        return 0U;
    }

    elapsed_ms = SolenoidValve_GetMs() - solenoid_valve_last_switch_ms[valve];
    return (elapsed_ms < SOLENOID_VALVE_RESPONSE_MS) ? 1U : 0U;
}

/**
 * @brief 获取软件记录的阀门状态。
 */
SolenoidValveState_e SolenoidValve_GetState(SolenoidValveId_e valve)
{
    if (SolenoidValve_IsValidId(valve) == 0U)
    {
        return SOLENOID_VALVE_STATE_OFF_NO_OPEN;
    }

    return solenoid_valve_state[valve];
}
