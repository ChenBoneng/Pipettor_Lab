#include "water_pump.h"
#include "io_output.h"

/*
 * 抽水泵和电磁阀虽然都是 GPIO 控制 24V 负载，
 * 但业务含义不同，所以这里单独保留 water_pump 模块。
 *
 * 当前只有一路抽水泵：PC15 / GPIO_CONTROL_SHUIBENG。
 */
static const IoOutputId_e water_pump_output[WATER_PUMP_ID_MAX] = {
    IO_OUTPUT_WATER_PUMP,
};

/*
 * 软件状态只记录最近一次启停命令。
 * 当前硬件没有抽水泵反馈信号，所以不能代表真实转动状态。
 */
static volatile WaterPumpState_e water_pump_state[WATER_PUMP_ID_MAX] = {
    WATER_PUMP_STATE_OFF,
};
static uint8_t water_pump_inited = 0U;

/**
 * @brief 判断抽水泵编号是否合法。
 * @param pump 抽水泵编号。
 * @return 1 表示合法；0 表示越界。
 */
static uint8_t WaterPump_IsValidId(WaterPumpId_e pump)
{
    return (pump < WATER_PUMP_ID_MAX) ? 1U : 0U;
}

/**
 * @brief 初始化抽水泵模块并关闭输出。
 */
void WaterPump_Init(void)
{
    for (uint8_t i = 0U; i < WATER_PUMP_ID_MAX; i++)
    {
        (void)IoOutput_Set(water_pump_output[i], IO_OUTPUT_STATE_OFF);
        water_pump_state[i] = WATER_PUMP_STATE_OFF;
    }

    water_pump_inited = 1U;
}

/**
 * @brief 设置抽水泵启停状态。
 */
uint8_t WaterPump_SetState(WaterPumpId_e pump, WaterPumpState_e state)
{
    IoOutputState_e output_state = IO_OUTPUT_STATE_OFF;

    if ((water_pump_inited == 0U) || (WaterPump_IsValidId(pump) == 0U))
    {
        return 0U;
    }

    if (state == WATER_PUMP_STATE_ON)
    {
        output_state = IO_OUTPUT_STATE_ON;
    }

    /* 当前硬件为正逻辑：GPIO 高电平导通抽水泵 24V 输出。 */
    if (IoOutput_Set(water_pump_output[pump], output_state) == 0U)
    {
        return 0U;
    }

    water_pump_state[pump] = state;
    return 1U;
}

/**
 * @brief 打开抽水泵。
 */
uint8_t WaterPump_Start(WaterPumpId_e pump)
{
    return WaterPump_SetState(pump, WATER_PUMP_STATE_ON);
}

/**
 * @brief 关闭抽水泵。
 */
void WaterPump_Stop(WaterPumpId_e pump)
{
    (void)WaterPump_SetState(pump, WATER_PUMP_STATE_OFF);
}

/**
 * @brief 关闭全部抽水泵。
 */
void WaterPump_StopAll(void)
{
    for (uint8_t i = 0U; i < WATER_PUMP_ID_MAX; i++)
    {
        WaterPump_Stop((WaterPumpId_e)i);
    }
}

/**
 * @brief 判断抽水泵是否打开。
 */
uint8_t WaterPump_IsOn(WaterPumpId_e pump)
{
    if (WaterPump_IsValidId(pump) == 0U)
    {
        return 0U;
    }

    return (water_pump_state[pump] == WATER_PUMP_STATE_ON) ? 1U : 0U;
}
