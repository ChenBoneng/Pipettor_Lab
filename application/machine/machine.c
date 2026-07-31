#include "machine.h"
#include "main.h"
#include "solenoid_valve.h"
#include "water_pump.h"

/*
 * 阀门/抽水泵顺序测试参数。
 *
 * 本测试目标：
 * 1. 打开阀 1，等待 2s；
 * 2. 打开阀 2，等待 2s；
 * 3. 打开抽水泵，等待 2s；
 * 4. 关闭阀 1，等待 2s；
 * 5. 关闭阀 2，等待 2s；
 * 6. 关闭抽水泵；
 * 7. 上述流程重复 5 次。
 *
 * 这里使用非阻塞状态机，不在 MachineTask() 中直接 osDelay(2000)，
 * 避免测试时卡住按键扫描、LCD 刷新和后续其它业务逻辑。
 */
#define MACHINE_OUTPUT_TEST_INTERVAL_MS      2000U
#define MACHINE_OUTPUT_TEST_REPEAT_COUNT     5U

typedef enum
{
    MACHINE_TEST_STATE_IDLE = 0,             // 空闲状态，测试未启动或已经结束
    MACHINE_TEST_STATE_OUTPUT_START,         // 启动测试，先确保阀门和抽水泵全关
    MACHINE_TEST_STATE_VALVE_1_ON,           // 打开阀 1
    MACHINE_TEST_STATE_VALVE_2_ON,           // 打开阀 2
    MACHINE_TEST_STATE_WATER_PUMP_ON,        // 打开抽水泵
    MACHINE_TEST_STATE_VALVE_1_OFF,          // 关闭阀 1
    MACHINE_TEST_STATE_VALVE_2_OFF,          // 关闭阀 2
    MACHINE_TEST_STATE_WATER_PUMP_OFF,       // 关闭抽水泵
    MACHINE_TEST_STATE_FINISHED,             // 五轮测试完成
} MachineTestState_e;

static MachineTestState_e machine_test_state = MACHINE_TEST_STATE_IDLE;
static uint32_t machine_test_state_start_ms = 0U;
static uint8_t machine_test_repeat_count = 0U;
static uint8_t machine_test_running = 0U;

static uint32_t Machine_GetMs(void);
static void Machine_EnterState(MachineTestState_e next_state);
static void Machine_ExecuteOutputTestState(void);
static void Machine_AdvanceOutputTest(void);

/**
 * @brief 获取当前毫秒时间轴。
 *
 * @return HAL_GetTick() 返回的系统毫秒数。
 */
static uint32_t Machine_GetMs(void)
{
    return HAL_GetTick();
}

/**
 * @brief 进入测试状态并记录进入时间。
 *
 * @param next_state 下一个测试状态。
 */
static void Machine_EnterState(MachineTestState_e next_state)
{
    machine_test_state = next_state;
    machine_test_state_start_ms = Machine_GetMs();
}

/**
 * @brief 执行当前测试状态对应的一次性动作。
 *
 * @note 本函数只做“进入该状态时要做的一次动作”，等待 2s 的逻辑放在
 *       Machine_AdvanceOutputTest() 中统一处理。
 */
static void Machine_ExecuteOutputTestState(void)
{
    switch (machine_test_state)
    {
    case MACHINE_TEST_STATE_OUTPUT_START:
        /*
         * 每次启动测试前先强制关闭全部输出。
         * 这样即使上一次测试中途复位，也不会保留旧输出状态。
         */
        SolenoidValve_AllOff();
        WaterPump_StopAll();
        break;

    case MACHINE_TEST_STATE_VALVE_1_ON:
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        break;

    case MACHINE_TEST_STATE_VALVE_2_ON:
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_MED,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        break;

    case MACHINE_TEST_STATE_WATER_PUMP_ON:
        (void)WaterPump_Start(WATER_PUMP_ID_MAIN);
        break;

    case MACHINE_TEST_STATE_VALVE_1_OFF:
        SolenoidValve_Off(SOLENOID_VALVE_ID_WATER);
        break;

    case MACHINE_TEST_STATE_VALVE_2_OFF:
        SolenoidValve_Off(SOLENOID_VALVE_ID_MED);
        break;

    case MACHINE_TEST_STATE_WATER_PUMP_OFF:
        WaterPump_Stop(WATER_PUMP_ID_MAIN);
        break;

    case MACHINE_TEST_STATE_FINISHED:
    case MACHINE_TEST_STATE_IDLE:
    default:
        break;
    }
}

/**
 * @brief 按测试步骤推进到下一个状态。
 */
static void Machine_AdvanceOutputTest(void)
{
    switch (machine_test_state)
    {
    case MACHINE_TEST_STATE_VALVE_1_ON:
        Machine_EnterState(MACHINE_TEST_STATE_VALVE_2_ON);
        break;

    case MACHINE_TEST_STATE_VALVE_2_ON:
        Machine_EnterState(MACHINE_TEST_STATE_WATER_PUMP_ON);
        break;

    case MACHINE_TEST_STATE_WATER_PUMP_ON:
        Machine_EnterState(MACHINE_TEST_STATE_VALVE_1_OFF);
        break;

    case MACHINE_TEST_STATE_VALVE_1_OFF:
        Machine_EnterState(MACHINE_TEST_STATE_VALVE_2_OFF);
        break;

    case MACHINE_TEST_STATE_VALVE_2_OFF:
        Machine_EnterState(MACHINE_TEST_STATE_WATER_PUMP_OFF);
        break;

    case MACHINE_TEST_STATE_WATER_PUMP_OFF:
        machine_test_repeat_count++;
        if (machine_test_repeat_count >= MACHINE_OUTPUT_TEST_REPEAT_COUNT)
        {
            SolenoidValve_AllOff();
            WaterPump_StopAll();
            machine_test_running = 0U;
            Machine_EnterState(MACHINE_TEST_STATE_FINISHED);
            return;
        }

        Machine_EnterState(MACHINE_TEST_STATE_VALVE_1_ON);
        break;

    default:
        return;
    }

    Machine_ExecuteOutputTestState();
}

void MachineInit(void)
{
    /*
     * 各底层模块已经在 AllTaskInit() 中初始化。
     * 这里仅启动 machine 层测试状态机，真实动作由 MachineControl() 周期执行。
     */
    MachineOutputTestStart();
}

void MachineControl(void)
{
    uint32_t elapsed_ms;

    if (machine_test_running == 0U)
    {
        return;
    }

    if (machine_test_state == MACHINE_TEST_STATE_OUTPUT_START)
    {
        Machine_ExecuteOutputTestState();
        Machine_EnterState(MACHINE_TEST_STATE_VALVE_1_ON);
        Machine_ExecuteOutputTestState();
        return;
    }

    elapsed_ms = Machine_GetMs() - machine_test_state_start_ms;
    if (elapsed_ms < MACHINE_OUTPUT_TEST_INTERVAL_MS)
    {
        return;
    }

    Machine_AdvanceOutputTest();
}

void MachineOutputTestStart(void)
{
    machine_test_repeat_count = 0U;
    machine_test_running = 1U;
    Machine_EnterState(MACHINE_TEST_STATE_OUTPUT_START);
}

uint8_t MachineOutputTestIsRunning(void)
{
    return machine_test_running;
}
