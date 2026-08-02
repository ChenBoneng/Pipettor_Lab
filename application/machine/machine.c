#include "machine.h"
#include "MachineCMD.h"
#include "pump_drive.h"
#include "solenoid_valve.h"
#include "step_motor.h"
#include "water_pump.h"
#include "main.h"

/*
 * 组合测试流程：
 * 1. 配药页确认配药量后启动；
 * 2. 导杆 A 拉 3cm；
 * 3. 导杆 B 推 5cm；
 * 4. 两个电磁阀打通；
 * 5. 泵1（300ul 定量泵）吸 100ul；
 * 6. 抽水泵打开 2s 后关闭；
 * 7. 等待用户按发药键并确认发药量；
 * 8. 泵2（100ul 定量泵）吸 20ul，流程结束。
 *
 * 每一步之间保留 500ms 间隔，不在 MachineTask 里阻塞等待。
 */
#define MACHINE_COMBO_STEP_GAP_MS             500U
#define MACHINE_COMBO_WATER_PUMP_MS           2000U
#define MACHINE_COMBO_PUMP_ACTION_MS          1000U
#define MACHINE_COMBO_STEP_SPEED_CM_S_X100    200U
#define MACHINE_COMBO_STEP_A_DISTANCE_CM_X100 300U
#define MACHINE_COMBO_STEP_B_DISTANCE_CM_X100 500U
#define MACHINE_COMBO_PUMP1_VOLUME_UL         100U
#define MACHINE_COMBO_PUMP2_VOLUME_UL         20U

typedef enum
{
    MACHINE_COMBO_STATE_IDLE = 0,       // 空闲，等待配药量确认
    MACHINE_COMBO_STATE_STEP_A_PULL,    // 导杆 A 拉 3cm
    MACHINE_COMBO_STATE_GAP_AFTER_A,    // A 动作完成后的间隔
    MACHINE_COMBO_STATE_STEP_B_PUSH,    // 导杆 B 推 5cm
    MACHINE_COMBO_STATE_GAP_AFTER_B,    // B 动作完成后的间隔
    MACHINE_COMBO_STATE_VALVES_ON,      // 打通两个电磁阀
    MACHINE_COMBO_STATE_GAP_AFTER_VALVE,// 阀门动作后的间隔
    MACHINE_COMBO_STATE_PUMP1_IN,       // 泵1吸 100ul
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1,// 泵1动作后的间隔
    MACHINE_COMBO_STATE_WATER_PUMP_ON,  // 抽水泵打开 2s
    MACHINE_COMBO_STATE_WATER_PUMP_OFF, // 关闭抽水泵后间隔
    MACHINE_COMBO_STATE_WAIT_DISPENSE,  // 等待发药量确认
    MACHINE_COMBO_STATE_PUMP2_IN,       // 泵2吸 20ul
    MACHINE_COMBO_STATE_FINISHED,       // 流程结束，关闭输出
    MACHINE_COMBO_STATE_ERROR           // 动作启动失败，关闭输出
} MachineComboState_e;

static MachineComboState_e machine_combo_state = MACHINE_COMBO_STATE_IDLE;
static uint32_t machine_combo_state_start_ms = 0U;
static uint8_t machine_combo_running = 0U;
static uint16_t machine_combo_prep_ml_x100 = 0U;
static uint16_t machine_combo_dispense_ml_x100 = 0U;

static uint32_t Machine_GetMs(void);
static void Machine_EnterComboState(MachineComboState_e next_state);
static void Machine_ExecuteComboState(void);
static void Machine_StopComboOutputs(void);
static void Machine_AbortCombo(void);
static void Machine_StartCombo(uint16_t prep_ml_x100);
static void Machine_UpdateCombo(void);

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
 * @brief 关闭组合测试相关输出。
 */
static void Machine_StopComboOutputs(void)
{
    StepMotor_StopAll();
    SolenoidValve_AllOff();
    WaterPump_StopAll();
}

/**
 * @brief 异常退出组合测试。
 */
static void Machine_AbortCombo(void)
{
    Machine_StopComboOutputs();
    machine_combo_running = 0U;
    machine_combo_state = MACHINE_COMBO_STATE_ERROR;
}

/**
 * @brief 进入组合测试状态并执行一次性动作。
 *
 * @param next_state 下一个状态。
 */
static void Machine_EnterComboState(MachineComboState_e next_state)
{
    machine_combo_state = next_state;
    machine_combo_state_start_ms = Machine_GetMs();
    Machine_ExecuteComboState();
}

/**
 * @brief 执行当前状态的一次性动作。
 */
static void Machine_ExecuteComboState(void)
{
    PumpDrive_s *pump;

    switch (machine_combo_state)
    {
    case MACHINE_COMBO_STATE_STEP_A_PULL:
        if (StepMotor_RunDistanceCmX100(STEP_MOTOR_ID_A,
                                        STEP_MOTOR_DIR_PULL,
                                        MACHINE_COMBO_STEP_A_DISTANCE_CM_X100,
                                        MACHINE_COMBO_STEP_SPEED_CM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_STEP_B_PUSH:
        if (StepMotor_RunDistanceCmX100(STEP_MOTOR_ID_B,
                                        STEP_MOTOR_DIR_PUSH,
                                        MACHINE_COMBO_STEP_B_DISTANCE_CM_X100,
                                        MACHINE_COMBO_STEP_SPEED_CM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_VALVES_ON:
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_MED,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        break;

    case MACHINE_COMBO_STATE_PUMP1_IN:
        pump = PumpDrive_GetPump1();
        if (PumpDrive_MoveInVolumeUl(pump, MACHINE_COMBO_PUMP1_VOLUME_UL) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
        if (WaterPump_Start(WATER_PUMP_ID_MAIN) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
        WaterPump_Stop(WATER_PUMP_ID_MAIN);
        break;

    case MACHINE_COMBO_STATE_PUMP2_IN:
        pump = PumpDrive_GetPump2();
        if (PumpDrive_MoveInVolumeUl(pump, MACHINE_COMBO_PUMP2_VOLUME_UL) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_FINISHED:
    case MACHINE_COMBO_STATE_ERROR:
        Machine_StopComboOutputs();
        machine_combo_running = 0U;
        break;

    case MACHINE_COMBO_STATE_IDLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_A:
    case MACHINE_COMBO_STATE_GAP_AFTER_B:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
    default:
        break;
    }
}

/**
 * @brief 启动组合测试。
 *
 * @param prep_ml_x100 配药页确认的配药量，单位 0.01ml。
 */
static void Machine_StartCombo(uint16_t prep_ml_x100)
{
    machine_combo_prep_ml_x100 = prep_ml_x100;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_running = 1U;

    Machine_StopComboOutputs();
    Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_A_PULL);
}

/**
 * @brief 推进组合测试状态机。
 */
static void Machine_UpdateCombo(void)
{
    uint32_t elapsed_ms;

    if (machine_combo_running == 0U)
    {
        return;
    }

    if ((MachineCMD_GetPage() == MACHINE_CMD_PAGE_PAUSED) ||
        (MachineCMD_IsRemoteMode() != 0U))
    {
        Machine_AbortCombo();
        return;
    }

    elapsed_ms = Machine_GetMs() - machine_combo_state_start_ms;

    switch (machine_combo_state)
    {
    case MACHINE_COMBO_STATE_STEP_A_PULL:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_A) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_A);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_A:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_B_PUSH);
        }
        break;

    case MACHINE_COMBO_STATE_STEP_B_PUSH:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_B) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_B);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_B:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_VALVES_ON);
        }
        break;

    case MACHINE_COMBO_STATE_VALVES_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_IN:
        if (elapsed_ms >= MACHINE_COMBO_PUMP_ACTION_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP1);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_WATER_PUMP_ON);
        }
        break;

    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
        if (elapsed_ms >= MACHINE_COMBO_WATER_PUMP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_WATER_PUMP_OFF);
        }
        break;

    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_DISPENSE);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
        if (MachineCMD_ConsumeDispenseConfirmed(&machine_combo_dispense_ml_x100) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_IN:
        if (elapsed_ms >= MACHINE_COMBO_PUMP_ACTION_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
        }
        break;

    case MACHINE_COMBO_STATE_FINISHED:
    case MACHINE_COMBO_STATE_ERROR:
    case MACHINE_COMBO_STATE_IDLE:
    default:
        break;
    }
}

void MachineInit(void)
{
    machine_combo_state = MACHINE_COMBO_STATE_IDLE;
    machine_combo_running = 0U;
    machine_combo_prep_ml_x100 = 0U;
    machine_combo_dispense_ml_x100 = 0U;
    Machine_StopComboOutputs();
}

void MachineControl(void)
{
    uint16_t prep_ml_x100;

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumePrepConfirmed(&prep_ml_x100) != 0U))
    {
        Machine_StartCombo(prep_ml_x100);
        return;
    }

    Machine_UpdateCombo();
}

uint8_t MachineCombinationTestIsRunning(void)
{
    return machine_combo_running;
}
