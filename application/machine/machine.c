#include "machine.h"
#include "MachineCMD.h"
#include "pump_drive.h"
#include "solenoid_valve.h"
#include "step_motor.h"
#include "water_pump.h"
#include "main.h"

/*
 * 配药/发药整机测试流程：
 * 1. 配药页确认“原药体积”和“目标浓度”后启动；
 * 2. 导杆 A 推 3cm，导杆 B 推 5cm；
 * 3. 抽水泵打开 5s，把铅罐中原药抽入活度计；
 * 4. 等待活度计读数稳定，当前未接入实物，先使用固定假活度；
 * 5. 按目标浓度、原药体积和实际活度计算需要补入的纯净水体积；
 * 6. 打开电磁阀 1，通过泵1抽取所需纯净水；
 * 7. 抽水泵再次打开 5s，把铅罐中的纯净水抽到活度计；
 * 8. 等待读数稳定后认为配药完成，并让导杆 A/B 回原点；
 * 9. 等待用户按发药键并确认发药量；
 * 10. 泵2吸入本次发药体积，流程结束。
 *
 * 每一步之间保留 500ms 间隔，定量泵按“体积换算步数 + 保守速度”等待完成，
 * 所有等待都由状态机推进，不在 MachineTask 里阻塞。
 */
#define MACHINE_COMBO_STEP_GAP_MS                500U
#define MACHINE_COMBO_WATER_PUMP_MS              5000U
#define MACHINE_COMBO_ACTIVITY_STABLE_MS         1000U
#define MACHINE_COMBO_PUMP_SPEED_PPS             1000U
#define MACHINE_COMBO_PUMP_WAIT_MARGIN_MS        1000U
#define MACHINE_COMBO_STEP_SPEED_MM_S_X100       2000U
#define MACHINE_COMBO_STEP_A_POSITION_MM_X100    3000U
#define MACHINE_COMBO_STEP_B_POSITION_MM_X100    5000U
#define MACHINE_COMBO_FAKE_RAW_ACTIVITY_MCI_X1000 20000U

typedef enum
{
    MACHINE_COMBO_STATE_IDLE = 0,              // 空闲，等待配药参数确认
    MACHINE_COMBO_STATE_STEP_A_PUSH,           // 导杆 A 推 3cm
    MACHINE_COMBO_STATE_GAP_AFTER_A,           // A 动作完成后的间隔
    MACHINE_COMBO_STATE_STEP_B_PUSH,           // 导杆 B 推 5cm
    MACHINE_COMBO_STATE_GAP_AFTER_B,           // B 动作完成后的间隔
    MACHINE_COMBO_STATE_RAW_PUMP_ON,           // 抽水泵抽原药进入活度计
    MACHINE_COMBO_STATE_RAW_PUMP_OFF,          // 关闭抽水泵并等待流路稳定
    MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY,     // 等待原药活度读数稳定
    MACHINE_COMBO_STATE_CALC_WATER,            // 根据假活度计算补水量
    MACHINE_COMBO_STATE_WATER_VALVE_ON,        // 打开电磁阀 1（水路阀）
    MACHINE_COMBO_STATE_GAP_AFTER_VALVE,       // 阀门动作后的间隔
    MACHINE_COMBO_STATE_PUMP1_WATER_IN,        // 泵1抽取纯净水
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1,       // 泵1动作后的间隔
    MACHINE_COMBO_STATE_WATER_PUMP_ON,         // 抽水泵抽纯净水进入活度计
    MACHINE_COMBO_STATE_WATER_PUMP_OFF,        // 关闭抽水泵并等待流路稳定
    MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY,   // 等待最终活度读数稳定
    MACHINE_COMBO_STATE_STEP_A_HOME,           // 导杆 A 回原点
    MACHINE_COMBO_STATE_GAP_AFTER_A_HOME,      // A 回原点后的间隔
    MACHINE_COMBO_STATE_STEP_B_HOME,           // 导杆 B 回原点
    MACHINE_COMBO_STATE_GAP_AFTER_B_HOME,      // B 回原点后的间隔
    MACHINE_COMBO_STATE_WAIT_DISPENSE,         // 等待发药量确认
    MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN,     // 泵2吸入发药体积
    MACHINE_COMBO_STATE_FINISHED,              // 流程结束，关闭输出
    MACHINE_COMBO_STATE_ERROR                  // 动作启动失败，关闭输出
} MachineComboState_e;

static MachineComboState_e machine_combo_state = MACHINE_COMBO_STATE_IDLE;
static uint32_t machine_combo_state_start_ms = 0U;
static uint8_t machine_combo_running = 0U;
static uint16_t machine_combo_raw_ml_x100 = 0U;
static uint16_t machine_combo_target_conc_x1000 = 0U;
static uint16_t machine_combo_dispense_ml_x100 = 0U;
static uint32_t machine_combo_water_ul = 0U;
static uint32_t machine_combo_dispense_ul = 0U;
static uint32_t machine_combo_raw_activity_mci_x1000 = 0U;
static uint32_t machine_combo_pump_wait_ms = 0U;

static uint32_t Machine_GetMs(void);
static uint32_t Machine_CalcPureWaterVolumeUl(uint16_t raw_ml_x100,
                                              uint16_t target_conc_x1000,
                                              uint32_t raw_activity_mci_x1000);
static uint32_t Machine_CalcPumpWaitMs(PumpDrive_s *pump, uint32_t volume_ul);
static void Machine_EnterComboState(MachineComboState_e next_state);
static void Machine_ExecuteComboState(void);
static void Machine_StopComboOutputs(void);
static void Machine_AbortCombo(void);
static void Machine_StartCombo(uint16_t raw_ml_x100, uint16_t target_conc_x1000);
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
 * @brief 根据目标浓度计算需要补入的纯净水体积。
 *
 * @param raw_ml_x100 原药体积，单位 0.01ml。
 * @param target_conc_x1000 目标浓度，单位 0.001mCi/ml。
 * @param raw_activity_mci_x1000 活度计读到的原药总活度，单位 0.001mCi。
 * @return 需要泵1抽取的纯净水体积，单位 ul。
 *
 * @note 稀释计算的本质是：
 *       最终总体积 = 原药总活度 / 目标浓度；
 *       补水体积 = 最终总体积 - 原药体积。
 *
 *       目前活度计还没有接入实物，raw_activity_mci_x1000 由固定假数据传入。
 *       如果目标浓度高于当前原药浓度，计算结果会小于原药体积，此时认为无需补水。
 */
static uint32_t Machine_CalcPureWaterVolumeUl(uint16_t raw_ml_x100,
                                              uint16_t target_conc_x1000,
                                              uint32_t raw_activity_mci_x1000)
{
    uint32_t final_ml_x100;
    uint32_t water_ml_x100;

    if (target_conc_x1000 == 0U)
    {
        return 0U;
    }

    final_ml_x100 = ((raw_activity_mci_x1000 * 100U) + (target_conc_x1000 / 2U)) /
                    target_conc_x1000;
    if (final_ml_x100 <= raw_ml_x100)
    {
        return 0U;
    }

    water_ml_x100 = final_ml_x100 - raw_ml_x100;
    return water_ml_x100 * 10U;
}

/**
 * @brief 根据本次定量泵体积估算动作完成等待时间。
 *
 * @param pump 定量泵实例。
 * @param volume_ul 本次吸入体积，单位 ul。
 * @return 建议等待时间，单位 ms。
 *
 * @note 当前 ISC1000 只下发一条 in/out 命令，没有“动作完成事件”主动上报。
 *       为了保证流程动作串行，machine 层按命令步数和保守 PPS 估算等待时间。
 *       后续如果要做到更准，可以改成周期发送 sta 并等 busy 位清零。
 */
static uint32_t Machine_CalcPumpWaitMs(PumpDrive_s *pump, uint32_t volume_ul)
{
    uint32_t steps;

    steps = PumpDrive_VolumeUlToSteps(pump, volume_ul);
    if (steps == 0U)
    {
        return 0U;
    }

    return ((steps * 1000U) + (MACHINE_COMBO_PUMP_SPEED_PPS - 1U)) /
           MACHINE_COMBO_PUMP_SPEED_PPS +
           MACHINE_COMBO_PUMP_WAIT_MARGIN_MS;
}

/**
 * @brief 关闭组合测试相关输出。
 */
static void Machine_StopComboOutputs(void)
{
    PumpDrive_s *pump;

    StepMotor_StopAll();
    SolenoidValve_AllOff();
    WaterPump_StopAll();

    /*
     * 定量泵是 RS485 命令设备，不会因为 MachineTask 状态结束自动停。
     * 本机流程暂停、复位、异常退出和流程结束时，都必须显式发送急停命令。
     */
    pump = PumpDrive_GetPump1();
    if (pump != NULL)
    {
        (void)PumpDrive_Stop(pump, 1U);
    }

    pump = PumpDrive_GetPump2();
    if (pump != NULL)
    {
        (void)PumpDrive_Stop(pump, 1U);
    }
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
    case MACHINE_COMBO_STATE_STEP_A_PUSH:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_A,
                                          MACHINE_COMBO_STEP_A_POSITION_MM_X100,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_STEP_B_PUSH:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_B,
                                          MACHINE_COMBO_STEP_B_POSITION_MM_X100,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
        if (WaterPump_Start(WATER_PUMP_ID_MAIN) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
        WaterPump_Stop(WATER_PUMP_ID_MAIN);
        break;

    case MACHINE_COMBO_STATE_CALC_WATER:
        /*
         * 活度计未接入前先使用固定假活度。后续接入真实活度计时，只需要把
         * machine_combo_raw_activity_mci_x1000 替换成 ActivityMeter_GetData()
         * 读取到的实际原药总活度即可。
         */
        machine_combo_raw_activity_mci_x1000 = MACHINE_COMBO_FAKE_RAW_ACTIVITY_MCI_X1000;
        machine_combo_water_ul = Machine_CalcPureWaterVolumeUl(machine_combo_raw_ml_x100,
                                                               machine_combo_target_conc_x1000,
                                                               machine_combo_raw_activity_mci_x1000);
        break;

    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        break;

    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
        if (machine_combo_water_ul == 0U)
        {
            machine_combo_pump_wait_ms = 0U;
            break;
        }

        pump = PumpDrive_GetPump1();
        machine_combo_pump_wait_ms = Machine_CalcPumpWaitMs(pump, machine_combo_water_ul);
        if (PumpDrive_MoveInVolumeUl(pump, machine_combo_water_ul) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_STEP_A_HOME:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_A,
                                          0U,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_STEP_B_HOME:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_B,
                                          0U,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
        machine_combo_dispense_ul = (uint32_t)machine_combo_dispense_ml_x100 * 10U;
        pump = PumpDrive_GetPump2();
        machine_combo_pump_wait_ms = Machine_CalcPumpWaitMs(pump, machine_combo_dispense_ul);
        if (PumpDrive_MoveInVolumeUl(pump, machine_combo_dispense_ul) == 0U)
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
    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
    case MACHINE_COMBO_STATE_GAP_AFTER_A_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_B_HOME:
    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
    default:
        break;
    }
}

/**
 * @brief 启动组合测试。
 *
 * @param raw_ml_x100 配药页确认的原药体积，单位 0.01ml。
 * @param target_conc_x1000 配药页确认的目标浓度，单位 0.001mCi/ml。
 */
static void Machine_StartCombo(uint16_t raw_ml_x100, uint16_t target_conc_x1000)
{
    machine_combo_raw_ml_x100 = raw_ml_x100;
    machine_combo_target_conc_x1000 = target_conc_x1000;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_raw_activity_mci_x1000 = 0U;
    machine_combo_pump_wait_ms = 0U;
    machine_combo_running = 1U;

    Machine_StopComboOutputs();
    Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_A_PUSH);
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
    case MACHINE_COMBO_STATE_STEP_A_PUSH:
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
            Machine_EnterComboState(MACHINE_COMBO_STATE_RAW_PUMP_ON);
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
        if (elapsed_ms >= MACHINE_COMBO_WATER_PUMP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_RAW_PUMP_OFF);
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
        if (elapsed_ms >= MACHINE_COMBO_ACTIVITY_STABLE_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_CALC_WATER);
        }
        break;

    case MACHINE_COMBO_STATE_CALC_WATER:
        Machine_EnterComboState(MACHINE_COMBO_STATE_WATER_VALVE_ON);
        break;

    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_WATER_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
        if (elapsed_ms >= machine_combo_pump_wait_ms)
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
            Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        if (elapsed_ms >= MACHINE_COMBO_ACTIVITY_STABLE_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_A_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_STEP_A_HOME:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_A) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_A_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_A_HOME:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_B_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_STEP_B_HOME:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_B) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_B_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_B_HOME:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_DISPENSE);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
        if (MachineCMD_ConsumeDispenseConfirmed(&machine_combo_dispense_ml_x100) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
        if (elapsed_ms >= machine_combo_pump_wait_ms)
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
    machine_combo_raw_ml_x100 = 0U;
    machine_combo_target_conc_x1000 = 0U;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_raw_activity_mci_x1000 = 0U;
    machine_combo_pump_wait_ms = 0U;
    Machine_StopComboOutputs();
}

void MachineControl(void)
{
    uint16_t raw_ml_x100;
    uint16_t target_conc_x1000;

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumePrepConfirmed(&raw_ml_x100, &target_conc_x1000) != 0U))
    {
        Machine_StartCombo(raw_ml_x100, target_conc_x1000);
        return;
    }

    Machine_UpdateCombo();
}

uint8_t MachineCombinationTestIsRunning(void)
{
    return machine_combo_running;
}
