#include "machine.h"
#include "MachineCMD.h"
#include "pump_drive.h"
#include "solenoid_valve.h"
#include "step_motor.h"
#include "water_pump.h"
#include "Communication.h"
#include "activity_meter.h"
#include "main.h"
#include <string.h>

/*
 * 配药/发药整机测试流程：
 * 1. 配药页确认“当前活度浓度、当前体积、目标活度浓度”后启动；
 * 2. 进罐导轨（电机A）推 3cm，插针导轨（电机B）推 5cm；
 * 3. 抽水泵打开 5s，把铅罐中当前溶液抽入活度计；
 * 4. 等待活度计读数稳定；
 * 5. 按当前浓度、当前体积和目标浓度计算需要补入的纯净水体积；
 * 6. 打开电磁阀 1，通过泵1抽取所需纯净水；
 * 7. 抽水泵再次打开 5s，把铅罐中的纯净水抽到活度计；
 * 8. 等待读数稳定后认为配药完成，先回退插针导轨（电机B），再回退进罐导轨（电机A）；
 * 9. 等待用户按发药键并确认发药量；
 * 10. 泵2吸入本次发药体积，流程结束。
 *
 * 每一步之间保留 500ms 间隔，定量泵由 pump_drive 周期轮询 Busy 位确认完成，
 * 所有等待都由状态机推进，不在 MachineTask 里阻塞。
 */
#define MACHINE_COMBO_STEP_GAP_MS                500U
#define MACHINE_COMBO_WATER_PUMP_MS              5000U
#define MACHINE_COMBO_ACTIVITY_STABLE_MS         1000U
#define MACHINE_COMBO_STEP_SPEED_MM_S_X100       2000U
#define MACHINE_COMBO_STEP_A_POSITION_MM_X100    3000U
#define MACHINE_COMBO_STEP_B_POSITION_MM_X100    5000U
#define MACHINE_DISPENSE_PUMP_SEGMENT_UL         PUMP_DRIVE_PUMP2_FULL_STROKE_UL
#define MACHINE_TRANSFER_TO_ACTIVITY_MS           2000U

typedef enum
{
    MACHINE_COMBO_STATE_IDLE = 0,              // 空闲，等待配药参数确认
    MACHINE_COMBO_STATE_STEP_A_PUSH,           // 进罐导轨（电机A）推 3cm
    MACHINE_COMBO_STATE_GAP_AFTER_A,           // 进罐导轨动作完成后的间隔
    MACHINE_COMBO_STATE_STEP_B_PUSH,           // 插针导轨（电机B）推 5cm
    MACHINE_COMBO_STATE_GAP_AFTER_B,           // 插针导轨动作完成后的间隔
    MACHINE_COMBO_STATE_RAW_PUMP_ON,           // 抽水泵抽当前溶液进入活度计
    MACHINE_COMBO_STATE_RAW_PUMP_OFF,          // 关闭抽水泵并等待流路稳定
    MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY,     // 等待当前溶液活度读数稳定
    MACHINE_COMBO_STATE_CALC_WATER,            // 根据当前浓度和目标浓度计算补水量
    MACHINE_COMBO_STATE_WATER_VALVE_ON,        // 打开电磁阀 1（水路阀）
    MACHINE_COMBO_STATE_GAP_AFTER_VALVE,       // 阀门动作后的间隔
    MACHINE_COMBO_STATE_PUMP1_ENABLE,          // 使能泵1，避免未使能时体积命令不动作
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE, // 泵1使能后的 RS485 间隔
    MACHINE_COMBO_STATE_PUMP1_WATER_IN,        // 泵1抽取纯净水
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1,       // 泵1动作后的间隔
    MACHINE_COMBO_STATE_WATER_PUMP_ON,         // 抽水泵抽纯净水进入活度计
    MACHINE_COMBO_STATE_WATER_PUMP_OFF,        // 关闭抽水泵并等待流路稳定
    MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY,   // 等待最终活度读数稳定
    MACHINE_COMBO_STATE_STEP_A_HOME,           // 进罐导轨（电机A）回原点
    MACHINE_COMBO_STATE_GAP_AFTER_A_HOME,      // 进罐导轨回原点后的间隔
    MACHINE_COMBO_STATE_STEP_B_HOME,           // 插针导轨（电机B）回原点
    MACHINE_COMBO_STATE_GAP_AFTER_B_HOME,      // 插针导轨回原点后的间隔
    MACHINE_COMBO_STATE_WAIT_DISPENSE,         // 等待发药量确认
    MACHINE_COMBO_STATE_PUMP2_ENABLE,          // 发送泵2使能命令
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE, // 泵2使能后等待总线和驱动响应
    MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN,     // 泵2吸入发药体积
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2,        // 泵2分段吸液后的间隔
    MACHINE_COMBO_STATE_FINISHED,              // 流程结束，关闭输出
    MACHINE_COMBO_STATE_ERROR                  // 动作启动失败，关闭输出
} MachineComboState_e;

typedef enum
{
    MACHINE_DIRECT_DISPENSE_STATE_IDLE = 0,    // 空闲，等待待机页发药量确认
    MACHINE_DIRECT_DISPENSE_STATE_ENABLE,      // 发送泵2使能命令
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE, // 使能后等待总线和驱动响应
    MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN,    // 泵2吸入本次发药体积
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2, // 泵2分段吸液后的间隔
    MACHINE_DIRECT_DISPENSE_STATE_FINISHED,    // 独立发药结束
    MACHINE_DIRECT_DISPENSE_STATE_ERROR        // 泵2动作启动失败
} MachineDirectDispenseState_e;

typedef enum
{
    MACHINE_FLOW_OWNER_LOCAL = 0,  // 本地 LCD/按键启动的流程
    MACHINE_FLOW_OWNER_REMOTE      // 上位机协议启动的流程
} MachineFlowOwner_e;

typedef enum
{
    MACHINE_ACTIVITY_WAIT_PENDING = 0, // 活度计读数仍在等待中
    MACHINE_ACTIVITY_WAIT_OK,          // 已收到进入等待状态后的新读数
    MACHINE_ACTIVITY_WAIT_ERROR        // 活度计通信超时、CRC 错误或响应格式错误
} MachineActivityWaitResult_e;

static MachineComboState_e machine_combo_state = MACHINE_COMBO_STATE_IDLE;
static uint32_t machine_combo_state_start_ms = 0U;
static uint8_t machine_combo_running = 0U;
static MachineFlowOwner_e machine_combo_owner = MACHINE_FLOW_OWNER_LOCAL;
static uint16_t machine_combo_current_conc_x1000 = 0U;
static uint16_t machine_combo_current_ml_x100 = 0U;
static uint16_t machine_combo_target_conc_x1000 = 0U;
static uint16_t machine_combo_dispense_ml_x100 = 0U;
static uint32_t machine_combo_water_ul = 0U;
static uint16_t machine_combo_remote_water_ml_x100 = 0U;
static uint16_t machine_combo_remote_final_ml_x100 = 0U;
static uint16_t machine_combo_remote_initial_activity_x100 = 0U;
static uint8_t machine_combo_remote_volume_valid = 0U;
static uint8_t machine_combo_remote_seq = 0U;
static uint32_t machine_combo_dispense_ul = 0U;
static uint32_t machine_combo_dispense_remaining_ul = 0U;
static uint32_t machine_combo_dispense_segment_ul = 0U;
static uint32_t machine_combo_dispense_done_ul = 0U;
static uint8_t machine_combo_paused = 0U;
static uint32_t machine_combo_pause_start_ms = 0U;

static MachineDirectDispenseState_e machine_direct_dispense_state = MACHINE_DIRECT_DISPENSE_STATE_IDLE;
static uint32_t machine_direct_dispense_state_start_ms = 0U;
static uint8_t machine_direct_dispense_running = 0U;
static MachineFlowOwner_e machine_direct_dispense_owner = MACHINE_FLOW_OWNER_LOCAL;
static uint16_t machine_direct_dispense_ml_x100 = 0U;
static uint32_t machine_direct_dispense_ul = 0U;
static uint32_t machine_direct_dispense_remaining_ul = 0U;
static uint32_t machine_direct_dispense_segment_ul = 0U;
static uint32_t machine_direct_dispense_done_ul = 0U;
static uint8_t machine_direct_dispense_paused = 0U;
static uint32_t machine_direct_dispense_pause_start_ms = 0U;
static uint8_t machine_dispense_progress_percent = 0U;
static uint8_t machine_transfer_running = 0U;
static uint8_t machine_transfer_done = 0U;
static uint8_t machine_transfer_activity_waiting = 0U;
static uint32_t machine_transfer_start_ms = 0U;
static uint8_t machine_activity_wait_active = 0U;
static uint8_t machine_activity_wait_started = 0U;
static uint32_t machine_activity_wait_start_ms = 0U;
static uint32_t machine_activity_wait_update_count = 0U;

static uint32_t Machine_GetMs(void);
static uint32_t Machine_CalcPureWaterVolumeUl(uint16_t current_conc_x1000,
                                              uint16_t current_ml_x100,
                                              uint16_t target_conc_x1000);
static uint8_t Machine_CalcDispenseProgressPercent(uint32_t done_ul, uint32_t target_ul);
static void Machine_EnterComboState(MachineComboState_e next_state);
static void Machine_ExecuteComboState(void);
static void Machine_StopComboOutputs(void);
static void Machine_AbortCombo(void);
static void Machine_PauseCombo(void);
static void Machine_ResumeCombo(void);
static PumpDrive_s *Machine_GetDispensePump(void);
static void Machine_StopDispensePumpOutput(void);
static uint8_t Machine_StartDispensePumpSegment(void);
static void Machine_StartCombo(uint16_t current_conc_x1000,
                               uint16_t current_ml_x100,
                               uint16_t target_conc_x1000,
                               MachineFlowOwner_e owner);
static void Machine_UpdateCombo(void);
static void Machine_EnterDirectDispenseState(MachineDirectDispenseState_e next_state);
static void Machine_ExecuteDirectDispenseState(void);
static void Machine_AbortDirectDispense(void);
static void Machine_PauseDirectDispense(void);
static void Machine_ResumeDirectDispense(void);
static uint8_t Machine_StartDirectDispenseSegment(void);
static void Machine_StartDirectDispense(uint16_t dispense_ml_x100, MachineFlowOwner_e owner);
static void Machine_UpdateDirectDispense(void);
static void Machine_AbortTransferToActivity(void);
static void Machine_UpdateTransferToActivity(void);
static void Machine_BeginActivityWait(void);
static MachineActivityWaitResult_e Machine_UpdateActivityWait(uint32_t stable_delay_ms);
static uint8_t Machine_IsActivityMeterErrorState(ActivityMeterState_e state);
static uint8_t Machine_IsRemoteFlowBlocked(MachineFlowOwner_e owner);
static uint16_t Machine_GetMeasuredActivityMciX100(void);
static uint16_t Machine_GetPreparedVolumeMlX100(void);
static void Machine_UpdateStandbyInventoryAfterPrepare(uint8_t use_measured_activity);
static uint8_t Machine_IsActivityWaitReadyForCombo(MachineActivityWaitResult_e result,
                                                   uint32_t elapsed_ms);
static void Machine_NotifyFlowStarted(MachineFlowOwner_e owner);
static void Machine_NotifyFlowStopped(MachineFlowOwner_e owner, uint8_t step);

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
 * @brief 根据当前浓度、当前体积和目标浓度计算需要补入的纯净水体积。
 *
 * @param current_conc_x1000 当前活度浓度，单位 0.001mCi/ml。
 * @param current_ml_x100 当前溶液体积，单位 0.01ml。
 * @param target_conc_x1000 目标活度浓度，单位 0.001mCi/ml。
 * @return 需要泵1抽取的纯净水体积，单位 ul。
 *
 * @note 稀释计算的本质是：
 *       当前总活度 = 当前浓度 * 当前体积；
 *       最终总体积 = 当前总活度 / 目标浓度；
 *       补水体积 = 最终总体积 - 当前体积。
 *
 *       只加纯净水时，浓度只能下降，不能上升。
 *       如果目标浓度大于等于当前浓度，则这里直接返回 0，不让泵1补水。
 */
static uint32_t Machine_CalcPureWaterVolumeUl(uint16_t current_conc_x1000,
                                              uint16_t current_ml_x100,
                                              uint16_t target_conc_x1000)
{
    uint32_t final_ml_x100;
    uint32_t water_ml_x100;

    if ((current_conc_x1000 == 0U) ||
        (current_ml_x100 == 0U) ||
        (target_conc_x1000 == 0U) ||
        (target_conc_x1000 >= current_conc_x1000))
    {
        return 0U;
    }

    final_ml_x100 = (((uint32_t)current_conc_x1000 * (uint32_t)current_ml_x100) +
                     (target_conc_x1000 / 2U)) /
                    target_conc_x1000;
    if (final_ml_x100 <= current_ml_x100)
    {
        return 0U;
    }

    water_ml_x100 = final_ml_x100 - current_ml_x100;
    return water_ml_x100 * 10U;
}

/**
 * @brief 根据泵2已经完成的发药体积计算进度。
 *
 * @note 当前泵驱动没有可靠的“已执行步数”反馈，光电门圈数也还没有完成现场标定。
 *       所以 LCD 进度只按已经确认完成的 100ul 分段推进，避免泵还在转时提前显示 100%。
 */
static uint8_t Machine_CalcDispenseProgressPercent(uint32_t done_ul, uint32_t target_ul)
{
    uint32_t percent;

    if (target_ul == 0U)
    {
        return 0U;
    }

    percent = (uint32_t)((((uint64_t)done_ul * 100ULL) + (target_ul / 2U)) / target_ul);
    if (percent > 100U)
    {
        percent = 100U;
    }

    return (uint8_t)percent;
}

/**
 * @brief 判断当前流程是否被远控状态阻断。
 *
 * 本地流程在上位机取得控制权后退出；远程流程只允许在 REMOTE 下继续。
 */
static uint8_t Machine_IsRemoteFlowBlocked(MachineFlowOwner_e owner)
{
    if (owner == MACHINE_FLOW_OWNER_REMOTE)
    {
        return (Communication_GetControlMode() == COMMUNICATION_CONTROL_REMOTE) ? 0U : 1U;
    }

    return (MachineCMD_IsRemoteMode() != 0U) ? 1U : 0U;
}

/**
 * @brief 获取最近一次活度计读数，并换算成 0.01mCi。
 *
 * @return 有效读数返回 mCi * 100；没有有效实测值时返回 0。
 */
static uint16_t Machine_GetMeasuredActivityMciX100(void)
{
    ActivityMeterData_s activity_data;
    float activity_mci;

    if ((ActivityMeter_GetData(&activity_data) == 0U) ||
        (activity_data.state != ACTIVITY_METER_STATE_OK) ||
        (activity_data.activity <= 0.0f))
    {
        return 0U;
    }

    switch (activity_data.activity_unit)
    {
    case ACTIVITY_METER_UNIT_UCI:
        activity_mci = activity_data.activity / 1000.0f;
        break;

    case ACTIVITY_METER_UNIT_MCI:
        activity_mci = activity_data.activity;
        break;

    case ACTIVITY_METER_UNIT_CI:
        activity_mci = activity_data.activity * 1000.0f;
        break;

    case ACTIVITY_METER_UNIT_BQ:
        activity_mci = activity_data.activity / 37000000.0f;
        break;

    case ACTIVITY_METER_UNIT_KBQ:
        activity_mci = activity_data.activity / 37000.0f;
        break;

    case ACTIVITY_METER_UNIT_MBQ:
        activity_mci = activity_data.activity / 37.0f;
        break;

    case ACTIVITY_METER_UNIT_GBQ:
        activity_mci = activity_data.activity * 27.027027f;
        break;

    default:
        return 0U;
    }

    if (activity_mci <= 0.0f)
    {
        return 0U;
    }
    if (activity_mci >= 655.35f)
    {
        return 0xFFFFU;
    }

    return (uint16_t)((activity_mci * 100.0f) + 0.5f);
}

/**
 * @brief 读取本次配药完成后的理论体积。
 *
 * @return 体积，单位 0.01ml。
 *
 * @note 本地配药的补水量由下位机计算，单位是 ul，需要换算回 0.01ml；
 *       远程配药的最终体积由上位机随 PREPARE_VOLUME_PARAM 下发，优先使用该值。
 */
static uint16_t Machine_GetPreparedVolumeMlX100(void)
{
    uint32_t volume_ml_x100;

    if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
        (machine_combo_remote_volume_valid != 0U))
    {
        return machine_combo_remote_final_ml_x100;
    }

    volume_ml_x100 = (uint32_t)machine_combo_current_ml_x100 +
                     (machine_combo_water_ul / 10U);
    if (volume_ml_x100 > 0xFFFFU)
    {
        volume_ml_x100 = 0xFFFFU;
    }

    return (uint16_t)volume_ml_x100;
}

/**
 * @brief 配药完成后同步待机页浓度、活度和体积。
 *
 * @note 浓度显示本次输入的目标浓度；活度只来自 RAM-100 当前有效读数，不再用
 *       “目标浓度 * 体积”反算；体积继续使用本次配药后的理论最终体积。
 */
static void Machine_UpdateStandbyInventoryAfterPrepare(uint8_t use_measured_activity)
{
    uint16_t volume_ml_x100 = Machine_GetPreparedVolumeMlX100();
    uint16_t activity_x100 = 0U;

    if (use_measured_activity != 0U)
    {
        activity_x100 = Machine_GetMeasuredActivityMciX100();
    }

    MachineCMD_SetStandbyInventory(machine_combo_target_conc_x1000,
                                   activity_x100,
                                   volume_ml_x100);
}

/**
 * @brief 判断组合配药流程是否可以离开活度等待状态。
 *
 * @note 本地按键流程要保证机械动作能走完，活度计读数作为检测更新库存，不能把泵和导轨永久卡住；
 *       上位机远控流程则继续严格等待真实新读数，保证协议结果可追溯。
 */
static uint8_t Machine_IsActivityWaitReadyForCombo(MachineActivityWaitResult_e result,
                                                   uint32_t elapsed_ms)
{
    if (result == MACHINE_ACTIVITY_WAIT_OK)
    {
        return 1U;
    }

    if ((machine_combo_owner == MACHINE_FLOW_OWNER_LOCAL) &&
        (elapsed_ms >= MACHINE_COMBO_ACTIVITY_STABLE_MS))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 同步流程开始状态给通信层。
 */
static void Machine_NotifyFlowStarted(MachineFlowOwner_e owner)
{
    Communication_SetSystemState(COMMUNICATION_SYS_RUNNING);
    if (owner == MACHINE_FLOW_OWNER_LOCAL)
    {
        Communication_OnLocalFlowStarted();
    }
}

/**
 * @brief 同步流程结束状态给通信层。
 */
static void Machine_NotifyFlowStopped(MachineFlowOwner_e owner, uint8_t step)
{
    uint16_t measured_activity_x100;
    uint16_t dispense_ml_x100 = 0U;

    if (owner == MACHINE_FLOW_OWNER_LOCAL)
    {
        Communication_OnLocalFlowStopped();
    }
    else
    {
        Communication_SetSystemState(COMMUNICATION_SYS_IDLE);
    }

    if ((owner == MACHINE_FLOW_OWNER_REMOTE) &&
        (step == COMMUNICATION_STEP_PREPARE_DONE))
    {
        measured_activity_x100 = Machine_GetMeasuredActivityMciX100();
        (void)Communication_SendPrepareResult(machine_combo_remote_final_ml_x100,
                                              measured_activity_x100,
                                              machine_combo_remote_seq);
    }

    if (step == COMMUNICATION_STEP_DISPENSE_DONE)
    {
        if (machine_combo_dispense_ul != 0U)
        {
            dispense_ml_x100 = machine_combo_dispense_ml_x100;
        }
        else if (machine_direct_dispense_ul != 0U)
        {
            dispense_ml_x100 = machine_direct_dispense_ml_x100;
        }

        MachineCMD_ConsumeStandbyInventory(dispense_ml_x100);
    }
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
 * @brief 获取现场用于发药的定量泵。
 *
 * @note 发药流程使用泵2；泵1只在配药补水阶段抽取纯净水。
 */
static PumpDrive_s *Machine_GetDispensePump(void)
{
    return PumpDrive_GetPump2();
}

/**
 * @brief 只停止发药定量泵输出。
 *
 * @note 待机直接发药流程只涉及发药泵，不应该顺手改动导杆、阀门或抽水泵。
 */
static void Machine_StopDispensePumpOutput(void)
{
    PumpDrive_s *pump = Machine_GetDispensePump();

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
    Machine_NotifyFlowStopped(machine_combo_owner, COMMUNICATION_STEP_FAILED);
    Machine_StopComboOutputs();
    machine_combo_running = 0U;
    machine_combo_state = MACHINE_COMBO_STATE_ERROR;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
}

/**
 * @brief 暂停当前组合流程。
 *
 * 暂停只停止当前物理输出，不清空状态机状态。
 * 启动键恢复时会从当前 machine_combo_state 继续推进。
 */
static void Machine_PauseCombo(void)
{
    if (machine_combo_paused != 0U)
    {
        return;
    }

    Machine_StopComboOutputs();
    machine_combo_paused = 1U;
    machine_combo_pause_start_ms = Machine_GetMs();
}

/**
 * @brief 从暂停页恢复当前组合流程。
 *
 * 对正在计时的状态，扣除暂停期间经过的时间；
 * 对正在动作的状态，重新执行该状态的一次性启动动作。
 */
static void Machine_ResumeCombo(void)
{
    uint32_t now_ms;

    if (machine_combo_paused == 0U)
    {
        return;
    }

    now_ms = Machine_GetMs();
    machine_combo_state_start_ms += now_ms - machine_combo_pause_start_ms;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;

    Machine_ExecuteComboState();
}

/**
 * @brief 启动发药泵的一段吸液。
 *
 * 发药泵按满行程体积分段，避免一次下发过大步数，也便于暂停和后续光电门校准。
 */
static uint8_t Machine_StartDispensePumpSegment(void)
{
    PumpDrive_s *pump = Machine_GetDispensePump();
    uint32_t segment_ul;

    if ((pump == NULL) || (machine_combo_dispense_remaining_ul == 0U))
    {
        return 0U;
    }

    segment_ul = machine_combo_dispense_segment_ul;
    if (segment_ul == 0U)
    {
        segment_ul = machine_combo_dispense_remaining_ul;
        if (segment_ul > MACHINE_DISPENSE_PUMP_SEGMENT_UL)
        {
            segment_ul = MACHINE_DISPENSE_PUMP_SEGMENT_UL;
        }
    }

    if (PumpDrive_MoveInVolumeUl(pump, segment_ul) == 0U)
    {
        return 0U;
    }

    machine_combo_dispense_segment_ul = segment_ul;
    return 1U;
}

/**
 * @brief 启动一次活度计读数等待。
 *
 * @note 活度计模块本身会周期轮询，但整机流程需要确认“进入本步骤以后”的新读数，
 *       不能直接使用旧缓存。因此这里记录当前 update_count，再主动请求一次读取；
 *       如果底层已经处于 WAITING，说明已有一帧读命令在路上，直接接管这次等待。
 */
static void Machine_BeginActivityWait(void)
{
    ActivityMeterData_s activity_data;

    memset(&activity_data, 0, sizeof(activity_data));
    (void)ActivityMeter_GetData(&activity_data);

    machine_activity_wait_active = 1U;
    machine_activity_wait_started = 0U;
    machine_activity_wait_start_ms = Machine_GetMs();
    machine_activity_wait_update_count = activity_data.update_count;

    if (ActivityMeter_GetState() == ACTIVITY_METER_STATE_WAITING)
    {
        machine_activity_wait_started = 1U;
    }
    else if (ActivityMeter_RequestRead() != 0U)
    {
        machine_activity_wait_started = 1U;
    }
}

/**
 * @brief 判断活度计状态是否为本次流程不可继续的通信错误。
 */
static uint8_t Machine_IsActivityMeterErrorState(ActivityMeterState_e state)
{
    return ((state == ACTIVITY_METER_STATE_TIMEOUT) ||
            (state == ACTIVITY_METER_STATE_CRC_ERROR) ||
            (state == ACTIVITY_METER_STATE_BAD_RESPONSE)) ? 1U : 0U;
}

/**
 * @brief 推进当前活度计读数等待。
 *
 * @param stable_delay_ms 读数等待状态至少保留的时间，单位 ms；0 表示收到新读数即可继续。
 * @return MACHINE_ACTIVITY_WAIT_PENDING/OK/ERROR。
 */
static MachineActivityWaitResult_e Machine_UpdateActivityWait(uint32_t stable_delay_ms)
{
    ActivityMeterData_s activity_data;
    ActivityMeterState_e state;
    uint32_t elapsed_ms;

    if (machine_activity_wait_active == 0U)
    {
        Machine_BeginActivityWait();
    }

    if (machine_activity_wait_started == 0U)
    {
        if (ActivityMeter_GetState() == ACTIVITY_METER_STATE_WAITING)
        {
            machine_activity_wait_started = 1U;
        }
        else if (ActivityMeter_RequestRead() != 0U)
        {
            machine_activity_wait_started = 1U;
        }
        else
        {
            return MACHINE_ACTIVITY_WAIT_PENDING;
        }
    }

    memset(&activity_data, 0, sizeof(activity_data));
    (void)ActivityMeter_GetData(&activity_data);
    state = ActivityMeter_GetState();

    if ((activity_data.state == ACTIVITY_METER_STATE_OK) &&
        (activity_data.update_count != machine_activity_wait_update_count))
    {
        elapsed_ms = Machine_GetMs() - machine_activity_wait_start_ms;
        if (elapsed_ms >= stable_delay_ms)
        {
            machine_activity_wait_active = 0U;
            machine_activity_wait_started = 0U;
            return MACHINE_ACTIVITY_WAIT_OK;
        }

        return MACHINE_ACTIVITY_WAIT_PENDING;
    }

    if ((machine_activity_wait_started != 0U) &&
        (Machine_IsActivityMeterErrorState(state) != 0U))
    {
        machine_activity_wait_active = 0U;
        machine_activity_wait_started = 0U;
        return MACHINE_ACTIVITY_WAIT_ERROR;
    }

    return MACHINE_ACTIVITY_WAIT_PENDING;
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
         * 本地配药使用 LCD 输入的当前浓度和当前体积计算补水量；远程配药必须使用上位机随
         * PREPARE_VOLUME_PARAM 下发的理论补水量，避免下位机缺参数硬算。
         */
        if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
            (machine_combo_remote_volume_valid != 0U))
        {
            machine_combo_water_ul = (uint32_t)machine_combo_remote_water_ml_x100 * 10U;
        }
        else
        {
            machine_combo_water_ul = Machine_CalcPureWaterVolumeUl(machine_combo_current_conc_x1000,
                                                                   machine_combo_current_ml_x100,
                                                                   machine_combo_target_conc_x1000);
        }
        break;

    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
        (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                     SOLENOID_VALVE_STATE_ON_NC_OPEN);
        break;

    case MACHINE_COMBO_STATE_PUMP1_ENABLE:
        /*
         * 泵1和泵2同属 ISC1000 定量泵，体积运动前都需要确认电机处于使能态。
         * 使能命令单独占一个状态，避免紧跟 in 命令时被 RS485 一问一答保护挡住。
         */
        if (PumpDrive_Enable(PumpDrive_GetPump1()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
        if (machine_combo_water_ul == 0U)
        {
            break;
        }

        pump = PumpDrive_GetPump1();
        if ((pump == NULL) || (PumpDrive_MoveInVolumeUl(pump, machine_combo_water_ul) == 0U))
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

    case MACHINE_COMBO_STATE_PUMP2_ENABLE:
        /*
         * 泵2发药前先显式使能，并等待一个状态间隔后再发 in 命令。
         * 这样避免上一条 stop / set / sta 命令后的 RS485 一问一答保护挡住吸液命令。
         */
        if (PumpDrive_Enable(Machine_GetDispensePump()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
        if (machine_combo_dispense_ul == 0U)
        {
            machine_combo_dispense_ul = (uint32_t)machine_combo_dispense_ml_x100 * 10U;
            machine_combo_dispense_remaining_ul = machine_combo_dispense_ul;
            machine_combo_dispense_segment_ul = 0U;
            machine_combo_dispense_done_ul = 0U;
            machine_dispense_progress_percent = 0U;
        }

        if (machine_combo_dispense_remaining_ul == 0U)
        {
            break;
        }

        if (Machine_StartDispensePumpSegment() == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        Machine_BeginActivityWait();
        break;

    case MACHINE_COMBO_STATE_FINISHED:
    case MACHINE_COMBO_STATE_ERROR:
        if ((machine_combo_state == MACHINE_COMBO_STATE_FINISHED) &&
            (machine_combo_dispense_ul != 0U))
        {
            machine_dispense_progress_percent = 100U;
        }
        Machine_StopComboOutputs();
        Machine_NotifyFlowStopped(machine_combo_owner,
                                  (machine_combo_state == MACHINE_COMBO_STATE_FINISHED) ?
                                  ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) ?
                                   COMMUNICATION_STEP_PREPARE_DONE :
                                   COMMUNICATION_STEP_DISPENSE_DONE) :
                                  COMMUNICATION_STEP_FAILED);
        machine_combo_running = 0U;
        break;

    case MACHINE_COMBO_STATE_IDLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_A:
    case MACHINE_COMBO_STATE_GAP_AFTER_B:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_GAP_AFTER_A_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_B_HOME:
    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2:
    default:
        break;
    }
}

/**
 * @brief 启动组合测试。
 *
 * @param current_conc_x1000 配药页确认的当前活度浓度，单位 0.001mCi/ml。
 * @param current_ml_x100 配药页确认的当前溶液体积，单位 0.01ml。
 * @param target_conc_x1000 配药页确认的目标活度浓度，单位 0.001mCi/ml。
 */
static void Machine_StartCombo(uint16_t current_conc_x1000,
                               uint16_t current_ml_x100,
                               uint16_t target_conc_x1000,
                               MachineFlowOwner_e owner)
{
    machine_combo_owner = owner;
    machine_combo_current_conc_x1000 = current_conc_x1000;
    machine_combo_current_ml_x100 = current_ml_x100;
    machine_combo_target_conc_x1000 = target_conc_x1000;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_dispense_progress_percent = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_running = 1U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;

    Machine_StopComboOutputs();
    Machine_NotifyFlowStarted(owner);
    Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_A_PUSH);
}

/**
 * @brief 推进组合测试状态机。
 */
static void Machine_UpdateCombo(void)
{
    uint32_t elapsed_ms;
    MachineActivityWaitResult_e activity_wait;

    if (machine_combo_running == 0U)
    {
        return;
    }

    if (Machine_IsRemoteFlowBlocked(machine_combo_owner) != 0U)
    {
        Machine_AbortCombo();
        return;
    }

    if ((machine_combo_owner == MACHINE_FLOW_OWNER_LOCAL) &&
        (MachineCMD_GetPage() == MACHINE_CMD_PAGE_PAUSED))
    {
        Machine_PauseCombo();
        return;
    }

    if (machine_combo_paused != 0U)
    {
        Machine_ResumeCombo();
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
        activity_wait = Machine_UpdateActivityWait(MACHINE_COMBO_ACTIVITY_STABLE_MS);
        if (Machine_IsActivityWaitReadyForCombo(activity_wait, elapsed_ms) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_CALC_WATER);
        }
        else if ((activity_wait == MACHINE_ACTIVITY_WAIT_ERROR) &&
                 (machine_combo_owner != MACHINE_FLOW_OWNER_LOCAL))
        {
            Machine_AbortCombo();
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
            if (machine_combo_water_ul == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP1);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_ENABLE);
            }
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_ENABLE:
        Machine_ExecuteComboState();
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_WATER_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
        if ((machine_combo_water_ul == 0U) ||
            (PumpDrive_IsMoveDone(PumpDrive_GetPump1()) != 0U))
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
        activity_wait = Machine_UpdateActivityWait(MACHINE_COMBO_ACTIVITY_STABLE_MS);
        if (Machine_IsActivityWaitReadyForCombo(activity_wait, elapsed_ms) != 0U)
        {
            Machine_UpdateStandbyInventoryAfterPrepare((activity_wait == MACHINE_ACTIVITY_WAIT_OK) ? 1U : 0U);

            /*
             * 回退导轨时先收插针导轨（电机B），再收进罐导轨（电机A）。
             * 插针先退出可以先解除针头和瓶口/管路的机械约束，再移动进罐方向。
             */
            Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_B_HOME);
        }
        else if ((activity_wait == MACHINE_ACTIVITY_WAIT_ERROR) &&
                 (machine_combo_owner != MACHINE_FLOW_OWNER_LOCAL))
        {
            Machine_AbortCombo();
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
            if (machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_DISPENSE);
            }
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
            Machine_EnterComboState(MACHINE_COMBO_STATE_STEP_A_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
        if (MachineCMD_ConsumeDispenseConfirmed(&machine_combo_dispense_ml_x100) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_ENABLE:
        Machine_ExecuteComboState();
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
        if (machine_combo_dispense_ul == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
        }
        else if (PumpDrive_IsMoveDone(Machine_GetDispensePump()) != 0U)
        {
            if (machine_combo_dispense_remaining_ul >= machine_combo_dispense_segment_ul)
            {
                machine_combo_dispense_remaining_ul -= machine_combo_dispense_segment_ul;
            }
            else
            {
                machine_combo_dispense_remaining_ul = 0U;
            }

            machine_combo_dispense_done_ul = machine_combo_dispense_ul - machine_combo_dispense_remaining_ul;
            machine_dispense_progress_percent =
                Machine_CalcDispenseProgressPercent(machine_combo_dispense_done_ul, machine_combo_dispense_ul);
            machine_combo_dispense_segment_ul = 0U;

            if (machine_combo_dispense_remaining_ul == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP2);
            }
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN);
        }
        break;

    case MACHINE_COMBO_STATE_FINISHED:
    case MACHINE_COMBO_STATE_ERROR:
    case MACHINE_COMBO_STATE_IDLE:
    default:
        break;
    }
}

/**
 * @brief 进入待机直接发药状态并执行一次性动作。
 *
 * @param next_state 下一个状态。
 */
static void Machine_EnterDirectDispenseState(MachineDirectDispenseState_e next_state)
{
    machine_direct_dispense_state = next_state;
    machine_direct_dispense_state_start_ms = Machine_GetMs();
    Machine_ExecuteDirectDispenseState();
}

/**
 * @brief 启动待机直接发药的一段发药泵吸液。
 *
 * @return 1 表示本段动作已经下发；0 表示发药泵不可用或启动失败。
 */
static uint8_t Machine_StartDirectDispenseSegment(void)
{
    PumpDrive_s *pump = Machine_GetDispensePump();
    uint32_t segment_ul;

    if ((pump == NULL) || (machine_direct_dispense_remaining_ul == 0U))
    {
        return 0U;
    }

    segment_ul = machine_direct_dispense_segment_ul;
    if (segment_ul == 0U)
    {
        segment_ul = machine_direct_dispense_remaining_ul;
        if (segment_ul > MACHINE_DISPENSE_PUMP_SEGMENT_UL)
        {
            segment_ul = MACHINE_DISPENSE_PUMP_SEGMENT_UL;
        }
    }

    if (PumpDrive_MoveInVolumeUl(pump, segment_ul) == 0U)
    {
        return 0U;
    }

    machine_direct_dispense_segment_ul = segment_ul;
    return 1U;
}

/**
 * @brief 执行待机直接发药状态的一次性动作。
 */
static void Machine_ExecuteDirectDispenseState(void)
{
    switch (machine_direct_dispense_state)
    {
    case MACHINE_DIRECT_DISPENSE_STATE_ENABLE:
        if (PumpDrive_Enable(Machine_GetDispensePump()) != 0U)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN:
        if (machine_direct_dispense_ul == 0U)
        {
            machine_direct_dispense_ul = (uint32_t)machine_direct_dispense_ml_x100 * 10U;
            machine_direct_dispense_remaining_ul = machine_direct_dispense_ul;
            machine_direct_dispense_segment_ul = 0U;
            machine_direct_dispense_done_ul = 0U;
            machine_dispense_progress_percent = 0U;
        }

        if (machine_direct_dispense_remaining_ul == 0U)
        {
            break;
        }

        if (Machine_StartDirectDispenseSegment() == 0U)
        {
            Machine_AbortDirectDispense();
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_FINISHED:
    case MACHINE_DIRECT_DISPENSE_STATE_ERROR:
        if ((machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_FINISHED) &&
            (machine_direct_dispense_ul != 0U))
        {
            machine_dispense_progress_percent = 100U;
        }
        Machine_StopDispensePumpOutput();
        Machine_NotifyFlowStopped(machine_direct_dispense_owner,
                                  (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_FINISHED) ?
                                  COMMUNICATION_STEP_DISPENSE_DONE :
                                  COMMUNICATION_STEP_FAILED);
        machine_direct_dispense_running = 0U;
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_IDLE:
    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE:
    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2:
    default:
        break;
    }
}

/**
 * @brief 异常退出待机直接发药流程。
 */
static void Machine_AbortDirectDispense(void)
{
    Machine_NotifyFlowStopped(machine_direct_dispense_owner, COMMUNICATION_STEP_FAILED);
    Machine_StopDispensePumpOutput();
    machine_direct_dispense_running = 0U;
    machine_direct_dispense_state = MACHINE_DIRECT_DISPENSE_STATE_ERROR;
    machine_direct_dispense_paused = 0U;
    machine_direct_dispense_pause_start_ms = 0U;
    machine_direct_dispense_segment_ul = 0U;
}

/**
 * @brief 暂停待机直接发药流程。
 */
static void Machine_PauseDirectDispense(void)
{
    if (machine_direct_dispense_paused != 0U)
    {
        return;
    }

    Machine_StopDispensePumpOutput();
    machine_direct_dispense_paused = 1U;
    machine_direct_dispense_pause_start_ms = Machine_GetMs();
}

/**
 * @brief 从暂停页恢复待机直接发药流程。
 */
static void Machine_ResumeDirectDispense(void)
{
    uint32_t now_ms;

    if (machine_direct_dispense_paused == 0U)
    {
        return;
    }

    now_ms = Machine_GetMs();
    machine_direct_dispense_state_start_ms += now_ms - machine_direct_dispense_pause_start_ms;
    machine_direct_dispense_paused = 0U;
    machine_direct_dispense_pause_start_ms = 0U;

    Machine_ExecuteDirectDispenseState();
}

/**
 * @brief 启动待机页直接发药流程。
 *
 * @param dispense_ml_x100 发药体积，单位 0.01ml。
 */
static void Machine_StartDirectDispense(uint16_t dispense_ml_x100, MachineFlowOwner_e owner)
{
    machine_direct_dispense_owner = owner;
    machine_direct_dispense_ml_x100 = dispense_ml_x100;
    machine_direct_dispense_ul = 0U;
    machine_direct_dispense_remaining_ul = 0U;
    machine_direct_dispense_segment_ul = 0U;
    machine_direct_dispense_done_ul = 0U;
    machine_dispense_progress_percent = 0U;
    machine_direct_dispense_paused = 0U;
    machine_direct_dispense_pause_start_ms = 0U;
    machine_direct_dispense_running = 1U;
    Machine_NotifyFlowStarted(owner);

    /*
     * 待机直接发药刚启动时不先发急停。
     * 急停命令会占用 RS485 一问一答保护窗口，马上发 in 命令会被总线忙挡掉。
     */
    Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_ENABLE);
}

/**
 * @brief 推进待机直接发药流程。
 */
static void Machine_UpdateDirectDispense(void)
{
    uint32_t elapsed_ms;

    if (machine_direct_dispense_running == 0U)
    {
        return;
    }

    if (Machine_IsRemoteFlowBlocked(machine_direct_dispense_owner) != 0U)
    {
        Machine_AbortDirectDispense();
        return;
    }

    if ((machine_direct_dispense_owner == MACHINE_FLOW_OWNER_LOCAL) &&
        (MachineCMD_GetPage() == MACHINE_CMD_PAGE_PAUSED))
    {
        Machine_PauseDirectDispense();
        return;
    }

    if (machine_direct_dispense_paused != 0U)
    {
        Machine_ResumeDirectDispense();
    }

    elapsed_ms = Machine_GetMs() - machine_direct_dispense_state_start_ms;

    switch (machine_direct_dispense_state)
    {
    case MACHINE_DIRECT_DISPENSE_STATE_ENABLE:
        Machine_ExecuteDirectDispenseState();
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN:
        if (machine_direct_dispense_ul == 0U)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_FINISHED);
        }
        else if (PumpDrive_IsMoveDone(Machine_GetDispensePump()) != 0U)
        {
            if (machine_direct_dispense_remaining_ul >= machine_direct_dispense_segment_ul)
            {
                machine_direct_dispense_remaining_ul -= machine_direct_dispense_segment_ul;
            }
            else
            {
                machine_direct_dispense_remaining_ul = 0U;
            }

            machine_direct_dispense_done_ul = machine_direct_dispense_ul - machine_direct_dispense_remaining_ul;
            machine_dispense_progress_percent =
                Machine_CalcDispenseProgressPercent(machine_direct_dispense_done_ul, machine_direct_dispense_ul);
            machine_direct_dispense_segment_ul = 0U;

            if (machine_direct_dispense_remaining_ul == 0U)
            {
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_FINISHED);
            }
            else
            {
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2);
            }
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_FINISHED:
    case MACHINE_DIRECT_DISPENSE_STATE_ERROR:
    case MACHINE_DIRECT_DISPENSE_STATE_IDLE:
    default:
        break;
    }
}

/**
 * @brief 终止药液转移到活度计的协议流程。
 *
 * @note 当前转移流程只承担新版 CAN 协议的步骤门控，尚未直接驱动泵阀。
 *       后续如果现场确认需要真实动作，可在这里统一关闭对应输出。
 */
static void Machine_AbortTransferToActivity(void)
{
    if (machine_transfer_running != 0U)
    {
        Communication_SetSystemState(COMMUNICATION_SYS_IDLE);
    }

    machine_transfer_running = 0U;
    machine_transfer_done = 0U;
    machine_transfer_activity_waiting = 0U;
    machine_transfer_start_ms = 0U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
}

/**
 * @brief 推进药液转移到活度计的非阻塞协议流程。
 *
 * @note 最新协议要求 processId=4 先上报 0x17，再在完成后上报 0x18。
 *       当前参考联调工程用 2 秒模拟转移动作，不在任务里阻塞，也不新增未确认的硬件输出。
 */
static void Machine_UpdateTransferToActivity(void)
{
    uint32_t now_ms;
    MachineActivityWaitResult_e activity_wait;

    if (machine_transfer_running == 0U)
    {
        return;
    }

    if (Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE)
    {
        Machine_AbortTransferToActivity();
        return;
    }

    now_ms = Machine_GetMs();
    if (machine_transfer_activity_waiting == 0U)
    {
        if ((uint32_t)(now_ms - machine_transfer_start_ms) < MACHINE_TRANSFER_TO_ACTIVITY_MS)
        {
            return;
        }

        machine_transfer_activity_waiting = 1U;
        Machine_BeginActivityWait();
        return;
    }

    activity_wait = Machine_UpdateActivityWait(0U);
    if (activity_wait == MACHINE_ACTIVITY_WAIT_OK)
    {
        machine_transfer_running = 0U;
        machine_transfer_done = 1U;
        machine_transfer_activity_waiting = 0U;
        machine_transfer_start_ms = 0U;
        Communication_SetSystemState(COMMUNICATION_SYS_IDLE);
    }
    else if (activity_wait == MACHINE_ACTIVITY_WAIT_ERROR)
    {
        Machine_AbortTransferToActivity();
    }
}

void MachineInit(void)
{
    machine_combo_state = MACHINE_COMBO_STATE_IDLE;
    machine_combo_running = 0U;
    machine_combo_owner = MACHINE_FLOW_OWNER_LOCAL;
    machine_combo_current_conc_x1000 = 0U;
    machine_combo_current_ml_x100 = 0U;
    machine_combo_target_conc_x1000 = 0U;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_remote_water_ml_x100 = 0U;
    machine_combo_remote_final_ml_x100 = 0U;
    machine_combo_remote_initial_activity_x100 = 0U;
    machine_combo_remote_volume_valid = 0U;
    machine_combo_remote_seq = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;

    machine_direct_dispense_state = MACHINE_DIRECT_DISPENSE_STATE_IDLE;
    machine_direct_dispense_state_start_ms = 0U;
    machine_direct_dispense_running = 0U;
    machine_direct_dispense_owner = MACHINE_FLOW_OWNER_LOCAL;
    machine_direct_dispense_ml_x100 = 0U;
    machine_direct_dispense_ul = 0U;
    machine_direct_dispense_remaining_ul = 0U;
    machine_direct_dispense_segment_ul = 0U;
    machine_direct_dispense_done_ul = 0U;
    machine_direct_dispense_paused = 0U;
    machine_direct_dispense_pause_start_ms = 0U;
    machine_dispense_progress_percent = 0U;
    machine_transfer_running = 0U;
    machine_transfer_done = 0U;
    machine_transfer_activity_waiting = 0U;
    machine_transfer_start_ms = 0U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;

    Machine_StopComboOutputs();
}

void MachineControl(void)
{
    uint16_t current_conc_x1000;
    uint16_t current_ml_x100;
    uint16_t target_conc_x1000;
    uint16_t dispense_ml_x100;

    if (MachineCMD_ConsumeResetRequested() != 0U)
    {
        if (machine_combo_running != 0U)
        {
            Machine_AbortCombo();
        }

        if (machine_direct_dispense_running != 0U)
        {
            Machine_AbortDirectDispense();
        }

        Machine_AbortTransferToActivity();

        return;
    }

    if (machine_transfer_running != 0U)
    {
        Machine_UpdateTransferToActivity();
        return;
    }

    if (machine_direct_dispense_running != 0U)
    {
        Machine_UpdateDirectDispense();
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumePrepConfirmed(&current_conc_x1000,
                                         &current_ml_x100,
                                         &target_conc_x1000) != 0U))
    {
        machine_combo_remote_volume_valid = 0U;
        Machine_StartCombo(current_conc_x1000,
                           current_ml_x100,
                           target_conc_x1000,
                           MACHINE_FLOW_OWNER_LOCAL);
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumeDispenseConfirmed(&dispense_ml_x100) != 0U))
    {
        Machine_StartDirectDispense(dispense_ml_x100, MACHINE_FLOW_OWNER_LOCAL);
        return;
    }

    Machine_UpdateCombo();
}

uint8_t MachineCombinationTestIsRunning(void)
{
    return machine_combo_running;
}

uint8_t MachineCombinationTestCanDispense(void)
{
    if ((machine_combo_running != 0U) &&
        (machine_combo_state == MACHINE_COMBO_STATE_WAIT_DISPENSE))
    {
        return 1U;
    }

    return 0U;
}

uint8_t Machine_StartRemotePrepare(uint16_t water_volume_x100,
                                   uint16_t final_volume_x100,
                                   uint16_t initial_activity_x100,
                                   uint16_t target_conc_x1000,
                                   uint8_t seq)
{
    uint16_t current_ml_x100 = 0U;

    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U) ||
        (machine_transfer_done == 0U) ||
        (final_volume_x100 == 0U) ||
        (target_conc_x1000 == 0U))
    {
        return 0U;
    }

    if (final_volume_x100 > water_volume_x100)
    {
        current_ml_x100 = final_volume_x100 - water_volume_x100;
    }

    machine_combo_remote_water_ml_x100 = water_volume_x100;
    machine_combo_remote_final_ml_x100 = final_volume_x100;
    machine_combo_remote_initial_activity_x100 = initial_activity_x100;
    machine_combo_remote_volume_valid = 1U;
    machine_combo_remote_seq = seq;
    machine_transfer_done = 0U;

    /*
     * 远程配药的补水量已经由上位机算好并随 PREPARE_VOLUME_PARAM 下发。
     * 这里保留当前体积用于状态记录，当前浓度填 0，不参与本地补水计算。
     */
    Machine_StartCombo(0U,
                       current_ml_x100,
                       target_conc_x1000,
                       MACHINE_FLOW_OWNER_REMOTE);
    return 1U;
}

uint8_t Machine_StartRemoteTransferToActivity(void)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U) ||
        (machine_transfer_running != 0U))
    {
        return 0U;
    }

    machine_transfer_running = 1U;
    machine_transfer_done = 0U;
    machine_transfer_activity_waiting = 0U;
    machine_transfer_start_ms = Machine_GetMs();
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    Communication_SetSystemState(COMMUNICATION_SYS_RUNNING);
    return 1U;
}

uint8_t Machine_IsTransferToActivityDone(void)
{
    return (machine_transfer_done != 0U) ? 1U : 0U;
}

uint8_t Machine_StartRemoteDispense(uint16_t volume_ml_x100)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U) ||
        (volume_ml_x100 == 0U))
    {
        return 0U;
    }

    Machine_StartDirectDispense(volume_ml_x100, MACHINE_FLOW_OWNER_REMOTE);
    return 1U;
}

uint8_t Machine_GetCommunicationStep(void)
{
    if (machine_transfer_running != 0U)
    {
        return COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY;
    }

    if (machine_direct_dispense_running != 0U)
    {
        if ((machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_ENABLE) ||
            (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE))
        {
            return COMMUNICATION_STEP_DISPENSE_START;
        }

        if ((machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN) ||
            (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2))
        {
            return COMMUNICATION_STEP_DISPENSE_RUNNING;
        }

        return COMMUNICATION_STEP_DISPENSE_DONE;
    }

    if (machine_combo_running == 0U)
    {
        if (machine_transfer_done != 0U)
        {
            return COMMUNICATION_STEP_TRANSFER_DONE;
        }

        return COMMUNICATION_STEP_IDLE;
    }

    switch (machine_combo_state)
    {
    case MACHINE_COMBO_STATE_STEP_A_PUSH:
    case MACHINE_COMBO_STATE_GAP_AFTER_A:
    case MACHINE_COMBO_STATE_STEP_B_PUSH:
    case MACHINE_COMBO_STATE_GAP_AFTER_B:
    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
        return COMMUNICATION_STEP_PREPARE_START;

    case MACHINE_COMBO_STATE_CALC_WATER:
    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
        return COMMUNICATION_STEP_PREPARE_WATER_FILL;

    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        return COMMUNICATION_STEP_PREPARE_WAIT_ACTIVITY;

    case MACHINE_COMBO_STATE_STEP_A_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_A_HOME:
    case MACHINE_COMBO_STATE_STEP_B_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_B_HOME:
        return COMMUNICATION_STEP_PREPARE_RESULT;

    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
        return COMMUNICATION_STEP_PREPARE_DONE;

    case MACHINE_COMBO_STATE_PUMP2_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE:
        return COMMUNICATION_STEP_DISPENSE_START;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2:
        return COMMUNICATION_STEP_DISPENSE_RUNNING;

    case MACHINE_COMBO_STATE_FINISHED:
        return (machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) ?
               COMMUNICATION_STEP_PREPARE_DONE :
               COMMUNICATION_STEP_DISPENSE_DONE;

    case MACHINE_COMBO_STATE_ERROR:
        return COMMUNICATION_STEP_FAILED;

    case MACHINE_COMBO_STATE_IDLE:
    default:
        return COMMUNICATION_STEP_IDLE;
    }
}

uint8_t Machine_GetDispenseProgressPercent(void)
{
    uint8_t percent;

    if ((machine_combo_running != 0U) && (machine_combo_dispense_ul != 0U))
    {
        percent = Machine_CalcDispenseProgressPercent(machine_combo_dispense_done_ul, machine_combo_dispense_ul);
        return (percent >= 100U) ? 99U : percent;
    }

    if ((machine_direct_dispense_running != 0U) && (machine_direct_dispense_ul != 0U))
    {
        percent = Machine_CalcDispenseProgressPercent(machine_direct_dispense_done_ul, machine_direct_dispense_ul);
        return (percent >= 100U) ? 99U : percent;
    }

    return machine_dispense_progress_percent;
}

uint8_t Machine_IsFlowRunning(void)
{
    return ((machine_combo_running != 0U) ||
            (machine_direct_dispense_running != 0U) ||
            (machine_transfer_running != 0U)) ? 1U : 0U;
}
