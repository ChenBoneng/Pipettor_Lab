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
 * 配药/发药整机流程按《分药仪各部件运行流程》实现：
 * - 每次配药前先用泵2按排气体积反向回吸发药管道残液，避免管道残留被遗留在外部；
 * - 本机配药最多 2 个铅罐，每罐流程为进罐 220mm、插针 120mm、抽水泵转移、
 *   泵1按 6ml 分段补水并由抽水泵送入活度计；
 * - 两罐完成后活度计稳定读数 15s，按配药有效容量计算单位浓度；
 * - 配药末尾预留排气定量，默认 10ml，可由上位机参数帧调整；
 * - 发药由泵2定量完成，完成后同样执行一次泵1定量冲洗。
 *
 * 所有动作均为非阻塞状态机推进，MachineTask 里不做长时间阻塞等待。
 */
#define MACHINE_COMBO_STEP_GAP_MS                1000U
#define MACHINE_DISPENSE_PUMP_GAP_MS             0U /* 泵2发药分段之间不额外等待，只等上一段 Busy 结束。 */
#define MACHINE_COMBO_WATER_PUMP_MS              10000U
#define MACHINE_COMBO_FINAL_WATER_PUMP_MS        15000U
#define MACHINE_COMBO_ACTIVITY_STABLE_MS         15000U
#define MACHINE_COMBO_STEP_SPEED_MM_S_X100       2000U
#define MACHINE_COMBO_TANK_POSITION_MM_X100      22000U
#define MACHINE_COMBO_NEEDLE_POSITION_MM_X100    12000U
#define MACHINE_PREP_ACTIVITY_CAPACITY_ML_X100   15000U
#define MACHINE_PREP_ACTIVITY_CAPACITY_UL        150000UL
#define MACHINE_PREP_DEAD_VOLUME_UL              1500UL
#define MACHINE_PREP_MAX_BOTTLE_COUNT            2U
#define MACHINE_PUMP1_WATER_SEGMENT_UL           6000UL

/* 上电只执行一次的导轨物理归零参数。 */
#define MACHINE_STARTUP_HOME_SPEED_MM_S_X100     500U
#define MACHINE_STARTUP_HOME_CONFIRM_MS          10U
#define MACHINE_STARTUP_HOME_NEEDLE_TIMEOUT_MS   40000U
#define MACHINE_STARTUP_HOME_TANK_TIMEOUT_MS     70000U

/* 自动排气/冲洗默认 10.00ml；单独远控排气/冲洗只覆盖本次流程。 */
#define MACHINE_DEFAULT_EXHAUST_VOLUME_ML_X100   1000U
#define MACHINE_DEFAULT_FLUSH_VOLUME_ML_X100     1000U
/*
 * 泵2发药尽量用单条 ISC1000 in 命令连续完成。
 * 之前按 100ul/圈切段，会导致每一圈都等待 Busy 清零再发下一条命令，现场表现为“停一下转一下”。
 * 这里按协议单条命令最大 60000 步换算成泵2整圈体积：60000 / 400 * 100ul = 15000ul。
 */
#define MACHINE_DISPENSE_PUMP_SEGMENT_UL         \
    ((PUMP_DRIVE_COMMAND_MAX_STEPS / PUMP_DRIVE_FULL_STROKE_STEPS) * PUMP_DRIVE_PUMP2_FULL_STROKE_UL)
#define MACHINE_REMOTE_DISPENSE_DONE_HOLD_MS     (COMMUNICATION_STATUS_PERIOD_MS * 2U)
#define MACHINE_REMOTE_STEP_HOLD_MS              (COMMUNICATION_STATUS_PERIOD_MS * 4U)

typedef enum
{
    MACHINE_COMBO_STATE_IDLE = 0,              // 空闲，等待配药参数确认
    MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON,  // 配药前切到发药流路，准备把发药管道残液回吸到活度计
    MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN_VALVE, // 回吸流路稳定等待
    MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE, // 使能泵2准备反向回吸
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_PREP_RETURN_ENABLE, // 泵2回吸使能后的间隔
    MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT, // 泵2按排气体积反向回吸到活度计
    MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN, // 配药前回吸完成后的间隔
    MACHINE_COMBO_STATE_TANK_PUSH,             // 进罐导轨（300mm，电机B）推 3cm
    MACHINE_COMBO_STATE_GAP_AFTER_TANK,        // 进罐导轨动作完成后的间隔
    MACHINE_COMBO_STATE_NEEDLE_PUSH,           // 插针导轨（150mm，电机A）推 5cm
    MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE,      // 插针导轨动作完成后的间隔
    MACHINE_COMBO_STATE_RAW_PUMP_ON,           // 抽水泵把当前铅罐药液抽入活度计
    MACHINE_COMBO_STATE_RAW_PUMP_OFF,          // 关闭抽水泵
    MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY,     // 等待当前溶液活度读数稳定
    MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE,   // 远控转移完成，等待上位机开始配药
    MACHINE_COMBO_STATE_CALC_WATER,            // 按药瓶量和余量计算本罐补水量
    MACHINE_COMBO_STATE_WATER_VALVE_ON,        // 阀1切到 C1-A1，纯净水进入铅罐
    MACHINE_COMBO_STATE_GAP_AFTER_VALVE,       // 阀门动作后的间隔
    MACHINE_COMBO_STATE_PUMP1_ENABLE,          // 使能泵1，避免未使能时体积命令不动作
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE, // 泵1使能后的 RS485 间隔
    MACHINE_COMBO_STATE_PUMP1_WATER_IN,        // 泵1按分段体积抽取纯净水
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1,       // 泵1动作后的间隔
    MACHINE_COMBO_STATE_WATER_PUMP_ON,         // 抽水泵运行 10s，把本段液体送入活度计
    MACHINE_COMBO_STATE_WATER_PUMP_OFF,        // 关闭抽水泵
    MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP,  // 抽水泵关闭后的间隔
    MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON,   // 本罐补水完成后抽水泵持续运行 15s
    MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF,  // 关闭最终转移抽水泵
    MACHINE_COMBO_STATE_TANK_HOME,             // 进罐导轨（300mm，电机B）回原点
    MACHINE_COMBO_STATE_GAP_AFTER_TANK_HOME,   // 进罐导轨回原点后的间隔
    MACHINE_COMBO_STATE_NEEDLE_HOME,           // 插针导轨（150mm，电机A）回原点
    MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE_HOME, // 插针导轨回原点后的间隔
    MACHINE_COMBO_STATE_WAIT_SWITCH_TANK,      // 等待用户换下一个铅罐并确认
    MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY,   // 两罐完成后等待活度计稳定读数
    MACHINE_COMBO_STATE_EXHAUST_VALVE_ON,      // 阀2切到 A2-C2，准备自动排气
    MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE, // 排气流路稳定等待
    MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE,  // 使能泵2准备排气
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE, // 泵2排气使能后的间隔
    MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN,      // 泵2按装机设定体积排气
    MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST,     // 排气完成后的间隔
    MACHINE_COMBO_STATE_FLUSH_VALVE_ON,        // 阀1/阀2切到 C1-B1-B2-C2 冲洗流路
    MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE, // 冲洗流路稳定等待
    MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE,    // 使能泵1准备冲洗
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE, // 泵1冲洗使能后的间隔
    MACHINE_COMBO_STATE_PUMP1_FLUSH_IN,        // 泵1按装机设定体积冲洗
    MACHINE_COMBO_STATE_GAP_AFTER_FLUSH,       // 冲洗完成后的间隔
    MACHINE_COMBO_STATE_WAIT_DISPENSE,         // 等待发药量确认
    MACHINE_COMBO_STATE_PUMP2_ENABLE,          // 发送泵2使能命令
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE, // 泵2使能后等待总线和驱动响应
    MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN,     // 泵2吸入发药体积
    MACHINE_COMBO_STATE_GAP_AFTER_PUMP2,        // 泵2分段吸液后的连续切段检查
    MACHINE_COMBO_STATE_FINISHED,              // 流程结束，关闭输出
    MACHINE_COMBO_STATE_ERROR                  // 动作启动失败，关闭输出
} MachineComboState_e;

typedef enum
{
    MACHINE_DIRECT_DISPENSE_STATE_IDLE = 0,    // 空闲，等待待机页发药量确认
    MACHINE_DIRECT_DISPENSE_STATE_ENABLE,      // 发送泵2使能命令
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_ENABLE, // 使能后等待总线和驱动响应
    MACHINE_DIRECT_DISPENSE_STATE_PUMP2_IN,    // 泵2吸入本次发药体积
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2, // 泵2分段吸液后的连续切段检查
    MACHINE_DIRECT_DISPENSE_STATE_FLUSH_VALVE_ON, // 发药完成后切到冲洗流路
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH_VALVE, // 冲洗流路稳定等待
    MACHINE_DIRECT_DISPENSE_STATE_PUMP1_ENABLE, // 使能泵1准备自动冲洗
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP1_ENABLE, // 泵1使能后的间隔
    MACHINE_DIRECT_DISPENSE_STATE_PUMP1_FLUSH_IN, // 泵1执行发药后定量冲洗
    MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH, // 冲洗完成后的间隔
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
    MACHINE_COMBO_UTILITY_NONE = 0U,      // 正常配药/远控组合流程
    MACHINE_COMBO_UTILITY_EXHAUST_ONLY,  // 本机单独排气：只走泵2排气段
    MACHINE_COMBO_UTILITY_EMPTY_ONLY,    // 本机单独排空：只走泵1冲洗/排空段
    MACHINE_COMBO_UTILITY_RESET_RECOVERY // 复位收尾：泵1冲洗后两根导轨回 0
} MachineComboUtilityMode_e;

typedef enum
{
    MACHINE_ACTIVITY_WAIT_PENDING = 0, // 活度计读数仍在等待中
    MACHINE_ACTIVITY_WAIT_OK,          // 已收到进入等待状态后的新读数
    MACHINE_ACTIVITY_WAIT_ERROR        // 活度计通信超时、CRC 错误或响应格式错误
} MachineActivityWaitResult_e;

typedef enum
{
    MACHINE_STARTUP_HOME_START = 0,
    MACHINE_STARTUP_HOME_SEEK,
    MACHINE_STARTUP_HOME_CONFIRM,
    MACHINE_STARTUP_HOME_DONE,
    MACHINE_STARTUP_HOME_ERROR
} MachineStartupHomeState_e;

typedef enum
{
    MACHINE_MOTOR_RESET_IDLE = 0,
    MACHINE_MOTOR_RESET_NEEDLE,
    MACHINE_MOTOR_RESET_TANK
} MachineMotorResetState_e;

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
static uint8_t machine_combo_remote_process_id = 0U;
static uint32_t machine_combo_dispense_ul = 0U;
static uint32_t machine_combo_dispense_remaining_ul = 0U;
static uint32_t machine_combo_dispense_segment_ul = 0U;
static uint32_t machine_combo_dispense_done_ul = 0U;
static uint8_t machine_combo_bottle_count = 0U;
static uint8_t machine_combo_bottle_index = 0U;
static uint16_t machine_combo_bottle_ml_x100[MACHINE_PREP_MAX_BOTTLE_COUNT] = {0U};
static uint16_t machine_combo_residual_ml_x100 = 0U;
static uint32_t machine_combo_water_per_bottle_ul = 0U;
static uint32_t machine_combo_pump1_remaining_ul = 0U;
static uint32_t machine_combo_pump1_segment_ul = 0U;
static uint32_t machine_combo_post_volume_ul = 0U;
static uint16_t machine_default_exhaust_volume_ml_x100 = MACHINE_DEFAULT_EXHAUST_VOLUME_ML_X100;
static uint16_t machine_default_flush_volume_ml_x100 = MACHINE_DEFAULT_FLUSH_VOLUME_ML_X100;
static uint16_t machine_active_exhaust_volume_ml_x100 = MACHINE_DEFAULT_EXHAUST_VOLUME_ML_X100;
static uint16_t machine_active_flush_volume_ml_x100 = MACHINE_DEFAULT_FLUSH_VOLUME_ML_X100;
static uint8_t machine_active_auto_exhaust = 1U;
static uint8_t machine_active_auto_flush = 0U;
static MachineComboUtilityMode_e machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
static uint8_t machine_combo_prepare_finished = 0U;
static uint16_t machine_combo_final_conc_x1000 = 0U;
static uint8_t machine_combo_paused = 0U;
static uint32_t machine_combo_pause_start_ms = 0U;
static uint8_t machine_combo_final_activity_ready = 0U;
static uint8_t machine_remote_bottle_confirm_seen = 0U;

static MachineDirectDispenseState_e machine_direct_dispense_state = MACHINE_DIRECT_DISPENSE_STATE_IDLE;
static uint32_t machine_direct_dispense_state_start_ms = 0U;
static uint8_t machine_direct_dispense_running = 0U;
static MachineFlowOwner_e machine_direct_dispense_owner = MACHINE_FLOW_OWNER_LOCAL;
static uint16_t machine_direct_dispense_ml_x100 = 0U;
static uint32_t machine_direct_dispense_ul = 0U;
static uint32_t machine_direct_dispense_remaining_ul = 0U;
static uint32_t machine_direct_dispense_segment_ul = 0U;
static uint32_t machine_direct_dispense_done_ul = 0U;
static uint32_t machine_direct_flush_ul = 0U;
static uint8_t machine_direct_dispense_paused = 0U;
static uint32_t machine_direct_dispense_pause_start_ms = 0U;
static uint8_t machine_direct_dispense_seq = 0U;
static uint32_t machine_remote_dispense_done_until_ms = 0U;
static uint8_t machine_remote_hold_step = COMMUNICATION_STEP_IDLE;
static uint32_t machine_remote_hold_until_ms = 0U;
static uint8_t machine_dispense_progress_percent = 0U;
static uint8_t machine_local_dispense_completed = 0U;
static uint8_t machine_transfer_done = 0U;
static uint8_t machine_activity_wait_active = 0U;
static uint8_t machine_activity_wait_started = 0U;
static uint32_t machine_activity_wait_start_ms = 0U;
static uint32_t machine_activity_wait_update_count = 0U;
static MachineStartupHomeState_e machine_startup_home_state = MACHINE_STARTUP_HOME_START;
static StepMotorId_e machine_startup_home_motor = STEP_MOTOR_ID_NEEDLE_RAIL;
static uint32_t machine_startup_home_state_start_ms = 0U;
static uint32_t machine_startup_home_axis_start_ms = 0U;
static uint8_t machine_startup_home_initialized = 0U;
static uint8_t machine_startup_home_complete = 0U;
static uint8_t machine_startup_home_failed = 0U;
static MachineMotorResetState_e machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
static uint8_t machine_motor_reset_running = 0U;

static uint32_t Machine_GetMs(void);
static void Machine_EnterStartupHomeState(MachineStartupHomeState_e next_state);
static void Machine_FailStartupHome(void);
static void Machine_UpdateStartupHome(void);
static void Machine_UpdateMotorReset(void);
static uint32_t Machine_GetPrepCapacityUl(void);
static uint16_t Machine_GetPrepCapacityMlX100(void);
static uint32_t Machine_SubtractVolumeUl(uint32_t volume_ul, uint32_t subtract_ul);
static uint8_t Machine_GetEffectiveBottleCount(uint8_t requested_count,
                                               uint16_t bottle1_ml_x100,
                                               uint16_t bottle2_ml_x100);
static uint32_t Machine_CalcPrepareWaterPerBottleUl(uint8_t bottle_count,
                                                    const uint16_t bottle_ml_x100[],
                                                    uint16_t residual_ml_x100);
static uint16_t Machine_CalcConcentrationFromActivity(uint16_t activity_x100);
static uint8_t Machine_CalcDispenseProgressPercent(uint32_t done_ul, uint32_t target_ul);
static uint32_t Machine_VolumeMlX100ToUl(uint16_t volume_ml_x100);
static uint16_t Machine_VolumeUlToMlX100(uint32_t volume_ul);
static void Machine_SetActivePipeConfig(uint16_t exhaust_volume_x100,
                                        uint16_t flush_volume_x100,
                                        uint8_t auto_exhaust,
                                        uint8_t auto_flush);
static void Machine_SetActivePipeVolumes(uint16_t exhaust_volume_x100, uint16_t flush_volume_x100);
static void Machine_UseDefaultPipeVolumes(void);
static uint32_t Machine_GetExhaustVolumeUl(void);
static uint32_t Machine_GetFlushVolumeUl(void);
static uint16_t Machine_EstimatePumpProgressMlX100(PumpDrive_s *pump, uint32_t total_ul);
static void Machine_UpdateFlushUiProgress(uint16_t done_ml_x100);
static void Machine_SetPrepareWaterFlow(void);
static void Machine_SetDispenseFlow(void);
static void Machine_SetFlushFlow(void);
static void Machine_UpdatePrepUiForState(MachineComboState_e state);
static void Machine_EnterComboState(MachineComboState_e next_state);
static void Machine_ExecuteComboState(void);
static void Machine_StopComboOutputs(void);
static void Machine_AbortCombo(void);
static void Machine_PauseCombo(void);
static void Machine_ResumeCombo(void);
static PumpDrive_s *Machine_GetDispensePump(void);
static void Machine_StopDispensePumpOutput(void);
static uint8_t Machine_StartDispensePumpSegment(void);
static uint8_t Machine_StartPump1WaterSegment(void);
static uint8_t Machine_StartComboPostPumpVolume(PumpDrive_s *pump, uint32_t volume_ul, uint8_t move_out);
static void Machine_StartCombo(uint16_t current_conc_x1000,
                               uint16_t current_ml_x100,
                               uint16_t target_conc_x1000,
                               MachineFlowOwner_e owner);
static void Machine_StartLocalPrepare(uint8_t bottle_count,
                                      uint16_t bottle1_ml_x100,
                                      uint16_t bottle2_ml_x100);
static void Machine_StartUtility(MachineComboUtilityMode_e utility_mode,
                                 MachineComboState_e first_state,
                                 MachineFlowOwner_e owner,
                                 uint8_t process_id,
                                 uint8_t seq);
static void Machine_StartLocalExhaust(void);
static void Machine_StartLocalEmpty(void);
static void Machine_StartResetRecovery(void);
static void Machine_UpdateCombo(void);
static void Machine_EnterDirectDispenseState(MachineDirectDispenseState_e next_state);
static void Machine_ExecuteDirectDispenseState(void);
static void Machine_AbortDirectDispense(void);
static void Machine_PauseDirectDispense(void);
static void Machine_ResumeDirectDispense(void);
static uint8_t Machine_StartDirectDispenseSegment(void);
static uint8_t Machine_StartDirectFlush(void);
static void Machine_StartDirectDispense(uint16_t dispense_ml_x100,
                                        MachineFlowOwner_e owner,
                                        uint16_t flush_volume_x100,
                                        uint8_t auto_flush);
static void Machine_UpdateDirectDispense(void);
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

static void Machine_EnterStartupHomeState(MachineStartupHomeState_e next_state)
{
    machine_startup_home_state = next_state;
    machine_startup_home_state_start_ms = Machine_GetMs();
}

static void Machine_FailStartupHome(void)
{
    StepMotor_StopAll();
    machine_startup_home_failed = 1U;
    Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_ERROR);
}

/**
 * @brief 非阻塞推进本次上电唯一一次导轨物理归零。
 *
 * 插针导轨先按拉方向寻找 PC4，完成后进罐导轨再寻找 PC5。下降沿中断只
 * 记录事件；MachineTask 在这里停止 PWM，并确认信号持续为低后设置软件零点。
 */
static void Machine_UpdateStartupHome(void)
{
    uint32_t now_ms = Machine_GetMs();
    uint32_t speed_pps;
    uint32_t timeout_ms;

    timeout_ms = (machine_startup_home_motor == STEP_MOTOR_ID_NEEDLE_RAIL) ?
                 MACHINE_STARTUP_HOME_NEEDLE_TIMEOUT_MS :
                 MACHINE_STARTUP_HOME_TANK_TIMEOUT_MS;

    switch (machine_startup_home_state)
    {
    case MACHINE_STARTUP_HOME_START:
        (void)StepMotor_ConsumeHomeSensorEvent(machine_startup_home_motor);
        machine_startup_home_axis_start_ms = now_ms;
        if (StepMotor_IsHomeSensorActive(machine_startup_home_motor) != 0U)
        {
            Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_CONFIRM);
            break;
        }

        speed_pps = StepMotor_MmPerSecX100ToPps(MACHINE_STARTUP_HOME_SPEED_MM_S_X100);
        if (StepMotor_RunDebugContinuous(machine_startup_home_motor,
                                         STEP_MOTOR_DIR_PULL,
                                         speed_pps) == 0U)
        {
            Machine_FailStartupHome();
            break;
        }
        Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_SEEK);
        break;

    case MACHINE_STARTUP_HOME_SEEK:
        if ((StepMotor_ConsumeHomeSensorEvent(machine_startup_home_motor) != 0U) ||
            (StepMotor_IsHomeSensorActive(machine_startup_home_motor) != 0U))
        {
            StepMotor_Stop(machine_startup_home_motor);
            Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_CONFIRM);
        }
        else if (((uint32_t)(now_ms - machine_startup_home_axis_start_ms) >=
                  timeout_ms) ||
                 (StepMotor_IsBusy(machine_startup_home_motor) == 0U))
        {
            Machine_FailStartupHome();
        }
        break;

    case MACHINE_STARTUP_HOME_CONFIRM:
        if ((uint32_t)(now_ms - machine_startup_home_state_start_ms) <
            MACHINE_STARTUP_HOME_CONFIRM_MS)
        {
            break;
        }

        if (StepMotor_IsHomeSensorActive(machine_startup_home_motor) != 0U)
        {
            StepMotor_Stop(machine_startup_home_motor);
            if (StepMotor_SetCurrentPositionZero(machine_startup_home_motor) == 0U)
            {
                Machine_FailStartupHome();
                break;
            }
            if (machine_startup_home_motor == STEP_MOTOR_ID_NEEDLE_RAIL)
            {
                machine_startup_home_motor = STEP_MOTOR_ID_TANK_RAIL;
                Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_START);
            }
            else
            {
                machine_startup_home_complete = 1U;
                Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_DONE);
            }
            break;
        }

        if ((uint32_t)(now_ms - machine_startup_home_axis_start_ms) >= timeout_ms)
        {
            Machine_FailStartupHome();
            break;
        }

        (void)StepMotor_ConsumeHomeSensorEvent(machine_startup_home_motor);
        speed_pps = StepMotor_MmPerSecX100ToPps(MACHINE_STARTUP_HOME_SPEED_MM_S_X100);
        if (StepMotor_RunDebugContinuous(machine_startup_home_motor,
                                         STEP_MOTOR_DIR_PULL,
                                         speed_pps) == 0U)
        {
            Machine_FailStartupHome();
            break;
        }
        Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_SEEK);
        break;

    case MACHINE_STARTUP_HOME_DONE:
    case MACHINE_STARTUP_HOME_ERROR:
    default:
        break;
    }
}

/**
 * @brief 获取配药目标容量，单位 ul。
 *
 * @note 上位机和本机配药目标仍按活度计标称 150ml 处理。
 *       配药前泵2回吸进来的管路残液不再降低容量上限，而是作为已进入活度计的体积，
 *       在后续泵1补水量里扣除，兼容上位机仍按 150ml 下发目标体积。
 */
static uint32_t Machine_GetPrepCapacityUl(void)
{
    return MACHINE_PREP_ACTIVITY_CAPACITY_UL;
}

/**
 * @brief 获取配药目标容量，单位 0.01ml。
 */
static uint16_t Machine_GetPrepCapacityMlX100(void)
{
    return Machine_VolumeUlToMlX100(Machine_GetPrepCapacityUl());
}

static uint32_t Machine_SubtractVolumeUl(uint32_t volume_ul, uint32_t subtract_ul)
{
    return (volume_ul > subtract_ul) ? (volume_ul - subtract_ul) : 0U;
}

/**
 * @brief 根据本机 UI 输入判断本次实际参与配药的药瓶数量。
 *
 * @param requested_count UI 选择的药瓶数量，最大 2。
 * @param bottle1_ml_x100 1 号药瓶量，单位 0.01ml。
 * @param bottle2_ml_x100 2 号药瓶量，单位 0.01ml。
 * @return 1/2 表示有效瓶数；0 表示没有有效药瓶量。
 *
 * @note 流程文档明确“药瓶量2为0”时按单瓶公式处理，所以这里以体积是否
 *       大于 0 判断有效数据，而不是盲目相信 UI 选择数量。
 */
static uint8_t Machine_GetEffectiveBottleCount(uint8_t requested_count,
                                               uint16_t bottle1_ml_x100,
                                               uint16_t bottle2_ml_x100)
{
    if ((requested_count == 0U) || (bottle1_ml_x100 == 0U))
    {
        return 0U;
    }

    if ((requested_count >= 2U) && (bottle2_ml_x100 != 0U))
    {
        return 2U;
    }

    return 1U;
}

/**
 * @brief 按文档公式计算每个药罐需要泵1补入的纯净水体积。
 *
 * @param bottle_count 实际有效药瓶数量，1 或 2。
 * @param bottle_ml_x100 药瓶量数组，单位 0.01ml。
 * @param residual_ml_x100 活度计当前余量，单位 0.01ml。
 * @return 每罐泵1补水体积，单位 ul。
 *
 * @note 文档公式：
 *       - 两瓶：纯净水总量 = 配药有效容量 - 药瓶1 + 1.5ml - 药瓶2 + 1.5ml；
 *       - 一瓶：纯净水总量 = 配药有效容量 - 药瓶1 + 1.5ml；
 *       - 有余量时，先从总补水量里扣除余量。
 *       - 配药前回吸只回收已经计入库存的管路液体，不再从补水量里扣除。
 */
static uint32_t Machine_CalcPrepareWaterPerBottleUl(uint8_t bottle_count,
                                                    const uint16_t bottle_ml_x100[],
                                                    uint16_t residual_ml_x100)
{
    uint32_t bottle_total_ul = 0U;
    uint32_t capacity_ul;
    uint32_t dead_total_ul;
    uint32_t residual_ul;
    uint32_t water_total_ul;

    if ((bottle_count == 0U) ||
        (bottle_count > MACHINE_PREP_MAX_BOTTLE_COUNT) ||
        (bottle_ml_x100 == NULL))
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < bottle_count; i++)
    {
        bottle_total_ul += (uint32_t)bottle_ml_x100[i] * 10U;
    }

    dead_total_ul = (uint32_t)bottle_count * MACHINE_PREP_DEAD_VOLUME_UL;
    capacity_ul = Machine_GetPrepCapacityUl();
    if ((capacity_ul == 0U) ||
        (bottle_total_ul >= (capacity_ul + dead_total_ul)))
    {
        return 0U;
    }

    water_total_ul = capacity_ul + dead_total_ul - bottle_total_ul;
    residual_ul = (uint32_t)residual_ml_x100 * 10U;
    water_total_ul = Machine_SubtractVolumeUl(water_total_ul, residual_ul);

    return (water_total_ul + ((uint32_t)bottle_count / 2U)) / (uint32_t)bottle_count;
}

/**
 * @brief 按当前配置的活度计容量计算单位浓度。
 *
 * @param activity_x100 活度，单位 0.01mCi。
 * @return 单位浓度，单位 0.001mCi/ml。
 */
static uint16_t Machine_CalcConcentrationFromActivity(uint16_t activity_x100)
{
    uint16_t volume_ml_x100 = Machine_GetPreparedVolumeMlX100();
    uint32_t conc_x1000;

    if (volume_ml_x100 == 0U)
    {
        return 0U;
    }

    conc_x1000 = (((uint32_t)activity_x100 * 1000U) +
                  (volume_ml_x100 / 2U)) /
                 volume_ml_x100;
    if (conc_x1000 > 0xFFFFU)
    {
        conc_x1000 = 0xFFFFU;
    }

    return (uint16_t)conc_x1000;
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

static uint16_t Machine_VolumeUlToMlX100(uint32_t volume_ul)
{
    return (uint16_t)((volume_ul + 5U) / 10U);
}

static uint32_t Machine_VolumeMlX100ToUl(uint16_t volume_ml_x100)
{
    return (uint32_t)volume_ml_x100 * 10UL;
}

static void Machine_SetActivePipeConfig(uint16_t exhaust_volume_x100,
                                        uint16_t flush_volume_x100,
                                        uint8_t auto_exhaust,
                                        uint8_t auto_flush)
{
    machine_active_exhaust_volume_ml_x100 = exhaust_volume_x100;
    machine_active_flush_volume_ml_x100 = flush_volume_x100;
    machine_active_auto_exhaust = (auto_exhaust != 0U) ? 1U : 0U;
    machine_active_auto_flush = (auto_flush != 0U) ? 1U : 0U;
}

static void Machine_SetActivePipeVolumes(uint16_t exhaust_volume_x100, uint16_t flush_volume_x100)
{
    Machine_SetActivePipeConfig(exhaust_volume_x100, flush_volume_x100, 1U, 0U);
}

static void Machine_UseDefaultPipeVolumes(void)
{
    Machine_SetActivePipeVolumes(machine_default_exhaust_volume_ml_x100,
                                 machine_default_flush_volume_ml_x100);
}

static uint32_t Machine_GetExhaustVolumeUl(void)
{
    return Machine_VolumeMlX100ToUl(machine_active_exhaust_volume_ml_x100);
}

static uint32_t Machine_GetFlushVolumeUl(void)
{
    return Machine_VolumeMlX100ToUl(machine_active_flush_volume_ml_x100);
}

static uint16_t Machine_EstimatePumpProgressMlX100(PumpDrive_s *pump, uint32_t total_ul)
{
    uint32_t elapsed_ms;
    uint32_t done_ul;

    if ((pump == NULL) || (total_ul == 0U))
    {
        return 0U;
    }

    if (pump->move_state == PUMP_DRIVE_MOVE_DONE)
    {
        return Machine_VolumeUlToMlX100(total_ul);
    }

    if ((pump->move_state == PUMP_DRIVE_MOVE_IDLE) ||
        (pump->move_start_ms == 0U) ||
        (pump->move_timeout_ms == 0U))
    {
        return 0U;
    }

    elapsed_ms = Machine_GetMs() - pump->move_start_ms;
    if (elapsed_ms >= pump->move_timeout_ms)
    {
        done_ul = total_ul;
    }
    else
    {
        done_ul = (uint32_t)(((uint64_t)total_ul * elapsed_ms) / pump->move_timeout_ms);
    }

    return Machine_VolumeUlToMlX100(done_ul);
}

static void Machine_UpdateFlushUiProgress(uint16_t done_ml_x100)
{
    uint16_t total_ml_x100 = machine_active_flush_volume_ml_x100;

    if (done_ml_x100 > total_ml_x100)
    {
        done_ml_x100 = total_ml_x100;
    }

    MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_FLUSH,
                               done_ml_x100,
                               total_ml_x100);
}

/**
 * @brief 设置泵1补水到铅罐的流路。
 *
 * @note 现场确认阀1 A/B 口与旧注释相反，阀1掉电表示 C1-A1 连通，
 *       用于“纯净水->泵1->铅罐”。
 */
static void Machine_SetPrepareWaterFlow(void)
{
    SolenoidValve_AllOff();
}

/**
 * @brief 设置泵2发药/排气流路。
 *
 * @note 阀2保持原始映射，掉电表示 A2-C2 连通，对应“活度计->泵2->出口”。
 */
static void Machine_SetDispenseFlow(void)
{
    SolenoidValve_AllOff();
}

/**
 * @brief 设置泵1定量冲洗流路。
 *
 * @note 按文档冲洗流向为“纯净水->泵1->C1->B1->B2->C2”。
 *       现场确认 A/B 口与旧注释相反，所以阀1上电走 C1-B1，阀2掉电走 B2-C2。
 */
static void Machine_SetFlushFlow(void)
{
    SolenoidValve_Off(SOLENOID_VALVE_ID_MED);
    (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                 SOLENOID_VALVE_STATE_ON_NC_OPEN);
}

/**
 * @brief 按当前整机状态同步 LCD 配药运行阶段。
 */
static void Machine_UpdatePrepUiForState(MachineComboState_e state)
{
    uint32_t done_ul;
    uint16_t done_ml_x100;
    uint16_t total_ml_x100;

    if (machine_combo_owner != MACHINE_FLOW_OWNER_LOCAL)
    {
        return;
    }

    if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_RESET_RECOVERY)
    {
        switch (state)
        {
        case MACHINE_COMBO_STATE_FLUSH_VALVE_ON:
        case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE:
        case MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE:
        case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE:
        case MACHINE_COMBO_STATE_PUMP1_FLUSH_IN:
            Machine_UpdateFlushUiProgress(0U);
            break;

        case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH:
            Machine_UpdateFlushUiProgress(machine_active_flush_volume_ml_x100);
            break;

        default:
            MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_ABORTING, 0U, 0U);
            break;
        }
        return;
    }

    switch (state)
    {
    case MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN_VALVE:
    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_PREP_RETURN_ENABLE:
    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT:
    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_EXHAUST, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_TANK_PUSH:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_IN_TANK, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_NEEDLE_PUSH:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_INSERT_NEEDLE, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_DRAW_WATER, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
    case MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF:
        done_ul = machine_combo_water_per_bottle_ul;
        if (machine_combo_pump1_remaining_ul < done_ul)
        {
            done_ul -= machine_combo_pump1_remaining_ul;
        }
        else
        {
            done_ul = 0U;
        }
        done_ml_x100 = (uint16_t)((done_ul + 5U) / 10U);
        total_ml_x100 = (uint16_t)((machine_combo_water_per_bottle_ul + 5U) / 10U);
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_WATER_FILL,
                                   done_ml_x100,
                                   total_ml_x100);
        break;

    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_ACTIVITY_CHECK, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_EXHAUST_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE:
    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE:
    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST:
        MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_EXHAUST, 0U, 0U);
        break;

    case MACHINE_COMBO_STATE_FLUSH_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE:
    case MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE:
        Machine_UpdateFlushUiProgress(0U);
        break;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_IN:
        Machine_UpdateFlushUiProgress(0U);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH:
        Machine_UpdateFlushUiProgress(machine_active_flush_volume_ml_x100);
        break;

    default:
        break;
    }
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
 * @note 本机配药按配药有效容量作为完成体积；
 *       远程配药仍优先使用上位机随 PREPARE_VOLUME_PARAM 下发的最终体积。
 */
static uint16_t Machine_GetPreparedVolumeMlX100(void)
{
    uint16_t capacity_ml_x100 = Machine_GetPrepCapacityMlX100();

    if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
        (machine_combo_remote_volume_valid != 0U))
    {
        return machine_combo_remote_final_ml_x100;
    }

    return capacity_ml_x100;
}

/**
 * @brief 配药完成后同步待机页浓度、活度和体积。
 *
 * @note 本机流程按“活度计读数 / 当前容量配置”得到单位浓度，并同步待机页
 *       的活度和体积。若读数不可用，则浓度和活度保留为 0，避免显示伪数据。
 */
static void Machine_UpdateStandbyInventoryAfterPrepare(uint8_t use_measured_activity)
{
    uint16_t volume_ml_x100 = Machine_GetPreparedVolumeMlX100();
    uint16_t activity_x100 = 0U;
    uint16_t conc_x1000 = machine_combo_target_conc_x1000;

    if (use_measured_activity != 0U)
    {
        activity_x100 = Machine_GetMeasuredActivityMciX100();
        conc_x1000 = Machine_CalcConcentrationFromActivity(activity_x100);
    }

    MachineCMD_SetStandbyInventory(conc_x1000,
                                   activity_x100,
                                   volume_ml_x100);
    machine_combo_final_conc_x1000 = conc_x1000;
}

/**
 * @brief 判断组合配药流程是否可以离开活度等待状态。
 *
 * @note 配药的核心安全动作是泵、阀和导轨按顺序走完。活度计读数用于计算浓度，
 *       但不能在机械动作已经完成后把整套流程永久卡住或误报失败；
 *       若等待期内没有有效新读数，浓度按 0 上报，上位机仍可通过活度计状态帧判断读数异常。
 */
static uint8_t Machine_IsActivityWaitReadyForCombo(MachineActivityWaitResult_e result,
                                                   uint32_t elapsed_ms)
{
    if (result == MACHINE_ACTIVITY_WAIT_OK)
    {
        return 1U;
    }

    if (elapsed_ms >= MACHINE_COMBO_ACTIVITY_STABLE_MS)
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
    if (owner == MACHINE_FLOW_OWNER_REMOTE)
    {
        Communication_OnRemoteFlowStarted();
        machine_remote_hold_step = COMMUNICATION_STEP_IDLE;
        machine_remote_hold_until_ms = 0U;
    }
    else
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
    uint16_t process_detail = 0U;
    uint8_t process_id = 0U;
    uint8_t process_result = COMMUNICATION_RESULT_OK;
    uint8_t process_seq = machine_combo_remote_seq;

    if (owner == MACHINE_FLOW_OWNER_LOCAL)
    {
        Communication_OnLocalFlowStopped();
    }
    else
    {
        Communication_OnRemoteFlowStopped();
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

        if (owner == MACHINE_FLOW_OWNER_LOCAL)
        {
            machine_local_dispense_completed = 1U;
        }
        else
        {
            machine_remote_dispense_done_until_ms =
                Machine_GetMs() + MACHINE_REMOTE_DISPENSE_DONE_HOLD_MS;
            MachineCMD_ReportRemoteDispenseDoneActivity();
            (void)Communication_SendAck(COMMUNICATION_CMD_START_PROCESS,
                                        COMMUNICATION_OBJ_SYSTEM,
                                        COMMUNICATION_RESULT_OK,
                                        COMMUNICATION_ERROR_NONE,
                                        COMMUNICATION_STEP_DISPENSE_DONE,
                                        machine_direct_dispense_seq);
        }
    }

    if (owner == MACHINE_FLOW_OWNER_REMOTE)
    {
        if (step == COMMUNICATION_STEP_FAILED)
        {
            process_result = COMMUNICATION_RESULT_FAILED;
            process_detail = COMMUNICATION_ERROR_PROCESS_FAILED;
            process_id = machine_combo_remote_process_id;
            if ((process_id == 0U) && (machine_direct_dispense_ul != 0U))
            {
                process_id = COMMUNICATION_PROCESS_DISPENSE;
                process_seq = machine_direct_dispense_seq;
            }
        }
        else if (step == COMMUNICATION_STEP_PREPARE_DONE)
        {
            process_id = (machine_combo_remote_process_id != 0U) ?
                         machine_combo_remote_process_id : COMMUNICATION_PROCESS_PREPARE;
            process_detail = Machine_GetPreparedVolumeMlX100();
        }
        else if (step == COMMUNICATION_STEP_DISPENSE_DONE)
        {
            process_id = COMMUNICATION_PROCESS_DISPENSE;
            process_seq = machine_direct_dispense_seq;
            process_detail = dispense_ml_x100;
        }
        else if (step == COMMUNICATION_STEP_EXHAUST_DONE)
        {
            process_id = COMMUNICATION_PROCESS_EXHAUST;
            process_detail = machine_active_exhaust_volume_ml_x100;
        }
        else if (step == COMMUNICATION_STEP_FLUSH_DONE)
        {
            process_id = COMMUNICATION_PROCESS_FLUSH;
            process_detail = machine_active_flush_volume_ml_x100;
        }
        else
        {
            process_id = machine_combo_remote_process_id;
        }

        if ((step != COMMUNICATION_STEP_IDLE) &&
            ((process_id != COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ||
             (step == COMMUNICATION_STEP_FAILED)))
        {
            machine_remote_hold_step = step;
            machine_remote_hold_until_ms = Machine_GetMs() + MACHINE_REMOTE_STEP_HOLD_MS;
        }

        if (process_id != 0U)
        {
            (void)Communication_SendProcessResult(process_id,
                                                  process_result,
                                                  process_detail,
                                                  step,
                                                  process_seq);
        }
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
 * @brief 停止待机直接发药流程相关输出。
 *
 * @note 新流程在泵2发药后会接泵1定量冲洗，因此复位、暂停和异常退出时需要同时停泵1/泵2并释放阀门。
 */
static void Machine_StopDispensePumpOutput(void)
{
    PumpDrive_s *pump;

    SolenoidValve_AllOff();

    pump = Machine_GetDispensePump();
    if (pump != NULL)
    {
        (void)PumpDrive_Stop(pump, 1U);
    }

    pump = PumpDrive_GetPump1();
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
    machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
    machine_combo_remote_process_id = 0U;
    machine_transfer_done = 0U;
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
 * 发药泵按 ISC1000 单条命令上限分段，尽量减少段间 Busy 查询造成的停顿。
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
 * @brief 启动泵1当前 6ml 分段补水。
 *
 * @return 1 表示本段命令已经下发；0 表示泵1不可用或命令失败。
 */
static uint8_t Machine_StartPump1WaterSegment(void)
{
    PumpDrive_s *pump = PumpDrive_GetPump1();
    uint32_t segment_ul;

    if ((pump == NULL) || (machine_combo_pump1_remaining_ul == 0U))
    {
        return 0U;
    }

    segment_ul = machine_combo_pump1_segment_ul;
    if (segment_ul == 0U)
    {
        segment_ul = machine_combo_pump1_remaining_ul;
        if (segment_ul > MACHINE_PUMP1_WATER_SEGMENT_UL)
        {
            segment_ul = MACHINE_PUMP1_WATER_SEGMENT_UL;
        }
    }

    if (PumpDrive_MoveInVolumeUl(pump, segment_ul) == 0U)
    {
        return 0U;
    }

    machine_combo_pump1_segment_ul = segment_ul;
    return 1U;
}

/**
 * @brief 启动组合流程末尾的泵体定量动作。
 *
 * @param pump 目标定量泵。
 * @param volume_ul 目标体积，单位 ul。
 * @param move_out 非 0 表示执行 out；0 表示执行 in。
 * @return 1 表示不需要动作或命令已下发；0 表示泵不可用或命令失败。
 */
static uint8_t Machine_StartComboPostPumpVolume(PumpDrive_s *pump, uint32_t volume_ul, uint8_t move_out)
{
    if (volume_ul == 0U)
    {
        return 1U;
    }

    if (pump == NULL)
    {
        return 0U;
    }

    if (move_out != 0U)
    {
        return PumpDrive_MoveOutVolumeUl(pump, volume_ul);
    }

    return PumpDrive_MoveInVolumeUl(pump, volume_ul);
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
    if (next_state == MACHINE_COMBO_STATE_WAIT_SWITCH_TANK)
    {
        machine_remote_bottle_confirm_seen = 0U;
    }
    Machine_UpdatePrepUiForState(next_state);
    Machine_ExecuteComboState();
}

/**
 * @brief 执行当前状态的一次性动作。
 */
static void Machine_ExecuteComboState(void)
{
    uint8_t final_step;

    switch (machine_combo_state)
    {
    case MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON:
        Machine_SetDispenseFlow();
        break;

    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE:
        if (PumpDrive_Enable(Machine_GetDispensePump()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_PREP_RETURN_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT:
        machine_combo_post_volume_ul = Machine_GetExhaustVolumeUl();
        if (Machine_StartComboPostPumpVolume(Machine_GetDispensePump(),
                                             machine_combo_post_volume_ul,
                                             1U) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_TANK_PUSH:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_TANK_RAIL,
                                          MACHINE_COMBO_TANK_POSITION_MM_X100,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_NEEDLE_PUSH:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_NEEDLE_RAIL,
                                          MACHINE_COMBO_NEEDLE_POSITION_MM_X100,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON:
        if (WaterPump_Start(WATER_PUMP_ID_MAIN) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF:
        WaterPump_Stop(WATER_PUMP_ID_MAIN);
        break;

    case MACHINE_COMBO_STATE_CALC_WATER:
        /*
         * 本机配药的单罐补水量在启动时按药瓶量统一算好；远程配药仍使用上位机
         * 下发的体积参数，避免下位机缺少远程业务参数时硬算。
         */
        if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
            (machine_combo_remote_volume_valid != 0U) &&
            (machine_combo_bottle_count == 0U))
        {
            machine_combo_water_ul = (uint32_t)machine_combo_remote_water_ml_x100 * 10U;
            machine_combo_water_per_bottle_ul = machine_combo_water_ul;
            machine_combo_pump1_remaining_ul = machine_combo_water_ul;
        }
        else
        {
            machine_combo_water_ul = machine_combo_water_per_bottle_ul;
            machine_combo_pump1_remaining_ul = machine_combo_water_per_bottle_ul;
        }
        machine_combo_pump1_segment_ul = 0U;
        break;

    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
        Machine_SetPrepareWaterFlow();
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
        if (machine_combo_pump1_remaining_ul == 0U)
        {
            break;
        }

        if (Machine_StartPump1WaterSegment() == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_TANK_HOME:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_TANK_RAIL,
                                          0U,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_NEEDLE_HOME:
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_NEEDLE_RAIL,
                                          0U,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_ENABLE:
        Machine_SetDispenseFlow();
        /*
         * 泵2发药前先显式使能，并等待一个状态间隔后再发 in 命令。
         * 这样避免上一条 stop / set / sta 命令后的 RS485 一问一答保护挡住吸液命令。
         */
        if (PumpDrive_Enable(Machine_GetDispensePump()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_EXHAUST_VALVE_ON:
        Machine_SetDispenseFlow();
        break;

    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE:
        if (PumpDrive_Enable(Machine_GetDispensePump()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN:
        machine_combo_post_volume_ul = Machine_GetExhaustVolumeUl();
        if (Machine_StartComboPostPumpVolume(Machine_GetDispensePump(),
                                             machine_combo_post_volume_ul,
                                             0U) == 0U)
        {
            Machine_AbortCombo();
        }
        break;

    case MACHINE_COMBO_STATE_FLUSH_VALVE_ON:
        Machine_SetFlushFlow();
        break;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE:
        if (PumpDrive_Enable(PumpDrive_GetPump1()) != 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_IN:
        machine_combo_post_volume_ul = Machine_GetFlushVolumeUl();
        if (Machine_StartComboPostPumpVolume(PumpDrive_GetPump1(),
                                             machine_combo_post_volume_ul,
                                             0U) == 0U)
        {
            Machine_AbortCombo();
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
        if (machine_combo_state == MACHINE_COMBO_STATE_FINISHED)
        {
            if (machine_combo_dispense_ul != 0U)
            {
                final_step = COMMUNICATION_STEP_DISPENSE_DONE;
            }
            else if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_EXHAUST_ONLY)
            {
                final_step = COMMUNICATION_STEP_EXHAUST_DONE;
            }
            else if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_EMPTY_ONLY)
            {
                final_step = COMMUNICATION_STEP_FLUSH_DONE;
            }
            else if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_RESET_RECOVERY)
            {
                final_step = COMMUNICATION_STEP_FLUSH_DONE;
            }
            else
            {
                final_step = COMMUNICATION_STEP_PREPARE_DONE;
            }
        }
        else
        {
            final_step = COMMUNICATION_STEP_FAILED;
        }

        if ((machine_combo_state == MACHINE_COMBO_STATE_FINISHED) &&
            (machine_combo_dispense_ul != 0U))
        {
            machine_dispense_progress_percent = 100U;
        }
        if ((machine_combo_state == MACHINE_COMBO_STATE_FINISHED) &&
            (machine_combo_owner == MACHINE_FLOW_OWNER_LOCAL) &&
            (machine_combo_dispense_ul == 0U))
        {
            if (machine_combo_utility_mode != MACHINE_COMBO_UTILITY_NONE)
            {
                MachineCMD_ReturnToStandby();
            }
            else if (machine_combo_prepare_finished != 0U)
            {
                MachineCMD_SetPrepFinished(machine_combo_final_conc_x1000);
            }
        }
        Machine_StopComboOutputs();
        Machine_NotifyFlowStopped(machine_combo_owner, final_step);
        machine_combo_running = 0U;
        machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
        machine_combo_remote_process_id = 0U;
        break;

    case MACHINE_COMBO_STATE_IDLE:
    case MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE:
    case MACHINE_COMBO_STATE_GAP_AFTER_TANK:
    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
    case MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP:
    case MACHINE_COMBO_STATE_WAIT_SWITCH_TANK:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST:
    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH:
    case MACHINE_COMBO_STATE_GAP_AFTER_TANK_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE_HOME:
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
    Machine_UseDefaultPipeVolumes();

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
    machine_combo_bottle_count = 0U;
    machine_combo_bottle_index = 0U;
    machine_combo_bottle_ml_x100[0] = 0U;
    machine_combo_bottle_ml_x100[1] = 0U;
    machine_combo_residual_ml_x100 = current_ml_x100;
    machine_combo_water_per_bottle_ul = 0U;
    machine_combo_pump1_remaining_ul = 0U;
    machine_combo_pump1_segment_ul = 0U;
    machine_combo_post_volume_ul = 0U;
    machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
    machine_combo_prepare_finished = 0U;
    machine_combo_final_conc_x1000 = 0U;
    machine_dispense_progress_percent = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_final_activity_ready = 0U;
    machine_combo_running = 1U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;

    Machine_StopComboOutputs();
    Machine_NotifyFlowStarted(owner);
    Machine_EnterComboState(MACHINE_COMBO_STATE_TANK_PUSH);
}

/**
 * @brief 启动独立维护动作。
 *
 * @param utility_mode 独立动作类型，用于决定排气后是否继续冲洗。
 * @param first_state 进入的第一段组合状态。
 * @param owner 本机或上位机流程归属，用于控制权和结果上报。
 * @param process_id 远程流程编号，本机流程填 0。
 * @param seq 远程 START_PROCESS 序号，本机流程填 0。
 *
 * @note 单独排气/排空不重新造流程，复用配药末尾已经验证过的泵和阀状态段，
 *       只清空与配药、发药相关的计量状态，避免把维护动作误当成配药完成。
 */
static void Machine_StartUtility(MachineComboUtilityMode_e utility_mode,
                                 MachineComboState_e first_state,
                                 MachineFlowOwner_e owner,
                                 uint8_t process_id,
                                 uint8_t seq)
{
    machine_combo_owner = owner;
    machine_combo_current_conc_x1000 = 0U;
    machine_combo_current_ml_x100 = 0U;
    machine_combo_target_conc_x1000 = 0U;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_combo_bottle_count = 0U;
    machine_combo_bottle_index = 0U;
    machine_combo_bottle_ml_x100[0] = 0U;
    machine_combo_bottle_ml_x100[1] = 0U;
    machine_combo_residual_ml_x100 = 0U;
    machine_combo_water_per_bottle_ul = 0U;
    machine_combo_pump1_remaining_ul = 0U;
    machine_combo_pump1_segment_ul = 0U;
    machine_combo_post_volume_ul = 0U;
    machine_combo_utility_mode = utility_mode;
    machine_combo_prepare_finished = 0U;
    machine_combo_final_conc_x1000 = 0U;
    machine_dispense_progress_percent = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_final_activity_ready = 0U;
    machine_combo_running = 1U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    machine_combo_remote_process_id = process_id;
    machine_combo_remote_seq = seq;

    Machine_StopComboOutputs();
    Machine_NotifyFlowStarted(owner);
    Machine_EnterComboState(first_state);
}

/**
 * @brief 启动本机单独排气动作。
 */
static void Machine_StartLocalExhaust(void)
{
    Machine_UseDefaultPipeVolumes();

    Machine_StartUtility(MACHINE_COMBO_UTILITY_EXHAUST_ONLY,
                         MACHINE_COMBO_STATE_EXHAUST_VALVE_ON,
                         MACHINE_FLOW_OWNER_LOCAL,
                         0U,
                         0U);
}

/**
 * @brief 启动本机单独排空动作。
 */
static void Machine_StartLocalEmpty(void)
{
    Machine_UseDefaultPipeVolumes();

    Machine_StartUtility(MACHINE_COMBO_UTILITY_EMPTY_ONLY,
                         MACHINE_COMBO_STATE_FLUSH_VALVE_ON,
                         MACHINE_FLOW_OWNER_LOCAL,
                         0U,
                         0U);
}

/**
 * @brief 启动复位收尾动作。
 *
 * @note 复位不是急停：它允许设备在已经停止当前输出后，按 LCD 提示执行管路冲洗和导轨回 0。
 *       因此这里先终止原流程，再复用泵1冲洗段和导轨回零段做收尾。
 */
static void Machine_StartResetRecovery(void)
{
    if (machine_combo_running != 0U)
    {
        Machine_AbortCombo();
    }

    if (machine_direct_dispense_running != 0U)
    {
        Machine_AbortDirectDispense();
    }

    Machine_UseDefaultPipeVolumes();
    MachineCMD_SetPrepRunStage(MACHINE_CMD_PREP_RUN_STAGE_ABORTING, 0U, 0U);
    Machine_StartUtility(MACHINE_COMBO_UTILITY_RESET_RECOVERY,
                         MACHINE_COMBO_STATE_FLUSH_VALVE_ON,
                         MACHINE_FLOW_OWNER_LOCAL,
                         0U,
                         0U);
}

/**
 * @brief 启动本机两罐配药流程。
 *
 * @param bottle_count UI 选择药瓶数，最大 2。
 * @param bottle1_ml_x100 1 号药瓶量，单位 0.01ml。
 * @param bottle2_ml_x100 2 号药瓶量，单位 0.01ml。
 */
static void Machine_StartLocalPrepare(uint8_t bottle_count,
                                      uint16_t bottle1_ml_x100,
                                      uint16_t bottle2_ml_x100)
{
    uint8_t effective_count;

    effective_count = Machine_GetEffectiveBottleCount(bottle_count,
                                                      bottle1_ml_x100,
                                                      bottle2_ml_x100);
    if (effective_count == 0U)
    {
        return;
    }

    Machine_UseDefaultPipeVolumes();

    machine_combo_owner = MACHINE_FLOW_OWNER_LOCAL;
    machine_combo_current_conc_x1000 = 0U;
    machine_combo_current_ml_x100 = 0U;
    machine_combo_target_conc_x1000 = 0U;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_combo_bottle_count = effective_count;
    machine_combo_bottle_index = 1U;
    machine_combo_bottle_ml_x100[0] = bottle1_ml_x100;
    machine_combo_bottle_ml_x100[1] = (effective_count >= 2U) ? bottle2_ml_x100 : 0U;
    machine_combo_residual_ml_x100 = MachineCMD_GetStandbyVolumeMlX100();
    machine_combo_water_per_bottle_ul =
        Machine_CalcPrepareWaterPerBottleUl(machine_combo_bottle_count,
                                            machine_combo_bottle_ml_x100,
                                            machine_combo_residual_ml_x100);
    machine_combo_pump1_remaining_ul = 0U;
    machine_combo_pump1_segment_ul = 0U;
    machine_combo_post_volume_ul = 0U;
    machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
    machine_combo_prepare_finished = 0U;
    machine_combo_final_conc_x1000 = 0U;
    machine_dispense_progress_percent = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_final_activity_ready = 0U;
    machine_combo_running = 1U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    machine_combo_remote_seq = 0U;
    machine_combo_remote_process_id = 0U;

    Machine_StopComboOutputs();
    Machine_NotifyFlowStarted(MACHINE_FLOW_OWNER_LOCAL);

    Machine_EnterComboState(MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON);
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
    case MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (Machine_GetExhaustVolumeUl() == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE);
            }
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE:
        Machine_ExecuteComboState();
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_PREP_RETURN_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT:
        if ((Machine_GetExhaustVolumeUl() == 0U) ||
            (PumpDrive_IsMoveDone(Machine_GetDispensePump()) != 0U))
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_TANK_PUSH);
        }
        break;

    case MACHINE_COMBO_STATE_TANK_PUSH:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_TANK_RAIL) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_TANK);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_TANK:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_NEEDLE_PUSH);
        }
        break;

    case MACHINE_COMBO_STATE_NEEDLE_PUSH:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_NEEDLE_RAIL) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE:
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
            if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
                (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY))
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_CALC_WATER);
            }
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
        activity_wait = Machine_UpdateActivityWait(MACHINE_COMBO_ACTIVITY_STABLE_MS);
        if (Machine_IsActivityWaitReadyForCombo(activity_wait, elapsed_ms) != 0U)
        {
            if (machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE)
            {
                machine_transfer_done = 1U;
                Communication_SetSystemState(COMMUNICATION_SYS_IDLE);
                Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_CALC_WATER);
            }
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

    case MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE:
        break;

    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (machine_combo_pump1_remaining_ul == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON);
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
        if ((machine_combo_pump1_remaining_ul == 0U) ||
            (PumpDrive_IsMoveDone(PumpDrive_GetPump1()) != 0U))
        {
            if (machine_combo_pump1_remaining_ul >= machine_combo_pump1_segment_ul)
            {
                machine_combo_pump1_remaining_ul -= machine_combo_pump1_segment_ul;
            }
            else
            {
                machine_combo_pump1_remaining_ul = 0U;
            }
            machine_combo_pump1_segment_ul = 0U;
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
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (machine_combo_pump1_remaining_ul != 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_WATER_IN);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON);
            }
        }
        break;

    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON:
        if (elapsed_ms >= MACHINE_COMBO_FINAL_WATER_PUMP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF);
        }
        break;

    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_NEEDLE_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        activity_wait = Machine_UpdateActivityWait(MACHINE_COMBO_ACTIVITY_STABLE_MS);
        if (Machine_IsActivityWaitReadyForCombo(activity_wait, elapsed_ms) != 0U)
        {
            Machine_UpdateStandbyInventoryAfterPrepare((activity_wait == MACHINE_ACTIVITY_WAIT_OK) ? 1U : 0U);
            machine_combo_prepare_finished = 1U;
            /*
             * 配药完成后的排气属于整机工艺收尾动作。
             * 远控 processId=1 和本机配药保持一致，不要求上位机额外发送排气命令。
             */
            if (machine_active_auto_exhaust != 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_EXHAUST_VALVE_ON);
            }
            else if (machine_active_auto_flush != 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FLUSH_VALVE_ON);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
        }
        else if ((activity_wait == MACHINE_ACTIVITY_WAIT_ERROR) &&
                 (machine_combo_owner != MACHINE_FLOW_OWNER_LOCAL))
        {
            /*
             * 活度计错误不再让已完成机械配药的远控流程失败。
             * 继续等待到稳定窗口结束；若仍无有效读数，完成结果中的活度/浓度按 0 上报。
             */
        }
        break;

    case MACHINE_COMBO_STATE_TANK_HOME:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_TANK_RAIL) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_TANK_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_TANK_HOME:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_RESET_RECOVERY)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
            else if ((machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) &&
                (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY))
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
            else if (machine_combo_bottle_index < machine_combo_bottle_count)
            {
                MachineCMD_SetPrepSwitchTank(machine_combo_bottle_index, machine_combo_bottle_index + 1U);
                Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_SWITCH_TANK);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY);
            }
        }
        break;

    case MACHINE_COMBO_STATE_NEEDLE_HOME:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_NEEDLE_RAIL) == 0U)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE_HOME:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_TANK_HOME);
        }
        break;

    case MACHINE_COMBO_STATE_WAIT_SWITCH_TANK:
        if (MachineCMD_ConsumeLocalPrepSwitchRequested(NULL) != 0U)
        {
            machine_remote_bottle_confirm_seen = 0U;
            machine_combo_bottle_index++;
            Machine_EnterComboState(MACHINE_COMBO_STATE_TANK_PUSH);
        }
        break;

    case MACHINE_COMBO_STATE_EXHAUST_VALVE_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (Machine_GetExhaustVolumeUl() == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE);
            }
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE:
        Machine_ExecuteComboState();
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN:
        if ((Machine_GetExhaustVolumeUl() == 0U) ||
            (PumpDrive_IsMoveDone(Machine_GetDispensePump()) != 0U))
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if ((machine_combo_utility_mode == MACHINE_COMBO_UTILITY_NONE) &&
                (machine_active_auto_flush != 0U))
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FLUSH_VALVE_ON);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
        }
        break;

    case MACHINE_COMBO_STATE_FLUSH_VALVE_ON:
        Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE);
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (Machine_GetFlushVolumeUl() == 0U)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_FLUSH);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE);
            }
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE:
        Machine_ExecuteComboState();
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_PUMP1_FLUSH_IN);
        }
        break;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_IN:
        Machine_UpdateFlushUiProgress(Machine_EstimatePumpProgressMlX100(PumpDrive_GetPump1(),
                                                                         Machine_GetFlushVolumeUl()));
        if ((Machine_GetFlushVolumeUl() == 0U) ||
            (PumpDrive_IsMoveDone(PumpDrive_GetPump1()) != 0U))
        {
            Machine_EnterComboState(MACHINE_COMBO_STATE_GAP_AFTER_FLUSH);
        }
        break;

    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_RESET_RECOVERY)
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_NEEDLE_HOME);
            }
            else
            {
                Machine_EnterComboState(MACHINE_COMBO_STATE_FINISHED);
            }
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
        if (elapsed_ms >= MACHINE_DISPENSE_PUMP_GAP_MS)
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
 * @brief 启动待机发药后的泵1定量冲洗。
 *
 * @return 1 表示不需要冲洗或冲洗命令已经下发；0 表示泵1不可用或命令失败。
 */
static uint8_t Machine_StartDirectFlush(void)
{
    if (machine_direct_flush_ul == 0U)
    {
        return 1U;
    }

    return Machine_StartComboPostPumpVolume(PumpDrive_GetPump1(), machine_direct_flush_ul, 0U);
}

/**
 * @brief 执行待机直接发药状态的一次性动作。
 */
static void Machine_ExecuteDirectDispenseState(void)
{
    switch (machine_direct_dispense_state)
    {
    case MACHINE_DIRECT_DISPENSE_STATE_ENABLE:
        Machine_SetDispenseFlow();
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

    case MACHINE_DIRECT_DISPENSE_STATE_FLUSH_VALVE_ON:
        Machine_SetFlushFlow();
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP1_ENABLE:
        if (PumpDrive_Enable(PumpDrive_GetPump1()) != 0U)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP1_ENABLE);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP1_FLUSH_IN:
        if (Machine_StartDirectFlush() == 0U)
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
    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH_VALVE:
    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP1_ENABLE:
    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH:
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
 * @brief 上位机急停：立即切断输出并终止当前整体流程。
 *
 * @note 急停和复位收尾必须分开。急停只负责停止，不再继续冲洗、导轨回零或其它后续动作。
 */
void Machine_EmergencyStop(void)
{
    machine_motor_reset_running = 0U;
    machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;

    if (machine_combo_running != 0U)
    {
        Machine_AbortCombo();
    }
    else
    {
        Machine_StopComboOutputs();
    }

    if (machine_direct_dispense_running != 0U)
    {
        Machine_AbortDirectDispense();
    }
    else
    {
        Machine_StopDispensePumpOutput();
    }
}

void Machine_ResetRuntimeState(void)
{
    Machine_StopComboOutputs();
    Machine_StopDispensePumpOutput();
    MachineInit();
}

uint8_t Machine_StartMotorReset(void)
{
    if (machine_motor_reset_running != 0U)
    {
        return 1U;
    }

    if ((machine_startup_home_complete == 0U) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U))
    {
        return 0U;
    }

    StepMotor_StopAll();
    machine_motor_reset_running = 1U;
    machine_motor_reset_state = MACHINE_MOTOR_RESET_NEEDLE;

    if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_NEEDLE_RAIL,
                                      0U,
                                      MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
    {
        machine_motor_reset_running = 0U;
        machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
        return 0U;
    }

    return 1U;
}

static void Machine_UpdateMotorReset(void)
{
    if (machine_motor_reset_running == 0U)
    {
        return;
    }

    if (machine_motor_reset_state == MACHINE_MOTOR_RESET_NEEDLE)
    {
        if (StepMotor_IsBusy(STEP_MOTOR_ID_NEEDLE_RAIL) != 0U)
        {
            return;
        }

        machine_motor_reset_state = MACHINE_MOTOR_RESET_TANK;
        if (StepMotor_RunToPositionMmX100(STEP_MOTOR_ID_TANK_RAIL,
                                          0U,
                                          MACHINE_COMBO_STEP_SPEED_MM_S_X100) == 0U)
        {
            machine_motor_reset_running = 0U;
            machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
        }
        return;
    }

    if ((machine_motor_reset_state == MACHINE_MOTOR_RESET_TANK) &&
        (StepMotor_IsBusy(STEP_MOTOR_ID_TANK_RAIL) == 0U))
    {
        machine_motor_reset_running = 0U;
        machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
    }
}

void Machine_ClearRemoteStepHold(void)
{
    machine_remote_hold_step = COMMUNICATION_STEP_IDLE;
    machine_remote_hold_until_ms = 0U;
    machine_remote_dispense_done_until_ms = 0U;
}

void Machine_SetPipeVolumes(uint16_t exhaust_volume_x100, uint16_t flush_volume_x100)
{
    machine_default_exhaust_volume_ml_x100 = exhaust_volume_x100;
    machine_default_flush_volume_ml_x100 = flush_volume_x100;
    Machine_UseDefaultPipeVolumes();
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
static void Machine_StartDirectDispense(uint16_t dispense_ml_x100,
                                        MachineFlowOwner_e owner,
                                        uint16_t flush_volume_x100,
                                        uint8_t auto_flush)
{
    Machine_SetActivePipeConfig(machine_default_exhaust_volume_ml_x100,
                                flush_volume_x100,
                                0U,
                                auto_flush);

    machine_direct_dispense_owner = owner;
    machine_direct_dispense_ml_x100 = dispense_ml_x100;
    machine_direct_dispense_ul = 0U;
    machine_direct_dispense_remaining_ul = 0U;
    machine_direct_dispense_segment_ul = 0U;
    machine_direct_dispense_done_ul = 0U;
    machine_direct_flush_ul = (auto_flush != 0U) ? Machine_GetFlushVolumeUl() : 0U;
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
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_FLUSH_VALVE_ON);
            }
            else
            {
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2);
            }
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_FLUSH_VALVE_ON:
        Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH_VALVE);
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH_VALVE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            if (machine_direct_flush_ul == 0U)
            {
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH);
            }
            else
            {
                Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_PUMP1_ENABLE);
            }
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP1_ENABLE:
        Machine_ExecuteDirectDispenseState();
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP1_ENABLE:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_PUMP1_FLUSH_IN);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_PUMP1_FLUSH_IN:
        if ((machine_direct_flush_ul == 0U) ||
            (PumpDrive_IsMoveDone(PumpDrive_GetPump1()) != 0U))
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH:
        if (elapsed_ms >= MACHINE_COMBO_STEP_GAP_MS)
        {
            Machine_EnterDirectDispenseState(MACHINE_DIRECT_DISPENSE_STATE_FINISHED);
        }
        break;

    case MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP2:
        if (elapsed_ms >= MACHINE_DISPENSE_PUMP_GAP_MS)
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
    machine_combo_remote_process_id = 0U;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_combo_bottle_count = 0U;
    machine_combo_bottle_index = 0U;
    machine_combo_bottle_ml_x100[0] = 0U;
    machine_combo_bottle_ml_x100[1] = 0U;
    machine_combo_residual_ml_x100 = 0U;
    machine_combo_water_per_bottle_ul = 0U;
    machine_combo_pump1_remaining_ul = 0U;
    machine_combo_pump1_segment_ul = 0U;
    machine_combo_post_volume_ul = 0U;
    machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
    machine_combo_prepare_finished = 0U;
    machine_combo_final_conc_x1000 = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_final_activity_ready = 0U;
    machine_remote_bottle_confirm_seen = 0U;

    machine_direct_dispense_state = MACHINE_DIRECT_DISPENSE_STATE_IDLE;
    machine_direct_dispense_state_start_ms = 0U;
    machine_direct_dispense_running = 0U;
    machine_direct_dispense_owner = MACHINE_FLOW_OWNER_LOCAL;
    machine_direct_dispense_ml_x100 = 0U;
    machine_direct_dispense_ul = 0U;
    machine_direct_dispense_remaining_ul = 0U;
    machine_direct_dispense_segment_ul = 0U;
    machine_direct_dispense_done_ul = 0U;
    machine_direct_flush_ul = 0U;
    machine_direct_dispense_paused = 0U;
    machine_direct_dispense_pause_start_ms = 0U;
    machine_direct_dispense_seq = 0U;
    machine_remote_dispense_done_until_ms = 0U;
    machine_remote_hold_step = COMMUNICATION_STEP_IDLE;
    machine_remote_hold_until_ms = 0U;
    machine_dispense_progress_percent = 0U;
    machine_local_dispense_completed = 0U;
    machine_transfer_done = 0U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
    machine_motor_reset_running = 0U;

    Machine_UseDefaultPipeVolumes();
    Machine_StopComboOutputs();

    if (machine_startup_home_initialized == 0U)
    {
        machine_startup_home_complete = 0U;
        machine_startup_home_failed = 0U;
        machine_startup_home_axis_start_ms = 0U;
        machine_startup_home_motor = STEP_MOTOR_ID_NEEDLE_RAIL;
        machine_startup_home_initialized = 1U;
        Machine_EnterStartupHomeState(MACHINE_STARTUP_HOME_START);
    }
}

void MachineControl(void)
{
    uint8_t bottle_count;
    uint16_t bottle1_ml_x100;
    uint16_t bottle2_ml_x100;
    uint16_t current_conc_x1000;
    uint16_t current_ml_x100;
    uint16_t target_conc_x1000;
    uint16_t dispense_ml_x100;

    if (machine_startup_home_complete == 0U)
    {
        Machine_UpdateStartupHome();
        return;
    }

    if (MachineCMD_ConsumeResetRequested() != 0U)
    {
        if ((machine_combo_running != 0U) ||
            (machine_direct_dispense_running != 0U))
        {
            Machine_StartResetRecovery();
            return;
        }

        Machine_StopComboOutputs();
        machine_motor_reset_running = 0U;
        machine_motor_reset_state = MACHINE_MOTOR_RESET_IDLE;
        return;
    }

    if (machine_motor_reset_running != 0U)
    {
        Machine_UpdateMotorReset();
        return;
    }

    if (machine_direct_dispense_running != 0U)
    {
        Machine_UpdateDirectDispense();
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumeLocalExhaustRequested() != 0U))
    {
        Machine_StartLocalExhaust();
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumeLocalEmptyRequested() != 0U))
    {
        Machine_StartLocalEmpty();
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumeLocalPrepStartRequested(&bottle_count,
                                                   &bottle1_ml_x100,
                                                   &bottle2_ml_x100) != 0U))
    {
        machine_combo_remote_volume_valid = 0U;
        Machine_StartLocalPrepare(bottle_count,
                                  bottle1_ml_x100,
                                  bottle2_ml_x100);
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
        (MachineCMD_ConsumeLocalDispenseStartRequested(&dispense_ml_x100) != 0U))
    {
        Machine_StartDirectDispense(dispense_ml_x100,
                                    MACHINE_FLOW_OWNER_LOCAL,
                                    machine_default_flush_volume_ml_x100,
                                    1U);
        return;
    }

    if ((machine_combo_running == 0U) &&
        (MachineCMD_ConsumeDispenseConfirmed(&dispense_ml_x100) != 0U))
    {
        Machine_StartDirectDispense(dispense_ml_x100,
                                    MACHINE_FLOW_OWNER_LOCAL,
                                    machine_default_flush_volume_ml_x100,
                                    1U);
        return;
    }

    Machine_UpdateCombo();
}

uint8_t Machine_IsStartupHomeComplete(void)
{
    return machine_startup_home_complete;
}

uint8_t Machine_IsStartupHomeFailed(void)
{
    return machine_startup_home_failed;
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

uint8_t Machine_ConsumeCombinationFinalActivityReady(void)
{
    uint8_t ready = machine_combo_final_activity_ready;

    machine_combo_final_activity_ready = 0U;
    return ready;
}

uint8_t Machine_ConsumeLocalDispenseCompleted(void)
{
    uint8_t completed = machine_local_dispense_completed;

    machine_local_dispense_completed = 0U;
    return completed;
}

uint8_t Machine_StartRemotePrepare(uint16_t water_volume_x100,
                                   uint16_t final_volume_x100,
                                   uint16_t initial_activity_x100,
                                   uint16_t target_conc_x1000,
                                   uint8_t seq)
{
    uint16_t current_ml_x100 = 0U;

    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running == 0U) ||
        (machine_combo_owner != MACHINE_FLOW_OWNER_REMOTE) ||
        (machine_combo_state != MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE) ||
        (machine_direct_dispense_running != 0U) ||
        (machine_transfer_done == 0U) ||
        (final_volume_x100 > COMMUNICATION_PREP_FINAL_VOLUME_MAX_X100) ||
        ((final_volume_x100 % COMMUNICATION_PREP_FINAL_VOLUME_STEP_X100) != 0U) ||
        (target_conc_x1000 == 0U))
    {
        return 0U;
    }

    if (final_volume_x100 > water_volume_x100)
    {
        current_ml_x100 = final_volume_x100 - water_volume_x100;
    }

    Machine_UseDefaultPipeVolumes();

    machine_combo_remote_water_ml_x100 = water_volume_x100;
    machine_combo_remote_final_ml_x100 = final_volume_x100;
    machine_combo_remote_initial_activity_x100 = initial_activity_x100;
    machine_combo_remote_volume_valid = 1U;
    machine_combo_remote_seq = seq;
    machine_combo_remote_process_id = COMMUNICATION_PROCESS_PREPARE;
    machine_transfer_done = 0U;

    /*
     * 远程配药的补水量已经由上位机算好并随 PREPARE_VOLUME_PARAM 下发。
     * 这里保留当前体积用于状态记录，当前浓度填 0，不参与本地补水计算。
     */
    machine_combo_current_conc_x1000 = 0U;
    machine_combo_current_ml_x100 = current_ml_x100;
    machine_combo_target_conc_x1000 = target_conc_x1000;
    Machine_NotifyFlowStarted(MACHINE_FLOW_OWNER_REMOTE);
    Machine_EnterComboState(MACHINE_COMBO_STATE_CALC_WATER);
    return 1U;
}

uint8_t Machine_StartRemotePrepareByBottle(uint16_t bottle1_ml_x100,
                                           uint16_t bottle2_ml_x100,
                                           uint16_t water_volume_x100,
                                           uint16_t final_volume_x100,
                                           uint16_t initial_activity_x100,
                                           uint16_t target_conc_x1000,
                                           uint16_t pipe_exhaust_volume_x100,
                                           uint16_t pipe_flush_volume_x100,
                                           uint8_t auto_exhaust,
                                           uint8_t auto_flush,
                                           uint8_t seq)
{
    uint8_t effective_count;
    uint32_t water_total_ul;

    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U) ||
        (final_volume_x100 > COMMUNICATION_PREP_FINAL_VOLUME_MAX_X100) ||
        ((final_volume_x100 % COMMUNICATION_PREP_FINAL_VOLUME_STEP_X100) != 0U))
    {
        return 0U;
    }

    effective_count = Machine_GetEffectiveBottleCount(MACHINE_PREP_MAX_BOTTLE_COUNT,
                                                      bottle1_ml_x100,
                                                      bottle2_ml_x100);
    if (effective_count == 0U)
    {
        return 0U;
    }

    Machine_SetActivePipeConfig(pipe_exhaust_volume_x100,
                                pipe_flush_volume_x100,
                                auto_exhaust,
                                auto_flush);

    water_total_ul = (uint32_t)water_volume_x100 * 10U;

    machine_combo_owner = MACHINE_FLOW_OWNER_REMOTE;
    machine_combo_current_conc_x1000 = 0U;
    machine_combo_current_ml_x100 = 0U;
    machine_combo_target_conc_x1000 = target_conc_x1000;
    machine_combo_dispense_ml_x100 = 0U;
    machine_combo_water_ul = water_total_ul;
    machine_combo_dispense_ul = 0U;
    machine_combo_dispense_remaining_ul = 0U;
    machine_combo_dispense_segment_ul = 0U;
    machine_combo_dispense_done_ul = 0U;
    machine_combo_bottle_count = effective_count;
    machine_combo_bottle_index = 1U;
    machine_combo_bottle_ml_x100[0] = bottle1_ml_x100;
    machine_combo_bottle_ml_x100[1] = (effective_count >= 2U) ? bottle2_ml_x100 : 0U;
    machine_combo_residual_ml_x100 = 0U;
    machine_combo_water_per_bottle_ul =
        (water_total_ul + ((uint32_t)effective_count / 2U)) / (uint32_t)effective_count;
    machine_combo_pump1_remaining_ul = 0U;
    machine_combo_pump1_segment_ul = 0U;
    machine_combo_post_volume_ul = 0U;
    machine_combo_utility_mode = MACHINE_COMBO_UTILITY_NONE;
    machine_combo_prepare_finished = 0U;
    machine_combo_final_conc_x1000 = 0U;
    machine_dispense_progress_percent = 0U;
    machine_combo_paused = 0U;
    machine_combo_pause_start_ms = 0U;
    machine_combo_final_activity_ready = 0U;
    machine_remote_bottle_confirm_seen = 0U;
    machine_combo_running = 1U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    machine_transfer_done = 0U;
    machine_combo_remote_water_ml_x100 = Machine_VolumeUlToMlX100(water_total_ul);
    machine_combo_remote_final_ml_x100 = final_volume_x100;
    machine_combo_remote_initial_activity_x100 = initial_activity_x100;
    machine_combo_remote_volume_valid = 1U;
    machine_combo_remote_seq = seq;
    machine_combo_remote_process_id = COMMUNICATION_PROCESS_PREPARE;

    /*
     * 最新上位机流程把 processId=1 定义为完整配药入口。
     * 这里只缓存上位机下发的体积结果，不再要求先走 processId=4 的转移等待态。
     */
    Machine_StopComboOutputs();
    Machine_NotifyFlowStarted(MACHINE_FLOW_OWNER_REMOTE);
    Machine_EnterComboState(MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON);
    return 1U;
}

uint8_t Machine_StartRemoteTransferToActivity(void)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U))
    {
        return 0U;
    }

    machine_transfer_done = 0U;
    machine_activity_wait_active = 0U;
    machine_activity_wait_started = 0U;
    machine_combo_remote_volume_valid = 0U;
    machine_combo_remote_process_id = COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY;
    Machine_StartCombo(0U, 0U, 0U, MACHINE_FLOW_OWNER_REMOTE);
    return 1U;
}

uint8_t Machine_IsTransferToActivityDone(void)
{
    return (machine_transfer_done != 0U) ? 1U : 0U;
}

uint8_t Machine_StartRemoteDispense(uint16_t volume_ml_x100,
                                    uint16_t flush_volume_x100,
                                    uint8_t auto_flush,
                                    uint8_t seq)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U) ||
        (volume_ml_x100 == 0U))
    {
        return 0U;
    }

    machine_direct_dispense_seq = seq;
    machine_remote_dispense_done_until_ms = 0U;
    Machine_StartDirectDispense(volume_ml_x100,
                                MACHINE_FLOW_OWNER_REMOTE,
                                flush_volume_x100,
                                auto_flush);
    return 1U;
}

uint8_t Machine_StartRemoteFlush(uint16_t flush_volume_x100, uint8_t seq)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U))
    {
        return 0U;
    }

    Machine_SetActivePipeConfig(machine_default_exhaust_volume_ml_x100,
                                flush_volume_x100,
                                0U,
                                1U);
    Machine_StartUtility(MACHINE_COMBO_UTILITY_EMPTY_ONLY,
                         MACHINE_COMBO_STATE_FLUSH_VALVE_ON,
                         MACHINE_FLOW_OWNER_REMOTE,
                         COMMUNICATION_PROCESS_FLUSH,
                         seq);
    return 1U;
}

uint8_t Machine_StartRemoteExhaust(uint16_t exhaust_volume_x100, uint8_t seq)
{
    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running != 0U) ||
        (machine_direct_dispense_running != 0U))
    {
        return 0U;
    }

    Machine_SetActivePipeConfig(exhaust_volume_x100,
                                machine_default_flush_volume_ml_x100,
                                1U,
                                0U);
    Machine_StartUtility(MACHINE_COMBO_UTILITY_EXHAUST_ONLY,
                         MACHINE_COMBO_STATE_EXHAUST_VALVE_ON,
                         MACHINE_FLOW_OWNER_REMOTE,
                         COMMUNICATION_PROCESS_EXHAUST,
                         seq);
    return 1U;
}

uint8_t Machine_ConfirmRemoteBottleChanged(uint8_t bottle_index,
                                           uint8_t confirm_step,
                                           uint8_t *ack_result,
                                           uint16_t *ack_error)
{
    uint8_t expected_bottle_index;

    if (ack_result != NULL)
    {
        *ack_result = COMMUNICATION_RESULT_OK;
    }
    if (ack_error != NULL)
    {
        *ack_error = COMMUNICATION_ERROR_NONE;
    }

    if ((confirm_step != COMMUNICATION_BOTTLE_CONFIRM_SEEN) &&
        (confirm_step != COMMUNICATION_BOTTLE_CONFIRM_CHANGED))
    {
        if (ack_result != NULL)
        {
            *ack_result = COMMUNICATION_RESULT_BAD_PARAM;
        }
        if (ack_error != NULL)
        {
            *ack_error = COMMUNICATION_ERROR_BAD_PARAM;
        }
        return 0U;
    }

    if ((Communication_GetControlMode() != COMMUNICATION_CONTROL_REMOTE) ||
        (machine_combo_running == 0U) ||
        (machine_combo_owner != MACHINE_FLOW_OWNER_REMOTE) ||
        (machine_combo_remote_process_id != COMMUNICATION_PROCESS_PREPARE) ||
        (machine_combo_bottle_count < 2U) ||
        (machine_combo_state != MACHINE_COMBO_STATE_WAIT_SWITCH_TANK))
    {
        if (ack_result != NULL)
        {
            *ack_result = COMMUNICATION_RESULT_BUSY;
        }
        if (ack_error != NULL)
        {
            *ack_error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
        return 0U;
    }

    expected_bottle_index = machine_combo_bottle_index + 1U;
    if (bottle_index != expected_bottle_index)
    {
        if (ack_result != NULL)
        {
            *ack_result = COMMUNICATION_RESULT_BAD_PARAM;
        }
        if (ack_error != NULL)
        {
            *ack_error = COMMUNICATION_ERROR_BAD_PARAM;
        }
        return 0U;
    }

    if (confirm_step == COMMUNICATION_BOTTLE_CONFIRM_SEEN)
    {
        machine_remote_bottle_confirm_seen = 1U;
        return 1U;
    }

    machine_remote_bottle_confirm_seen = 0U;
    machine_combo_bottle_index++;
    Machine_EnterComboState(MACHINE_COMBO_STATE_TANK_PUSH);
    return 1U;
}

uint8_t Machine_GetCommunicationStep(void)
{
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

        if ((machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_FLUSH_VALVE_ON) ||
            (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH_VALVE) ||
            (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_PUMP1_ENABLE) ||
            (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_PUMP1_ENABLE))
        {
            return COMMUNICATION_STEP_FLUSH_START;
        }

        if (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_PUMP1_FLUSH_IN)
        {
            return COMMUNICATION_STEP_FLUSH_RUNNING;
        }

        if (machine_direct_dispense_state == MACHINE_DIRECT_DISPENSE_STATE_GAP_AFTER_FLUSH)
        {
            return COMMUNICATION_STEP_FLUSH_DONE;
        }

        return COMMUNICATION_STEP_DISPENSE_DONE;
    }

    if (machine_combo_running == 0U)
    {
        if ((machine_remote_hold_until_ms != 0U) &&
            ((int32_t)(machine_remote_hold_until_ms - Machine_GetMs()) > 0))
        {
            return machine_remote_hold_step;
        }

        machine_remote_hold_step = COMMUNICATION_STEP_IDLE;
        machine_remote_hold_until_ms = 0U;

        if ((machine_remote_dispense_done_until_ms != 0U) &&
            ((int32_t)(machine_remote_dispense_done_until_ms - Machine_GetMs()) > 0))
        {
            return COMMUNICATION_STEP_DISPENSE_DONE;
        }

        machine_remote_dispense_done_until_ms = 0U;

        if (machine_transfer_done != 0U)
        {
            return COMMUNICATION_STEP_TRANSFER_DONE;
        }

        return COMMUNICATION_STEP_IDLE;
    }

    switch (machine_combo_state)
    {
    case MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE:
        return COMMUNICATION_STEP_TRANSFER_DONE;

    case MACHINE_COMBO_STATE_PREP_RETURN_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN_VALVE:
    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_PREP_RETURN_ENABLE:
    case MACHINE_COMBO_STATE_PUMP2_PREP_RETURN_OUT:
    case MACHINE_COMBO_STATE_GAP_AFTER_PREP_RETURN:
        return COMMUNICATION_STEP_EXHAUST_RUNNING;

    case MACHINE_COMBO_STATE_TANK_PUSH:
    case MACHINE_COMBO_STATE_GAP_AFTER_TANK:
        return (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_CANISTER_IN;

    case MACHINE_COMBO_STATE_NEEDLE_PUSH:
    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE:
        return (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_NEEDLE_IN;

    case MACHINE_COMBO_STATE_RAW_PUMP_ON:
    case MACHINE_COMBO_STATE_RAW_PUMP_OFF:
        return (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_WATER_PUMP;

    case MACHINE_COMBO_STATE_CALC_WATER:
    case MACHINE_COMBO_STATE_WATER_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_VALVE:
    case MACHINE_COMBO_STATE_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_ENABLE:
    case MACHINE_COMBO_STATE_PUMP1_WATER_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1:
        return COMMUNICATION_STEP_PREPARE_PUMP1_FILL;

    case MACHINE_COMBO_STATE_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_WATER_PUMP_OFF:
    case MACHINE_COMBO_STATE_GAP_AFTER_WATER_PUMP:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_ON:
    case MACHINE_COMBO_STATE_FINAL_WATER_PUMP_OFF:
        return COMMUNICATION_STEP_PREPARE_WATER_PUMP;

    case MACHINE_COMBO_STATE_WAIT_SWITCH_TANK:
        return COMMUNICATION_STEP_PREPARE_WAIT_CONFIRM;

    case MACHINE_COMBO_STATE_WAIT_RAW_ACTIVITY:
        return (machine_combo_owner == MACHINE_FLOW_OWNER_REMOTE) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_MEASURING;

    case MACHINE_COMBO_STATE_WAIT_FINAL_ACTIVITY:
        return COMMUNICATION_STEP_PREPARE_MEASURING;

    case MACHINE_COMBO_STATE_TANK_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_TANK_HOME:
        return (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_CANISTER_IN;

    case MACHINE_COMBO_STATE_NEEDLE_HOME:
    case MACHINE_COMBO_STATE_GAP_AFTER_NEEDLE_HOME:
        return (machine_combo_remote_process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY) ?
               COMMUNICATION_STEP_TRANSFER_TO_ACTIVITY :
               COMMUNICATION_STEP_PREPARE_NEEDLE_IN;

    case MACHINE_COMBO_STATE_EXHAUST_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST_VALVE:
    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_EXHAUST_ENABLE:
    case MACHINE_COMBO_STATE_PUMP2_EXHAUST_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_EXHAUST:
        return COMMUNICATION_STEP_EXHAUST_RUNNING;

    case MACHINE_COMBO_STATE_FLUSH_VALVE_ON:
    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH_VALVE:
    case MACHINE_COMBO_STATE_PUMP1_FLUSH_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP1_FLUSH_ENABLE:
        return COMMUNICATION_STEP_FLUSH_START;

    case MACHINE_COMBO_STATE_PUMP1_FLUSH_IN:
        return COMMUNICATION_STEP_FLUSH_RUNNING;

    case MACHINE_COMBO_STATE_GAP_AFTER_FLUSH:
        return COMMUNICATION_STEP_FLUSH_DONE;

    case MACHINE_COMBO_STATE_WAIT_DISPENSE:
        return COMMUNICATION_STEP_PREPARE_DONE;

    case MACHINE_COMBO_STATE_PUMP2_ENABLE:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2_ENABLE:
        return COMMUNICATION_STEP_DISPENSE_START;

    case MACHINE_COMBO_STATE_PUMP2_DISPENSE_IN:
    case MACHINE_COMBO_STATE_GAP_AFTER_PUMP2:
        return COMMUNICATION_STEP_DISPENSE_RUNNING;

    case MACHINE_COMBO_STATE_FINISHED:
        if (machine_combo_dispense_ul != 0U)
        {
            return COMMUNICATION_STEP_DISPENSE_DONE;
        }
        if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_EXHAUST_ONLY)
        {
            return COMMUNICATION_STEP_EXHAUST_DONE;
        }
        if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_EMPTY_ONLY)
        {
            return COMMUNICATION_STEP_FLUSH_DONE;
        }
        if (machine_combo_utility_mode == MACHINE_COMBO_UTILITY_RESET_RECOVERY)
        {
            return COMMUNICATION_STEP_FLUSH_DONE;
        }
        return COMMUNICATION_STEP_PREPARE_DONE;

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
    return (((machine_combo_running != 0U) &&
             (machine_combo_state != MACHINE_COMBO_STATE_WAIT_REMOTE_PREPARE)) ||
            (machine_direct_dispense_running != 0U) ||
            (machine_motor_reset_running != 0U)) ? 1U : 0U;
}
