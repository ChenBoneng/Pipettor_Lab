//
// Created by lenovo on 26-7-30.
//

#include "MachineCMD.h"
#include <stdio.h>
#include <string.h>
#include "bsp_dwt.h"
#include "display_lcd.h"
#include "Keyboard.h"
#include "MachineCMD_Text.h"
#include "machine.h"
#include "Communication.h"
#include "activity_meter.h"
#include "pump_drive.h"
#include "solenoid_valve.h"
#include "step_motor.h"
#include "water_pump.h"

#define MACHINE_CMD_LCD_LINE_BYTES        16U
#define MACHINE_CMD_INPUT_MAX_LEN         5U
#define MACHINE_CMD_BOOT_HOLD_MS          2000U
#define MACHINE_CMD_MEASURE_HOLD_MS       15000U
#define MACHINE_CMD_DISPENSE_DONE_HOLD_MS 3000U
#define MACHINE_CMD_REMOTE_DIR_REVERSE    0U
#define MACHINE_CMD_REMOTE_DIR_FORWARD    1U
#define MACHINE_CMD_REMOTE_PUMP_STOP      0U
#define MACHINE_CMD_REMOTE_PUMP_IN        1U
#define MACHINE_CMD_REMOTE_PUMP_OUT       2U
/** 上位机泵控制未携带体积时，默认按 10 圈点动；角度单位为 0.1 度。 */
#define MACHINE_CMD_REMOTE_PUMP_DEFAULT_ANGLE_DEG_X10 36000U
#define MACHINE_CMD_REMOTE_MOTOR_A_BUSY   (1U << 0)
#define MACHINE_CMD_REMOTE_MOTOR_B_BUSY   (1U << 1)
#define MACHINE_CMD_REMOTE_PUMP1_BUSY     (1U << 2)
#define MACHINE_CMD_REMOTE_PUMP2_BUSY     (1U << 3)
#define MACHINE_CMD_REMOTE_WATER_VALVE_ON (1U << 0)
#define MACHINE_CMD_REMOTE_MED_VALVE_ON   (1U << 1)
#define MACHINE_CMD_REMOTE_WATER_PUMP_ON  (1U << 2)
#define MACHINE_CMD_ACTIVITY_PUSH_SEQ      0U

typedef enum
{
    MACHINE_CMD_PREP_FOCUS_CURRENT_CONC = 0, // 配药设置页：当前输入现有活度浓度
    MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME,   // 配药设置页：当前输入现有溶液体积
    MACHINE_CMD_PREP_FOCUS_TARGET_CONC,      // 配药设置页：当前输入目标活度浓度
} MachineCmdPrepFocus_e;

typedef enum
{
    MACHINE_CMD_MANUAL_ACTION_IDLE = 0,       // 空闲
    MACHINE_CMD_MANUAL_ACTION_IN_TANK,        // 进罐导轨
    MACHINE_CMD_MANUAL_ACTION_OUT_TANK,       // 出罐导轨
    MACHINE_CMD_MANUAL_ACTION_NEEDLE_IN,      // 插针
    MACHINE_CMD_MANUAL_ACTION_NEEDLE_OUT,     // 收针
    MACHINE_CMD_MANUAL_ACTION_DRAW_MED,       // 抽药
    MACHINE_CMD_MANUAL_ACTION_EXHAUST,        // 定量排气
    MACHINE_CMD_MANUAL_ACTION_REMOTE,         // 上位机远控
    MACHINE_CMD_MANUAL_ACTION_WATER_IN,       // 水进
    MACHINE_CMD_MANUAL_ACTION_MED_IN,         // 药进
    MACHINE_CMD_MANUAL_ACTION_WATER_OUT,      // 水出
    MACHINE_CMD_MANUAL_ACTION_MED_OUT,        // 药出
} MachineCmdManualAction_e;

typedef struct
{
    MachineCmdPage_e page;                              // 当前 LCD 页面
    uint32_t boot_start_ms;                             // 开机页进入时间
    uint8_t manual_switches;                            // 手动调试开关位
    uint32_t measure_start_ms;                           // 活度计测量页进入时间
    uint32_t dispense_done_start_ms;                     // 发药完成页开始保持 100% 的时间
    uint8_t dispense_done_holding;                       // 发药完成后正在保持 100% 页面
    MachineCmdPage_e paused_page;                        // 暂停前所在运行页
    uint16_t prep_current_conc_x1000;                    // 当前活度浓度，单位 0.001mCi/ml
    uint16_t prep_current_ml_x100;                       // 当前溶液体积，单位 0.01ml
    uint16_t prep_target_conc_x1000;                     // 目标活度浓度，单位 0.001mCi/ml
    uint16_t dispense_ml_x100;                          // 发药体积，单位 0.01ml
    uint16_t left_ml_x100;                               // 当前可发药余量，单位 0.01ml
    uint16_t standby_conc_x1000;                         // 待机页当前浓度，单位 0.001mCi/ml
    uint16_t standby_activity_x100;                      // 待机页活度计实时活度，单位 0.01mCi
    uint16_t standby_volume_ml_x100;                     // 待机页当前剩余体积，单位 0.01ml
    uint8_t prep_confirmed;                              // 配药量确认事件，machine 层读取后清零
    uint8_t dispense_confirmed;                          // 发药量确认事件，machine 层读取后清零
    uint8_t dispense_input_error;                         // 发药输入超余量提示标志
    uint8_t reset_requested;                              // 复位键事件，machine 层读取后终止当前流程
    uint8_t remote_enabled;                               // 上位机远控接管标志
    uint8_t remote_paused;                                // 远控暂停标志，本地暂停键置位
    uint16_t remote_prepare_initial_activity_x100;         // 远程配药初始活度，单位 0.01mCi
    uint16_t remote_prepare_target_conc_x1000;             // 远程配药目标浓度，单位 0.001mCi/ml
    uint16_t remote_prepare_water_volume_x100;             // 远程配药补水量，单位 0.01ml
    uint16_t remote_prepare_final_volume_x100;             // 远程配药理论最终体积，单位 0.01ml
    uint16_t remote_dispense_volume_x100;                  // 远程发药体积，单位 0.01ml
    uint16_t remote_dispense_target_activity_x100;         // 远程发药目标活度，单位 0.01mCi
    uint8_t remote_prepare_param_ready;                    // 已收到 PREPARE_PARAM
    uint8_t remote_prepare_volume_ready;                   // 已收到 PREPARE_VOLUME_PARAM
    uint8_t remote_dispense_param_ready;                   // 已收到 DISPENSE_PARAM
    uint8_t remote_transfer_activity_reported;             // 转移完成后的活度主动上报已经发送
    uint32_t remote_status_last_ms;                         // 0x181 周期状态上报时间
    uint8_t activity_request_pending;                       // READ_ACTIVITY 异步查询正在等待最终读数
    uint8_t activity_request_started;                       // 本次查询已经触发或接上活度计读取
    uint8_t activity_request_seq;                           // 待补发活度结果的上位机 SEQ
    uint32_t activity_request_update_count;                 // 查询开始时的活度计成功解析计数
    uint32_t activity_reported_update_count;                // 已主动上报给上位机的最新解析计数
    MachineCmdPrepFocus_e prep_focus;                     // 配药设置页当前输入焦点
    char input[MACHINE_CMD_INPUT_MAX_LEN + 1U];          // 当前输入缓冲区
    MachineCmdManualAction_e manual_action;              // 手动页当前动作说明
} MachineCmdContext_s;

static MachineCmdContext_s machine_cmd = {0};

static uint32_t MachineCMD_GetMs(void);
static void MachineCMD_EnterPage(MachineCmdPage_e page);
static void MachineCMD_TryEnterDispSettingPage(uint8_t allow_direct_dispense);
static void MachineCMD_ClearInput(void);
static uint8_t MachineCMD_KeyToDigit(KeypadState_e key, uint8_t *digit);
static uint8_t MachineCMD_IsNumberKey(KeypadState_e key);
static void MachineCMD_AppendDigit(uint8_t digit);
static void MachineCMD_AppendDot(void);
static uint16_t MachineCMD_InputToScaledValue(uint16_t scale);
static uint16_t MachineCMD_InputToMlX100(void);
static uint16_t MachineCMD_InputToConcX1000(void);
static uint16_t MachineCMD_CalcPreparedLeftMlX100(uint16_t current_conc_x1000,
                                                  uint16_t current_ml_x100,
                                                  uint16_t target_conc_x1000);
static uint16_t MachineCMD_ActivityToMciX100(const ActivityMeterData_s *activity_data);
static void MachineCMD_UpdateStandbyActivityFromMeter(void);
static void MachineCMD_FormatFixed1Ascii(char *buffer, uint8_t size, uint32_t value_x10);
static void MachineCMD_FormatMlX100Ascii(char *buffer, uint8_t size, uint16_t value_x100);
static void MachineCMD_FormatConcX1000Ascii(char *buffer, uint8_t size, uint16_t value_x1000);
static void MachineCMD_HandleStandbyKey(KeypadState_e key);
static void MachineCMD_HandlePrepSettingKey(KeypadState_e key);
static void MachineCMD_HandlePrepRunningKey(KeypadState_e key);
static void MachineCMD_HandlePrepMeasureKey(KeypadState_e key);
static void MachineCMD_HandleDispSettingKey(KeypadState_e key);
static void MachineCMD_HandleDispRunningKey(KeypadState_e key);
static void MachineCMD_HandleRemoteKey(KeypadState_e key);
static void MachineCMD_HandleManualKey(KeypadState_e key);
static void MachineCMD_HandlePausedKey(KeypadState_e key);
static void MachineCMD_SetManualAction(KeypadState_e key);
static void MachineCMD_ToggleManualSwitch(uint8_t switch_mask);
static void MachineCMD_ClearManualSwitches(void);
static void MachineCMD_ApplyManualOutputs(void);
static void MachineCMD_PauseCurrentFlow(void);
static void MachineCMD_EnterRemoteMode(void);
static void MachineCMD_StopRemoteOutputs(void);
static void MachineCMD_PauseRemoteMode(void);
static void MachineCMD_SyncRemoteState(void);
static void MachineCMD_ApplyCommunicationSafetyAction(void);
static void MachineCMD_ProcessRemoteCommand(void);
static void MachineCMD_ExecuteRemoteCommand(const CommunicationHostCommand_s *command);
static uint8_t MachineCMD_SendActivityData(const ActivityMeterData_s *activity_data, uint8_t seq);
static uint8_t MachineCMD_SendActivityState(const ActivityMeterData_s *activity_data,
                                            uint8_t state,
                                            uint8_t seq);
static void MachineCMD_HandleRemoteActivityRead(uint8_t seq);
static void MachineCMD_CompleteActivityRequest(void);
static void MachineCMD_ReportActivityUpdate(void);
static void MachineCMD_ReportTransferActivityIfReady(void);
static void MachineCMD_HandleRemoteSetParam(const CommunicationHostCommand_s *command);
static void MachineCMD_HandleRemoteStartProcess(const CommunicationHostCommand_s *command);
static uint8_t MachineCMD_HandleRemoteStepper(const CommunicationHostCommand_s *command);
static uint8_t MachineCMD_HandleRemoteValve(const CommunicationHostCommand_s *command);
static uint8_t MachineCMD_HandleRemotePump(const CommunicationHostCommand_s *command);
static uint8_t MachineCMD_HandleRemoteStopObject(const CommunicationHostCommand_s *command);
static void MachineCMD_SendRemoteStatus(uint8_t seq);
static uint8_t MachineCMD_LineAppendBytes(uint8_t *line, uint8_t offset, const uint8_t *data, uint8_t len);
static uint8_t MachineCMD_LineAppendText(uint8_t *line, uint8_t offset, const MachineCmdText_s *text);
static uint8_t MachineCMD_LineAppendString(uint8_t *line, uint8_t offset, const char *text);
static uint8_t MachineCMD_LineAppendSwitchState(uint8_t *line, uint8_t offset, uint8_t switch_mask);
static void MachineCMD_WriteBytes(DisplayLcdRow_e row, const uint8_t *data, uint8_t len);
static void MachineCMD_WriteText(DisplayLcdRow_e row, const MachineCmdText_s *text);
static void MachineCMD_FormatActivityAscii(char *buffer, uint8_t size, float value, const char *unit);
static const MachineCmdText_s *MachineCMD_GetManualActionText(void);
static const MachineCmdText_s *MachineCMD_GetPrepStepTitle(void);
static const MachineCmdText_s *MachineCMD_GetPrepStepLabel(void);
static const MachineCmdText_s *MachineCMD_GetPrepStepHint(void);
static void MachineCMD_FormatPrepFocusValue(char *buffer, uint8_t size);
static void MachineCMD_MovePrepFocusBack(void);
static void MachineCMD_ShowBootPage(void);
static void MachineCMD_ShowReadyPage(void);
static void MachineCMD_ShowStandbyPage(void);
static void MachineCMD_ShowPrepSettingPage(void);
static void MachineCMD_ShowPrepRunningPage(void);
static void MachineCMD_ShowPrepMeasurePage(void);
static void MachineCMD_ShowDispSettingPage(void);
static void MachineCMD_ShowDispRunningPage(void);
static void MachineCMD_ShowRemotePage(void);
static void MachineCMD_ShowManualPage(void);
static void MachineCMD_ShowCleanPage(void);
static void MachineCMD_ShowPausedPage(void);
static void MachineCMD_ShowAlarmPage(void);

/**
 * @brief 初始化 LCD/按键命令层。
 *
 * @note 本函数只初始化界面状态和手动调试输出状态，不负责初始化 LCD、
 *       键盘扫描或底层电机驱动。底层模块仍然由 AllTaskInit() 统一初始化。
 */
void MachineCMD_Init(void)
{
    memset(&machine_cmd, 0, sizeof(machine_cmd));

    machine_cmd.page = MACHINE_CMD_PAGE_BOOT;
    machine_cmd.boot_start_ms = MachineCMD_GetMs();
    machine_cmd.left_ml_x100 = 0U;
    machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_CONC;
    machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_REMOTE;
    machine_cmd.remote_enabled = 0U;
    machine_cmd.remote_paused = 0U;
    MachineCMD_ClearManualSwitches();
}

/**
 * @brief 处理一次按键事件和页面状态跳转。
 *
 * @note 本函数不阻塞等待按键，只读取 Keypad_GetPressedState() 产生的一次性
 *       稳定按下事件。数字键在配药/发药设置页作为数值输入，在手动调试页
 *       作为水进、药进、水出、药出四个独立开关。
 */
void MachineCMD_Process(void)
{
    KeypadState_e key;
    uint32_t now_ms;

    now_ms = MachineCMD_GetMs();
    MachineCMD_SyncRemoteState();
    MachineCMD_ApplyCommunicationSafetyAction();
    MachineCMD_ReportTransferActivityIfReady();

    if ((uint32_t)(now_ms - machine_cmd.remote_status_last_ms) >=
        COMMUNICATION_STATUS_PERIOD_MS)
    {
        machine_cmd.remote_status_last_ms = now_ms;
        MachineCMD_SendRemoteStatus(0U);
    }

    /*
     * 开机页停留 2 秒，用于让用户看到系统已经进入初始化。
     * 当前设备默认交给上位机控制，所以启动结束后直接进入远控页。
     */
    if ((machine_cmd.page == MACHINE_CMD_PAGE_BOOT) &&
        ((now_ms - machine_cmd.boot_start_ms) >= MACHINE_CMD_BOOT_HOLD_MS))
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
    }

    if ((machine_cmd.page == MACHINE_CMD_PAGE_PREP_MEASURE) &&
        ((now_ms - machine_cmd.measure_start_ms) >= MACHINE_CMD_MEASURE_HOLD_MS))
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_STANDBY);
    }

    if ((machine_cmd.page == MACHINE_CMD_PAGE_PREP_RUNNING) &&
        (Machine_ConsumeCombinationFinalActivityReady() != 0U))
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_PREP_MEASURE);
    }

    if (Machine_ConsumeLocalDispenseCompleted() != 0U)
    {
        if (machine_cmd.page == MACHINE_CMD_PAGE_DISP_RUNNING)
        {
            machine_cmd.dispense_done_start_ms = now_ms;
            machine_cmd.dispense_done_holding = 1U;
        }
    }

    if ((machine_cmd.page == MACHINE_CMD_PAGE_DISP_RUNNING) &&
        (machine_cmd.dispense_done_holding != 0U) &&
        ((now_ms - machine_cmd.dispense_done_start_ms) >= MACHINE_CMD_DISPENSE_DONE_HOLD_MS))
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_STANDBY);
    }

    key = Keypad_GetPressedState();
    if (key == KEYPAD_STATE_NONE)
    {
        MachineCMD_ProcessRemoteCommand();
        MachineCMD_CompleteActivityRequest();
        MachineCMD_ReportActivityUpdate();
        MachineCMD_SyncRemoteState();
        return;
    }

    /*
     * 启动就绪页是旧本机优先流程的保留页，当前默认不会主动进入。
     * 如果后续从其它逻辑切到本页，仍按一次任意键回到本机待机页。
     */
    if (machine_cmd.page == MACHINE_CMD_PAGE_READY)
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_STANDBY);
        return;
    }

    /*
     * 复位键作为当前 UI 层的统一退出键：
     * - 清空当前数字输入；
     * - 关闭手动调试输出；
     * - 回到待机看板。
     */
    if (key == KEYPAD_STATE_RESET)
    {
        uint8_t mode = Communication_GetControlMode();

        if ((mode == COMMUNICATION_CONTROL_REMOTE) ||
            (mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING))
        {
            /*
             * 新版控制调度要求 REMOTE 下复位键不能直接抢回本地控制权。
             * 用户必须先按暂停键让输出进入安全暂停，再按复位完成本地接管。
             */
            MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
            MachineCMD_SyncRemoteState();
            return;
        }

        /*
         * REMOTE_PAUSED 下复位表示本地接管；LOCAL 下复位保持本地控制。
         * REMOTE 下直接复位已在上面拦截，避免绕过“先暂停、再接管”的安全步骤。
         */
        Communication_OnLocalResetKey();
        MachineCMD_ClearInput();
        MachineCMD_ClearManualSwitches();
        MachineCMD_StopRemoteOutputs();
        machine_cmd.prep_confirmed = 0U;
        machine_cmd.dispense_confirmed = 0U;
        machine_cmd.dispense_input_error = 0U;
        machine_cmd.reset_requested = 1U;
        machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_CONC;
        machine_cmd.remote_enabled = 0U;
        machine_cmd.remote_paused = 0U;
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_STANDBY);
        MachineCMD_SyncRemoteState();
        return;
    }

    switch (machine_cmd.page)
    {
    case MACHINE_CMD_PAGE_STANDBY:
        MachineCMD_HandleStandbyKey(key);
        break;

    case MACHINE_CMD_PAGE_PREP_SETTING:
        MachineCMD_HandlePrepSettingKey(key);
        break;

    case MACHINE_CMD_PAGE_PREP_RUNNING:
        MachineCMD_HandlePrepRunningKey(key);
        break;

    case MACHINE_CMD_PAGE_PREP_MEASURE:
        MachineCMD_HandlePrepMeasureKey(key);
        break;

    case MACHINE_CMD_PAGE_DISP_SETTING:
        MachineCMD_HandleDispSettingKey(key);
        break;

    case MACHINE_CMD_PAGE_DISP_RUNNING:
        MachineCMD_HandleDispRunningKey(key);
        break;

    case MACHINE_CMD_PAGE_REMOTE:
        MachineCMD_HandleRemoteKey(key);
        break;

    case MACHINE_CMD_PAGE_MANUAL:
        MachineCMD_HandleManualKey(key);
        break;

    case MACHINE_CMD_PAGE_PAUSED:
        MachineCMD_HandlePausedKey(key);
        break;

    case MACHINE_CMD_PAGE_CLEAN:
    case MACHINE_CMD_PAGE_ALARM:
    case MACHINE_CMD_PAGE_READY:
    case MACHINE_CMD_PAGE_BOOT:
    default:
        break;
    }
}

/**
 * @brief 按当前页面刷新 LCD 四行文本。
 *
 * @note DisplayLcd_UpdateLine() 内部已有行缓存，相同内容不会重复写 LCD。
 *       因此 LCDTask 可以周期调用本函数，不需要在页面切换时反复清屏。
 */
void MachineCMD_LCDTask(void)
{
    switch (machine_cmd.page)
    {
    case MACHINE_CMD_PAGE_BOOT:
        MachineCMD_ShowBootPage();
        break;

    case MACHINE_CMD_PAGE_READY:
        MachineCMD_ShowReadyPage();
        break;

    case MACHINE_CMD_PAGE_STANDBY:
        MachineCMD_ShowStandbyPage();
        break;

    case MACHINE_CMD_PAGE_PREP_SETTING:
        MachineCMD_ShowPrepSettingPage();
        break;

    case MACHINE_CMD_PAGE_PREP_RUNNING:
        MachineCMD_ShowPrepRunningPage();
        break;

    case MACHINE_CMD_PAGE_PREP_MEASURE:
        MachineCMD_ShowPrepMeasurePage();
        break;

    case MACHINE_CMD_PAGE_DISP_SETTING:
        MachineCMD_ShowDispSettingPage();
        break;

    case MACHINE_CMD_PAGE_DISP_RUNNING:
        MachineCMD_ShowDispRunningPage();
        break;

    case MACHINE_CMD_PAGE_REMOTE:
        MachineCMD_ShowRemotePage();
        break;

    case MACHINE_CMD_PAGE_MANUAL:
        MachineCMD_ShowManualPage();
        break;

    case MACHINE_CMD_PAGE_CLEAN:
        MachineCMD_ShowCleanPage();
        break;

    case MACHINE_CMD_PAGE_PAUSED:
        MachineCMD_ShowPausedPage();
        break;

    case MACHINE_CMD_PAGE_ALARM:
    default:
        MachineCMD_ShowAlarmPage();
        break;
    }
}

/**
 * @brief 获取当前 LCD 页面。
 *
 * @return 当前 MachineCMD 页面状态。
 */
MachineCmdPage_e MachineCMD_GetPage(void)
{
    return machine_cmd.page;
}

/**
 * @brief 获取当前手动调试开关位。
 *
 * @return bit0=水进，bit1=药进，bit2=水出，bit3=药出。
 */
uint8_t MachineCMD_GetManualSwitches(void)
{
    return machine_cmd.manual_switches;
}

/**
 * @brief 判断当前是否已经交给上位机远控。
 *
 * @return 1 表示远控接管中；0 表示本机普通 UI 控制。
 */
uint8_t MachineCMD_IsRemoteMode(void)
{
    return Communication_IsRemoteControlActive();
}

/**
 * @brief 读取并清除复位键事件。
 *
 * @return 1 表示本次读到了复位事件；0 表示没有新事件。
 *
 * UI 层按下复位后会回到待机页；machine 层需要独立消费这个事件，
 * 用来区分“用户复位终止流程”和“普通页面切换”。
 */
uint8_t MachineCMD_ConsumeResetRequested(void)
{
    if (machine_cmd.reset_requested == 0U)
    {
        return 0U;
    }

    machine_cmd.reset_requested = 0U;
    return 1U;
}

/**
 * @brief 读取并清除配药参数确认事件。
 *
 * @param current_conc_x1000 输出当前活度浓度，单位 0.001mCi/ml，可为 NULL。
 * @param current_ml_x100 输出当前溶液体积，单位 0.01ml，可为 NULL。
 * @param target_conc_x1000 输出目标活度浓度，单位 0.001mCi/ml，可为 NULL。
 * @return 1 表示本次读到了新的确认事件；0 表示没有新事件。
 */
uint8_t MachineCMD_ConsumePrepConfirmed(uint16_t *current_conc_x1000,
                                        uint16_t *current_ml_x100,
                                        uint16_t *target_conc_x1000)
{
    if (machine_cmd.prep_confirmed == 0U)
    {
        return 0U;
    }

    if (current_conc_x1000 != NULL)
    {
        *current_conc_x1000 = machine_cmd.prep_current_conc_x1000;
    }

    if (current_ml_x100 != NULL)
    {
        *current_ml_x100 = machine_cmd.prep_current_ml_x100;
    }

    if (target_conc_x1000 != NULL)
    {
        *target_conc_x1000 = machine_cmd.prep_target_conc_x1000;
    }

    machine_cmd.prep_confirmed = 0U;
    return 1U;
}

/**
 * @brief 读取并清除发药量确认事件。
 *
 * @param volume_ml_x100 输出发药量，单位 0.01ml，可为 NULL。
 * @return 1 表示本次读到了新的确认事件；0 表示没有新事件。
 */
uint8_t MachineCMD_ConsumeDispenseConfirmed(uint16_t *volume_ml_x100)
{
    if (machine_cmd.dispense_confirmed == 0U)
    {
        return 0U;
    }

    if (volume_ml_x100 != NULL)
    {
        *volume_ml_x100 = machine_cmd.dispense_ml_x100;
    }

    machine_cmd.dispense_confirmed = 0U;
    return 1U;
}

void MachineCMD_SetStandbyInventory(uint16_t conc_x1000,
                                    uint16_t activity_x100,
                                    uint16_t volume_ml_x100)
{
    machine_cmd.standby_conc_x1000 = conc_x1000;
    machine_cmd.standby_activity_x100 = activity_x100;
    machine_cmd.standby_volume_ml_x100 = volume_ml_x100;

    /* 发药设置页的超余量判断仍然按“剩余可发体积”拦截。 */
    machine_cmd.left_ml_x100 = volume_ml_x100;
}

void MachineCMD_ConsumeStandbyInventory(uint16_t volume_ml_x100)
{
    uint16_t next_volume_ml_x100;
    if ((volume_ml_x100 == 0U) || (machine_cmd.standby_volume_ml_x100 == 0U))
    {
        return;
    }

    if (volume_ml_x100 >= machine_cmd.standby_volume_ml_x100)
    {
        MachineCMD_SetStandbyInventory(0U, 0U, 0U);
        return;
    }

    next_volume_ml_x100 = machine_cmd.standby_volume_ml_x100 - volume_ml_x100;
    MachineCMD_SetStandbyInventory(machine_cmd.standby_conc_x1000,
                                   machine_cmd.standby_activity_x100,
                                   next_volume_ml_x100);
}

/**
 * @brief 获取 DWT 毫秒时间轴。
 *
 * @return 从 DWT_Init() 后累计的毫秒数。
 */
static uint32_t MachineCMD_GetMs(void)
{
    return (uint32_t)(DWT_GetTimeline_us() / 1000ULL);
}

/**
 * @brief 进入指定 LCD 页面。
 *
 * @param page 目标页面。
 *
 * @note 进入参数输入页时自动清空输入缓存，避免上一次输入残留到新页面。
 */
static void MachineCMD_EnterPage(MachineCmdPage_e page)
{
    machine_cmd.page = page;

    if ((page == MACHINE_CMD_PAGE_PREP_SETTING) ||
        (page == MACHINE_CMD_PAGE_DISP_SETTING))
    {
        MachineCMD_ClearInput();
    }

    if (page == MACHINE_CMD_PAGE_PREP_SETTING)
    {
        /* 新一轮配药开始前丢弃上一次可能残留的测量完成事件。 */
        (void)Machine_ConsumeCombinationFinalActivityReady();
        machine_cmd.prep_current_conc_x1000 = 0U;
        machine_cmd.prep_current_ml_x100 = 0U;
        machine_cmd.prep_target_conc_x1000 = 0U;
        machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_CONC;
        machine_cmd.prep_confirmed = 0U;
    }

    if (page == MACHINE_CMD_PAGE_DISP_SETTING)
    {
        machine_cmd.dispense_confirmed = 0U;
        machine_cmd.dispense_input_error = 0U;
    }

    if (page == MACHINE_CMD_PAGE_PREP_MEASURE)
    {
        machine_cmd.measure_start_ms = MachineCMD_GetMs();
    }

    if (page == MACHINE_CMD_PAGE_DISP_RUNNING)
    {
        /* 新一次发药开始前丢弃上一次可能残留的完成事件。 */
        (void)Machine_ConsumeLocalDispenseCompleted();
        machine_cmd.dispense_done_start_ms = 0U;
        machine_cmd.dispense_done_holding = 0U;
    }
    else
    {
        machine_cmd.dispense_done_start_ms = 0U;
        machine_cmd.dispense_done_holding = 0U;
    }
}

/**
 * @brief 在允许发药时进入发药参数页。
 *
 * @param allow_direct_dispense 是否允许待机页直接进入独立发药流程。
 *
 * @note 待机页允许直接发药，用于单独控制泵2。
 *       配药运行中仍必须等 machine 组合流程走到 WAIT_DISPENSE，避免配药未完成时误发药。
 */
static void MachineCMD_TryEnterDispSettingPage(uint8_t allow_direct_dispense)
{
    if (MachineCombinationTestCanDispense() != 0U)
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_DISP_SETTING);
        return;
    }

    if ((allow_direct_dispense == 0U) ||
        (MachineCombinationTestIsRunning() != 0U))
    {
        return;
    }

    MachineCMD_EnterPage(MACHINE_CMD_PAGE_DISP_SETTING);
}

/**
 * @brief 清空当前数值输入缓存。
 */
static void MachineCMD_ClearInput(void)
{
    memset(machine_cmd.input, 0, sizeof(machine_cmd.input));
}

/**
 * @brief 把键盘功能枚举转换成数字值。
 *
 * @param key 当前按键功能枚举。
 * @param digit 输出数字，范围 0~9。
 * @return 1 表示该按键是数字键；0 表示不是数字键或参数错误。
 */
static uint8_t MachineCMD_KeyToDigit(KeypadState_e key, uint8_t *digit)
{
    if (digit == NULL)
    {
        return 0U;
    }

    switch (key)
    {
    case KEYPAD_STATE_NUM_0:
        *digit = 0U;
        return 1U;
    case KEYPAD_STATE_NUM_1:
        *digit = 1U;
        return 1U;
    case KEYPAD_STATE_NUM_2:
        *digit = 2U;
        return 1U;
    case KEYPAD_STATE_NUM_3:
        *digit = 3U;
        return 1U;
    case KEYPAD_STATE_NUM_4:
        *digit = 4U;
        return 1U;
    case KEYPAD_STATE_NUM_5:
        *digit = 5U;
        return 1U;
    case KEYPAD_STATE_NUM_6:
        *digit = 6U;
        return 1U;
    case KEYPAD_STATE_NUM_7:
        *digit = 7U;
        return 1U;
    case KEYPAD_STATE_NUM_8:
        *digit = 8U;
        return 1U;
    case KEYPAD_STATE_NUM_9:
        *digit = 9U;
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 判断按键是否属于数字键。
 *
 * @param key 当前按键功能枚举。
 * @return 1 表示数字键；0 表示非数字键。
 */
static uint8_t MachineCMD_IsNumberKey(KeypadState_e key)
{
    uint8_t digit;

    return MachineCMD_KeyToDigit(key, &digit);
}

/**
 * @brief 向当前输入缓存追加一个数字字符。
 *
 * @param digit 要追加的数字，范围 0~9。
 *
 * @note 输入缓存长度限制为 MACHINE_CMD_INPUT_MAX_LEN，超过后直接忽略。
 */
static void MachineCMD_AppendDigit(uint8_t digit)
{
    uint32_t len = strlen(machine_cmd.input);

    if ((digit > 9U) || (len >= MACHINE_CMD_INPUT_MAX_LEN))
    {
        return;
    }

    machine_cmd.input[len] = (char)('0' + digit);
    machine_cmd.input[len + 1U] = '\0';
}

/**
 * @brief 向当前输入缓存追加小数点。
 *
 * @note 为了保持输入简单，小数点不能作为第一个字符，也不能重复输入。
 */
static void MachineCMD_AppendDot(void)
{
    uint32_t len = strlen(machine_cmd.input);

    if ((len >= MACHINE_CMD_INPUT_MAX_LEN) ||
        (strchr(machine_cmd.input, '.') != NULL))
    {
        return;
    }

    if (len == 0U)
    {
        if (MACHINE_CMD_INPUT_MAX_LEN < 2U)
        {
            return;
        }

        machine_cmd.input[0] = '0';
        machine_cmd.input[1] = '.';
        machine_cmd.input[2] = '\0';
        return;
    }

    machine_cmd.input[len] = '.';
    machine_cmd.input[len + 1U] = '\0';
}

/**
 * @brief 将当前输入字符串转换成指定倍率的整数值。
 *
 * @param scale 放大倍率。例如 ml 使用 100，浓度使用 1000。
 * @return 输入值乘以 scale 后的整数，超过 uint16 上限时截断。
 *
 * @note 这里不用 atof()，避免在 STM32F103 上引入不必要的浮点格式化依赖。
 *       scale 只按 10 的倍数使用，当前用于 0.01ml 和 0.001mCi/ml。
 */
static uint16_t MachineCMD_InputToScaledValue(uint16_t scale)
{
    uint32_t integer_part = 0U;
    uint32_t decimal_part = 0U;
    uint32_t decimal_scale;
    uint8_t dot_seen = 0U;

    if (scale == 0U)
    {
        return 0U;
    }

    decimal_scale = scale;
    for (uint8_t i = 0U; machine_cmd.input[i] != '\0'; i++)
    {
        char ch = machine_cmd.input[i];

        if (ch == '.')
        {
            dot_seen = 1U;
            continue;
        }

        if ((ch < '0') || (ch > '9'))
        {
            continue;
        }

        if (dot_seen == 0U)
        {
            integer_part = integer_part * 10U + (uint32_t)(ch - '0');
        }
        else if (decimal_scale > 1U)
        {
            decimal_scale /= 10U;
            decimal_part += (uint32_t)(ch - '0') * decimal_scale;
        }
    }

    integer_part = integer_part * (uint32_t)scale + decimal_part;
    if (integer_part > 65535U)
    {
        integer_part = 65535U;
    }

    return (uint16_t)integer_part;
}

/**
 * @brief 将当前输入字符串转换成 0.01ml 单位的整数值。
 */
static uint16_t MachineCMD_InputToMlX100(void)
{
    return MachineCMD_InputToScaledValue(100U);
}

/**
 * @brief 将当前输入字符串转换成 0.001mCi/ml 单位的整数值。
 */
static uint16_t MachineCMD_InputToConcX1000(void)
{
    return MachineCMD_InputToScaledValue(1000U);
}

/**
 * @brief 格式化 1 位小数的 ASCII 数字。
 *
 * @param value_x10 已放大 10 倍的值，例如 1205 表示 120.5。
 */
static void MachineCMD_FormatFixed1Ascii(char *buffer, uint8_t size, uint32_t value_x10)
{
    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }

    (void)snprintf(buffer,
                   size,
                   "%lu.%01lu",
                   (unsigned long)(value_x10 / 10U),
                   (unsigned long)(value_x10 % 10U));
}

/**
 * @brief 把 0.01ml 单位的值格式化成 1 位小数 ASCII。
 */
static void MachineCMD_FormatMlX100Ascii(char *buffer, uint8_t size, uint16_t value_x100)
{
    MachineCMD_FormatFixed1Ascii(buffer, size, ((uint32_t)value_x100 + 5U) / 10U);
}

/**
 * @brief 把 0.001mCi/ml 单位的值格式化成 1 位小数 ASCII。
 */
static void MachineCMD_FormatConcX1000Ascii(char *buffer, uint8_t size, uint16_t value_x1000)
{
    MachineCMD_FormatFixed1Ascii(buffer, size, ((uint32_t)value_x1000 + 50U) / 100U);
}

/**
 * @brief 计算配药完成后可发药余量。
 *
 * @param current_conc_x1000 当前活度浓度，单位 0.001mCi/ml。
 * @param current_ml_x100 当前溶液体积，单位 0.01ml。
 * @param target_conc_x1000 目标活度浓度，单位 0.001mCi/ml。
 * @return 配药完成后的理论总体积，单位 0.01ml。
 *
 * @note 这里和 machine 层补水计算保持同一套稀释公式：
 *       当前总活度不变，目标浓度更低时，最终体积 = 当前浓度 * 当前体积 / 目标浓度。
 *       如果目标浓度大于等于当前浓度，则不补水，余量就是当前体积。
 */
static uint16_t MachineCMD_CalcPreparedLeftMlX100(uint16_t current_conc_x1000,
                                                  uint16_t current_ml_x100,
                                                  uint16_t target_conc_x1000)
{
    uint32_t final_ml_x100;

    if (current_ml_x100 == 0U)
    {
        return 0U;
    }

    if ((current_conc_x1000 == 0U) ||
        (target_conc_x1000 == 0U) ||
        (target_conc_x1000 >= current_conc_x1000))
    {
        return current_ml_x100;
    }

    final_ml_x100 = (((uint32_t)current_conc_x1000 * (uint32_t)current_ml_x100) +
                     (target_conc_x1000 / 2U)) /
                    target_conc_x1000;
    if (final_ml_x100 > 0xFFFFU)
    {
        final_ml_x100 = 0xFFFFU;
    }

    return (uint16_t)final_ml_x100;
}

/**
 * @brief 将 RAM-100 活度读数统一换算为 mCi * 100。
 *
 * @param activity_data 活度计最近一次有效数据。
 * @return 活度，单位 0.01mCi；没有有效读数时返回 0。
 *
 * @note 待机页“活度”只反映活度计实时读数，不再由目标浓度和体积反算。
 */
static uint16_t MachineCMD_ActivityToMciX100(const ActivityMeterData_s *activity_data)
{
    float activity_mci;

    if ((activity_data == NULL) ||
        (activity_data->state != ACTIVITY_METER_STATE_OK) ||
        (activity_data->update_count == 0U))
    {
        return 0U;
    }

    switch (activity_data->activity_unit)
    {
    case ACTIVITY_METER_UNIT_UCI:
        activity_mci = activity_data->activity / 1000.0f;
        break;

    case ACTIVITY_METER_UNIT_MCI:
        activity_mci = activity_data->activity;
        break;

    case ACTIVITY_METER_UNIT_CI:
        activity_mci = activity_data->activity * 1000.0f;
        break;

    case ACTIVITY_METER_UNIT_BQ:
        activity_mci = activity_data->activity / 37000000.0f;
        break;

    case ACTIVITY_METER_UNIT_KBQ:
        activity_mci = activity_data->activity / 37000.0f;
        break;

    case ACTIVITY_METER_UNIT_MBQ:
        activity_mci = activity_data->activity / 37.0f;
        break;

    case ACTIVITY_METER_UNIT_GBQ:
        activity_mci = activity_data->activity * 27.027027f;
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
 * @brief 用活度计最新成功读数刷新待机页活度。
 *
 * @note 只在有 OK 读数时覆盖，通信异常时保留上一笔有效活度，避免待机页闪成 0。
 */
static void MachineCMD_UpdateStandbyActivityFromMeter(void)
{
    ActivityMeterData_s activity_data;
    uint16_t activity_x100;

    memset(&activity_data, 0, sizeof(activity_data));
    if (ActivityMeter_GetData(&activity_data) == 0U)
    {
        return;
    }

    if ((activity_data.state == ACTIVITY_METER_STATE_OK) &&
        (activity_data.update_count != 0U))
    {
        activity_x100 = MachineCMD_ActivityToMciX100(&activity_data);
        machine_cmd.standby_activity_x100 = activity_x100;
    }
}

/**
 * @brief 处理待机页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 待机页只负责入口分流：
 *       - 配药键进入配药参数设置；
 *       - 发药键进入发药参数设置；
 *       - 动作类按键进入手动调试页。
 */
static void MachineCMD_HandleStandbyKey(KeypadState_e key)
{
    switch (key)
    {
    case KEYPAD_STATE_PREPARE_MEDICINE:
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_PREP_SETTING);
        break;

    case KEYPAD_STATE_SEND_MEDICINE:
        MachineCMD_TryEnterDispSettingPage(1U);
        break;

    case KEYPAD_STATE_CLEAR_ALL:
        MachineCMD_ClearManualSwitches();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_CLEAN);
        break;

    case KEYPAD_STATE_FAULT:
        MachineCMD_ClearManualSwitches();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_ALARM);
        break;

    case KEYPAD_STATE_REMOTE:
        MachineCMD_EnterRemoteMode();
        break;

    case KEYPAD_STATE_NUM_1:
    case KEYPAD_STATE_NUM_2:
    case KEYPAD_STATE_NUM_4:
    case KEYPAD_STATE_NUM_5:
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_MANUAL);
        MachineCMD_HandleManualKey(key);
        break;

    case KEYPAD_STATE_IN_TANK:
    case KEYPAD_STATE_OUT_TANK:
    case KEYPAD_STATE_INSERT_NEEDLE:
    case KEYPAD_STATE_RETRACT_NEEDLE:
    case KEYPAD_STATE_DRAW_MEDICINE:
    case KEYPAD_STATE_EXHAUST_FIXED:
        MachineCMD_SetManualAction(key);
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_MANUAL);
        break;

    default:
        break;
    }
}

/**
 * @brief 处理配药参数设置页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 配药设置页使用三步向导，而不是把三项参数挤在同一页：
 *       1. 当前活度浓度，单位 0.001mCi/ml；
 *       2. 当前溶液体积，单位 0.01ml；
 *       3. 目标活度浓度，单位 0.001mCi/ml。
 *
 *       这样每页只有一个输入框，启动键语义固定为“确认当前项”。
 *       清除键在输入框为空时退回上一项，用来弥补面板没有方向键的问题。
 */
static void MachineCMD_HandlePrepSettingKey(KeypadState_e key)
{
    uint8_t digit;

    if (MachineCMD_KeyToDigit(key, &digit) != 0U)
    {
        MachineCMD_AppendDigit(digit);
        return;
    }

    if (key == KEYPAD_STATE_DOT)
    {
        MachineCMD_AppendDot();
        return;
    }

    if (key == KEYPAD_STATE_CLEAR_INPUT)
    {
        if (machine_cmd.input[0] != '\0')
        {
            MachineCMD_ClearInput();
            return;
        }

        MachineCMD_MovePrepFocusBack();
        return;
    }

    if (key == KEYPAD_STATE_START)
    {
        if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_CONC)
        {
            if (machine_cmd.input[0] != '\0')
            {
                machine_cmd.prep_current_conc_x1000 = MachineCMD_InputToConcX1000();
            }
            machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME;
            MachineCMD_ClearInput();
            return;
        }

        if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME)
        {
            if (machine_cmd.input[0] != '\0')
            {
                machine_cmd.prep_current_ml_x100 = MachineCMD_InputToMlX100();
            }
            machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_TARGET_CONC;
            MachineCMD_ClearInput();
            return;
        }

        if (machine_cmd.input[0] != '\0')
        {
            machine_cmd.prep_target_conc_x1000 = MachineCMD_InputToConcX1000();
        }
        machine_cmd.left_ml_x100 = MachineCMD_CalcPreparedLeftMlX100(machine_cmd.prep_current_conc_x1000,
                                                                     machine_cmd.prep_current_ml_x100,
                                                                     machine_cmd.prep_target_conc_x1000);
        machine_cmd.prep_confirmed = 1U;
        MachineCMD_ClearInput();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_PREP_RUNNING);
    }
}

/**
 * @brief 处理配药运行提示页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 最终活度等待结束后由页面状态同步逻辑自动进入测量倒计时，
 *       不再要求用户额外按启动键。暂停键仍用于挂起当前流程。
 */
static void MachineCMD_HandlePrepRunningKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_SEND_MEDICINE)
    {
        MachineCMD_TryEnterDispSettingPage(0U);
        return;
    }

    if (key == KEYPAD_STATE_PAUSE)
    {
        MachineCMD_PauseCurrentFlow();
    }
}

/**
 * @brief 处理配药活度测量页按键。
 *
 * @param key 当前一次性按下事件。
 */
static void MachineCMD_HandlePrepMeasureKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_SEND_MEDICINE)
    {
        MachineCMD_TryEnterDispSettingPage(0U);
        return;
    }

    if (key == KEYPAD_STATE_PAUSE)
    {
        MachineCMD_PauseCurrentFlow();
    }
}

/**
 * @brief 处理发药参数设置页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 发药页只有一个目标体积输入框。启动键确认输入后进入发药运行提示页。
 */
static void MachineCMD_HandleDispSettingKey(KeypadState_e key)
{
    uint8_t digit;

    if (MachineCMD_KeyToDigit(key, &digit) != 0U)
    {
        machine_cmd.dispense_input_error = 0U;
        MachineCMD_AppendDigit(digit);
        return;
    }

    if (key == KEYPAD_STATE_DOT)
    {
        machine_cmd.dispense_input_error = 0U;
        MachineCMD_AppendDot();
        return;
    }

    if (key == KEYPAD_STATE_CLEAR_INPUT)
    {
        machine_cmd.dispense_input_error = 0U;
        MachineCMD_ClearInput();
        return;
    }

    if (key == KEYPAD_STATE_START)
    {
        machine_cmd.dispense_ml_x100 = MachineCMD_InputToMlX100();
        if (machine_cmd.dispense_ml_x100 > machine_cmd.left_ml_x100)
        {
            /*
             * 发药目标体积不能超过当前余量。
             * 目前蜂鸣器没有开放模块接口，这里先做 UI 层拦截：清空输入并留在本页。
             */
            machine_cmd.dispense_input_error = 1U;
            machine_cmd.dispense_confirmed = 0U;
            MachineCMD_ClearInput();
            return;
        }

        machine_cmd.dispense_input_error = 0U;
        machine_cmd.dispense_confirmed = 1U;
        MachineCMD_ClearInput();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_DISP_RUNNING);
    }
}

/**
 * @brief 处理发药运行提示页按键。
 *
 * @param key 当前一次性按下事件。
 */
static void MachineCMD_HandleDispRunningKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_PAUSE)
    {
        MachineCMD_PauseCurrentFlow();
    }
}

/**
 * @brief 处理手动调试页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 本页面下数字 1/2/4/5 不再参与数值输入，而是分别翻转水进、药进、
 *       水出、药出四个逻辑开关。其它动作键只更新当前动作提示文本。
 */
static void MachineCMD_HandleManualKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_NUM_1)
    {
        MachineCMD_ToggleManualSwitch(MACHINE_CMD_MANUAL_WATER_IN);
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_WATER_IN;
        return;
    }

    if (key == KEYPAD_STATE_NUM_2)
    {
        MachineCMD_ToggleManualSwitch(MACHINE_CMD_MANUAL_MED_IN);
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_MED_IN;
        return;
    }

    if (key == KEYPAD_STATE_NUM_4)
    {
        MachineCMD_ToggleManualSwitch(MACHINE_CMD_MANUAL_WATER_OUT);
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_WATER_OUT;
        return;
    }

    if (key == KEYPAD_STATE_NUM_5)
    {
        MachineCMD_ToggleManualSwitch(MACHINE_CMD_MANUAL_MED_OUT);
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_MED_OUT;
        return;
    }

    if (MachineCMD_IsNumberKey(key) == 0U)
    {
        MachineCMD_SetManualAction(key);
    }
}

/**
 * @brief 处理流程暂停页按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 启动键只恢复到暂停前的提示页。真实运动继续由 machine 层接管后再补。
 */
static void MachineCMD_HandlePausedKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_START)
    {
        MachineCMD_EnterPage(machine_cmd.paused_page);
    }
}

/**
 * @brief 根据动作键更新手动调试页当前动作说明。
 *
 * @param key 当前动作按键。
 */
static void MachineCMD_SetManualAction(KeypadState_e key)
{
    switch (key)
    {
    case KEYPAD_STATE_IN_TANK:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_IN_TANK;
        break;

    case KEYPAD_STATE_OUT_TANK:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_OUT_TANK;
        break;

    case KEYPAD_STATE_INSERT_NEEDLE:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_NEEDLE_IN;
        break;

    case KEYPAD_STATE_RETRACT_NEEDLE:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_NEEDLE_OUT;
        break;

    case KEYPAD_STATE_DRAW_MEDICINE:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_DRAW_MED;
        break;

    case KEYPAD_STATE_EXHAUST_FIXED:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_EXHAUST;
        break;

    case KEYPAD_STATE_REMOTE:
        machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_REMOTE;
        break;

    default:
        break;
    }
}

/**
 * @brief 翻转一个手动调试逻辑开关。
 *
 * @param switch_mask 要翻转的开关位，见 MACHINE_CMD_MANUAL_xxx。
 *
 * @note 每次开关变化后立即调用 MachineCMD_ApplyManualOutputs() 同步到底层输出模块。
 */
static void MachineCMD_ToggleManualSwitch(uint8_t switch_mask)
{
    machine_cmd.manual_switches ^= switch_mask;
    MachineCMD_ApplyManualOutputs();
}

/**
 * @brief 关闭所有手动调试逻辑开关。
 *
 * @note 复位退出手动页时调用，确保阀门和泵不会保持上一次手动输出状态。
 */
static void MachineCMD_ClearManualSwitches(void)
{
    machine_cmd.manual_switches = 0U;
    SolenoidValve_AllOff();
    WaterPump_StopAll();
}

/**
 * @brief 将手动调试逻辑开关同步到底层阀门和抽水泵。
 *
 * @note LCD 页面和按键处理只关心“水进/药进/水出/药出”，不再直接依赖具体引脚。
 */
static void MachineCMD_ApplyManualOutputs(void)
{
    uint8_t water_valve_on;
    uint8_t med_valve_on;
    uint8_t pump_on;

    /*
     * 三路已确认硬件输出映射：
     * - 水进/水出打开水路阀；
     * - 药进/药出打开药路阀；
     * - 水出/药出打开抽水泵。
     *
     * “进”方向目前只作为阀门开关，不额外打开泵，避免没有方向控制时误动作。
     */
    water_valve_on = ((machine_cmd.manual_switches &
                       (MACHINE_CMD_MANUAL_WATER_IN | MACHINE_CMD_MANUAL_WATER_OUT)) != 0U) ? 1U : 0U;
    med_valve_on = ((machine_cmd.manual_switches &
                     (MACHINE_CMD_MANUAL_MED_IN | MACHINE_CMD_MANUAL_MED_OUT)) != 0U) ? 1U : 0U;
    pump_on = ((machine_cmd.manual_switches &
                (MACHINE_CMD_MANUAL_WATER_OUT | MACHINE_CMD_MANUAL_MED_OUT)) != 0U) ? 1U : 0U;

    (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_WATER,
                                 water_valve_on ?
                                 SOLENOID_VALVE_STATE_ON_NC_OPEN :
                                 SOLENOID_VALVE_STATE_OFF_NO_OPEN);
    (void)SolenoidValve_SetState(SOLENOID_VALVE_ID_MED,
                                 med_valve_on ?
                                 SOLENOID_VALVE_STATE_ON_NC_OPEN :
                                 SOLENOID_VALVE_STATE_OFF_NO_OPEN);
    (void)WaterPump_SetState(WATER_PUMP_ID_MAIN,
                             pump_on ? WATER_PUMP_STATE_ON : WATER_PUMP_STATE_OFF);
}

/**
 * @brief 暂停当前运行提示页。
 *
 * @note 暂停必须先让所有已知执行器停到安全状态，再切换 UI 页面。
 *       这里直接停 DM542、两只电磁阀、抽水泵和两台 ISC1000 定量泵。
 */
static void MachineCMD_PauseCurrentFlow(void)
{
    machine_cmd.paused_page = machine_cmd.page;
    MachineCMD_ClearManualSwitches();
    MachineCMD_StopRemoteOutputs();
    MachineCMD_EnterPage(MACHINE_CMD_PAGE_PAUSED);
}

/**
 * @brief 处理上位机远控接管页的本机按键。
 *
 * @param key 当前一次性按下事件。
 *
 * @note 远控页只保留本机安全相关按键：
 *       - 暂停键：立即停止当前输出，但仍保持远控模式；
 *       - 启动键：仅在本地暂停后恢复接收上位机动作命令；
 *       - 复位键：在 MachineCMD_Process() 的统一复位分支中退出远控模式。
 *       启动键不会自动恢复被暂停前的动作，上位机需要重新下发动作命令。
 *       其它按键不再触发本机流程，避免本机和上位机同时抢控制权。
 */
static void MachineCMD_HandleRemoteKey(KeypadState_e key)
{
    if (key == KEYPAD_STATE_PAUSE)
    {
        MachineCMD_PauseRemoteMode();
    }
    else if (key == KEYPAD_STATE_START)
    {
        Communication_OnLocalStartKey();
        MachineCMD_SyncRemoteState();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
    }
}

/**
 * @brief 从本机待机页进入上位机远控接管模式。
 *
 * @note 进入远控时先清掉本机输入事件和手动输出，再交给上位机。
 *       这样可以避免用户刚才在配药/发药页留下的确认事件继续被 machine 层消费。
 */
static void MachineCMD_EnterRemoteMode(void)
{
    MachineCMD_ClearInput();
    MachineCMD_ClearManualSwitches();
    MachineCMD_StopRemoteOutputs();

    machine_cmd.prep_confirmed = 0U;
    machine_cmd.dispense_confirmed = 0U;
    machine_cmd.dispense_input_error = 0U;
    machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_CONC;
    machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_REMOTE;
    Communication_OnLocalRemoteKey();
    MachineCMD_SyncRemoteState();

    MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
}

/**
 * @brief 停止远控模式能直接管理的所有执行器。
 *
 * @note 本函数只调用各 modules 层公开接口，不接触 BSP 和具体引脚。
 *       这样远控暂停、复位、上位机 STOP_OBJECT 都能复用同一套停机动作。
 */
static void MachineCMD_StopRemoteOutputs(void)
{
    PumpDrive_s *pump;

    StepMotor_StopAll();
    SolenoidValve_AllOff();
    WaterPump_StopAll();

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
 * @brief 暂停上位机远控模式。
 *
 * @note 暂停后仍保持 remote_enabled=1，上位机仍可查询状态或发送停止/复位类命令。
 *       本机复位键才会真正退出远控模式并回到待机页。
 */
static void MachineCMD_PauseRemoteMode(void)
{
    Communication_OnLocalPauseKey();
    MachineCMD_ApplyCommunicationSafetyAction();
    MachineCMD_SyncRemoteState();
    MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
}

/**
 * @brief 根据通信控制权状态刷新 UI 缓存标志。
 */
static void MachineCMD_SyncRemoteState(void)
{
    uint8_t mode = Communication_GetControlMode();

    machine_cmd.remote_enabled = (mode != COMMUNICATION_CONTROL_LOCAL) ? 1U : 0U;
    machine_cmd.remote_paused =
        (mode == COMMUNICATION_CONTROL_REMOTE_PAUSED) ? 1U : 0U;
}

/**
 * @brief 落实通信层要求的安全停机动作。
 */
static void MachineCMD_ApplyCommunicationSafetyAction(void)
{
    uint8_t action;

    do
    {
        action = Communication_ConsumeSafetyAction();
        if ((action == COMMUNICATION_SAFETY_ACTION_PAUSE_REMOTE) ||
            (action == COMMUNICATION_SAFETY_ACTION_TAKEOVER_LOCAL))
        {
            /*
             * 通信层只记录“需要安全停机”的事件，不直接操作电机、阀和泵。
             * 真正停输出放在 MachineCMD，是为了所有执行器仍通过 modules 层公开接口关闭，
             * 避免通信中断、授权超时、本地接管几条路径各自写一套停机代码。
             */
            MachineCMD_StopRemoteOutputs();
        }
    } while (action != COMMUNICATION_SAFETY_ACTION_NONE);
}

/**
 * @brief 周期消费一帧上位机命令。
 *
 * @note Communication 层只负责收帧、授权和缓存最新命令，不直接驱动机械动作。
 *       MachineCMD 作为“命令解释层”，只在远控接管后执行这些命令。
 */
static void MachineCMD_ProcessRemoteCommand(void)
{
    CommunicationHostCommand_s command;

    if (Communication_HasNewCommand() == 0U)
    {
        return;
    }

    if (Communication_GetHostCommand(&command) == 0U)
    {
        Communication_ClearNewCommandFlag();
        return;
    }

    Communication_ClearNewCommandFlag();
    MachineCMD_ExecuteRemoteCommand(&command);
}

/**
 * @brief 根据协议命令分发上位机远控动作。
 *
 * @param command 已通过 Communication 层授权检查的上位机命令。
 *
 * @note 当前只落地底层执行器直控和查询类命令。START_PROCESS/SET_PARAM 这类完整流程命令
 *       后续应交给 machine 主状态机实现，避免 UI 层越权拼业务流程。
 */
static void MachineCMD_ExecuteRemoteCommand(const CommunicationHostCommand_s *command)
{
    if (command == NULL)
    {
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_STOP_PROCESS)
    {
        /*
         * 上位机急停按钮当前按 STOP_PROCESS 进入这里。
         * 本项目暂时不要求 LCD 切到急停页，但物理输出必须立即停止：
         * 步进电机停 PWM，阀和水泵断开，泵1/泵2发送 stp 1 急停命令。
         */
        MachineCMD_StopRemoteOutputs();
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_RESET_ERROR)
    {
        MachineCMD_StopRemoteOutputs();
        Communication_OnLocalStartKey();
        MachineCMD_SyncRemoteState();
        machine_cmd.remote_paused = 0U;
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_REMOTE);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_QUERY_STATUS)
    {
        MachineCMD_SendRemoteStatus(command->seq);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_QUERY_VERSION)
    {
        (void)Communication_SendVersion(1U, 0U, 1U, 0U, 1U, 0U, command->seq);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_READ_ACTIVITY)
    {
        MachineCMD_HandleRemoteActivityRead(command->seq);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_STOP_OBJECT)
    {
        (void)MachineCMD_HandleRemoteStopObject(command);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_SET_PARAM)
    {
        MachineCMD_HandleRemoteSetParam(command);
        return;
    }

    if (command->cmd == COMMUNICATION_CMD_START_PROCESS)
    {
        MachineCMD_HandleRemoteStartProcess(command);
        return;
    }

    /*
     * 远控暂停后只接受停止、复位和查询类命令。
     * 如果继续执行运动/阀/泵命令，本机暂停键就失去了安全意义。
     */
    if (machine_cmd.remote_paused != 0U)
    {
        return;
    }

    switch (command->cmd)
    {
    case COMMUNICATION_CMD_MOVE_STEPPER:
        (void)MachineCMD_HandleRemoteStepper(command);
        break;

    case COMMUNICATION_CMD_VALVE_CONTROL:
        (void)MachineCMD_HandleRemoteValve(command);
        break;

    case COMMUNICATION_CMD_PUMP_CONTROL:
        (void)MachineCMD_HandleRemotePump(command);
        break;

    default:
        break;
    }
}

/**
 * @brief 按协议发送一组完整活度数据帧。
 *
 * @param activity_data 活度计最近一次成功解析的数据。
 * @param seq 上位机查询序号；主动实时上报时固定为 0。
 * @return 1 表示活度值帧和附加信息帧都已进入 CAN 发送队列；0 表示发送失败。
 *
 * @note 0x183 / 07 01 发送 float 活度值和单位，0x183 / 07 02 发送核素、扣本底、
 *       通道和状态。这里特意使用 ActivityMeter_GetNuclideMassNumber()，
 *       因为 RAM-100 的 nuclide_id 是内部表下标，不是协议 isotope 字段。
 */
static uint8_t MachineCMD_SendActivityData(const ActivityMeterData_s *activity_data, uint8_t seq)
{
    if (activity_data == NULL)
    {
        return 0U;
    }

    if (Communication_SendActivity(activity_data->activity,
                                   (uint8_t)activity_data->activity_unit,
                                   seq) == 0U)
    {
        return 0U;
    }

    return Communication_SendActivityInfo(
        ActivityMeter_GetNuclideMassNumber(activity_data->nuclide_id),
        activity_data->background_subtracted,
        activity_data->channel,
        (uint8_t)activity_data->state,
        seq);
}

/**
 * @brief 只发送活度计状态附加信息帧。
 *
 * @param activity_data 最近一次活度缓存，可为 NULL。
 * @param state 本次要上报的活度计通信状态。
 * @param seq 上位机查询序号。
 * @return 1 表示状态帧已进入 CAN 发送队列；0 表示发送失败。
 *
 * @note 当还没有有效读数时，核素、扣本底和通道都填 0；如果之前已经有过成功读数，
 *       即使本次处于 WAITING/TIMEOUT，也保留最近一次元数据，便于上位机界面不闪成未知核素。
 */
static uint8_t MachineCMD_SendActivityState(const ActivityMeterData_s *activity_data,
                                            uint8_t state,
                                            uint8_t seq)
{
    uint16_t isotope = 0U;
    uint8_t background_subtracted = 0U;
    uint8_t channel = 0U;

    if ((activity_data != NULL) && (activity_data->update_count != 0U))
    {
        isotope = ActivityMeter_GetNuclideMassNumber(activity_data->nuclide_id);
        background_subtracted = activity_data->background_subtracted;
        channel = activity_data->channel;
    }

    return Communication_SendActivityInfo(isotope,
                                          background_subtracted,
                                          channel,
                                          state,
                                          seq);
}

/**
 * @brief 处理上位机 READ_ACTIVITY 查询。
 *
 * @param seq 本次查询帧 Byte6 的请求序号。
 *
 * @note 如果当前缓存就是 OK，按文档立即返回 07 01 + 07 02 两帧。
 *       如果当前没有有效读数或正在等待 RAM-100 响应，则先返回一帧 07 02 状态帧，
 *       保存本次 SEQ，等后续 update_count 变化后再用同一个 SEQ 补发最终结果。
 *       当前只保留一个待完成查询，上位机应等本次查询结束后再发下一条 READ_ACTIVITY。
 */
static void MachineCMD_HandleRemoteActivityRead(uint8_t seq)
{
    ActivityMeterData_s activity_data;
    uint8_t state;

    memset(&activity_data, 0, sizeof(activity_data));
    (void)ActivityMeter_GetData(&activity_data);

    if (Machine_IsTransferToActivityDone() == 0U)
    {
        state = (uint8_t)ActivityMeter_GetState();
        if ((state != (uint8_t)ACTIVITY_METER_STATE_TIMEOUT) &&
            (state != (uint8_t)ACTIVITY_METER_STATE_CRC_ERROR) &&
            (state != (uint8_t)ACTIVITY_METER_STATE_BAD_RESPONSE))
        {
            state = COMMUNICATION_ACTIVITY_NOT_READ;
        }

        (void)MachineCMD_SendActivityState(&activity_data,
                                           state,
                                           seq);
        return;
    }

    if (activity_data.state == ACTIVITY_METER_STATE_OK)
    {
        if (MachineCMD_SendActivityData(&activity_data, seq) != 0U)
        {
            machine_cmd.activity_reported_update_count = activity_data.update_count;
        }
        return;
    }

    machine_cmd.activity_request_seq = seq;
    machine_cmd.activity_request_update_count = activity_data.update_count;
    machine_cmd.activity_request_pending = 1U;
    machine_cmd.activity_request_started =
        (ActivityMeter_GetState() == ACTIVITY_METER_STATE_WAITING) ? 1U : 0U;

    if ((machine_cmd.activity_request_started == 0U) &&
        (ActivityMeter_RequestRead() != 0U))
    {
        machine_cmd.activity_request_started = 1U;
    }

    state = (uint8_t)ActivityMeter_GetState();
    (void)MachineCMD_SendActivityState(&activity_data, state, seq);
}

/**
 * @brief 活度计异步查询完成后，使用原 READ_ACTIVITY 序号补发结果。
 *
 * @note MachineCMD_Process() 周期调用本函数。它不阻塞等待 RAM-100，只观察
 *       ActivityMeterData_s.update_count 和通信状态：
 *       - update_count 变化且状态 OK，说明新读数到了，补发完整两帧；
 *       - 状态进入 TIMEOUT/CRC/BAD_RESPONSE，说明本次读取失败，只补发最终状态帧。
 */
static void MachineCMD_CompleteActivityRequest(void)
{
    ActivityMeterData_s activity_data;
    uint8_t state;

    if (machine_cmd.activity_request_pending == 0U)
    {
        return;
    }

    memset(&activity_data, 0, sizeof(activity_data));
    (void)ActivityMeter_GetData(&activity_data);
    state = (uint8_t)ActivityMeter_GetState();

    if (machine_cmd.activity_request_started == 0U)
    {
        if (state == (uint8_t)ACTIVITY_METER_STATE_WAITING)
        {
            machine_cmd.activity_request_started = 1U;
        }
        else if (ActivityMeter_RequestRead() != 0U)
        {
            machine_cmd.activity_request_started = 1U;
        }
        return;
    }

    if ((activity_data.state == ACTIVITY_METER_STATE_OK) &&
        (activity_data.update_count != machine_cmd.activity_request_update_count))
    {
        if (MachineCMD_SendActivityData(&activity_data,
                                        machine_cmd.activity_request_seq) != 0U)
        {
            machine_cmd.activity_request_pending = 0U;
            machine_cmd.activity_request_started = 0U;
            machine_cmd.activity_reported_update_count = activity_data.update_count;
        }
        return;
    }

    if ((state != (uint8_t)ACTIVITY_METER_STATE_NOT_READ) &&
        (state != (uint8_t)ACTIVITY_METER_STATE_WAITING) &&
        (state != (uint8_t)ACTIVITY_METER_STATE_OK) &&
        (MachineCMD_SendActivityState(&activity_data,
                                      state,
                                      machine_cmd.activity_request_seq) != 0U))
    {
        machine_cmd.activity_request_pending = 0U;
        machine_cmd.activity_request_started = 0U;
    }
}

/**
 * @brief 活度计成功更新后主动推送给上位机实时显示。
 *
 * @note 主动上报不对应任何上位机请求，所以 SEQ 固定为 0。若当前已有 READ_ACTIVITY
 *       查询等待完成，则由 MachineCMD_CompleteActivityRequest() 使用原 SEQ 返回，
 *       这里跳过，避免同一读数同时以“查询返回”和“主动推送”两种身份重复发送。
 */
static void MachineCMD_ReportActivityUpdate(void)
{
    ActivityMeterData_s activity_data;

    if (Machine_IsTransferToActivityDone() == 0U)
    {
        return;
    }

    if (machine_cmd.activity_request_pending != 0U)
    {
        return;
    }

    memset(&activity_data, 0, sizeof(activity_data));
    if ((ActivityMeter_GetData(&activity_data) == 0U) ||
        (activity_data.state != ACTIVITY_METER_STATE_OK) ||
        (activity_data.update_count == 0U) ||
        (activity_data.update_count == machine_cmd.activity_reported_update_count))
    {
        return;
    }

    if (MachineCMD_SendActivityData(&activity_data, MACHINE_CMD_ACTIVITY_PUSH_SEQ) != 0U)
    {
        machine_cmd.activity_reported_update_count = activity_data.update_count;
    }
}

/**
 * @brief 药液转移完成后主动上报一次稳定活度。
 *
 * @note processId=4 完成后，状态帧会进入 0x18。若 RAM-100 已经有稳定读数，
 *       这里主动发送 07 01 + 07 02，SEQ 固定为 0；若暂时没有稳定读数，
 *       不伪造成功状态，等待上位机 READ_ACTIVITY 或后续真实读数更新。
 */
static void MachineCMD_ReportTransferActivityIfReady(void)
{
    ActivityMeterData_s activity_data;

    if ((Machine_IsTransferToActivityDone() == 0U) ||
        (machine_cmd.remote_transfer_activity_reported != 0U))
    {
        return;
    }

    memset(&activity_data, 0, sizeof(activity_data));
    if ((ActivityMeter_GetData(&activity_data) == 0U) ||
        (activity_data.state != ACTIVITY_METER_STATE_OK))
    {
        return;
    }

    if (MachineCMD_SendActivityData(&activity_data, MACHINE_CMD_ACTIVITY_PUSH_SEQ) != 0U)
    {
        machine_cmd.remote_transfer_activity_reported = 1U;
        machine_cmd.activity_reported_update_count = activity_data.update_count;
    }
}

/**
 * @brief 缓存上位机 SET_PARAM 参数。
 */
static void MachineCMD_HandleRemoteSetParam(const CommunicationHostCommand_s *command)
{
    uint8_t result = COMMUNICATION_RESULT_OK;
    uint16_t error = COMMUNICATION_ERROR_NONE;

    if (command == NULL)
    {
        return;
    }

    switch (command->obj)
    {
    case COMMUNICATION_OBJ_PREPARE_PARAM:
        machine_cmd.remote_prepare_initial_activity_x100 =
            Communication_ReadU16LE(&command->data[2]);
        machine_cmd.remote_prepare_target_conc_x1000 =
            Communication_ReadU16LE(&command->data[4]);
        machine_cmd.remote_prepare_param_ready = 1U;
        break;

    case COMMUNICATION_OBJ_PREPARE_VOLUME_PARAM:
        machine_cmd.remote_prepare_water_volume_x100 =
            Communication_ReadU16LE(&command->data[2]);
        machine_cmd.remote_prepare_final_volume_x100 =
            Communication_ReadU16LE(&command->data[4]);
        machine_cmd.remote_prepare_volume_ready = 1U;
        break;

    case COMMUNICATION_OBJ_DISPENSE_PARAM:
        machine_cmd.remote_dispense_volume_x100 =
            Communication_ReadU16LE(&command->data[2]);
        machine_cmd.remote_dispense_target_activity_x100 =
            Communication_ReadU16LE(&command->data[4]);
        machine_cmd.remote_dispense_param_ready = 1U;
        break;

    default:
        result = COMMUNICATION_RESULT_BAD_PARAM;
        error = COMMUNICATION_ERROR_BAD_PARAM;
        break;
    }

    (void)Communication_SendAck(command->cmd,
                                command->obj,
                                result,
                                error,
                                0U,
                                command->seq);
}

/**
 * @brief 按上位机 START_PROCESS 启动完整业务流程。
 */
static void MachineCMD_HandleRemoteStartProcess(const CommunicationHostCommand_s *command)
{
    uint8_t process_id;
    uint8_t result = COMMUNICATION_RESULT_OK;
    uint16_t error = COMMUNICATION_ERROR_NONE;

    if ((command == NULL) || (command->obj != COMMUNICATION_OBJ_SYSTEM))
    {
        if (command != NULL)
        {
            (void)Communication_SendAck(command->cmd,
                                        command->obj,
                                        COMMUNICATION_RESULT_BAD_PARAM,
                                        COMMUNICATION_ERROR_BAD_PARAM,
                                        0U,
                                        command->seq);
        }
        return;
    }

    process_id = command->data[2];
    if (process_id == COMMUNICATION_PROCESS_PREPARE)
    {
        if (Machine_IsTransferToActivityDone() == 0U)
        {
            result = COMMUNICATION_RESULT_BUSY;
            error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
        else if ((machine_cmd.remote_prepare_param_ready == 0U) ||
            (machine_cmd.remote_prepare_volume_ready == 0U))
        {
            result = COMMUNICATION_RESULT_BAD_PARAM;
            error = COMMUNICATION_ERROR_BAD_PARAM;
        }
        else if (Machine_StartRemotePrepare(machine_cmd.remote_prepare_water_volume_x100,
                                            machine_cmd.remote_prepare_final_volume_x100,
                                            machine_cmd.remote_prepare_initial_activity_x100,
                                            machine_cmd.remote_prepare_target_conc_x1000,
                                            command->seq) != 0U)
        {
            machine_cmd.remote_prepare_param_ready = 0U;
            machine_cmd.remote_prepare_volume_ready = 0U;
        }
        else
        {
            result = COMMUNICATION_RESULT_BUSY;
            error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
    }
    else if (process_id == COMMUNICATION_PROCESS_DISPENSE)
    {
        if (machine_cmd.remote_dispense_param_ready == 0U)
        {
            result = COMMUNICATION_RESULT_BAD_PARAM;
            error = COMMUNICATION_ERROR_BAD_PARAM;
        }
        else if (Machine_StartRemoteDispense(machine_cmd.remote_dispense_volume_x100) != 0U)
        {
            machine_cmd.remote_dispense_param_ready = 0U;
        }
        else
        {
            result = COMMUNICATION_RESULT_BUSY;
            error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
    }
    else if (process_id == COMMUNICATION_PROCESS_TRANSFER_TO_ACTIVITY)
    {
        if (Machine_StartRemoteTransferToActivity() != 0U)
        {
            machine_cmd.remote_prepare_param_ready = 0U;
            machine_cmd.remote_prepare_volume_ready = 0U;
            machine_cmd.remote_transfer_activity_reported = 0U;
        }
        else
        {
            result = COMMUNICATION_RESULT_BUSY;
            error = COMMUNICATION_ERROR_STATE_NOT_ALLOWED;
        }
    }
    else
    {
        result = COMMUNICATION_RESULT_UNSUPPORTED;
        error = COMMUNICATION_ERROR_NONE;
    }

    (void)Communication_SendAck(command->cmd,
                                command->obj,
                                result,
                                error,
                                0U,
                                command->seq);
}

/**
 * @brief 执行上位机单轴步进电机命令。
 *
 * @param command 上位机命令缓存。
 * @return 1 表示已成功启动或停止；0 表示参数不支持或底层拒绝启动。
 *
 * @note 当前远控执行层约定：
 *       Byte1=对象，MOTOR_A/MOTOR_B；
 *       Byte2=方向，0 表示反向，1 表示正向；
 *       Byte3~4=steps，小端；
 *       Byte5~6=frequencyHz，小端，也就是每秒输出多少个 PUL 脉冲；
 *       Byte7=SEQ，已经在 Communication 层特殊处理。
 */
static uint8_t MachineCMD_HandleRemoteStepper(const CommunicationHostCommand_s *command)
{
    StepMotorId_e motor;
    StepMotorDirection_e direction;
    uint16_t steps;
    uint16_t frequency_hz;

    if (command == NULL)
    {
        return 0U;
    }

    if (command->obj == COMMUNICATION_OBJ_MOTOR_A)
    {
        motor = STEP_MOTOR_ID_A;
    }
    else if (command->obj == COMMUNICATION_OBJ_MOTOR_B)
    {
        motor = STEP_MOTOR_ID_B;
    }
    else
    {
        return 0U;
    }

    /*
     * CAN 协议使用“正/反向”，底层模块使用具体的 DIR 电平枚举。
     * 这里集中做一次翻译，上层协议就不需要关心电机接线和机械语义。
     */
    direction = (command->data[2] == MACHINE_CMD_REMOTE_DIR_REVERSE) ?
                STEP_MOTOR_DIR_REVERSE :
                STEP_MOTOR_DIR_FORWARD;
    steps = Communication_ReadU16LE(&command->data[3]);
    frequency_hz = Communication_ReadU16LE(&command->data[5]);

    if (frequency_hz == 0U)
    {
        return 0U;
    }

    if (steps == 0U)
    {
        return StepMotor_RunContinuous(motor, direction, frequency_hz);
    }

    return StepMotor_RunSteps(motor, direction, frequency_hz, steps);
}

/**
 * @brief 执行上位机电磁阀控制命令。
 *
 * @param command 上位机命令缓存。
 * @return 1 表示写入成功；0 表示对象不支持或阀模块暂时拒绝切换。
 *
 * @note 当前远控执行层约定 Byte2=0 关闭阀，Byte2 非 0 打开阀。
 */
static uint8_t MachineCMD_HandleRemoteValve(const CommunicationHostCommand_s *command)
{
    SolenoidValveId_e valve;
    SolenoidValveState_e state;

    if (command == NULL)
    {
        return 0U;
    }

    if (command->obj == COMMUNICATION_OBJ_VALVE_1)
    {
        valve = SOLENOID_VALVE_ID_WATER;
    }
    else if (command->obj == COMMUNICATION_OBJ_VALVE_2)
    {
        valve = SOLENOID_VALVE_ID_MED;
    }
    else
    {
        return 0U;
    }

    state = (command->data[2] != 0U) ?
            SOLENOID_VALVE_STATE_ON_NC_OPEN :
            SOLENOID_VALVE_STATE_OFF_NO_OPEN;

    return SolenoidValve_SetState(valve, state);
}

/**
 * @brief 执行上位机泵控制命令。
 *
 * @param command 上位机命令缓存。
 * @return 1 表示命令已下发；0 表示对象/动作不支持或底层发送失败。
 *
 * @note WATER_PUMP 使用 Byte2=0/1 控制关/开。
 *       PUMP_1/PUMP_2 使用 Byte2=0/1/2 表示停止/吸入/排出。
 *       Byte3~4 非 0 时按体积 ul 精确运动；Byte3~4 为 0 时作为上位机点动命令。
 *       点动命令固定按 10 圈处理，底层通过独立的每圈步数常量换算成 ISC1000 步数。
 *       注意不要把满行程 1600 步当成电机一圈，也不要把 `in/out 10` 误认为 10 圈。
 */
static uint8_t MachineCMD_HandleRemotePump(const CommunicationHostCommand_s *command)
{
    PumpDrive_s *pump;
    uint8_t action;
    uint16_t volume_ul;

    if (command == NULL)
    {
        return 0U;
    }

    action = command->data[2];

    if (command->obj == COMMUNICATION_OBJ_WATER_PUMP)
    {
        return WaterPump_SetState(WATER_PUMP_ID_MAIN,
                                  (action == MACHINE_CMD_REMOTE_PUMP_STOP) ?
                                  WATER_PUMP_STATE_OFF :
                                  WATER_PUMP_STATE_ON);
    }

    if (command->obj == COMMUNICATION_OBJ_PUMP_1)
    {
        pump = PumpDrive_GetPump1();
    }
    else if (command->obj == COMMUNICATION_OBJ_PUMP_2)
    {
        pump = PumpDrive_GetPump2();
    }
    else
    {
        return 0U;
    }

    if (pump == NULL)
    {
        return 0U;
    }

    volume_ul = Communication_ReadU16LE(&command->data[3]);
    switch (action)
    {
    case MACHINE_CMD_REMOTE_PUMP_STOP:
        return PumpDrive_Stop(pump, 1U);

    case MACHINE_CMD_REMOTE_PUMP_IN:
        if (volume_ul == 0U)
        {
            return PumpDrive_MoveInAngleDegX10(pump, MACHINE_CMD_REMOTE_PUMP_DEFAULT_ANGLE_DEG_X10);
        }
        return PumpDrive_MoveInVolumeUl(pump, volume_ul);

    case MACHINE_CMD_REMOTE_PUMP_OUT:
        if (volume_ul == 0U)
        {
            return PumpDrive_MoveOutAngleDegX10(pump, MACHINE_CMD_REMOTE_PUMP_DEFAULT_ANGLE_DEG_X10);
        }
        return PumpDrive_MoveOutVolumeUl(pump, volume_ul);

    default:
        return 0U;
    }
}

/**
 * @brief 停止上位机指定对象。
 *
 * @param command 上位机命令缓存。
 * @return 1 表示对象已处理；0 表示对象不支持。
 */
static uint8_t MachineCMD_HandleRemoteStopObject(const CommunicationHostCommand_s *command)
{
    PumpDrive_s *pump;

    if (command == NULL)
    {
        return 0U;
    }

    switch (command->obj)
    {
    case COMMUNICATION_OBJ_SYSTEM:
        MachineCMD_StopRemoteOutputs();
        return 1U;

    case COMMUNICATION_OBJ_MOTOR_A:
        StepMotor_Stop(STEP_MOTOR_ID_A);
        return 1U;

    case COMMUNICATION_OBJ_MOTOR_B:
        StepMotor_Stop(STEP_MOTOR_ID_B);
        return 1U;

    case COMMUNICATION_OBJ_VALVE_1:
        SolenoidValve_Off(SOLENOID_VALVE_ID_WATER);
        return 1U;

    case COMMUNICATION_OBJ_VALVE_2:
        SolenoidValve_Off(SOLENOID_VALVE_ID_MED);
        return 1U;

    case COMMUNICATION_OBJ_WATER_PUMP:
        WaterPump_Stop(WATER_PUMP_ID_MAIN);
        return 1U;

    case COMMUNICATION_OBJ_PUMP_1:
        pump = PumpDrive_GetPump1();
        return (pump != NULL) ? PumpDrive_Stop(pump, 1U) : 0U;

    case COMMUNICATION_OBJ_PUMP_2:
        pump = PumpDrive_GetPump2();
        return (pump != NULL) ? PumpDrive_Stop(pump, 1U) : 0U;

    default:
        return 0U;
    }
}

/**
 * @brief 向上位机返回远控状态帧。
 *
 * @param seq 当前查询命令序号，0x181 状态帧没有独立 seq 字段，保留该参数方便后续扩展。
 */
static void MachineCMD_SendRemoteStatus(uint8_t seq)
{
    CommunicationStatus_s status;
    const CommunicationControlContext_s *control;
    PumpDrive_s *pump;
    uint8_t activity_state;

    (void)seq;
    memset(&status, 0, sizeof(status));
    control = Communication_GetControlContext();
    status.step = Machine_GetCommunicationStep();

    if (StepMotor_IsBusy(STEP_MOTOR_ID_A) != 0U)
    {
        status.motor_state |= MACHINE_CMD_REMOTE_MOTOR_A_BUSY;
    }
    if (StepMotor_IsBusy(STEP_MOTOR_ID_B) != 0U)
    {
        status.motor_state |= MACHINE_CMD_REMOTE_MOTOR_B_BUSY;
    }

    pump = PumpDrive_GetPump1();
    if ((pump != NULL) && (pump->status.busy != 0U))
    {
        status.motor_state |= MACHINE_CMD_REMOTE_PUMP1_BUSY;
    }
    pump = PumpDrive_GetPump2();
    if ((pump != NULL) && (pump->status.busy != 0U))
    {
        status.motor_state |= MACHINE_CMD_REMOTE_PUMP2_BUSY;
    }

    if (SolenoidValve_GetState(SOLENOID_VALVE_ID_WATER) == SOLENOID_VALVE_STATE_ON_NC_OPEN)
    {
        status.output_state |= MACHINE_CMD_REMOTE_WATER_VALVE_ON;
    }
    if (SolenoidValve_GetState(SOLENOID_VALVE_ID_MED) == SOLENOID_VALVE_STATE_ON_NC_OPEN)
    {
        status.output_state |= MACHINE_CMD_REMOTE_MED_VALVE_ON;
    }
    if (WaterPump_IsOn(WATER_PUMP_ID_MAIN) != 0U)
    {
        status.output_state |= MACHINE_CMD_REMOTE_WATER_PUMP_ON;
    }

    if (Communication_IsUnlocked() == 0U)
    {
        status.sys_state = COMMUNICATION_SYS_AUTH_LOCKED;
    }
    else if (Communication_GetControlMode() == COMMUNICATION_CONTROL_REMOTE_PAUSED)
    {
        status.sys_state = COMMUNICATION_SYS_PAUSED;
        status.step = COMMUNICATION_STEP_REMOTE_PAUSED;
    }
    else if ((control != NULL) && (control->local_takeover_latched != 0U))
    {
        status.sys_state = COMMUNICATION_SYS_IDLE;
        status.step = COMMUNICATION_STEP_LOCAL_TAKEOVER;
    }
    else if ((Machine_IsFlowRunning() != 0U) ||
             (status.motor_state != 0U) ||
             (status.output_state != 0U))
    {
        status.sys_state = COMMUNICATION_SYS_RUNNING;
    }
    else
    {
        status.sys_state = COMMUNICATION_SYS_IDLE;
        if ((machine_cmd.remote_dispense_param_ready != 0U) &&
            (status.step == COMMUNICATION_STEP_IDLE))
        {
            status.step = COMMUNICATION_STEP_DISPENSE_PARAM_READY;
        }
        else if (((machine_cmd.remote_prepare_param_ready != 0U) ||
                  (machine_cmd.remote_prepare_volume_ready != 0U)) &&
                 (status.step == COMMUNICATION_STEP_IDLE))
        {
            status.step = COMMUNICATION_STEP_PREPARE_PARAM_READY;
        }
    }

    activity_state = (uint8_t)ActivityMeter_GetState();
    if (Machine_IsTransferToActivityDone() == 0U)
    {
        if ((activity_state == (uint8_t)COMMUNICATION_ACTIVITY_TIMEOUT) ||
            (activity_state == (uint8_t)COMMUNICATION_ACTIVITY_CRC_ERROR) ||
            (activity_state == (uint8_t)COMMUNICATION_ACTIVITY_BAD_RESPONSE))
        {
            status.activity_state = activity_state;
        }
        else
        {
            status.activity_state = COMMUNICATION_ACTIVITY_NOT_READ;
        }
    }
    else
    {
        status.activity_state = (activity_state <= (uint8_t)COMMUNICATION_ACTIVITY_BAD_RESPONSE) ?
                                activity_state :
                                COMMUNICATION_ACTIVITY_NOT_READ;
    }

    (void)Communication_SendStatus(&status);
}

/**
 * @brief 向 16 字节 LCD 行缓冲追加一段原始字节。
 *
 * @param line 目标行缓冲，长度至少 MACHINE_CMD_LCD_LINE_BYTES。
 * @param offset 当前写入偏移。
 * @param data 待追加数据，可以是 GB2312 中文字节，也可以是 ASCII。
 * @param len 待追加数据长度。
 * @return 追加后的偏移，最大不超过 MACHINE_CMD_LCD_LINE_BYTES。
 */
static uint8_t MachineCMD_LineAppendBytes(uint8_t *line, uint8_t offset, const uint8_t *data, uint8_t len)
{
    if ((line == NULL) || (data == NULL) || (offset >= MACHINE_CMD_LCD_LINE_BYTES))
    {
        return offset;
    }

    while ((len > 0U) && (offset < MACHINE_CMD_LCD_LINE_BYTES))
    {
        line[offset] = *data;
        offset++;
        data++;
        len--;
    }

    return offset;
}

/**
 * @brief 向 16 字节 LCD 行缓冲追加一条文案。
 *
 * @param line 目标行缓冲。
 * @param offset 当前写入偏移。
 * @param text 待追加文案。
 * @return 追加后的偏移。
 */
static uint8_t MachineCMD_LineAppendText(uint8_t *line, uint8_t offset, const MachineCmdText_s *text)
{
    if (text == NULL)
    {
        return offset;
    }

    if ((offset + text->len) > MACHINE_CMD_LCD_LINE_BYTES)
    {
        return offset;
    }

    return MachineCMD_LineAppendBytes(line, offset, text->data, text->len);
}

/**
 * @brief 向 16 字节 LCD 行缓冲追加 ASCII 字符串。
 *
 * @param line 目标行缓冲。
 * @param offset 当前写入偏移。
 * @param text 待追加 ASCII 字符串。
 * @return 追加后的偏移。
 */
static uint8_t MachineCMD_LineAppendString(uint8_t *line, uint8_t offset, const char *text)
{
    if (text == NULL)
    {
        return offset;
    }

    return MachineCMD_LineAppendBytes(line, offset, (const uint8_t *)text, (uint8_t)strlen(text));
}

/**
 * @brief 向行缓冲追加一个开关状态。
 *
 * @param line 目标行缓冲。
 * @param offset 当前写入偏移。
 * @param switch_mask 手动调试开关位。
 * @return 追加后的偏移。
 */
static uint8_t MachineCMD_LineAppendSwitchState(uint8_t *line, uint8_t offset, uint8_t switch_mask)
{
    const MachineCmdText_s *state_text;

    state_text = ((machine_cmd.manual_switches & switch_mask) != 0U) ?
                 &machine_cmd_text_on :
                 &machine_cmd_text_off;

    return MachineCMD_LineAppendText(line, offset, state_text);
}

/**
 * @brief 把活度值转换成 LCD 可显示的 ASCII 文本。
 *
 * @param buffer 输出缓冲区。
 * @param size 输出缓冲区长度。
 * @param value 活度计返回的浮点值。
 * @param unit 单位字符串，例如 uCi、mCi。
 *
 * @note 这里不用 printf 的浮点格式，避免在 STM32F103 上额外拉入较大的格式化代码。
 */
static void MachineCMD_FormatActivityAscii(char *buffer, uint8_t size, float value, const char *unit)
{
    uint32_t value_x10;

    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }

    if (value < 0.0f)
    {
        value = 0.0f;
    }

    value_x10 = (uint32_t)(value * 10.0f + 0.5f);
    (void)snprintf(buffer,
                   size,
                   "%lu.%01lu%s",
                   (unsigned long)(value_x10 / 10U),
                   (unsigned long)(value_x10 % 10U),
                   unit);
}

/**
 * @brief 更新 LCD 指定行的原始字节。
 *
 * @param row LCD 行号。
 * @param data 待显示字节，可以是 GB2312 中文字节或 ASCII。
 * @param len 待显示字节数。
 *
 * @note DisplayLcd_UpdateLine() 会补空格并做行缓存比较，因此这里不主动清屏。
 */
static void MachineCMD_WriteBytes(DisplayLcdRow_e row, const uint8_t *data, uint8_t len)
{
    DisplayLcd_UpdateLine(row, data, len);
}

/**
 * @brief 更新 LCD 指定行为一条固定文案。
 *
 * @param row LCD 行号。
 * @param text 待显示文案。
 */
static void MachineCMD_WriteText(DisplayLcdRow_e row, const MachineCmdText_s *text)
{
    if (text == NULL)
    {
        MachineCMD_WriteBytes(row, NULL, 0U);
        return;
    }

    MachineCMD_WriteBytes(row, text->data, text->len);
}

/**
 * @brief 获取手动动作键对应的提示文案。
 *
 * @return 当前动作键文案；NULL 表示当前是数字开关动作或空闲。
 *
 * @note 手动页的主体已经用中文显示四个开关状态。
 *       这里仅用于动作键反馈，避免按【进罐】【插针】这类键时 LCD 看起来没有变化。
 */
static const MachineCmdText_s *MachineCMD_GetManualActionText(void)
{
    switch (machine_cmd.manual_action)
    {
    case MACHINE_CMD_MANUAL_ACTION_IN_TANK:
        return &machine_cmd_text_in_tank;

    case MACHINE_CMD_MANUAL_ACTION_OUT_TANK:
        return &machine_cmd_text_out_tank;

    case MACHINE_CMD_MANUAL_ACTION_NEEDLE_IN:
        return &machine_cmd_text_needle_in;

    case MACHINE_CMD_MANUAL_ACTION_NEEDLE_OUT:
        return &machine_cmd_text_needle_out;

    case MACHINE_CMD_MANUAL_ACTION_DRAW_MED:
        return &machine_cmd_text_draw_med;

    case MACHINE_CMD_MANUAL_ACTION_EXHAUST:
        return &machine_cmd_text_exhaust;

    default:
        return NULL;
    }
}

/**
 * @brief 获取配药向导当前步骤标题。
 *
 * @return 当前步骤对应的 LCD 文案。
 */
static const MachineCmdText_s *MachineCMD_GetPrepStepTitle(void)
{
    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME)
    {
        return &machine_cmd_text_prep_step2;
    }

    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_TARGET_CONC)
    {
        return &machine_cmd_text_prep_step3;
    }

    return &machine_cmd_text_prep_step1;
}

/**
 * @brief 获取配药向导当前输入项说明。
 *
 * @return 当前输入项对应的 LCD 文案。
 */
static const MachineCmdText_s *MachineCMD_GetPrepStepLabel(void)
{
    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME)
    {
        return &machine_cmd_text_current_volume_unit;
    }

    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_TARGET_CONC)
    {
        return &machine_cmd_text_target_conc_unit;
    }

    return &machine_cmd_text_current_conc_unit;
}

/**
 * @brief 获取配药向导当前启动键提示。
 *
 * @return 当前步骤对应的 LCD 文案。
 */
static const MachineCmdText_s *MachineCMD_GetPrepStepHint(void)
{
    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_TARGET_CONC)
    {
        return &machine_cmd_text_start_prep;
    }

    return &machine_cmd_text_start_next_step;
}

/**
 * @brief 格式化配药向导当前步骤已经保存的数值。
 *
 * @param buffer 输出 ASCII 缓冲。
 * @param size 缓冲区长度。
 *
 * @note 用户按【清除】退回上一项时，已保存的值仍然显示出来。
 *       如果继续直接按【启动】，该值会被保留；如果重新输入数字，则用新输入覆盖。
 */
static void MachineCMD_FormatPrepFocusValue(char *buffer, uint8_t size)
{
    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }

    buffer[0] = '\0';
    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_CONC)
    {
        if (machine_cmd.prep_current_conc_x1000 != 0U)
        {
            MachineCMD_FormatConcX1000Ascii(buffer, size, machine_cmd.prep_current_conc_x1000);
        }
        return;
    }

    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME)
    {
        if (machine_cmd.prep_current_ml_x100 != 0U)
        {
            MachineCMD_FormatMlX100Ascii(buffer, size, machine_cmd.prep_current_ml_x100);
        }
        return;
    }

    if (machine_cmd.prep_target_conc_x1000 != 0U)
    {
        MachineCMD_FormatConcX1000Ascii(buffer, size, machine_cmd.prep_target_conc_x1000);
    }
}

/**
 * @brief 配药向导退回上一项。
 *
 * @note 面板没有上下方向键，所以复用【清除】键：
 *       当前输入框为空时，再按【清除】就回到上一个输入步骤。
 */
static void MachineCMD_MovePrepFocusBack(void)
{
    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_TARGET_CONC)
    {
        machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME;
        return;
    }

    if (machine_cmd.prep_focus == MACHINE_CMD_PREP_FOCUS_CURRENT_VOLUME)
    {
        machine_cmd.prep_focus = MACHINE_CMD_PREP_FOCUS_CURRENT_CONC;
    }
}

/**
 * @brief 显示开机初始化页。
 */
static void MachineCMD_ShowBootPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_title);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_init);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_eeprom);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_ready);
}

/**
 * @brief 显示系统启动就绪页。
 *
 * @note 本页面等待任意按键确认，确认后才进入待机看板页。
 */
static void MachineCMD_ShowReadyPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_title);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_system_ready);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_any_key_start);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, NULL);
}

/**
 * @brief 显示待机看板页。
 *
 * @note 浓度显示配药后的目标浓度；活度来自 RAM-100 最新有效读数；体积仍表示当前可发药体积。
 */
static void MachineCMD_ShowStandbyPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    char ascii[12];

    MachineCMD_UpdateStandbyActivityFromMeter();

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_standby);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_concentration);
    MachineCMD_FormatConcX1000Ascii(ascii, sizeof(ascii), machine_cmd.standby_conc_x1000);
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    (void)MachineCMD_LineAppendString(line, offset, "mCi/ml");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_activity);
    MachineCMD_FormatFixed1Ascii(ascii,
                                 sizeof(ascii),
                                 ((uint32_t)machine_cmd.standby_activity_x100 + 5U) / 10U);
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    (void)MachineCMD_LineAppendString(line, offset, "mCi");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_volume_prefix);
    MachineCMD_FormatMlX100Ascii(ascii, sizeof(ascii), machine_cmd.standby_volume_ml_x100);
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    (void)MachineCMD_LineAppendString(line, offset, "ml");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_4, line, sizeof(line));
}

/**
 * @brief 显示配药参数输入页。
 *
 * @note 这里使用向导式单项输入，而不是同屏显示三项参数。
 *       操作员每次只看到一个焦点，避免误以为第一下【启动】会直接开机动作。
 */
static void MachineCMD_ShowPrepSettingPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    char ascii[8];

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, MachineCMD_GetPrepStepTitle());
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, MachineCMD_GetPrepStepLabel());

    /*
     * 第 3 行：> ____
     * 当前步骤只显示一个输入框。若用户从后续步骤退回来，优先显示已经保存的值；
     * 一旦重新输入数字，则显示新的输入缓冲。
     */
    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendString(line, 0U, "> ");
    if (machine_cmd.input[0] != '\0')
    {
        offset = MachineCMD_LineAppendString(line, offset, machine_cmd.input);
    }
    else
    {
        MachineCMD_FormatPrepFocusValue(ascii, sizeof(ascii));
        offset = MachineCMD_LineAppendString(line, offset, ascii[0] != '\0' ? ascii : "____");
    }
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, MachineCMD_GetPrepStepHint());
}

/**
 * @brief 显示配药运行提示页。
 *
 * @note 目前这里只做 UI 提示，真正配药流程后续应由 machine 层状态机接管。
 */
static void MachineCMD_ShowPrepRunningPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_prep_run);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_current_measure);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_wait_activity);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_pause_reset_hint);
}

/**
 * @brief 显示配药活度计测量页。
 */
static void MachineCMD_ShowPrepMeasurePage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    uint32_t elapsed_ms;
    uint32_t left_sec;
    char ascii[12];
    ActivityMeterData_s activity_data;

    elapsed_ms = MachineCMD_GetMs() - machine_cmd.measure_start_ms;
    if (elapsed_ms >= MACHINE_CMD_MEASURE_HOLD_MS)
    {
        left_sec = 0U;
    }
    else
    {
        left_sec = (MACHINE_CMD_MEASURE_HOLD_MS - elapsed_ms + 999U) / 1000U;
    }

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_prep_run);

    /* 轮询进入 WAITING 时继续显示最近一次有效值，避免数值和“检测中”来回闪烁。 */
    if ((ActivityMeter_GetData(&activity_data) != 0U) &&
        (activity_data.update_count != 0U))
    {
        memset(line, ' ', sizeof(line));
        offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_activity);
        MachineCMD_FormatActivityAscii(ascii,
                                       sizeof(ascii),
                                       activity_data.activity,
                                       ActivityMeter_GetUnitString(activity_data.activity_unit));
        (void)MachineCMD_LineAppendString(line, offset, ascii);
        MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));
    }
    else if (ActivityMeter_GetState() == ACTIVITY_METER_STATE_TIMEOUT)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_activity_timeout);
    }
    else
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_wait_activity);
    }

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_countdown);
    (void)snprintf(ascii, sizeof(ascii), "%2u", (unsigned int)left_sec);
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    (void)MachineCMD_LineAppendString(line, offset, "s");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_pause_reset_hint);
}

/**
 * @brief 显示发药参数输入页。
 */
static void MachineCMD_ShowDispSettingPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    char ascii[8];

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_disp_title);

    /*
     * 第 2 行：目标:____ml
     * 目标输入只使用 ASCII 数字和小数点，避免和中文 GB2312 字节混写后出现错位。
     */
    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_target);
    offset = MachineCMD_LineAppendString(line, offset, machine_cmd.input[0] != '\0' ? machine_cmd.input : "____");
    (void)MachineCMD_LineAppendString(line, offset, "ml");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    /*
     * 第 3 行： （余量：xxx.xml)
     * 配药确认后，left_ml_x100 保存“当前原药体积 + 计算出的补水体积”。
     * 未配药直接进入发药页时保持 0.0ml，用于拦截超余量输入。
     */
    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_remaining_prefix);
    MachineCMD_FormatMlX100Ascii(ascii, sizeof(ascii), machine_cmd.left_ml_x100);
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    offset = MachineCMD_LineAppendString(line, offset, "ml");
    (void)MachineCMD_LineAppendString(line, offset, ")");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    if (machine_cmd.dispense_input_error != 0U)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_input_over_left);
    }
    else
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_disp_start);
    }
}

/**
 * @brief 显示发药运行提示页。
 *
 * @note 完成进度由 machine 层按泵2已经确认完成的分段体积换算。
 */
static void MachineCMD_ShowDispRunningPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    char ascii[8];

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_disp_run);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_pump2);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_progress_prefix);
    (void)snprintf(ascii, sizeof(ascii), "%03u%%", (unsigned int)Machine_GetDispenseProgressPercent());
    (void)MachineCMD_LineAppendString(line, offset, ascii);
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_pause_reset_hint);
}

/**
 * @brief 显示上位机远控接管页。
 *
 * @note 本页全部使用 MachineCMD_Text.c 中的 GB2312 字节表。
 *       每行均不超过 ST7920 文本模式 16 字节上限，避免中文错位或乱码。
 */
static void MachineCMD_ShowRemotePage(void)
{
    const CommunicationControlContext_s *control = Communication_GetControlContext();
    uint8_t mode = Communication_GetControlMode();

    MachineCMD_SyncRemoteState();
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_remote_title);

    if (mode == COMMUNICATION_CONTROL_REMOTE_PAUSED)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_remote_paused);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_wait_host);
        if ((control != NULL) && (control->remote_resume_allowed != 0U))
        {
            MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_remote_start_continue);
        }
        else
        {
            MachineCMD_WriteText(DISPLAY_LCD_ROW_3, NULL);
        }
    }
    else if (mode == COMMUNICATION_CONTROL_REMOTE_SWITCHING)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_remote_switching);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_wait_host);
    }
    else if (mode == COMMUNICATION_CONTROL_REMOTE)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_remote_takeover);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_remote_pause);
    }
    else if ((control != NULL) && (control->local_takeover_latched != 0U))
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_local_takeover);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_remote_key_request);
    }
    else if (Communication_IsUnlocked() == 0U)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_wait_auth);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_wait_host);
    }
    else
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_wait_host);
        MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_remote_key_request);
    }

    if (mode == COMMUNICATION_CONTROL_REMOTE)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_remote_pause_first);
    }
    else
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_remote_reset);
    }
}

/**
 * @brief 显示手动调试页。
 *
 * @note 直接显示四个手动开关的中文开/关状态。
 *       按 1/2/4/5 翻转后，下一轮刷新会立即反映到对应项目。
 */
static void MachineCMD_ShowManualPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    const MachineCmdText_s *action_text;

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_manual);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_water_in);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_full_colon);
    offset = MachineCMD_LineAppendSwitchState(line, offset, MACHINE_CMD_MANUAL_WATER_IN);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_med_in);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_full_colon);
    (void)MachineCMD_LineAppendSwitchState(line, offset, MACHINE_CMD_MANUAL_MED_IN);
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_water_out);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_full_colon);
    offset = MachineCMD_LineAppendSwitchState(line, offset, MACHINE_CMD_MANUAL_WATER_OUT);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_med_out);
    offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_full_colon);
    (void)MachineCMD_LineAppendSwitchState(line, offset, MACHINE_CMD_MANUAL_MED_OUT);
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    action_text = MachineCMD_GetManualActionText();
    if (action_text != NULL)
    {
        memset(line, ' ', sizeof(line));
        offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_last);
        offset = MachineCMD_LineAppendText(line, offset, &machine_cmd_text_full_colon);
        (void)MachineCMD_LineAppendText(line, offset, action_text);
        MachineCMD_WriteBytes(DISPLAY_LCD_ROW_4, line, sizeof(line));
        return;
    }

    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_manual_switch_hint);
}

/**
 * @brief 显示自动清洗页。
 *
 * @note 当前页只完成 UI 入口提示，泵 1 的实际冲洗动作应由后续 machine 层接管。
 */
static void MachineCMD_ShowCleanPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_clean_title);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_clean_pipe);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_pump1_run);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_waste_cup);
}

/**
 * @brief 显示流程暂停页。
 */
static void MachineCMD_ShowPausedPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_paused_title);

    if (machine_cmd.paused_page == MACHINE_CMD_PAGE_DISP_RUNNING)
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_disp_paused);
    }
    else
    {
        MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_prep_paused);
    }

    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_start_continue);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_reset_stop);
}

/**
 * @brief 显示报警页。
 *
 * @note 当前还没有接入真实报警源，先保留固定文案和复位清除提示。
 */
static void MachineCMD_ShowAlarmPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_alarm);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_fault);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_locked);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_rst_alarm);
}
