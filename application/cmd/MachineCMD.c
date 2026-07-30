//
// Created by lenovo on 26-7-30.
//

#include "MachineCMD.h"
#include <stdio.h>
#include <string.h>
#include "bsp_dwt.h"
#include "display_lcd.h"
#include "Keyboard.h"
#include "main.h"
#include "MachineCMD_Text.h"

#define MACHINE_CMD_LCD_LINE_BYTES        16U
#define MACHINE_CMD_INPUT_MAX_LEN         8U
#define MACHINE_CMD_PREP_BOTTLE_COUNT     3U
#define MACHINE_CMD_BOOT_HOLD_MS          2000U

/*
 * 当前硬件表中能确认的三路 24V 输出：
 * - PC13：阀 1；
 * - PC14：阀 2；
 * - PC15：抽水泵。
 *
 * 手动页先把“水进/药进/水出/药出”作为逻辑开关保存，再映射到这三路硬件。
 * 如果后续增加独立药出泵或陶瓷泵方向控制，只需要改 MachineCMD_ApplyManualOutputs()。
 */
#define MACHINE_CMD_VALVE_WATER_PORT      GPIOC
#define MACHINE_CMD_VALVE_WATER_PIN       GPIO_PIN_13
#define MACHINE_CMD_VALVE_MED_PORT        GPIOC
#define MACHINE_CMD_VALVE_MED_PIN         GPIO_PIN_14
#define MACHINE_CMD_WATER_PUMP_PORT       GPIOC
#define MACHINE_CMD_WATER_PUMP_PIN        GPIO_PIN_15

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
    uint8_t prep_index;                                 // 当前正在输入的瓶号：0~2
    uint16_t prep_bottle_ml_x100[MACHINE_CMD_PREP_BOTTLE_COUNT]; // 配药瓶体积，单位 0.01ml
    uint16_t dispense_ml_x100;                          // 发药体积，单位 0.01ml
    char input[MACHINE_CMD_INPUT_MAX_LEN + 1U];          // 当前输入缓冲区
    MachineCmdManualAction_e manual_action;              // 手动页当前动作说明
} MachineCmdContext_s;

static MachineCmdContext_s machine_cmd = {0};

static uint32_t MachineCMD_GetMs(void);
static void MachineCMD_EnterPage(MachineCmdPage_e page);
static void MachineCMD_ClearInput(void);
static uint8_t MachineCMD_KeyToDigit(KeypadState_e key, uint8_t *digit);
static uint8_t MachineCMD_IsNumberKey(KeypadState_e key);
static void MachineCMD_AppendDigit(uint8_t digit);
static void MachineCMD_AppendDot(void);
static uint16_t MachineCMD_InputToMlX100(void);
static void MachineCMD_HandleStandbyKey(KeypadState_e key);
static void MachineCMD_HandlePrepSettingKey(KeypadState_e key);
static void MachineCMD_HandleDispSettingKey(KeypadState_e key);
static void MachineCMD_HandleManualKey(KeypadState_e key);
static void MachineCMD_SetManualAction(KeypadState_e key);
static void MachineCMD_ToggleManualSwitch(uint8_t switch_mask);
static void MachineCMD_ClearManualSwitches(void);
static void MachineCMD_ApplyManualOutputs(void);
static uint8_t MachineCMD_LineAppendBytes(uint8_t *line, uint8_t offset, const uint8_t *data, uint8_t len);
static uint8_t MachineCMD_LineAppendText(uint8_t *line, uint8_t offset, const MachineCmdText_s *text);
static uint8_t MachineCMD_LineAppendString(uint8_t *line, uint8_t offset, const char *text);
static void MachineCMD_WriteBytes(DisplayLcdRow_e row, const uint8_t *data, uint8_t len);
static void MachineCMD_WriteText(DisplayLcdRow_e row, const MachineCmdText_s *text);
static const MachineCmdText_s *MachineCMD_GetManualActionText(void);
static void MachineCMD_ShowBootPage(void);
static void MachineCMD_ShowReadyPage(void);
static void MachineCMD_ShowStandbyPage(void);
static void MachineCMD_ShowPrepSettingPage(void);
static void MachineCMD_ShowPrepRunningPage(void);
static void MachineCMD_ShowDispSettingPage(void);
static void MachineCMD_ShowDispRunningPage(void);
static void MachineCMD_ShowManualPage(void);
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
    machine_cmd.manual_action = MACHINE_CMD_MANUAL_ACTION_IDLE;
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

    /*
     * 开机页停留 2 秒，用于让用户看到系统已经进入初始化。
     * 后续如果接入 EEPROM 断电恢复，可以在这里改成等待人工确认。
     */
    if ((machine_cmd.page == MACHINE_CMD_PAGE_BOOT) &&
        ((MachineCMD_GetMs() - machine_cmd.boot_start_ms) >= MACHINE_CMD_BOOT_HOLD_MS))
    {
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_READY);
    }

    key = Keypad_GetPressedState();
    if (key == KEYPAD_STATE_NONE)
    {
        return;
    }

    /*
     * 启动就绪页只等待一次用户确认。
     * 任意按键都只进入待机页，不继续执行按键本身的功能。
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
        MachineCMD_ClearInput();
        MachineCMD_ClearManualSwitches();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_STANDBY);
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

    case MACHINE_CMD_PAGE_DISP_SETTING:
        MachineCMD_HandleDispSettingKey(key);
        break;

    case MACHINE_CMD_PAGE_MANUAL:
        MachineCMD_HandleManualKey(key);
        break;

    case MACHINE_CMD_PAGE_PREP_RUNNING:
    case MACHINE_CMD_PAGE_DISP_RUNNING:
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

    case MACHINE_CMD_PAGE_DISP_SETTING:
        MachineCMD_ShowDispSettingPage();
        break;

    case MACHINE_CMD_PAGE_DISP_RUNNING:
        MachineCMD_ShowDispRunningPage();
        break;

    case MACHINE_CMD_PAGE_MANUAL:
        MachineCMD_ShowManualPage();
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

    if ((len == 0U) || (len >= MACHINE_CMD_INPUT_MAX_LEN) ||
        (strchr(machine_cmd.input, '.') != NULL))
    {
        return;
    }

    machine_cmd.input[len] = '.';
    machine_cmd.input[len + 1U] = '\0';
}

/**
 * @brief 将当前输入字符串转换成 0.01ml 单位的整数值。
 *
 * @return 输入值放大 100 倍后的整数。例如 "2.50" 返回 250。
 *
 * @note 这里不用 atof()，避免在 STM32F103 上引入不必要的浮点格式化依赖。
 *       最多保留两位小数，多余的小数位直接忽略。
 */
static uint16_t MachineCMD_InputToMlX100(void)
{
    uint32_t integer_part = 0U;
    uint32_t decimal_part = 0U;
    uint8_t decimal_count = 0U;
    uint8_t dot_seen = 0U;

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
        else if (decimal_count < 2U)
        {
            decimal_part = decimal_part * 10U + (uint32_t)(ch - '0');
            decimal_count++;
        }
    }

    if (decimal_count == 1U)
    {
        decimal_part *= 10U;
    }

    integer_part = integer_part * 100U + decimal_part;
    if (integer_part > 65535U)
    {
        integer_part = 65535U;
    }

    return (uint16_t)integer_part;
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
        machine_cmd.prep_index = 0U;
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_PREP_SETTING);
        break;

    case KEYPAD_STATE_SEND_MEDICINE:
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_DISP_SETTING);
        break;

    case KEYPAD_STATE_IN_TANK:
    case KEYPAD_STATE_OUT_TANK:
    case KEYPAD_STATE_INSERT_NEEDLE:
    case KEYPAD_STATE_RETRACT_NEEDLE:
    case KEYPAD_STATE_DRAW_MEDICINE:
    case KEYPAD_STATE_EXHAUST_FIXED:
    case KEYPAD_STATE_REMOTE:
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
 * @note 数字和小数点写入 input；启动键保存当前瓶号输入值。
 *       3 个瓶号依次输入完成后，页面进入配药运行提示页。
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
        MachineCMD_ClearInput();
        return;
    }

    /*
     * 启动键在本页面不是立即驱动流程，而是作为“确认当前输入”的按键。
     * 前两个瓶号确认后跳到下一格，第三个瓶号确认后才进入运行提示页。
     */
    if (key == KEYPAD_STATE_START)
    {
        machine_cmd.prep_bottle_ml_x100[machine_cmd.prep_index] = MachineCMD_InputToMlX100();
        MachineCMD_ClearInput();

        if (machine_cmd.prep_index < (MACHINE_CMD_PREP_BOTTLE_COUNT - 1U))
        {
            machine_cmd.prep_index++;
        }
        else
        {
            MachineCMD_EnterPage(MACHINE_CMD_PAGE_PREP_RUNNING);
        }
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
        MachineCMD_ClearInput();
        return;
    }

    if (key == KEYPAD_STATE_START)
    {
        machine_cmd.dispense_ml_x100 = MachineCMD_InputToMlX100();
        MachineCMD_ClearInput();
        MachineCMD_EnterPage(MACHINE_CMD_PAGE_DISP_RUNNING);
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
 * @note 每次开关变化后立即调用 MachineCMD_ApplyManualOutputs() 同步到底层 GPIO。
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
    MachineCMD_ApplyManualOutputs();
}

/**
 * @brief 将手动调试逻辑开关同步到 GPIO 输出。
 *
 * @note 这里是目前唯一直接碰手动输出 GPIO 的地方。
 *       后续如果独立出阀门 BSP 或泵 BSP，应优先只替换本函数内部实现，
 *       不让 LCD 页面和按键处理代码直接依赖具体引脚。
 */
static void MachineCMD_ApplyManualOutputs(void)
{
    uint8_t water_valve_on;
    uint8_t med_valve_on;
    uint8_t pump_on;

    /*
     * 三路已确认硬件输出的临时映射：
     * - 水进/水出打开阀 1；
     * - 药进/药出打开阀 2；
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

    HAL_GPIO_WritePin(MACHINE_CMD_VALVE_WATER_PORT,
                      MACHINE_CMD_VALVE_WATER_PIN,
                      water_valve_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MACHINE_CMD_VALVE_MED_PORT,
                      MACHINE_CMD_VALVE_MED_PIN,
                      med_valve_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MACHINE_CMD_WATER_PUMP_PORT,
                      MACHINE_CMD_WATER_PUMP_PIN,
                      pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
 * @brief 获取手动调试页当前动作的 GB2312 文案。
 *
 * @return 当前动作对应的文案。
 */
static const MachineCmdText_s *MachineCMD_GetManualActionText(void)
{
    const MachineCmdText_s *text = &machine_cmd_text_idle;

    switch (machine_cmd.manual_action)
    {
    case MACHINE_CMD_MANUAL_ACTION_IN_TANK:
        text = &machine_cmd_text_in_tank;
        break;

    case MACHINE_CMD_MANUAL_ACTION_OUT_TANK:
        text = &machine_cmd_text_out_tank;
        break;

    case MACHINE_CMD_MANUAL_ACTION_NEEDLE_IN:
        text = &machine_cmd_text_needle_in;
        break;

    case MACHINE_CMD_MANUAL_ACTION_NEEDLE_OUT:
        text = &machine_cmd_text_needle_out;
        break;

    case MACHINE_CMD_MANUAL_ACTION_DRAW_MED:
        text = &machine_cmd_text_draw_med;
        break;

    case MACHINE_CMD_MANUAL_ACTION_EXHAUST:
        text = &machine_cmd_text_exhaust;
        break;

    case MACHINE_CMD_MANUAL_ACTION_REMOTE:
        text = &machine_cmd_text_remote;
        break;

    case MACHINE_CMD_MANUAL_ACTION_WATER_IN:
        text = &machine_cmd_text_water_in;
        break;

    case MACHINE_CMD_MANUAL_ACTION_MED_IN:
        text = &machine_cmd_text_med_in;
        break;

    case MACHINE_CMD_MANUAL_ACTION_WATER_OUT:
        text = &machine_cmd_text_water_out;
        break;

    case MACHINE_CMD_MANUAL_ACTION_MED_OUT:
        text = &machine_cmd_text_med_out;
        break;

    case MACHINE_CMD_MANUAL_ACTION_IDLE:
    default:
        break;
    }

    return text;
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
 * @note 当前浓度、余量、体积先使用占位值。后续接入活度计和业务状态后，
 *       只需要替换对应 GB2312 + ASCII 混合行内容。
 */
static void MachineCMD_ShowStandbyPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_standby);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_conc);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_left);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_volume);
}

/**
 * @brief 显示配药参数输入页。
 *
 * @note 第 2 行显示当前正在输入的瓶号，第 3 行显示已经确认的前两个瓶号整数 ml。
 *       动态数字仍用 ASCII，中文前缀使用 GB2312 字节。
 */
static void MachineCMD_ShowPrepSettingPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    char ascii[12];

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_prep_title);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_bottle);
    (void)snprintf(ascii, sizeof(ascii), "%u:", (unsigned int)(machine_cmd.prep_index + 1U));
    offset = MachineCMD_LineAppendString(line, offset, ascii);
    offset = MachineCMD_LineAppendString(line, offset, machine_cmd.input[0] != '\0' ? machine_cmd.input : "__");
    (void)MachineCMD_LineAppendString(line, offset, "ml");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_saved);
    (void)snprintf(ascii, sizeof(ascii), "%u/%u",
                   (unsigned int)(machine_cmd.prep_bottle_ml_x100[0] / 100U),
                   (unsigned int)(machine_cmd.prep_bottle_ml_x100[1] / 100U));
    (void)MachineCMD_LineAppendString(line, offset, ascii);
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_3, line, sizeof(line));

    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_start);
}

/**
 * @brief 显示配药运行提示页。
 *
 * @note 目前这里只做 UI 提示，真正配药流程后续应由 machine 层状态机接管。
 */
static void MachineCMD_ShowPrepRunningPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_prep_run);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, NULL);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_put_tank);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_start_ok);
}

/**
 * @brief 显示发药参数输入页。
 */
static void MachineCMD_ShowDispSettingPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_disp_title);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_target);
    offset = MachineCMD_LineAppendString(line, offset, machine_cmd.input[0] != '\0' ? machine_cmd.input : "____");
    (void)MachineCMD_LineAppendString(line, offset, "ml");
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, NULL);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_start);
}

/**
 * @brief 显示发药运行提示页。
 *
 * @note 当前还没有接入真实流程进度，先按 LCD 方案显示固定进度占位。
 */
static void MachineCMD_ShowDispRunningPage(void)
{
    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_disp_run);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_2, &machine_cmd_text_pump2);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_progress);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_dont_move);
}

/**
 * @brief 显示手动调试页。
 *
 * @note 第 3、4 行给出四个数字键复用说明：
 *       1=水进，2=药进，4=水出，5=药出。
 */
static void MachineCMD_ShowManualPage(void)
{
    uint8_t line[MACHINE_CMD_LCD_LINE_BYTES];
    uint8_t offset;
    const MachineCmdText_s *text;

    MachineCMD_WriteText(DISPLAY_LCD_ROW_1, &machine_cmd_text_manual);

    memset(line, ' ', sizeof(line));
    offset = MachineCMD_LineAppendText(line, 0U, &machine_cmd_text_current);
    text = MachineCMD_GetManualActionText();
    (void)MachineCMD_LineAppendText(line, offset, text);
    MachineCMD_WriteBytes(DISPLAY_LCD_ROW_2, line, sizeof(line));

    MachineCMD_WriteText(DISPLAY_LCD_ROW_3, &machine_cmd_text_key_wi);
    MachineCMD_WriteText(DISPLAY_LCD_ROW_4, &machine_cmd_text_key_wo);
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
