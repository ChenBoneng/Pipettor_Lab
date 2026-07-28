//
// Created by lenovo on 26-7-26.
//

#include "Keyboard.h"
#include "main.h"

/*
 * 矩阵键盘扫描说明：
 * 1. 行引脚默认保持高电平；
 * 2. 扫描时逐行拉低，只检测当前被拉低这一行；
 * 3. 列引脚使用上拉输入，读到低电平表示当前行、当前列的按键被按下。
 */
#define KEYPAD_ROW_COUNT         5U
#define KEYPAD_COL_COUNT         6U
#define KEYPAD_PRESS_DEBOUNCE_COUNT    8U
#define KEYPAD_RELEASE_DEBOUNCE_COUNT  30U
#define KEYPAD_MULTIPLE_KEYS     0xFFU

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} Keypad_Pin_s;

static const Keypad_Pin_s keypad_rows[KEYPAD_ROW_COUNT] = {
    {GPIOB, GPIO_PIN_0},
    {GPIOB, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_3},
    {GPIOB, GPIO_PIN_4},
    {GPIOB, GPIO_PIN_5},
};

/* 列引脚：PC6-PC11，使用上拉输入，和被拉低的行组合判断具体按键。 */
static const Keypad_Pin_s keypad_cols[KEYPAD_COL_COUNT] = {
    {GPIOC, GPIO_PIN_6},
    {GPIOC, GPIO_PIN_7},
    {GPIOC, GPIO_PIN_8},
    {GPIOC, GPIO_PIN_9},
    {GPIOC, GPIO_PIN_10},
    {GPIOC, GPIO_PIN_11},
};

/*
 * 按键编号映射表：
 * 行号和列号确定一个按键编号；
 * 5 行 x 6 列共 30 个位置，每个位置都分配一个非 0 编号。
 * 0 专门保留给“当前没有按键”的状态。
 */
static const uint8_t keypad_map[KEYPAD_ROW_COUNT][KEYPAD_COL_COUNT] = {
    { 1U,  2U,  3U,  4U,  5U,  6U},
    { 7U,  8U,  9U, 10U, 11U, 12U},
    {13U, 14U, 15U, 16U, 17U, 18U},
    {19U, 20U, 21U, 22U, 23U, 24U},
    {25U, 26U, 27U, 28U, 29U, 30U},
};

/*
 * 当前最终确认得到的按键序号记录在这里：
 * - 变量名：g_keypad_now_key
 * - 取值 1-30：当前通过消抖确认的有效按键编号
 * - 取值 0：当前没有按键，或者当前状态不是单个有效按键
 *
 * 其他模块如果要读取“当前最终得到的键盘按键序号”，就读取这个变量。
 */
static volatile uint8_t g_keypad_now_key = 0U;
static volatile KeypadState_e g_keypad_now_state = KEYPAD_STATE_NONE;
static volatile KeypadState_e g_keypad_pressed_state = KEYPAD_STATE_NONE;

/*
 * 消抖状态：
 * keypad_candidate      当前候选按键；
 * keypad_stable_count   候选按键连续稳定出现的次数；
 * keypad_press_locked   已经确认一次按下后锁定，必须稳定释放后才允许下一次按下；
 * g_keypad_now_key     只在候选状态稳定后更新。
 */
static uint8_t keypad_candidate = 0U;
static uint8_t keypad_stable_count = 0U;
static uint8_t keypad_press_locked = 0U;

static KeypadState_e Keypad_KeyToState(uint8_t key)
{
    switch (key)
    {
    case 1U:
        return KEYPAD_STATE_IN_TANK;
    case 2U:
        return KEYPAD_STATE_OUT_TANK;
    case 3U:
        return KEYPAD_STATE_NUM_7;
    case 4U:
        return KEYPAD_STATE_NUM_8;
    case 5U:
        return KEYPAD_STATE_NUM_9;
    case 6U:
        return KEYPAD_STATE_NUM_0;
    case 7U:
        return KEYPAD_STATE_FAULT;
    case 8U:
        return KEYPAD_STATE_DRAW_MEDICINE;
    case 9U:
        return KEYPAD_STATE_PREPARE_MEDICINE;
    case 10U:
        return KEYPAD_STATE_SEND_MEDICINE;
    case 11U:
        return KEYPAD_STATE_EXHAUST_FIXED;
    case 12U:
        return KEYPAD_STATE_CLEAR_ALL;
    case 13U:
        return KEYPAD_STATE_INSERT_NEEDLE;
    case 14U:
        return KEYPAD_STATE_RETRACT_NEEDLE;
    case 15U:
        return KEYPAD_STATE_NUM_4;
    case 16U:
        return KEYPAD_STATE_NUM_5;
    case 17U:
        return KEYPAD_STATE_NUM_6;
    case 18U:
        return KEYPAD_STATE_DOT;
    case 21U:
        return KEYPAD_STATE_NUM_1;
    case 22U:
        return KEYPAD_STATE_NUM_2;
    case 23U:
        return KEYPAD_STATE_NUM_3;
    case 24U:
        return KEYPAD_STATE_CLEAR_INPUT;
    case 25U:
        return KEYPAD_STATE_RESET;
    case 26U:
        return KEYPAD_STATE_START;
    case 27U:
        return KEYPAD_STATE_PAUSE;
    case 28U:
        return KEYPAD_STATE_REMOTE;
    default:
        return KEYPAD_STATE_NONE;
    }
}

static void Keypad_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 配置为输出前，先确保所有行保持高电平。 */
    for (uint8_t row = 0U; row < KEYPAD_ROW_COUNT; row++)
    {
        HAL_GPIO_WritePin(keypad_rows[row].port, keypad_rows[row].pin, GPIO_PIN_SET);
    }

    /* 行：PB0/PB1/PB3/PB4/PB5，推挽输出，空闲高电平。 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 列：PC6-PC11，上拉输入，读到低电平表示按下。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                          GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static uint8_t Keypad_Scan_Raw(void)
{
    uint8_t detected_count = 0U;
    uint8_t detected_key = 0U;

    /*
     * 原始扫描只负责判断当前物理状态：
     * - 没有按键返回 0；
     * - 单个有效按键返回按键编号；
     * - 多键、鬼键、无效位置返回 KEYPAD_MULTIPLE_KEYS。
     */
    for (uint8_t row = 0U; row < KEYPAD_ROW_COUNT; row++)
    {
        HAL_GPIO_WritePin(keypad_rows[row].port, keypad_rows[row].pin, GPIO_PIN_RESET);
        __NOP();
        __NOP();

        /* 仅当前行被拉低时，读取所有列状态。 */
        for (uint8_t col = 0U; col < KEYPAD_COL_COUNT; col++)
        {
            if (HAL_GPIO_ReadPin(keypad_cols[col].port, keypad_cols[col].pin) == GPIO_PIN_RESET)
            {
                detected_count++;
                detected_key = keypad_map[row][col];
            }
        }

        HAL_GPIO_WritePin(keypad_rows[row].port, keypad_rows[row].pin, GPIO_PIN_SET);
    }

    if (detected_count == 0U)
    {
        return 0U;
    }

    /* 忽略多键、鬼键以及未使用的矩阵位置。 */
    if ((detected_count != 1U) || (detected_key == 0U))
    {
        return KEYPAD_MULTIPLE_KEYS;
    }

    return detected_key;
}

void Keypad_Init(void)
{
    Keypad_GPIO_Init();

    g_keypad_now_key = 0U;
    g_keypad_now_state = KEYPAD_STATE_NONE;
    g_keypad_pressed_state = KEYPAD_STATE_NONE;
    keypad_candidate = 0U;
    keypad_stable_count = 0U;
    keypad_press_locked = 0U;
}

void Keypad_Process(void)
{
    const uint8_t raw_key = Keypad_Scan_Raw();
    uint8_t stable_key = 0U;
    uint8_t debounce_count = KEYPAD_PRESS_DEBOUNCE_COUNT;

    /*
     * 消抖原则：
     * 只有连续多次扫描结果相同，
     * 才认为这个按键状态稳定。
     *
     * 按下和释放分开处理：
     * - 按下需要连续 KEYPAD_PRESS_DEBOUNCE_COUNT 次稳定；
     * - 释放需要连续 KEYPAD_RELEASE_DEBOUNCE_COUNT 次稳定。
     *
     * 25 号复位键之前偶发“一次按下识别多次”，本质上是机械抖动过程中
     * 出现了短暂释放再按下。这里要求必须稳定释放后才解锁下一次按下，
     * 可以避免同一次物理按压被上层判断成多次。
     */
    if (raw_key != keypad_candidate)
    {
        keypad_candidate = raw_key;
        keypad_stable_count = 1U;
        return;
    }

    if (keypad_candidate == 0U)
    {
        debounce_count = KEYPAD_RELEASE_DEBOUNCE_COUNT;
    }

    if (keypad_stable_count < debounce_count)
    {
        keypad_stable_count++;
    }

    if (keypad_stable_count < debounce_count)
    {
        return;
    }

    if (keypad_candidate == 0U)
    {
        /*
         * 只有稳定无按键达到释放消抖次数，才认为本次按压结束。
         * 释放完成后解锁，下一次按下才能重新被识别。
         */
        g_keypad_now_key = 0U;
        g_keypad_now_state = KEYPAD_STATE_NONE;
        keypad_press_locked = 0U;
        return;
    }

    if (keypad_candidate == KEYPAD_MULTIPLE_KEYS)
    {
        /*
         * 多键或鬼键不当作有效按键，也不当作稳定释放。
         * 这样按键抖动过程中短暂扫到多键，不会清掉当前按下状态。
         */
        return;
    }

    if (keypad_press_locked != 0U)
    {
        return;
    }

    if (keypad_candidate != KEYPAD_MULTIPLE_KEYS)
    {
        stable_key = keypad_candidate;
    }

    /*
     * g_keypad_now_key 表示“当前稳定按键”：
     * - 稳定无按键时写 0；
     * - 稳定单键时写对应编号；
     * - 多键或无效状态时也写 0，避免保留上一次按键。
     */
    if (g_keypad_now_key != stable_key)
    {
        g_keypad_now_key = stable_key;
        g_keypad_now_state = Keypad_KeyToState(stable_key);
        g_keypad_pressed_state = g_keypad_now_state;
        keypad_press_locked = 1U;
    }
}

/*
 * @brief 获取当前稳定的按键编号
 * @return uint8_t 当前稳定按键编号，1-30 表示有效按键，0 表示无按键或多键状态
 */
uint8_t Keypad_GetNowKey()
{
    return g_keypad_now_key;
}

/*
 * @brief 获取当前稳定按键对应的功能枚举
 * @return KeypadState_e 当前按键功能，KEYPAD_STATE_NONE 表示没有按键或未映射功能
 */
KeypadState_e Keypad_GetNowState(void)
{
    return g_keypad_now_state;
}

/*
 * @brief 获取一次稳定按下事件对应的功能枚举
 * @return KeypadState_e 当前按下事件，读取后清除；没有新按下事件时返回 KEYPAD_STATE_NONE
 */
KeypadState_e Keypad_GetPressedState(void)
{
    KeypadState_e state = g_keypad_pressed_state;

    g_keypad_pressed_state = KEYPAD_STATE_NONE;
    return state;
}
