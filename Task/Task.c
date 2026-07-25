//
// Created by lenovo on 26-7-25.
//

#include "Task.h"
#include "cmsis_os2.h"
#include "main.h"

#define KEYPAD_ROW_COUNT         5U
#define KEYPAD_COL_COUNT         6U
#define KEYPAD_DEBOUNCE_COUNT    4U
#define KEYPAD_MULTIPLE_KEYS     0xFFU
#define KEYPAD_SCAN_PERIOD_MS    1U

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

static const Keypad_Pin_s keypad_cols[KEYPAD_COL_COUNT] = {
    {GPIOB, GPIO_PIN_6},
    {GPIOB, GPIO_PIN_7},
    {GPIOB, GPIO_PIN_8},
    {GPIOB, GPIO_PIN_9},
    {GPIOB, GPIO_PIN_10},
    {GPIOB, GPIO_PIN_11},
};

static const uint8_t keypad_map[KEYPAD_ROW_COUNT][KEYPAD_COL_COUNT] = {
    { 1U,  2U,  3U,  4U,  5U,  6U},
    { 7U,  8U,  9U, 10U, 11U, 12U},
    {13U, 14U, 15U, 16U, 17U, 18U},
    { 0U,  0U, 19U, 20U, 21U, 22U},
    {23U, 24U, 25U, 26U,  0U,  0U},
};

volatile uint8_t g_keypad_last_key = 0U;

static uint8_t keypad_candidate = 0U;
static uint8_t keypad_stable_count = 0U;
static uint8_t keypad_ready_for_press = 1U;

static void Keypad_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    for (uint8_t row = 0U; row < KEYPAD_ROW_COUNT; row++)
    {
        HAL_GPIO_WritePin(keypad_rows[row].port, keypad_rows[row].pin, GPIO_PIN_SET);
    }

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                          GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static uint8_t Keypad_Scan_Raw(void)
{
    uint8_t detected_count = 0U;
    uint8_t detected_key = 0U;

    for (uint8_t row = 0U; row < KEYPAD_ROW_COUNT; row++)
    {
        HAL_GPIO_WritePin(keypad_rows[row].port, keypad_rows[row].pin, GPIO_PIN_RESET);
        __NOP();
        __NOP();

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

    if ((detected_count != 1U) || (detected_key == 0U))
    {
        return KEYPAD_MULTIPLE_KEYS;
    }

    return detected_key;
}

static void Keypad_Init(void)
{
    Keypad_GPIO_Init();

    g_keypad_last_key = 0U;
    keypad_candidate = 0U;
    keypad_stable_count = 0U;
    keypad_ready_for_press = 1U;
}

static void Keypad_Process(void)
{
    const uint8_t raw_key = Keypad_Scan_Raw();

    if (raw_key != keypad_candidate)
    {
        keypad_candidate = raw_key;
        keypad_stable_count = 1U;
        return;
    }

    if (keypad_stable_count < KEYPAD_DEBOUNCE_COUNT)
    {
        keypad_stable_count++;
    }

    if (keypad_stable_count < KEYPAD_DEBOUNCE_COUNT)
    {
        return;
    }

    if (keypad_candidate == 0U)
    {
        keypad_ready_for_press = 1U;
    }
    else if ((keypad_candidate != KEYPAD_MULTIPLE_KEYS) && (keypad_ready_for_press != 0U))
    {
        g_keypad_last_key = keypad_candidate;
        keypad_ready_for_press = 0U;
    }
}


void KeyboardTask(void *argument)
{
    (void)argument;

    Keypad_Init();

    for(;;)
    {
        Keypad_Process();
        osDelay(KEYPAD_SCAN_PERIOD_MS);
    }

}
