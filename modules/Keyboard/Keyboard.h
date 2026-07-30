//
// Created by lenovo on 26-7-26.
//

#ifndef KEYBOARD_H
#define KEYBOARD_H
#include <stdint.h>

/**
 * @brief 矩阵键盘功能状态枚举。
 *
 * 枚举值直接使用物理按键编号，方便调试时把 Keypad_GetNowKey() 的数字
 * 和 Keypad_GetNowState() 的功能状态对应起来。
 */
typedef enum
{
    KEYPAD_STATE_NONE = 0U,              // 当前没有稳定按键，或按键未映射具体功能

    KEYPAD_STATE_IN_TANK = 1U,           // 进罐
    KEYPAD_STATE_OUT_TANK = 2U,          // 出罐
    KEYPAD_STATE_NUM_7 = 3U,             // 数字 7
    KEYPAD_STATE_NUM_8 = 4U,             // 数字 8
    KEYPAD_STATE_NUM_9 = 5U,             // 数字 9
    KEYPAD_STATE_NUM_0 = 6U,             // 数字 0

    KEYPAD_STATE_FAULT = 7U,             // 故障
    KEYPAD_STATE_DRAW_MEDICINE = 8U,     // 抽药
    KEYPAD_STATE_PREPARE_MEDICINE = 9U,  // 配药
    KEYPAD_STATE_SEND_MEDICINE = 10U,    // 发药
    KEYPAD_STATE_EXHAUST_FIXED = 11U,    // 定量排气
    KEYPAD_STATE_CLEAR_ALL = 12U,        // 清空

    KEYPAD_STATE_INSERT_NEEDLE = 13U,    // 插针
    KEYPAD_STATE_RETRACT_NEEDLE = 14U,   // 收针
    KEYPAD_STATE_NUM_4 = 15U,            // 数字 4，手动调试模式下为水出
    KEYPAD_STATE_NUM_5 = 16U,            // 数字 5，手动调试模式下为药出
    KEYPAD_STATE_NUM_6 = 17U,            // 数字 6
    KEYPAD_STATE_DOT = 18U,              // 小数点 .

    KEYPAD_STATE_NUM_1 = 21U,            // 数字 1,手动调试模式下为水进
    KEYPAD_STATE_NUM_2 = 22U,            // 数字 2，手动调试模式下为药进
    KEYPAD_STATE_NUM_3 = 23U,            // 数字 3
    KEYPAD_STATE_CLEAR_INPUT = 24U,      // 清除
    KEYPAD_STATE_RESET = 25U,            // 复位
    KEYPAD_STATE_START = 26U,            // 启动
    KEYPAD_STATE_PAUSE = 27U,            // 暂停
    KEYPAD_STATE_REMOTE = 28U,           // 远控
} KeypadState_e;

//矩阵键盘初始化
void Keypad_Init(void);
//轮询检测矩阵键盘
void Keypad_Process(void);
//获取当前稳定的按键编号,1-30 表示有效按键，0 表示无按键或多键状态
uint8_t Keypad_GetNowKey(void);
//获取当前稳定按键对应的功能枚举
KeypadState_e Keypad_GetNowState(void);
//获取一次按下事件对应的功能枚举，读取后自动清除，适合业务层做单次触发
KeypadState_e Keypad_GetPressedState(void);

#endif //KEYBOARD_H
