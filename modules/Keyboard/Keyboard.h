//
// Created by lenovo on 26-7-26.
//

#ifndef KEYBOARD_H
#define KEYBOARD_H
#include <stdint.h>


//矩阵键盘初始化
void Keypad_Init(void);
//轮询检测矩阵键盘
void Keypad_Process(void);
//获取当前稳定的按键编号,1-30 表示有效按键，0 表示无按键或多键状态
uint8_t Keypad_GetNowKey(void);

#endif //KEYBOARD_H
