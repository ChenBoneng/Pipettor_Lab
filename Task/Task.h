//
// 创建时间：26-7-25
//

#ifndef TASK_H
#define TASK_H

#include <stdint.h>

/*
 * 记录当前最终确认得到的按键序号：
 * - 1-30 表示当前通过消抖确认的有效按键编号；
 * - 0 表示当前没有按键，或者当前不是单个有效按键。
 */
extern volatile uint8_t g_keypad_last_key;

#endif /* TASK_H 结束 */
