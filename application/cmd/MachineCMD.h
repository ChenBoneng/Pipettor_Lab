//
// Created by lenovo on 26-7-30.
//

#ifndef MACHINECMD_H
#define MACHINECMD_H

#include <stdint.h>

/**
 * @brief LCD 页面状态。
 *
 * LCD 使用 12864 文本模式，每次只更新 4 行缓存内容。
 * 本枚举只描述“当前应该显示什么页面”，不直接驱动电机或泵。
 */
typedef enum
{
    MACHINE_CMD_PAGE_BOOT = 0,       // 开机初始化页
    MACHINE_CMD_PAGE_READY,          // 启动就绪确认页
    MACHINE_CMD_PAGE_STANDBY,        // 待机看板页
    MACHINE_CMD_PAGE_PREP_SETTING,   // 配药参数输入页
    MACHINE_CMD_PAGE_PREP_RUNNING,   // 配药运行提示页
    MACHINE_CMD_PAGE_PREP_MEASURE,   // 配药活度测量页
    MACHINE_CMD_PAGE_DISP_SETTING,   // 发药参数输入页
    MACHINE_CMD_PAGE_DISP_RUNNING,   // 发药运行提示页
    MACHINE_CMD_PAGE_MANUAL,         // 手动调试页
    MACHINE_CMD_PAGE_CLEAN,          // 自动清洗页
    MACHINE_CMD_PAGE_PAUSED,         // 流程暂停页
    MACHINE_CMD_PAGE_ALARM,          // 报警页
} MachineCmdPage_e;

/**
 * @brief 手动调试开关位。
 *
 * 数字键在手动调试页不再作为数值输入，而是作为独立动作开关：
 * - 数字 1：水进；
 * - 数字 2：药进；
 * - 数字 4：水出；
 * - 数字 5：药出。
 */
#define MACHINE_CMD_MANUAL_WATER_IN   (1U << 0)
#define MACHINE_CMD_MANUAL_MED_IN     (1U << 1)
#define MACHINE_CMD_MANUAL_WATER_OUT  (1U << 2)
#define MACHINE_CMD_MANUAL_MED_OUT    (1U << 3)

// 初始化按键/LCD 命令层状态，并关闭手动输出。
void MachineCMD_Init(void);

// 处理按键事件和页面状态，建议放在 MachineTask() 中周期调用。
void MachineCMD_Process(void);

// 刷新 LCD 显示内容，建议放在 LCDTask() 中周期调用。
void MachineCMD_LCDTask(void);

// 读取当前 LCD 页面，便于业务层判断当前交互状态。
MachineCmdPage_e MachineCMD_GetPage(void);

// 读取当前手动调试开关位。
uint8_t MachineCMD_GetManualSwitches(void);

#endif //MACHINECMD_H
