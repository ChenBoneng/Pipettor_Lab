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
    MACHINE_CMD_PAGE_REMOTE,         // 上位机远控接管页
    MACHINE_CMD_PAGE_MANUAL,         // 手动调试页
    MACHINE_CMD_PAGE_DEBUG,          // 故障键进入的设备调试页
    MACHINE_CMD_PAGE_CLEAN,          // 自动清洗页
    MACHINE_CMD_PAGE_PAUSED,         // 流程暂停页
    MACHINE_CMD_PAGE_ALARM,          // 报警页
} MachineCmdPage_e;

/**
 * @brief 本机配药 UI 运行阶段。
 *
 * @note 本枚举只描述 LCD 应显示的阶段，不直接驱动电机、泵或阀。
 *       后续 machine 层接入真实流程时，可按状态机阶段调用 MachineCMD_SetPrepRunStage()。
 */
typedef enum
{
    MACHINE_CMD_PREP_RUN_STAGE_READY = 0U,         /**< 操作员确认 1 号罐、水和接药杯。 */
    MACHINE_CMD_PREP_RUN_STAGE_IN_TANK,           /**< 进罐导轨运行。 */
    MACHINE_CMD_PREP_RUN_STAGE_INSERT_NEEDLE,      /**< 插针导轨运行。 */
    MACHINE_CMD_PREP_RUN_STAGE_DRAW_WATER,         /**< 抽水泵抽液到活度计。 */
    MACHINE_CMD_PREP_RUN_STAGE_WATER_FILL,         /**< 泵1补水，done/total 显示补水量。 */
    MACHINE_CMD_PREP_RUN_STAGE_ACTIVITY_CHECK,     /**< 活度计稳定读数。 */
    MACHINE_CMD_PREP_RUN_STAGE_SWITCH_TANK,        /**< 当前药罐完成，等待换下一罐。 */
    MACHINE_CMD_PREP_RUN_STAGE_EXHAUST,            /**< 泵2排气。 */
    MACHINE_CMD_PREP_RUN_STAGE_FLUSH,              /**< 泵1冲洗。 */
    MACHINE_CMD_PREP_RUN_STAGE_DONE,               /**< 配药完成，等待发药。 */
    MACHINE_CMD_PREP_RUN_STAGE_ABORTING            /**< 正在中止并等待收尾动作。 */
} MachineCmdPrepRunStage_e;

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

// 判断当前是否处于上位机远控接管模式。
uint8_t MachineCMD_IsRemoteMode(void);

// 读取并清除“复位键已经按下”事件，machine 层用它终止当前自动流程。
uint8_t MachineCMD_ConsumeResetRequested(void);

// 读取并清除“配药参数已经确认”事件，浓度单位 0.001mCi/ml，体积单位 0.01ml。
uint8_t MachineCMD_ConsumePrepConfirmed(uint16_t *current_conc_x1000,
                                        uint16_t *current_ml_x100,
                                        uint16_t *target_conc_x1000);

// 读取并清除“发药量已经确认”事件，volume_ml_x100 单位为 0.01ml。
uint8_t MachineCMD_ConsumeDispenseConfirmed(uint16_t *volume_ml_x100);

/* 本机 UI 事件接口：MachineControl() 周期消费后启动对应 machine 流程。 */
uint8_t MachineCMD_ConsumeLocalPrepStartRequested(uint8_t *bottle_count,
                                                  uint16_t *bottle1_ml_x100,
                                                  uint16_t *bottle2_ml_x100);
uint8_t MachineCMD_ConsumeLocalPrepSwitchRequested(uint8_t *next_bottle_index);
uint8_t MachineCMD_ConsumeLocalDispenseStartRequested(uint16_t *volume_ml_x100);
uint8_t MachineCMD_ConsumeLocalExhaustRequested(void);
uint8_t MachineCMD_ConsumeLocalEmptyRequested(void);

/* 本机 UI 阶段显示接口：machine 状态机按真实流程阶段更新页面和进度。 */
void MachineCMD_SetPrepRunStage(MachineCmdPrepRunStage_e stage,
                                uint16_t done_ml_x100,
                                uint16_t total_ml_x100);
void MachineCMD_SetPrepSwitchTank(uint8_t done_bottle_index, uint8_t next_bottle_index);
void MachineCMD_SetPrepFinished(uint16_t final_conc_x1000);
void MachineCMD_ReturnToStandby(void);

/* 待机页库存状态接口，供 machine 层在流程完成时同步真实数据。 */
void MachineCMD_SetStandbyInventory(uint16_t conc_x1000,
                                    uint16_t activity_x100,
                                    uint16_t volume_ml_x100);
void MachineCMD_ConsumeStandbyInventory(uint16_t volume_ml_x100);
uint16_t MachineCMD_GetStandbyVolumeMlX100(void);

/* 远控发药完成专用上报：0x181/0x23 -> 0x183/07 01 -> 0x181/0x23。 */
void MachineCMD_ReportRemoteDispenseDoneActivity(void);

#endif //MACHINECMD_H
