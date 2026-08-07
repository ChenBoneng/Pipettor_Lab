#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>

void MachineInit(void);
void MachineControl(void);

/* 上位机急停入口：立即停止输出并终止当前整体流程，不执行复位收尾。 */
void Machine_EmergencyStop(void);
void Machine_ResetRuntimeState(void);

/* 清除远控流程结束/失败后的短暂步骤保持，用于上位机复位后立即回到 IDLE 步骤。 */
void Machine_ClearRemoteStepHold(void);

/* 判断当前组合流程是否正在运行。 */
uint8_t MachineCombinationTestIsRunning(void);

/* 判断当前组合流程是否已经走到允许输入发药量的阶段。 */
uint8_t MachineCombinationTestCanDispense(void);

/* 读取并清除“本地组合流程最终活度等待已经完成”事件。 */
uint8_t Machine_ConsumeCombinationFinalActivityReady(void);

/* 读取并清除“本地发药已经成功完成”事件。 */
uint8_t Machine_ConsumeLocalDispenseCompleted(void);

/* 上位机启动配药流程，体积单位均为 0.01ml。 */
uint8_t Machine_StartRemotePrepare(uint16_t water_volume_x100,
                                   uint16_t final_volume_x100,
                                   uint16_t initial_activity_x100,
                                   uint16_t target_conc_x1000,
                                   uint8_t seq);

/* 上位机按老板最新流程启动完整配药，药瓶量和体积单位均为 0.01ml。 */
uint8_t Machine_StartRemotePrepareByBottle(uint16_t bottle1_ml_x100,
                                           uint16_t bottle2_ml_x100,
                                           uint16_t water_volume_x100,
                                           uint16_t final_volume_x100,
                                           uint16_t initial_activity_x100,
                                           uint16_t target_conc_x1000,
                                           uint8_t seq);

/* 上位机启动独立发药流程，volume_ml_x100 单位为 0.01ml，seq 用于完成 ACK 回传。 */
uint8_t Machine_StartRemoteDispense(uint16_t volume_ml_x100, uint8_t seq);

/* 上位机启动单独冲洗流程，seq 用于 PROCESS_RESULT 回传。 */
uint8_t Machine_StartRemoteFlush(uint8_t seq);

/* 上位机启动单独排气流程，seq 用于 PROCESS_RESULT 回传。 */
uint8_t Machine_StartRemoteExhaust(uint8_t seq);
uint8_t Machine_ConfirmRemoteBottleChanged(uint8_t bottle_index,
                                           uint8_t confirm_step,
                                           uint8_t *ack_result,
                                           uint16_t *ack_error);

/* 上位机启动“转移药液进活度计”流程，执行至原液活度读取完成后等待开始配药。 */
uint8_t Machine_StartRemoteTransferToActivity(void);

/* 判断药液是否已经完成转移到活度计，可用于 READ_ACTIVITY 和配药启动保护。 */
uint8_t Machine_IsTransferToActivityDone(void);

/* 读取当前流程步骤，供 0x181 状态帧 Byte1 使用。 */
uint8_t Machine_GetCommunicationStep(void);

/* 读取当前或最近一次发药进度百分比，0~100。 */
uint8_t Machine_GetDispenseProgressPercent(void);

/* 判断 machine 当前是否有自动流程正在运行。 */
uint8_t Machine_IsFlowRunning(void);

#endif /* MACHINE_H */
