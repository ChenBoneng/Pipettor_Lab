//
// Created by lenovo on 26-7-27.
//

#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>

void MachineInit(void);
void MachineControl(void);

// 判断当前组合测试流程是否正在运行。
uint8_t MachineCombinationTestIsRunning(void);

// 判断当前组合测试流程是否已经走到允许输入发药量的阶段。
uint8_t MachineCombinationTestCanDispense(void);

// 读取并清除“本地组合流程最终活度等待已经完成”事件。
uint8_t Machine_ConsumeCombinationFinalActivityReady(void);

// 读取并清除“本地发药已经成功完成”事件。
uint8_t Machine_ConsumeLocalDispenseCompleted(void);

// 上位机启动配药流程，体积单位均为 0.01ml。
uint8_t Machine_StartRemotePrepare(uint16_t water_volume_x100,
                                   uint16_t final_volume_x100,
                                   uint16_t initial_activity_x100,
                                   uint16_t target_conc_x1000,
                                   uint8_t seq);

// 上位机启动独立发药流程，volume_ml_x100 单位为 0.01ml，seq 用于完成 ACK 回传。
uint8_t Machine_StartRemoteDispense(uint16_t volume_ml_x100, uint8_t seq);

// 上位机启动“转移药液进活度计”流程，执行至原液活度读取完成后等待开始配药。
uint8_t Machine_StartRemoteTransferToActivity(void);

// 判断药液是否已经完成转移到活度计，可用于 READ_ACTIVITY 和配药启动保护。
uint8_t Machine_IsTransferToActivityDone(void);

// 读取当前流程步骤，供 0x181 状态帧 Byte1 使用。
uint8_t Machine_GetCommunicationStep(void);

// 读取当前或最近一次发药进度百分比，0~100。
uint8_t Machine_GetDispenseProgressPercent(void);

// 判断 machine 当前是否有自动流程正在运行。
uint8_t Machine_IsFlowRunning(void);

#endif //MACHINE_H
