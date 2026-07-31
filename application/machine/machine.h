//
// Created by lenovo on 26-7-27.
//

#ifndef MACHINE_H
#define MACHINE_H

#include <stdint.h>

void MachineInit(void);
void MachineControl(void);

// 启动阀门/抽水泵顺序测试：阀1、阀2、抽水泵依次开关，重复 5 次。
void MachineOutputTestStart(void);

// 判断阀门/抽水泵顺序测试是否正在运行。
uint8_t MachineOutputTestIsRunning(void);

#endif //MACHINE_H
