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

#endif //MACHINE_H
