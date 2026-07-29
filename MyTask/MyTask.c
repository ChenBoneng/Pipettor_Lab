//
// 创建时间：26-7-25
//

#include "MyTask.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Keyboard.h"
#include "Communication.h"
#include "machine.h"
#include "step_motor.h"

void AllTaskInit(void)
{
    DWT_Init(72); // 初始化DWT,用于获取时间间隔
    Keypad_Init();
    Communication_Init();
    StepMotor_Init();
    MachineInit();
}


void KeyboardTask(void *argument)
{
    (void)argument;

    for(;;)
    {
        /* 每 1ms 扫描一次，连续 4 次稳定后确认按键。 */
        Keypad_Process();
        osDelay(1);
    }

}

void MachineTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* 处理机器人的状态机逻辑 */
        // MachineControl();


        DWT_SysTimeUpdate();
        osDelay(2); // 每 2ms 执行一次
    }
}
