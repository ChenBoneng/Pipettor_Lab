//
// 创建时间：26-7-25
//

#include "MyTask.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Keyboard.h"
#include "Communication.h"
#include "display_lcd.h"
#include "MachineCMD.h"
#include "machine.h"
#include "io_output.h"
#include "solenoid_valve.h"
#include "step_motor.h"
#include "water_pump.h"
#include "activity_meter.h"
#include "pump_drive.h"

void AllTaskInit(void)
{
    DWT_Init(72); // 初始化DWT,用于获取时间间隔
    DisplayLcd_Init();
    Keypad_Init();
    Communication_Init();
    StepMotor_Init();
    IoOutput_Init();
    SolenoidValve_Init();
    WaterPump_Init();
    ActivityMeter_Init();
    (void)PumpDrive_BoardInit();
    MachineCMD_Init();
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
        /* 处理机器主流程状态机，不直接扫描按键或维护底层模块。 */
        MachineControl();
        osDelay(2);
    }
}

void LCDTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        MachineCMD_LCDTask();

        osDelay(100); // 每 100ms 刷新一次 LCD 内容
    }
}

void MachineCMDTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /*
         * 处理控制信号：
         * - 当前先处理矩阵键盘产生的一次性按键事件；
         * - 后续上位机 CAN 指令、远控接管也放在这里统一分发。
         */
        MachineCMD_Process();
        osDelay(2);
    }
}

void ModuleTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /*
         * 底层模块周期维护任务：
         * - StepMotor_Process() 负责步进电机加减速状态维护；
         * - ActivityMeter_Process() 负责活度计轮询和超时维护；
         * - PumpDrive_Process() 负责 ISC1000 总线等待超时维护；
         * - DWT_SysTimeUpdate() 周期更新时间轴，防止 CYCCNT 长时间无人读取。
         *
         * 这里不处理业务流程，也不处理按键/CAN 控制命令。
         */
        StepMotor_Process();
        ActivityMeter_Process();
        PumpDrive_Process();
        DWT_SysTimeUpdate();

        osDelay(2);
    }
}
