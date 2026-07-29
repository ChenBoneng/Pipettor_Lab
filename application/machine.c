#include "machine.h"
#include "step_motor.h"

/*
 * 步进电机测试参数。
 *
 * 本测试目标：
 * 1. 只让电机 A 按“拉”方向运行；
 * 2. 按丝杆导程换算接口移动 3cm；
 * 3. 电机 B 不参与本次测试。
 */
#define MACHINE_TEST_DISTANCE_MM_X100  3000U
#define MACHINE_TEST_SPEED_MM_S_X100   100U

typedef enum
{
    MACHINE_TEST_STATE_IDLE = 0,        // 空闲状态，一般用于测试完成或启动失败
    MACHINE_TEST_STATE_MOTOR_A_START,   // 启动电机 A 拉 3cm
    MACHINE_TEST_STATE_MOTOR_A_WAIT,    // 等待电机 A 自动完成指定步数
    MACHINE_TEST_STATE_FINISHED,        // 测试流程完成
} MachineTestState_e;

static MachineTestState_e machine_test_state = MACHINE_TEST_STATE_IDLE;
static uint8_t machine_test_finished = 0U;

static void Machine_EnterState(MachineTestState_e next_state)
{
    machine_test_state = next_state;
}

void MachineInit(void)
{
    /*
     * StepMotor_Init() 已经在 AllTaskInit() 中调用。
     * 这里仅初始化 machine 层测试状态机，不重复初始化底层硬件。
     */
    machine_test_finished = 0U;
    Machine_EnterState(MACHINE_TEST_STATE_MOTOR_A_START);
}

void MachineControl(void)
{
    if (machine_test_finished != 0U)
    {
        return;
    }

    switch (machine_test_state)
    {
    case MACHINE_TEST_STATE_MOTOR_A_START:
        /*
         * 电机 A 按“拉”方向运行 3cm。
         * StepMotor_RunDistanceMmX100() 会按 4mm 导程在模块层换算出脉冲数和 PPS。
         */
        if (StepMotor_RunDistanceMmX100(STEP_MOTOR_ID_A,
                                        STEP_MOTOR_DIR_PULL,
                                        MACHINE_TEST_DISTANCE_MM_X100,
                                        MACHINE_TEST_SPEED_MM_S_X100) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_MOTOR_A_WAIT);
        }
        else
        {
            Machine_EnterState(MACHINE_TEST_STATE_IDLE);
            machine_test_finished = 1U;
        }
        break;

    case MACHINE_TEST_STATE_MOTOR_A_WAIT:
        /*
         * 不使用延时等待，而是读取步进驱动状态。
         * 电机 A 达到目标步数后，step_motor 会在 TIM2 中断中自动停止 PWM。
         */
        if (StepMotor_IsBusy(STEP_MOTOR_ID_A) == 0U)
        {
            StepMotor_StopAll();
            Machine_EnterState(MACHINE_TEST_STATE_FINISHED);
            machine_test_finished = 1U;
        }
        break;

    case MACHINE_TEST_STATE_FINISHED:
    case MACHINE_TEST_STATE_IDLE:
    default:
        break;
    }
}
