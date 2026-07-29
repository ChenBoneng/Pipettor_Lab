#include "machine.h"
#include "step_motor.h"

/*
 * 步进电机测试参数。
 *
 * 本测试目标：
 * 1. 电机 A 正向移动 3cm，要求 2 秒完成；
 * 2. 电机 A 停止后，电机 B 正向移动 3cm，要求 2 秒完成；
 * 3. 两个电机不会同时运行。
 *
 * 重要：步进电机驱动只能控制“脉冲数”，3cm 必须换算成脉冲数。
 *
 * 当前先按常见机械参数估算：
 * - DM542 细分：1600 脉冲/圈；
 * - 丝杆导程：8mm/圈；
 * - 所以 1mm = 1600 / 8 = 200 脉冲。
 *
 * 如果你的实际细分或导程不同，只需要修改 MACHINE_STEP_PULSE_PER_MM。
 * 例如实际为 400 脉冲/mm，则 30mm = 12000 脉冲，2 秒完成速度为 6000pps。
 */
#define MACHINE_TEST_DISTANCE_MM       30U
#define MACHINE_TEST_RUN_TIME_MS       2000U
#define MACHINE_STEP_PULSE_PER_MM      100U
#define MACHINE_TEST_STEPS             (MACHINE_TEST_DISTANCE_MM * MACHINE_STEP_PULSE_PER_MM)
#define MACHINE_TEST_SPEED_PPS         ((MACHINE_TEST_STEPS * 1000U) / MACHINE_TEST_RUN_TIME_MS)

typedef enum
{
    MACHINE_TEST_STATE_IDLE = 0,        // 空闲状态，一般用于测试完成或启动失败
    MACHINE_TEST_STATE_MOTOR_A_START,   // 启动电机 A 正向移动 3cm
    MACHINE_TEST_STATE_MOTOR_A_WAIT,    // 等待电机 A 自动完成指定步数
    MACHINE_TEST_STATE_MOTOR_B_START,   // 启动电机 B 正向移动 3cm
    MACHINE_TEST_STATE_MOTOR_B_WAIT,    // 等待电机 B 自动完成指定步数
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
         * 电机 A 正向运行 MACHINE_TEST_STEPS 个脉冲。
         * 由于 MACHINE_TEST_SPEED_PPS = 步数 / 2 秒，所以理论运行时间为 2 秒。
         * StepMotor_RunSteps() 是非阻塞函数，启动成功后马上返回。
         */
        if (StepMotor_RunSteps(STEP_MOTOR_ID_A,
                               STEP_MOTOR_DIR_FORWARD,
                               MACHINE_TEST_SPEED_PPS,
                               MACHINE_TEST_STEPS) != 0U)
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
            Machine_EnterState(MACHINE_TEST_STATE_MOTOR_B_START);
        }
        break;

    case MACHINE_TEST_STATE_MOTOR_B_START:
        /*
         * 电机 B 使用同样的距离和速度参数。
         * 由于电机 A 已确认空闲，此时不会和电机 A 抢占 TIM2。
         */
        if (StepMotor_RunSteps(STEP_MOTOR_ID_B,
                               STEP_MOTOR_DIR_FORWARD,
                               MACHINE_TEST_SPEED_PPS,
                               MACHINE_TEST_STEPS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_MOTOR_B_WAIT);
        }
        else
        {
            Machine_EnterState(MACHINE_TEST_STATE_IDLE);
            machine_test_finished = 1U;
        }
        break;

    case MACHINE_TEST_STATE_MOTOR_B_WAIT:
        if (StepMotor_IsBusy(STEP_MOTOR_ID_B) == 0U)
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
