#include "machine.h"
#include "bsp_dwt.h"
#include "pump_drive.h"

/*
 * 电机测试参数：
 * - 速度 2000pps；
 * - 正转 2 秒，所以步数 = 2000 * 2 = 4000 step；
 * - 停止 1 秒；
 * - 反转 2 秒，所以步数同样为 4000 step。
 *
 * 当前约定：
 * - PumpDrive_MoveOut() 对应正转；
 * - PumpDrive_MoveIn()  对应反转。
 */
#define MACHINE_TEST_SPEED_PPS        2000U
#define MACHINE_TEST_FORWARD_MS       2000U
#define MACHINE_TEST_STOP_MS          1000U
#define MACHINE_TEST_REVERSE_MS       2000U
#define MACHINE_TEST_FORWARD_STEPS    ((MACHINE_TEST_SPEED_PPS * MACHINE_TEST_FORWARD_MS) / 1000U)
#define MACHINE_TEST_REVERSE_STEPS    ((MACHINE_TEST_SPEED_PPS * MACHINE_TEST_REVERSE_MS) / 1000U)
#define MACHINE_PUMP_DEVICE_ID        1U

/*
 * 如果你的 ISC1000 接的是 RS485 总线，把这里改成 PUMP_DRIVE_BUS_RS485。
 * 当前先按点对点串口模式处理，命令格式为：set spd=2000\n。
 */
#define MACHINE_PUMP_BUS_MODE         PUMP_DRIVE_BUS_RS485

typedef enum
{
    MACHINE_TEST_STATE_IDLE = 0,       // 空闲状态，通常表示初始化失败或测试未开始
    MACHINE_TEST_STATE_FORWARD_START,  // 发送正转命令
    MACHINE_TEST_STATE_FORWARD_WAIT,   // 等待正转 2 秒结束
    MACHINE_TEST_STATE_STOP_WAIT,      // 中间停止 1 秒
    MACHINE_TEST_STATE_REVERSE_START,  // 发送反转命令
    MACHINE_TEST_STATE_REVERSE_WAIT,   // 等待反转 2 秒结束
    MACHINE_TEST_STATE_FINISHED,       // 测试完成，保持停止
} MachineTestState_e;

static PumpDrive_s machine_pump;
static MachineTestState_e machine_test_state = MACHINE_TEST_STATE_IDLE;
static uint32_t machine_state_start_ms = 0U;
static uint8_t machine_pump_ready = 0U;

static uint8_t Machine_IsTimeReached(uint32_t duration_ms)
{
    uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();

    return ((uint32_t)(now_ms - machine_state_start_ms) >= duration_ms) ? 1U : 0U;
}

void MachineInit(void)
{
    /*
     * 初始化 ISC1000 驱动。
     * PumpDrive_Init() 内部固定注册 USART3，不会占用其它串口。
     */
    machine_pump_ready = PumpDrive_Init(&machine_pump,
                                        MACHINE_PUMP_BUS_MODE,
                                        MACHINE_PUMP_DEVICE_ID);

    if (machine_pump_ready == 0U)
    {
        machine_test_state = MACHINE_TEST_STATE_IDLE;
        return;
    }

    /*
     * 上电后先使能电机并设置测试速度。
     * 后续正反转都使用同一个 2000pps 速度。
     */
    (void)PumpDrive_Enable(&machine_pump);
    (void)PumpDrive_SetSpeed(&machine_pump, MACHINE_TEST_SPEED_PPS);

    machine_state_start_ms = (uint32_t)DWT_GetTimeline_ms();
    machine_test_state = MACHINE_TEST_STATE_FORWARD_START;
}


void MachineControl(void)
{
    if (machine_pump_ready == 0U)
    {
        return;
    }

    switch (machine_test_state)
    {
    case MACHINE_TEST_STATE_FORWARD_START:
        /*
         * 正转测试：2000pps * 2s = 4000 step。
         * 使用步数命令比“发送后强行延时再急停”更清楚，也更符合驱动器协议。
         */
        (void)PumpDrive_MoveOut(&machine_pump, MACHINE_TEST_FORWARD_STEPS);
        machine_state_start_ms = (uint32_t)DWT_GetTimeline_ms();
        machine_test_state = MACHINE_TEST_STATE_FORWARD_WAIT;
        break;

    case MACHINE_TEST_STATE_FORWARD_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_FORWARD_MS) != 0U)
        {
            /*
             * 正转时间到后发送减速停止命令。
             * 理论上 4000 step 已经正好跑完，这里再发 stop 是为了让测试状态明确进入停止段。
             */
            (void)PumpDrive_Stop(&machine_pump, 0U);
            machine_state_start_ms = (uint32_t)DWT_GetTimeline_ms();
            machine_test_state = MACHINE_TEST_STATE_STOP_WAIT;
        }
        break;

    case MACHINE_TEST_STATE_STOP_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_STOP_MS) != 0U)
        {
            machine_test_state = MACHINE_TEST_STATE_REVERSE_START;
        }
        break;

    case MACHINE_TEST_STATE_REVERSE_START:
        /* 反转测试：仍然是 2000pps，持续 2 秒，对应 4000 step。 */
        (void)PumpDrive_MoveIn(&machine_pump, MACHINE_TEST_REVERSE_STEPS);
        machine_state_start_ms = (uint32_t)DWT_GetTimeline_ms();
        machine_test_state = MACHINE_TEST_STATE_REVERSE_WAIT;
        break;

    case MACHINE_TEST_STATE_REVERSE_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_REVERSE_MS) != 0U)
        {
            (void)PumpDrive_Stop(&machine_pump, 0U);
            machine_test_state = MACHINE_TEST_STATE_FINISHED;
        }
        break;

    case MACHINE_TEST_STATE_FINISHED:
        /* 测试完成后保持停止状态，不重复发送命令。 */
        break;

    case MACHINE_TEST_STATE_IDLE:
    default:
        break;
    }
}
