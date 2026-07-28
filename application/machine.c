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
#define MACHINE_CMD_GAP_MS            50U
#define MACHINE_TEST_REPEAT_COUNT     5U

/*
 * 如果你的 ISC1000 接的是 RS485 总线，把这里改成 PUMP_DRIVE_BUS_RS485。
 * 当前使用 RS485 模式，命令格式为：1 set spd=2000\n。
 */
#define MACHINE_PUMP_BUS_MODE         PUMP_DRIVE_BUS_RS485

typedef enum
{
    MACHINE_TEST_STATE_IDLE = 0,          // 空闲状态，通常表示初始化失败
    MACHINE_TEST_STATE_ENABLE,            // 发送电机使能命令
    MACHINE_TEST_STATE_ENABLE_WAIT,       // 等待驱动器解析使能命令
    MACHINE_TEST_STATE_SET_SPEED,         // 发送 2000pps 速度设置命令
    MACHINE_TEST_STATE_SET_SPEED_WAIT,    // 等待驱动器解析速度设置命令
    MACHINE_TEST_STATE_FORWARD_START,     // 发送正转命令
    MACHINE_TEST_STATE_FORWARD_WAIT,      // 等待正转 2 秒
    MACHINE_TEST_STATE_FORWARD_STOP_WAIT, // 正转结束后停止 1 秒
    MACHINE_TEST_STATE_REVERSE_START,     // 发送反转命令
    MACHINE_TEST_STATE_REVERSE_WAIT,      // 等待反转 2 秒
    MACHINE_TEST_STATE_REVERSE_STOP_WAIT, // 非最后一轮反转结束后停止 1 秒
    MACHINE_TEST_STATE_FINISHED,          // 测试完成
} MachineTestState_e;

static PumpDrive_s machine_pump;
static uint8_t machine_pump_ready = 0U;
static uint8_t machine_test_finished = 0U;
static uint8_t machine_test_round = 0U;
static uint32_t machine_state_start_ms = 0U;
static MachineTestState_e machine_test_state = MACHINE_TEST_STATE_IDLE;

static uint32_t Machine_GetDwtMs(void)
{
    /*
     * 使用 DWT 时间轴做非阻塞计时。
     * DWT_GetTimeline_us() 内部会更新时间轴，这里转换成毫秒给状态机使用。
     */
    return (uint32_t)(DWT_GetTimeline_us() / 1000ULL);
}

static uint8_t Machine_IsTimeReached(uint32_t duration_ms)
{
    uint32_t now_ms = Machine_GetDwtMs();

    return ((uint32_t)(now_ms - machine_state_start_ms) >= duration_ms) ? 1U : 0U;
}

static void Machine_EnterState(MachineTestState_e next_state)
{
    machine_test_state = next_state;
    machine_state_start_ms = Machine_GetDwtMs();
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

    machine_test_round = 0U;
    machine_test_finished = 0U;
    Machine_EnterState(MACHINE_TEST_STATE_ENABLE);
}


void MachineControl(void)
{
    if ((machine_pump_ready == 0U) || (machine_test_finished != 0U))
    {
        return;
    }

    switch (machine_test_state)
    {
    case MACHINE_TEST_STATE_ENABLE:
        /*
         * 每个状态只发送一次命令，然后立即切到等待状态。
         * 后续等待由时间戳判断完成，不使用 DWT_Delay() 忙等。
         */
        (void)PumpDrive_Enable(&machine_pump);
        Machine_EnterState(MACHINE_TEST_STATE_ENABLE_WAIT);
        break;

    case MACHINE_TEST_STATE_ENABLE_WAIT:
        if (Machine_IsTimeReached(MACHINE_CMD_GAP_MS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_SET_SPEED);
        }
        break;

    case MACHINE_TEST_STATE_SET_SPEED:
        (void)PumpDrive_SetSpeed(&machine_pump, MACHINE_TEST_SPEED_PPS);
        Machine_EnterState(MACHINE_TEST_STATE_SET_SPEED_WAIT);
        break;

    case MACHINE_TEST_STATE_SET_SPEED_WAIT:
        if (Machine_IsTimeReached(MACHINE_CMD_GAP_MS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_FORWARD_START);
        }
        break;

    case MACHINE_TEST_STATE_FORWARD_START:
        /* out 4000 是定步正转运动，2000pps 下理论运行 2 秒。 */
        (void)PumpDrive_MoveOut(&machine_pump, MACHINE_TEST_FORWARD_STEPS);
        Machine_EnterState(MACHINE_TEST_STATE_FORWARD_WAIT);
        break;

    case MACHINE_TEST_STATE_FORWARD_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_FORWARD_MS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_FORWARD_STOP_WAIT);
        }
        break;

    case MACHINE_TEST_STATE_FORWARD_STOP_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_STOP_MS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_REVERSE_START);
        }
        break;

    case MACHINE_TEST_STATE_REVERSE_START:
        /* in 4000 是定步反转运动，2000pps 下理论运行 2 秒。 */
        (void)PumpDrive_MoveIn(&machine_pump, MACHINE_TEST_REVERSE_STEPS);
        Machine_EnterState(MACHINE_TEST_STATE_REVERSE_WAIT);
        break;

    case MACHINE_TEST_STATE_REVERSE_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_REVERSE_MS) == 0U)
        {
            break;
        }

        machine_test_round++;
        if (machine_test_round >= MACHINE_TEST_REPEAT_COUNT)
        {
            (void)PumpDrive_Stop(&machine_pump, 0U);
            machine_test_finished = 1U;
            Machine_EnterState(MACHINE_TEST_STATE_FINISHED);
        }
        else
        {
            Machine_EnterState(MACHINE_TEST_STATE_REVERSE_STOP_WAIT);
        }
        break;

    case MACHINE_TEST_STATE_REVERSE_STOP_WAIT:
        if (Machine_IsTimeReached(MACHINE_TEST_STOP_MS) != 0U)
        {
            Machine_EnterState(MACHINE_TEST_STATE_FORWARD_START);
        }
        break;

    case MACHINE_TEST_STATE_FINISHED:
    case MACHINE_TEST_STATE_IDLE:
    default:
        break;
    }
}
