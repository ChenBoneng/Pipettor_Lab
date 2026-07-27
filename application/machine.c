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
#define MACHINE_MS_TO_SECOND          0.001f

/*
 * 如果你的 ISC1000 接的是 RS485 总线，把这里改成 PUMP_DRIVE_BUS_RS485。
 * 当前使用 RS485 模式，命令格式为：1 set spd=2000\n。
 */
#define MACHINE_PUMP_BUS_MODE         PUMP_DRIVE_BUS_RS485

static PumpDrive_s machine_pump;
static uint8_t machine_pump_ready = 0U;
static uint8_t machine_test_finished = 0U;

static void Machine_DelayMs(uint32_t delay_ms)
{
    /*
     * 本测试明确要求使用 DWT 模块做时间延时。
     * DWT_Delay() 的单位是秒，所以这里把毫秒转换成秒。
     */
    DWT_Delay((float)delay_ms * MACHINE_MS_TO_SECOND);
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
        return;
    }
}


void MachineControl(void)
{
    if ((machine_pump_ready == 0U) || (machine_test_finished != 0U))
    {
        return;
    }

    /*
     * 这是一次性电机测试流程，不做复杂状态机：
     * 1. 任务启动后先使能电机；
     * 2. 设置目标速度为 2000pps；
     * 3. 循环 5 次执行：正转 4000 步 -> 停止 1 秒 -> 反转 4000 步；
     * 4. 最后发送停止命令并结束测试，不重复执行。
     *
     * 命令之间保留 50ms 间隔，是为了让 ISC1000 有时间解析上一条 ASCII 命令，
     * 避免连续发送过快时后一条命令被驱动器忽略。
     */
    machine_test_finished = 1U;

    (void)PumpDrive_Enable(&machine_pump);
    Machine_DelayMs(MACHINE_CMD_GAP_MS);

    (void)PumpDrive_SetSpeed(&machine_pump, MACHINE_TEST_SPEED_PPS);
    Machine_DelayMs(MACHINE_CMD_GAP_MS);

    for (uint8_t i = 0U; i < MACHINE_TEST_REPEAT_COUNT; i++)
    {
        /*
         * out 4000 是定步正转运动，2000pps 下理论运行 2 秒。
         * 驱动器完成定步运动后会自动停止，所以这里不额外发送 stp 0。
         */
        (void)PumpDrive_MoveOut(&machine_pump, MACHINE_TEST_FORWARD_STEPS);
        Machine_DelayMs(MACHINE_TEST_FORWARD_MS);

        /* 正转结束后保持停止 1 秒，再切换到反转。 */
        Machine_DelayMs(MACHINE_TEST_STOP_MS);

        /* in 4000 是定步反转运动，2000pps 下理论运行 2 秒。 */
        (void)PumpDrive_MoveIn(&machine_pump, MACHINE_TEST_REVERSE_STEPS);
        Machine_DelayMs(MACHINE_TEST_REVERSE_MS);

        /*
         * 如果还要进入下一轮，反转结束后也停 1 秒再重新正转。
         * 这样每次换向前都有明确的静止间隔，减少驱动器忙状态下漏命令的概率。
         */
        if (i < (MACHINE_TEST_REPEAT_COUNT - 1U))
        {
            Machine_DelayMs(MACHINE_TEST_STOP_MS);
        }
    }

    (void)PumpDrive_Stop(&machine_pump, 0U);
}
