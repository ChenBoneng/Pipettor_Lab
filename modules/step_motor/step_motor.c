#include "step_motor.h"
#include <string.h>
#include "bsp_dwt.h"
#include "main.h"
#include "tim.h"

/*
 * DM542 要求：
 * - PUL 上升沿有效；
 * - PUL 高电平和低电平都要大于 1.2us；
 * - DIR 改变后至少提前 5us，再输出 PUL。
 *
 * 这里把 TIM2 设置成 1MHz 计数基准：
 * - 1 个计数 = 1us；
 * - speed_pps = 2000 时，周期 = 1000000 / 2000 = 500us；
 * - 50% 占空比时，高/低电平各 250us，远大于 1.2us。
 */
#define STEP_MOTOR_TIMER_BASE_HZ       1000000UL
#define STEP_MOTOR_DIR_SETUP_DELAY_S   0.000010f
#define STEP_MOTOR_MIN_PULSE_TICKS     4UL

/*
 * 位移换算校准值。
 *
 * 实测条件：
 * - 驱动器细分数：1600 脉冲/圈；
 * - 发送脉冲数：12000；
 * - 实际移动距离：28.54mm = 28540um。
 *
 * 因此：
 * - 脉冲/毫米 = 12000 / 28.54 ≈ 420.46 pulse/mm；
 * - 应用层不直接使用浮点数，而是统一换算到 um 后用整数计算。
 */
#define STEP_MOTOR_CALIBRATION_PULSES       12000UL
#define STEP_MOTOR_CALIBRATION_DISTANCE_UM  28540UL
#define STEP_MOTOR_MM_X100_TO_UM            10UL
#define STEP_MOTOR_CM_X100_TO_UM            100UL

typedef struct
{
    GPIO_TypeDef *dir_plus_port;
    uint16_t dir_plus_pin;
    GPIO_TypeDef *dir_minus_port;
    uint16_t dir_minus_pin;
    uint32_t tim_channel;
} StepMotorHw_s;

static const StepMotorHw_s step_motor_hw[STEP_MOTOR_ID_MAX] = {
    {GPIOA, GPIO_PIN_7, GPIOA, GPIO_PIN_6, TIM_CHANNEL_1},
    {GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_2},
};

static volatile StepMotorStatus_s step_motor_status[STEP_MOTOR_ID_MAX];
static volatile StepMotorId_e step_motor_active_id = STEP_MOTOR_ID_MAX;
static uint8_t step_motor_inited = 0U;

static uint8_t StepMotor_IsValidId(StepMotorId_e motor);
static uint32_t StepMotor_GetTIM2ClockHz(void);
static uint8_t StepMotor_ConfigTimer(uint32_t speed_pps, uint32_t channel);
static uint32_t StepMotor_UmToSteps(uint32_t distance_um);
static uint32_t StepMotor_UmPerSecToPps(uint32_t speed_um_s);
static void StepMotor_SetDirection(StepMotorId_e motor, StepMotorDirection_e direction);
static void StepMotor_StopOutput(StepMotorId_e motor);
static uint8_t StepMotor_Start(StepMotorId_e motor,
                               StepMotorDirection_e direction,
                               uint32_t speed_pps,
                               uint32_t steps);

/**
 * @brief 判断电机编号是否合法。
 *
 * @param motor 电机编号。
 * @return 1 表示合法；0 表示越界。
 */
static uint8_t StepMotor_IsValidId(StepMotorId_e motor)
{
    return (motor < STEP_MOTOR_ID_MAX) ? 1U : 0U;
}

/**
 * @brief 获取 TIM2 的实际输入时钟。
 *
 * STM32F103 的 TIM2 挂在 APB1 上。若 APB1 分频不为 1，定时器时钟会自动
 * 变成 PCLK1 的 2 倍。当前工程 APB1=36MHz，所以 TIM2 实际为 72MHz。
 *
 * @return TIM2 输入时钟，单位 Hz。
 */
static uint32_t StepMotor_GetTIM2ClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = pclk1;

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        tim_clk = pclk1 * 2U;
    }

    return tim_clk;
}

/**
 * @brief 按目标 PPS 配置 TIM2 周期和指定通道占空比。
 *
 * @param speed_pps 目标脉冲频率，单位 PPS。
 * @param channel TIM2 通道，当前只允许 TIM_CHANNEL_1 或 TIM_CHANNEL_2。
 * @return 1 表示配置成功；0 表示速度非法或周期太小。
 */
static uint8_t StepMotor_ConfigTimer(uint32_t speed_pps, uint32_t channel)
{
    uint32_t tim_clk;
    uint32_t prescaler;
    uint32_t period_ticks;
    uint32_t pulse_ticks;

    if (speed_pps == 0U)
    {
        return 0U;
    }

    tim_clk = StepMotor_GetTIM2ClockHz();
    prescaler = (tim_clk / STEP_MOTOR_TIMER_BASE_HZ);
    if (prescaler == 0U)
    {
        prescaler = 1U;
    }

    period_ticks = STEP_MOTOR_TIMER_BASE_HZ / speed_pps;
    if (period_ticks < STEP_MOTOR_MIN_PULSE_TICKS)
    {
        return 0U;
    }

    pulse_ticks = period_ticks / 2U;
    if (pulse_ticks == 0U)
    {
        pulse_ticks = 1U;
    }

    /*
     * 修改 PSC/ARR/CCR 前先停表，避免运行中改周期造成毛刺。
     * PSC 寄存器实际写入的是“分频值 - 1”。
     */
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_PRESCALER(&htim2, prescaler - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period_ticks - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, channel, pulse_ticks);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    /*
     * 触发一次更新事件，让新的 PSC/ARR 立即装载。
     * 之后清掉更新标志，防止刚启动时进入无关中断。
     */
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2);

    return 1U;
}

/**
 * @brief 把微米距离换算成脉冲数。
 *
 * @param distance_um 目标距离，单位 um。
 * @return 换算后的脉冲数。
 *
 * @note 公式：steps = distance_um * 12000 / 28540。
 *       这里加上除数的一半做四舍五入，减少整数截断误差。
 */
static uint32_t StepMotor_UmToSteps(uint32_t distance_um)
{
    uint64_t numerator;

    numerator = (uint64_t)distance_um * STEP_MOTOR_CALIBRATION_PULSES;
    numerator += STEP_MOTOR_CALIBRATION_DISTANCE_UM / 2U;

    return (uint32_t)(numerator / STEP_MOTOR_CALIBRATION_DISTANCE_UM);
}

/**
 * @brief 把微米每秒速度换算成 PPS。
 *
 * @param speed_um_s 目标速度，单位 um/s。
 * @return 换算后的 PPS。
 *
 * @note 公式：pps = speed_um_s * 12000 / 28540。
 */
static uint32_t StepMotor_UmPerSecToPps(uint32_t speed_um_s)
{
    uint64_t numerator;

    numerator = (uint64_t)speed_um_s * STEP_MOTOR_CALIBRATION_PULSES;
    numerator += STEP_MOTOR_CALIBRATION_DISTANCE_UM / 2U;

    return (uint32_t)(numerator / STEP_MOTOR_CALIBRATION_DISTANCE_UM);
}

/**
 * @brief 设置 DM542 方向差分输入。
 *
 * @param motor 电机编号。
 * @param direction 目标方向。
 *
 * @note 当前约定正向为 DIR+ 高、DIR- 低；反向为 DIR+ 低、DIR- 高。
 */
static void StepMotor_SetDirection(StepMotorId_e motor, StepMotorDirection_e direction)
{
    GPIO_PinState plus_state = GPIO_PIN_RESET;
    GPIO_PinState minus_state = GPIO_PIN_SET;

    if (direction == STEP_MOTOR_DIR_FORWARD)
    {
        plus_state = GPIO_PIN_SET;
        minus_state = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(step_motor_hw[motor].dir_plus_port,
                      step_motor_hw[motor].dir_plus_pin,
                      plus_state);
    HAL_GPIO_WritePin(step_motor_hw[motor].dir_minus_port,
                      step_motor_hw[motor].dir_minus_pin,
                      minus_state);
}

/**
 * @brief 关闭某一路 PWM 输出并清理运行状态。
 *
 * @param motor 电机编号。
 */
static void StepMotor_StopOutput(StepMotorId_e motor)
{
    if (StepMotor_IsValidId(motor) == 0U)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop_IT(&htim2, step_motor_hw[motor].tim_channel);
    __HAL_TIM_SET_COMPARE(&htim2, step_motor_hw[motor].tim_channel, 0U);

    step_motor_status[motor].state = STEP_MOTOR_STATE_IDLE;
    if (step_motor_active_id == motor)
    {
        step_motor_active_id = STEP_MOTOR_ID_MAX;
    }
}

/**
 * @brief 启动某一路步进电机。
 *
 * @param motor 电机编号。
 * @param direction 运行方向。
 * @param speed_pps 脉冲频率，单位 PPS。
 * @param steps 目标步数，0 表示连续运行。
 * @return 1 表示启动成功；0 表示启动失败。
 */
static uint8_t StepMotor_Start(StepMotorId_e motor,
                               StepMotorDirection_e direction,
                               uint32_t speed_pps,
                               uint32_t steps)
{
    if ((step_motor_inited == 0U) ||
        (StepMotor_IsValidId(motor) == 0U) ||
        (speed_pps == 0U))
    {
        return 0U;
    }

    /*
     * 当前硬件只有一个 TIM2 周期源。
     * 业务已确认两个电机不会同时运行，因此这里直接拒绝第二路启动请求。
     */
    if (step_motor_active_id != STEP_MOTOR_ID_MAX)
    {
        return 0U;
    }

    StepMotor_StopAll();

    if (StepMotor_ConfigTimer(speed_pps, step_motor_hw[motor].tim_channel) == 0U)
    {
        return 0U;
    }

    StepMotor_SetDirection(motor, direction);

    /*
     * 这里只等待 10us，用于满足 DM542 的 DIR 提前建立时间。
     * 这不是业务流程里的长阻塞延时，不会影响 FreeRTOS 任务调度。
     */
    DWT_Delay(STEP_MOTOR_DIR_SETUP_DELAY_S);

    step_motor_status[motor].state = STEP_MOTOR_STATE_RUNNING;
    step_motor_status[motor].direction = direction;
    step_motor_status[motor].speed_pps = speed_pps;
    step_motor_status[motor].target_steps = steps;
    step_motor_status[motor].finished_steps = 0U;
    step_motor_active_id = motor;

    if (steps == 0U)
    {
        /*
         * 连续运行不需要统计目标步数，因此不开比较中断，减少 CPU 开销。
         * 停止动作由业务层调用 StepMotor_Stop() 或 StepMotor_StopAll() 完成。
         */
        if (HAL_TIM_PWM_Start(&htim2, step_motor_hw[motor].tim_channel) != HAL_OK)
        {
            StepMotor_StopOutput(motor);
            return 0U;
        }
    }
    else if (HAL_TIM_PWM_Start_IT(&htim2, step_motor_hw[motor].tim_channel) != HAL_OK)
    {
        StepMotor_StopOutput(motor);
        return 0U;
    }

    return 1U;
}

void StepMotor_Init(void)
{
    memset((void *)step_motor_status, 0, sizeof(step_motor_status));

    StepMotor_StopAll();

    /*
     * 初始化后方向默认给正向电平，但不输出脉冲。
     * PA4~PA7 已由 CubeMX 配成普通推挽输出，这里只写默认电平。
     */
    StepMotor_SetDirection(STEP_MOTOR_ID_A, STEP_MOTOR_DIR_FORWARD);
    StepMotor_SetDirection(STEP_MOTOR_ID_B, STEP_MOTOR_DIR_FORWARD);

    step_motor_active_id = STEP_MOTOR_ID_MAX;
    step_motor_inited = 1U;
}

uint8_t StepMotor_RunContinuous(StepMotorId_e motor,
                                StepMotorDirection_e direction,
                                uint32_t speed_pps)
{
    return StepMotor_Start(motor, direction, speed_pps, 0U);
}

uint8_t StepMotor_RunSteps(StepMotorId_e motor,
                           StepMotorDirection_e direction,
                           uint32_t speed_pps,
                           uint32_t steps)
{
    if (steps == 0U)
    {
        return 0U;
    }

    return StepMotor_Start(motor, direction, speed_pps, steps);
}

uint32_t StepMotor_MmX100ToSteps(uint32_t distance_mm_x100)
{
    return StepMotor_UmToSteps(distance_mm_x100 * STEP_MOTOR_MM_X100_TO_UM);
}

uint32_t StepMotor_MmPerSecX100ToPps(uint32_t speed_mm_s_x100)
{
    return StepMotor_UmPerSecToPps(speed_mm_s_x100 * STEP_MOTOR_MM_X100_TO_UM);
}

uint8_t StepMotor_RunDistanceMmX100(StepMotorId_e motor,
                                    StepMotorDirection_e direction,
                                    uint32_t distance_mm_x100,
                                    uint32_t speed_mm_s_x100)
{
    uint32_t steps;
    uint32_t speed_pps;

    steps = StepMotor_MmX100ToSteps(distance_mm_x100);
    speed_pps = StepMotor_MmPerSecX100ToPps(speed_mm_s_x100);

    if ((steps == 0U) || (speed_pps == 0U))
    {
        return 0U;
    }

    return StepMotor_RunSteps(motor, direction, speed_pps, steps);
}

uint8_t StepMotor_RunDistanceCmX100(StepMotorId_e motor,
                                    StepMotorDirection_e direction,
                                    uint32_t distance_cm_x100,
                                    uint32_t speed_cm_s_x100)
{
    uint32_t distance_um;
    uint32_t speed_um_s;
    uint32_t steps;
    uint32_t speed_pps;

    distance_um = distance_cm_x100 * STEP_MOTOR_CM_X100_TO_UM;
    speed_um_s = speed_cm_s_x100 * STEP_MOTOR_CM_X100_TO_UM;
    steps = StepMotor_UmToSteps(distance_um);
    speed_pps = StepMotor_UmPerSecToPps(speed_um_s);

    if ((steps == 0U) || (speed_pps == 0U))
    {
        return 0U;
    }

    return StepMotor_RunSteps(motor, direction, speed_pps, steps);
}

void StepMotor_Stop(StepMotorId_e motor)
{
    StepMotor_StopOutput(motor);
}

void StepMotor_StopAll(void)
{
    (void)HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);

    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_ID_MAX; i++)
    {
        step_motor_status[i].state = STEP_MOTOR_STATE_IDLE;
    }

    step_motor_active_id = STEP_MOTOR_ID_MAX;
}

uint8_t StepMotor_IsBusy(StepMotorId_e motor)
{
    if (StepMotor_IsValidId(motor) == 0U)
    {
        return 0U;
    }

    return (step_motor_status[motor].state == STEP_MOTOR_STATE_RUNNING) ? 1U : 0U;
}

uint8_t StepMotor_GetStatus(StepMotorId_e motor, StepMotorStatus_s *status)
{
    if ((StepMotor_IsValidId(motor) == 0U) || (status == NULL))
    {
        return 0U;
    }

    status->state = step_motor_status[motor].state;
    status->direction = step_motor_status[motor].direction;
    status->speed_pps = step_motor_status[motor].speed_pps;
    status->target_steps = step_motor_status[motor].target_steps;
    status->finished_steps = step_motor_status[motor].finished_steps;

    return 1U;
}

/**
 * @brief PWM 高电平结束回调。
 *
 * HAL_TIM_PWM_Start_IT() 会打开对应通道的比较中断。
 * PWM1 模式下，比较点就是高电平结束的位置；每进入一次该回调，
 * 说明刚刚完成了一个有效 PUL 上升沿对应的高电平脉冲。
 *
 * @param htim 触发回调的定时器句柄。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    StepMotorId_e motor;

    if ((htim == NULL) || (htim->Instance != TIM2))
    {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        motor = STEP_MOTOR_ID_A;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        motor = STEP_MOTOR_ID_B;
    }
    else
    {
        return;
    }

    if ((step_motor_active_id != motor) ||
        (step_motor_status[motor].state != STEP_MOTOR_STATE_RUNNING))
    {
        return;
    }

    step_motor_status[motor].finished_steps++;

    /*
     * target_steps 为 0 表示连续运行，不在中断里自动停止。
     * 非 0 时，达到目标步数立即关断 PWM，避免下一个周期再次产生上升沿。
     */
    if ((step_motor_status[motor].target_steps != 0U) &&
        (step_motor_status[motor].finished_steps >= step_motor_status[motor].target_steps))
    {
        StepMotor_StopOutput(motor);
    }
}
