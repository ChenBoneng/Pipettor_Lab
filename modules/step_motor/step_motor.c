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
 * 梯形加减速参数。
 *
 * 当前策略：
 * - 从 STEP_MOTOR_START_PPS 起步，避免电机从 0 直接跳到目标速度；
 * - 每 STEP_MOTOR_ACC_UPDATE_MS 调整一次速度；
 * - 每次调整 STEP_MOTOR_ACC_STEP_PPS；
 * - 接近目标步数时按同样斜率减速。
 */
#define STEP_MOTOR_START_PPS           200UL
#define STEP_MOTOR_ACC_STEP_PPS        50UL
#define STEP_MOTOR_ACC_UPDATE_MS       10UL

/*
 * 丝杆位移换算参数。
 *
 * 已确认机械数据：
 * - 驱动器细分数：1600 脉冲/圈；
 * - 丝杆导程：4mm/圈，也就是电机转 1 圈滑台移动 4mm；
 * - 电机 A 所在滑台有效行程：150mm；
 * - 电机 B 所在滑台有效行程：300mm。
 *
 * 因此：
 * - 脉冲/毫米 = 1600 / 4 = 400 pulse/mm；
 * - 30mm 位移 = 30 * 400 = 12000 pulse；
 * - 应用层不直接使用浮点数，而是统一换算到 um 后用整数计算。
 */
#define STEP_MOTOR_DRIVER_MICROSTEP_PPR     1600UL
#define STEP_MOTOR_SCREW_LEAD_UM            4000UL
#define STEP_MOTOR_A_TRAVEL_UM              150000UL
#define STEP_MOTOR_B_TRAVEL_UM              300000UL
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

typedef struct
{
    uint8_t enabled;          // 1 表示当前需要 StepMotor_Process() 调整速度
    uint32_t target_speed_pps; // 应用层要求的目标速度
    uint32_t current_speed_pps; // 当前实际输出速度
    uint32_t last_update_ms;   // 上一次调整速度的时间戳
} StepMotorRamp_s;

static const StepMotorHw_s step_motor_hw[STEP_MOTOR_ID_MAX] = {
    {GPIOA, GPIO_PIN_7, GPIOA, GPIO_PIN_6, TIM_CHANNEL_1},
    {GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_4, TIM_CHANNEL_2},
};

static const uint32_t step_motor_max_travel_um[STEP_MOTOR_ID_MAX] = {
    STEP_MOTOR_A_TRAVEL_UM,
    STEP_MOTOR_B_TRAVEL_UM,
};

static volatile StepMotorStatus_s step_motor_status[STEP_MOTOR_ID_MAX];
static volatile StepMotorId_e step_motor_active_id = STEP_MOTOR_ID_MAX;
static StepMotorRamp_s step_motor_ramp = {0};
static uint8_t step_motor_inited = 0U;

static uint8_t StepMotor_IsValidId(StepMotorId_e motor);
static uint32_t StepMotor_GetDwtMs(void);
static uint32_t StepMotor_GetTIM2ClockHz(void);
static uint8_t StepMotor_ConfigTimer(uint32_t speed_pps, uint32_t channel);
static uint32_t StepMotor_LimitStartSpeed(uint32_t target_speed_pps);
static uint32_t StepMotor_CalcDecelSteps(uint32_t current_speed_pps);
static uint32_t StepMotor_GetMaxTravelUm(StepMotorId_e motor);
static uint8_t StepMotor_IsDistanceAllowed(StepMotorId_e motor, uint32_t distance_um);
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
 * @brief 获取 DWT 毫秒时间轴。
 *
 * @return 从 DWT_Init() 后累计的毫秒数。
 */
static uint32_t StepMotor_GetDwtMs(void)
{
    return (uint32_t)(DWT_GetTimeline_us() / 1000ULL);
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
    uint32_t timer_was_enabled;

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
     * 修改 PSC/ARR/CCR 前先短暂停表，避免运行中改周期造成毛刺。
     * PSC 寄存器实际写入的是“分频值 - 1”。
     * 如果调用前定时器正在运行，修改完成后会恢复运行。
     */
    timer_was_enabled = htim2.Instance->CR1 & TIM_CR1_CEN;
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

    if (timer_was_enabled != 0U)
    {
        __HAL_TIM_ENABLE(&htim2);
    }

    return 1U;
}

/**
 * @brief 根据目标速度得到起步速度。
 *
 * @param target_speed_pps 目标速度，单位 PPS。
 * @return 实际起步速度，目标速度低于起步速度时直接用目标速度。
 */
static uint32_t StepMotor_LimitStartSpeed(uint32_t target_speed_pps)
{
    if (target_speed_pps < STEP_MOTOR_START_PPS)
    {
        return target_speed_pps;
    }

    return STEP_MOTOR_START_PPS;
}

/**
 * @brief 估算从当前速度减到起步速度所需的步数。
 *
 * @param current_speed_pps 当前速度，单位 PPS。
 * @return 估算减速步数。
 *
 * @note 这是给梯形减速用的轻量估算：
 *       减速次数 = (当前速度 - 起步速度) / 每次降速；
 *       减速时间 = 减速次数 * 调速周期；
 *       减速距离 = 平均速度 * 减速时间。
 */
static uint32_t StepMotor_CalcDecelSteps(uint32_t current_speed_pps)
{
    uint32_t decel_count;
    uint32_t average_speed_pps;
    uint64_t decel_steps;

    if (current_speed_pps <= STEP_MOTOR_START_PPS)
    {
        return 0U;
    }

    decel_count = (current_speed_pps - STEP_MOTOR_START_PPS + STEP_MOTOR_ACC_STEP_PPS - 1U) /
                  STEP_MOTOR_ACC_STEP_PPS;
    average_speed_pps = (current_speed_pps + STEP_MOTOR_START_PPS) / 2U;

    decel_steps = (uint64_t)average_speed_pps * decel_count * STEP_MOTOR_ACC_UPDATE_MS;
    decel_steps = (decel_steps + 999U) / 1000U;

    return (uint32_t)decel_steps;
}

/**
 * @brief 获取指定电机对应滑台的有效行程。
 *
 * @param motor 电机编号。
 * @return 有效行程，单位 um；电机编号非法时返回 0。
 */
static uint32_t StepMotor_GetMaxTravelUm(StepMotorId_e motor)
{
    if (StepMotor_IsValidId(motor) == 0U)
    {
        return 0U;
    }

    return step_motor_max_travel_um[motor];
}

/**
 * @brief 判断单次目标位移是否超过对应滑台的有效行程。
 *
 * @param motor 电机编号。
 * @param distance_um 本次目标位移，单位 um。
 * @return 1 表示允许执行；0 表示参数非法、距离为 0 或超过单次有效行程。
 *
 * @note 当前没有原点、限位或绝对位置反馈，所以这里仅限制“单次命令距离”
 *       不超过滑台总有效行程。若要防止累计移动撞端点，还需要增加回零、
 *       当前位置记录或限位开关检测。
 */
static uint8_t StepMotor_IsDistanceAllowed(StepMotorId_e motor, uint32_t distance_um)
{
    uint32_t max_travel_um;

    max_travel_um = StepMotor_GetMaxTravelUm(motor);
    if ((max_travel_um == 0U) ||
        (distance_um == 0U) ||
        (distance_um > max_travel_um))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 把微米距离换算成脉冲数。
 *
 * @param distance_um 目标距离，单位 um。
 * @return 换算后的脉冲数。
 *
 * @note 公式：steps = distance_um * 1600 / 4000。
 *       这里加上除数的一半做四舍五入，减少整数截断误差。
 */
static uint32_t StepMotor_UmToSteps(uint32_t distance_um)
{
    uint64_t numerator;

    numerator = (uint64_t)distance_um * STEP_MOTOR_DRIVER_MICROSTEP_PPR;
    numerator += STEP_MOTOR_SCREW_LEAD_UM / 2U;

    return (uint32_t)(numerator / STEP_MOTOR_SCREW_LEAD_UM);
}

/**
 * @brief 把微米每秒速度换算成 PPS。
 *
 * @param speed_um_s 目标速度，单位 um/s。
 * @return 换算后的 PPS。
 *
 * @note 公式：pps = speed_um_s * 1600 / 4000。
 */
static uint32_t StepMotor_UmPerSecToPps(uint32_t speed_um_s)
{
    uint64_t numerator;

    numerator = (uint64_t)speed_um_s * STEP_MOTOR_DRIVER_MICROSTEP_PPR;
    numerator += STEP_MOTOR_SCREW_LEAD_UM / 2U;

    return (uint32_t)(numerator / STEP_MOTOR_SCREW_LEAD_UM);
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

    step_motor_ramp.enabled = 0U;
    step_motor_ramp.target_speed_pps = 0U;
    step_motor_ramp.current_speed_pps = 0U;
    step_motor_ramp.last_update_ms = 0U;
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
    uint32_t start_speed_pps;

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

    start_speed_pps = StepMotor_LimitStartSpeed(speed_pps);

    if (StepMotor_ConfigTimer(start_speed_pps, step_motor_hw[motor].tim_channel) == 0U)
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
    step_motor_status[motor].speed_pps = start_speed_pps;
    step_motor_status[motor].target_steps = steps;
    step_motor_status[motor].finished_steps = 0U;
    step_motor_active_id = motor;

    step_motor_ramp.enabled = 1U;
    step_motor_ramp.target_speed_pps = speed_pps;
    step_motor_ramp.current_speed_pps = start_speed_pps;
    step_motor_ramp.last_update_ms = StepMotor_GetDwtMs();

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
    memset(&step_motor_ramp, 0, sizeof(step_motor_ramp));

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

void StepMotor_Process(void)
{
    StepMotorId_e motor;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint32_t current_speed_pps;
    uint32_t next_speed_pps;
    uint32_t remaining_steps;
    uint32_t decel_steps;

    if ((step_motor_inited == 0U) ||
        (step_motor_ramp.enabled == 0U) ||
        (step_motor_active_id >= STEP_MOTOR_ID_MAX))
    {
        return;
    }

    motor = step_motor_active_id;
    if (step_motor_status[motor].state != STEP_MOTOR_STATE_RUNNING)
    {
        return;
    }

    now_ms = StepMotor_GetDwtMs();
    elapsed_ms = now_ms - step_motor_ramp.last_update_ms;
    if (elapsed_ms < STEP_MOTOR_ACC_UPDATE_MS)
    {
        return;
    }

    step_motor_ramp.last_update_ms = now_ms;
    current_speed_pps = step_motor_ramp.current_speed_pps;
    next_speed_pps = current_speed_pps;

    if ((step_motor_status[motor].target_steps != 0U) &&
        (step_motor_status[motor].finished_steps < step_motor_status[motor].target_steps))
    {
        remaining_steps = step_motor_status[motor].target_steps -
                          step_motor_status[motor].finished_steps;
        decel_steps = StepMotor_CalcDecelSteps(current_speed_pps);

        /*
         * 剩余步数小于等于估算减速步数时开始减速。
         * 如果距离较短，电机会自动形成“加速后马上减速”的三角速度曲线。
         */
        if ((remaining_steps <= decel_steps) && (current_speed_pps > STEP_MOTOR_START_PPS))
        {
            if ((current_speed_pps - STEP_MOTOR_START_PPS) <= STEP_MOTOR_ACC_STEP_PPS)
            {
                next_speed_pps = STEP_MOTOR_START_PPS;
            }
            else
            {
                next_speed_pps = current_speed_pps - STEP_MOTOR_ACC_STEP_PPS;
            }
        }
        else if (current_speed_pps < step_motor_ramp.target_speed_pps)
        {
            next_speed_pps = current_speed_pps + STEP_MOTOR_ACC_STEP_PPS;
            if (next_speed_pps > step_motor_ramp.target_speed_pps)
            {
                next_speed_pps = step_motor_ramp.target_speed_pps;
            }
        }
    }
    else if ((step_motor_status[motor].target_steps == 0U) &&
             (current_speed_pps < step_motor_ramp.target_speed_pps))
    {
        /*
         * 连续运行没有目标步数，只做缓启动加速。
         * 停止时由业务层调用 Stop 接口。
         */
        next_speed_pps = current_speed_pps + STEP_MOTOR_ACC_STEP_PPS;
        if (next_speed_pps > step_motor_ramp.target_speed_pps)
        {
            next_speed_pps = step_motor_ramp.target_speed_pps;
        }
    }

    if (next_speed_pps == current_speed_pps)
    {
        return;
    }

    if (StepMotor_ConfigTimer(next_speed_pps, step_motor_hw[motor].tim_channel) != 0U)
    {
        step_motor_ramp.current_speed_pps = next_speed_pps;
        step_motor_status[motor].speed_pps = next_speed_pps;
    }
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
    if ((StepMotor_IsValidId(motor) == 0U) ||
        (steps == 0U) ||
        (steps > StepMotor_UmToSteps(StepMotor_GetMaxTravelUm(motor))))
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
    uint32_t distance_um;
    uint32_t steps;
    uint32_t speed_pps;

    distance_um = distance_mm_x100 * STEP_MOTOR_MM_X100_TO_UM;
    if (StepMotor_IsDistanceAllowed(motor, distance_um) == 0U)
    {
        return 0U;
    }

    steps = StepMotor_UmToSteps(distance_um);
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
    if (StepMotor_IsDistanceAllowed(motor, distance_um) == 0U)
    {
        return 0U;
    }

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
    memset(&step_motor_ramp, 0, sizeof(step_motor_ramp));
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
