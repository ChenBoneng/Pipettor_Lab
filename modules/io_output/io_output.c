#include "io_output.h"
#include "main.h"

/*
 * 这里单独做一层 io_output，是为了把“MCU 哪个引脚控制哪一路 24V 输出”
 * 和“这一路输出后面接了什么设备”分开。
 *
 * 上层 solenoid_valve / water_pump 只关心自己的设备编号，不直接写 GPIO。
 * 这样以后如果 PCB 引脚调整，只需要优先检查这张表。
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} IoOutputHw_s;

/*
 * 三路 24V 输出来自当前引脚表：
 * - PC13：GPIO_CONTROL_FAMEN_1；
 * - PC14：GPIO_CONTROL_FAMEN_2；
 * - PC15：GPIO_CONTROL_SHUIBENG。
 *
 * gpio.c 中已经把 PC13/PC14/PC15 配置为推挽输出，这里只保存端口和引脚。
 */
static const IoOutputHw_s io_output_hw[IO_OUTPUT_MAX] = {
    {GPIOC, GPIO_PIN_13},
    {GPIOC, GPIO_PIN_14},
    {GPIOC, GPIO_PIN_15},
};

/*
 * 软件状态只记录最近一次写入值。
 * 本项目当前没有硬件反馈脚，所以不能把它理解成真实负载状态。
 */
static volatile IoOutputState_e io_output_state[IO_OUTPUT_MAX] = {IO_OUTPUT_STATE_OFF};
static uint8_t io_output_inited = 0U;

/**
 * @brief 判断输出编号是否合法。
 * @param output 输出编号。
 * @return 1 表示合法；0 表示越界。
 */
static uint8_t IoOutput_IsValidId(IoOutputId_e output)
{
    return (output < IO_OUTPUT_MAX) ? 1U : 0U;
}

/**
 * @brief 初始化三路 24V 输出，并全部拉低。
 */
void IoOutput_Init(void)
{
    for (uint8_t i = 0U; i < IO_OUTPUT_MAX; i++)
    {
        HAL_GPIO_WritePin(io_output_hw[i].port, io_output_hw[i].pin, GPIO_PIN_RESET);
        io_output_state[i] = IO_OUTPUT_STATE_OFF;
    }

    io_output_inited = 1U;
}

/**
 * @brief 设置一路 24V 输出的 GPIO 电平。
 */
uint8_t IoOutput_Set(IoOutputId_e output, IoOutputState_e state)
{
    GPIO_PinState pin_state = GPIO_PIN_RESET;

    if ((io_output_inited == 0U) || (IoOutput_IsValidId(output) == 0U))
    {
        return 0U;
    }

    if (state == IO_OUTPUT_STATE_ON)
    {
        pin_state = GPIO_PIN_SET;
    }

    /* 当前硬件为正逻辑：GPIO 高电平导通外部功率输出。 */
    HAL_GPIO_WritePin(io_output_hw[output].port, io_output_hw[output].pin, pin_state);
    io_output_state[output] = state;
    return 1U;
}

/**
 * @brief 关闭全部 24V 输出。
 */
void IoOutput_AllOff(void)
{
    for (uint8_t i = 0U; i < IO_OUTPUT_MAX; i++)
    {
        (void)IoOutput_Set((IoOutputId_e)i, IO_OUTPUT_STATE_OFF);
    }
}

/**
 * @brief 读取软件记录的输出状态。
 */
IoOutputState_e IoOutput_GetState(IoOutputId_e output)
{
    if (IoOutput_IsValidId(output) == 0U)
    {
        return IO_OUTPUT_STATE_OFF;
    }

    return io_output_state[output];
}

/**
 * @brief 判断指定输出是否打开。
 */
uint8_t IoOutput_IsOn(IoOutputId_e output)
{
    return (IoOutput_GetState(output) == IO_OUTPUT_STATE_ON) ? 1U : 0U;
}
