#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>

/**
 * @file step_motor.h
 * @brief DM542 脉冲方向式步进电机驱动模块。
 *
 * 本模块位于 modules 层，只负责把上层的“运行方向、速度、步数”
 * 转换成 DM542 可以识别的 PUL/DIR 时序。
 *
 * 当前硬件连接：
 * - 电机 A：PA0/TIM2_CH1 输出 PUL，PA7/PA6 输出 DIR+/DIR-；
 * - 电机 B：PA1/TIM2_CH2 输出 PUL，PA5/PA4 输出 DIR+/DIR-；
 * - ENA 未看到独立 MCU 控制网络，按 DM542 默认悬空使能处理。
 *
 * @note TIM2_CH1 和 TIM2_CH2 共用同一个计数周期，因此理论上两路同时运行时
 *       只能同频。本项目约定两个步进电机不会同时运行，所以模块内部直接
 *       做单电机互斥控制，避免误操作造成两轴抢占 TIM2。
 */

/**
 * @brief 步进电机编号。
 *
 * 该枚举用于指定当前要控制的物理电机。
 */
typedef enum
{
    STEP_MOTOR_ID_A = 0, /**< 电机 A，对应 CN6 接口，PUL 使用 TIM2_CH1/PA0。 */
    STEP_MOTOR_ID_B,     /**< 电机 B，对应 CN5 接口，PUL 使用 TIM2_CH2/PA1。 */
    STEP_MOTOR_ID_MAX    /**< 电机数量边界值，仅用于数组长度和参数检查。 */
} StepMotorId_e;

/**
 * @brief 步进电机方向。
 *
 * DM542 的 DIR 信号本质上只区分高低电平，实际正反方向还受电机接线影响。
 * 如果发现方向与机械定义相反，优先对调电机其中一相接线，或在本模块中
 * 调换对应方向的 GPIO 电平。
 */
typedef enum
{
    STEP_MOTOR_DIR_FORWARD = 0, /**< 正向：DIR+ 输出高电平，DIR- 输出低电平。 */
    STEP_MOTOR_DIR_REVERSE      /**< 反向：DIR+ 输出低电平，DIR- 输出高电平。 */
} StepMotorDirection_e;

/**
 * @brief 步进电机运行状态。
 */
typedef enum
{
    STEP_MOTOR_STATE_IDLE = 0, /**< 空闲：当前没有输出脉冲。 */
    STEP_MOTOR_STATE_RUNNING   /**< 运行中：当前正在输出 PWM 脉冲。 */
} StepMotorState_e;

/**
 * @brief 单个步进电机的运行信息。
 *
 * 业务层可通过 StepMotor_GetStatus() 获取该结构体，查看电机是否忙碌、
 * 当前方向、当前速度和已经完成的脉冲数量。
 */
typedef struct
{
    StepMotorState_e state;         /**< 当前状态：空闲或运行中。 */
    StepMotorDirection_e direction; /**< 当前方向。 */
    uint32_t speed_pps;             /**< 当前脉冲频率，单位 PPS，即每秒输出多少个上升沿。 */
    uint32_t target_steps;          /**< 目标步数；0 表示连续运行，不自动停。 */
    uint32_t finished_steps;        /**< 已完成步数，在 PWM 高电平结束中断里累加。 */
} StepMotorStatus_s;

/**
 * @brief 初始化 DM542 步进电机驱动模块。
 *
 * @note 本函数只初始化方向 GPIO 的默认电平并停止 TIM2 PWM 输出。
 *       TIM2 全局中断由 CubeMX 在 MX_TIM2_Init() 中配置，
 *       本函数不会主动让任何电机转动，可以安全放在 AllTaskInit() 中调用。
 */
void StepMotor_Init(void);

/**
 * @brief 让指定电机按固定速度连续运行。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @param direction 运行方向，取值见 StepMotorDirection_e。
 * @param speed_pps 脉冲频率，单位 PPS。2000 表示每秒 2000 个脉冲。
 * @return 1 表示启动成功；0 表示参数错误、模块未初始化或已有电机正在运行。
 *
 * @note 连续运行不会自动停止，业务层必须调用 StepMotor_Stop() 或
 *       StepMotor_StopAll() 停止输出脉冲。
 */
uint8_t StepMotor_RunContinuous(StepMotorId_e motor,
                                StepMotorDirection_e direction,
                                uint32_t speed_pps);

/**
 * @brief 让指定电机按固定速度运行指定步数。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @param direction 运行方向，取值见 StepMotorDirection_e。
 * @param speed_pps 脉冲频率，单位 PPS。
 * @param steps 目标步数，也就是要输出的 PUL 上升沿数量。
 * @return 1 表示启动成功；0 表示参数错误、模块未初始化或已有电机正在运行。
 *
 * @note 本函数非阻塞。启动成功后立即返回，步数统计在 TIM2 PWM 中断中完成，
 *       达到 steps 后自动停止 PWM。
 */
uint8_t StepMotor_RunSteps(StepMotorId_e motor,
                           StepMotorDirection_e direction,
                           uint32_t speed_pps,
                           uint32_t steps);

/**
 * @brief 停止指定电机。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 *
 * @note 如果指定电机当前没有运行，本函数不会产生额外动作。
 */
void StepMotor_Stop(StepMotorId_e motor);

/**
 * @brief 停止所有步进电机输出。
 *
 * @note 当前模块内部约定任意时刻最多只有一个电机运行，但该函数会同时关闭
 *       TIM2_CH1 和 TIM2_CH2，适合急停或初始化时调用。
 */
void StepMotor_StopAll(void);

/**
 * @brief 判断指定电机是否正在运行。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @return 1 表示运行中；0 表示空闲或参数错误。
 */
uint8_t StepMotor_IsBusy(StepMotorId_e motor);

/**
 * @brief 读取指定电机的最近状态。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @param status 输出参数，用于保存电机状态，不能为 NULL。
 * @return 1 表示读取成功；0 表示参数错误。
 */
uint8_t StepMotor_GetStatus(StepMotorId_e motor, StepMotorStatus_s *status);

#endif //STEP_MOTOR_H
