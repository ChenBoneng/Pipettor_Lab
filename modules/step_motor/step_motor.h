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
 *
 * 当前机械语义约定：
 * - 正向等价于“拉”；
 * - 反向等价于“推”。
 *
 * 因此业务层建议优先使用 STEP_MOTOR_DIR_PULL / STEP_MOTOR_DIR_PUSH，
 * 少直接使用 FORWARD / REVERSE，避免阅读代码时还要再翻译机械动作。
 */
typedef enum
{
    STEP_MOTOR_DIR_FORWARD = 0,              /**< 正向：DIR+ 输出高电平，DIR- 输出低电平，机械动作定义为拉。 */
    STEP_MOTOR_DIR_REVERSE = 1,              /**< 反向：DIR+ 输出低电平，DIR- 输出高电平，机械动作定义为推。 */
    STEP_MOTOR_DIR_PULL = STEP_MOTOR_DIR_FORWARD, /**< 拉：业务语义方向，等价于当前正向。 */
    STEP_MOTOR_DIR_PUSH = STEP_MOTOR_DIR_REVERSE, /**< 推：业务语义方向，等价于当前反向。 */
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
 * @brief 步进电机周期处理函数。
 *
 * @note 本函数用于处理梯形加减速：
 *       - 刚启动时从低 PPS 逐步升到目标速度；
 *       - 接近目标步数时逐步降速；
 *       - PWM 脉冲计数仍然由 TIM2 中断完成。
 *
 *       建议在 MachineTask() 这类周期任务中调用，当前任务周期 2ms 足够使用。
 */
void StepMotor_Process(void);

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
 * @return 1 表示启动成功；0 表示参数错误、超过单次有效行程、模块未初始化或已有电机正在运行。
 *
 * @note 本函数非阻塞。启动成功后立即返回，步数统计在 TIM2 PWM 中断中完成，
 *       达到 steps 后自动停止 PWM。当前按 A 轴 100mm、B 轴 150mm 的滑台有效行程
 *       限制单次输出步数，但不记录滑台绝对位置。
 */
uint8_t StepMotor_RunSteps(StepMotorId_e motor,
                           StepMotorDirection_e direction,
                           uint32_t speed_pps,
                           uint32_t steps);

/**
 * @brief 把距离从 0.01mm 单位换算成脉冲数。
 *
 * @param distance_mm_x100 目标距离，单位 0.01mm。比如 3000 表示 30.00mm。
 * @return 换算后的脉冲数。
 *
 * @note 当前换算基于丝杆数据：驱动器细分 1600 脉冲/圈，丝杆导程 4mm/圈，
 *       因此 1mm = 400 个脉冲，30mm = 12000 个脉冲。
 */
uint32_t StepMotor_MmX100ToSteps(uint32_t distance_mm_x100);

/**
 * @brief 把速度从 0.01mm/s 单位换算成 PPS。
 *
 * @param speed_mm_s_x100 目标速度，单位 0.01mm/s。比如 357 表示 3.57mm/s。
 * @return 换算后的 PPS，即每秒需要输出的脉冲数。
 *
 * @note 当前换算基于丝杆数据：驱动器细分 1600 脉冲/圈，丝杆导程 4mm/圈，
 *       因此 1mm/s = 400 PPS。
 */
uint32_t StepMotor_MmPerSecX100ToPps(uint32_t speed_mm_s_x100);

/**
 * @brief 按毫米距离和毫米每秒速度运行指定电机。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @param direction 运行方向，建议使用 STEP_MOTOR_DIR_PULL 或 STEP_MOTOR_DIR_PUSH。
 * @param distance_mm_x100 目标距离，单位 0.01mm。比如 3000 表示 30.00mm。
 * @param speed_mm_s_x100 目标速度，单位 0.01mm/s。比如 1500 表示 15.00mm/s。
 * @return 1 表示启动成功；0 表示参数错误、超过单次有效行程、换算结果为 0 或电机启动失败。
 *
 * @note 本函数非阻塞。内部会把距离换算成 steps，把速度换算成 pps，
 *       再调用 StepMotor_RunSteps() 输出实际脉冲。当前 A 轴单次最大 100mm，
 *       B 轴单次最大 150mm。
 */
uint8_t StepMotor_RunDistanceMmX100(StepMotorId_e motor,
                                    StepMotorDirection_e direction,
                                    uint32_t distance_mm_x100,
                                    uint32_t speed_mm_s_x100);

/**
 * @brief 按厘米距离和厘米每秒速度运行指定电机。
 *
 * @param motor 电机编号，取值见 StepMotorId_e。
 * @param direction 运行方向，建议使用 STEP_MOTOR_DIR_PULL 或 STEP_MOTOR_DIR_PUSH。
 * @param distance_cm_x100 目标距离，单位 0.01cm。比如 300 表示 3.00cm。
 * @param speed_cm_s_x100 目标速度，单位 0.01cm/s。比如 150 表示 1.50cm/s。
 * @return 1 表示启动成功；0 表示参数错误、超过单次有效行程、换算结果为 0 或电机启动失败。
 *
 * @note 该接口便于应用层直接写“几厘米、几厘米每秒”。当前 A 轴单次最大 10cm，
 *       B 轴单次最大 15cm。
 */
uint8_t StepMotor_RunDistanceCmX100(StepMotorId_e motor,
                                    StepMotorDirection_e direction,
                                    uint32_t distance_cm_x100,
                                    uint32_t speed_cm_s_x100);

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
