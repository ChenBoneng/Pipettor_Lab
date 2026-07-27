#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>
#include "main.h"

/**
 * @file bsp_dwt.h
 * @brief STM32F103RCT6 DWT 周期计数器延时与时间戳模块。
 *
 * DWT(Data Watchpoint and Trace) 是 Cortex-M3 内核自带的调试/跟踪单元。
 * 其中 CYCCNT 是一个 32 位 CPU 周期计数器，每经过 1 个 CPU 时钟周期加 1。
 *
 * 本模块用 CYCCNT 做两类事情：
 * - 获取高精度时间间隔；
 * - 实现不依赖 SysTick / FreeRTOS 调度的短延时。
 *
 * @note STM32F103RCT6 是 Cortex-M3 内核，支持 DWT->CYCCNT。
 *       使用前必须先调用 DWT_Init()，并传入当前 CPU 主频，单位 MHz。
 */

/**
 * @brief DWT 软件维护的系统时间。
 *
 * CYCCNT 硬件寄存器只有 32 位，会周期性溢出。
 * 本结构体保存经过溢出扩展后的时间轴，拆成秒、毫秒、微秒三段，便于阅读和计算。
 */
typedef struct
{
    uint32_t s;  /**< 秒部分，从 DWT_Init() 调用成功后开始累计。 */
    uint16_t ms; /**< 当前秒内的毫秒部分，范围 0~999。 */
    uint16_t us; /**< 当前毫秒内的微秒部分，范围 0~999。 */
} DWT_Time_t;

/**
 * @brief 计算一段代码的执行时间，单位秒。
 *
 * @param dt 保存执行时间的变量，类型建议为 float。
 * @param code 需要测量的代码段，可以是一条语句，也可以是花括号包起来的多条语句。
 *
 * @note F407 原版宏会通过日志打印结果，本工程当前没有引入 bsp_log，
 *       因此这里只保留测量功能，不做日志输出。
 */
#define TIME_ELAPSE(dt, code)               \
    do                                      \
    {                                       \
        float dwt_time_start = DWT_GetTimeline_s(); \
        code;                               \
        dt = DWT_GetTimeline_s() - dwt_time_start;  \
    } while (0)

/**
 * @brief 初始化 DWT 周期计数器。
 *
 * @param CPU_Freq_mHz CPU 主频，单位 MHz。STM32F103RCT6 常见配置为 72。
 *
 * @note 该函数会清零 CYCCNT，并开启 Cortex-M3 的 DWT 计数功能。
 *       如果系统时钟配置改变，需要重新调用本函数更新频率参数。
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔，单位秒，float 精度。
 *
 * @param cnt_last 上一次保存的 CYCCNT 计数值指针。
 *                 第一次使用前可先赋值为 DWT->CYCCNT 或 0。
 * @return 距离上一次调用经过的时间，单位秒。
 *
 * @note 函数内部会把 *cnt_last 更新为当前 CYCCNT，方便下次继续计算。
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔，单位秒，double 精度。
 *
 * @param cnt_last 上一次保存的 CYCCNT 计数值指针。
 * @return 距离上一次调用经过的时间，单位秒。
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 获取从 DWT_Init() 后开始累计的时间，单位秒。
 *
 * @return 当前时间轴，单位秒。
 */
float DWT_GetTimeline_s(void);

/**
 * @brief 获取从 DWT_Init() 后开始累计的时间，单位毫秒。
 *
 * @return 当前时间轴，单位毫秒。
 */
float DWT_GetTimeline_ms(void);

/**
 * @brief 获取从 DWT_Init() 后开始累计的时间，单位微秒。
 *
 * @return 当前时间轴，单位微秒。
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief 使用 DWT 实现阻塞延时，单位秒。
 *
 * @param Delay 延时时间，单位秒。例如 0.001f 表示 1ms。
 *
 * @note 本延时只依赖 CPU 周期计数器，不依赖 SysTick。
 *       因此即使在临界区、关闭中断或调度器未启动时，也能用于短延时。
 */
void DWT_Delay(float Delay);

/**
 * @brief 手动更新时间轴。
 *
 * @note CYCCNT 是 32 位计数器，72MHz 下约 59.65 秒溢出一次。
 *       如果长时间不调用任何 DWT_GetTimeline_xx() 函数，建议周期性调用本函数，
 *       否则软件层无法准确统计多次溢出。
 */
void DWT_SysTimeUpdate(void);

#endif //BSP_DWT_H
