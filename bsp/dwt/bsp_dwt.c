#include "bsp_dwt.h"

/*
 * CYCCNT 是 32 位无符号计数器。
 * 从 0xFFFFFFFF 再加 1 会回到 0，因此一次完整计数周期是 2^32。
 */
#define DWT_CYCCNT_PERIOD      (4294967296ULL)
#define DWT_MHZ_TO_HZ          (1000000UL)
#define DWT_MS_PER_SECOND      (1000UL)
#define DWT_US_PER_SECOND      (1000000UL)
#define DWT_US_PER_MS          (1000UL)

static DWT_Time_t dwt_sys_time = {0};
static uint32_t dwt_cpu_freq_hz = 0U;
static uint32_t dwt_cpu_freq_hz_ms = 0U;
static uint32_t dwt_cpu_freq_hz_us = 0U;
static uint32_t dwt_overflow_count = 0U;
static uint32_t dwt_cyccnt_last = 0U;
static uint64_t dwt_cyccnt_64 = 0ULL;
static uint8_t dwt_is_inited = 0U;

static void DWT_CNT_Update(void);

/**
 * @brief 更新 CYCCNT 溢出次数。
 *
 * CYCCNT 是递增计数器，正常情况下新值一定大于等于旧值。
 * 如果本次读到的值小于上一次记录值，说明硬件计数器已经从 0xFFFFFFFF 回绕到 0。
 *
 * @note 本函数假设两次调用之间最多只发生 1 次溢出。
 *       对 72MHz 的 STM32F103 来说，CYCCNT 约 59.65 秒溢出一次；
 *       因此只要业务中周期性调用 DWT_GetTimeline_xx() 或 DWT_SysTimeUpdate() 即可。
 */
static void DWT_CNT_Update(void)
{
    static volatile uint8_t update_lock = 0U;
    uint32_t cnt_now;

    /*
     * 简单的软件锁，避免中断和主循环同时更新时间轴时重复统计溢出。
     * 这里不关闭中断，是为了保持 DWT 模块轻量。
     */
    if (update_lock != 0U)
    {
        return;
    }

    update_lock = 1U;
    cnt_now = DWT->CYCCNT;

    if (cnt_now < dwt_cyccnt_last)
    {
        dwt_overflow_count++;
    }

    dwt_cyccnt_last = cnt_now;
    update_lock = 0U;
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    if (CPU_Freq_mHz == 0U)
    {
        return;
    }

    /*
     * 1. 打开 CoreDebug 的 TRACE 使能位。
     *    没有这个位，DWT 模块不会工作。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /*
     * 2. 清零 CYCCNT。
     *    从初始化时刻开始，所有 timeline 函数都以这个时刻为 0 点。
     */
    DWT->CYCCNT = 0U;

    /*
     * 3. 打开 CYCCNT 计数使能位。
     *    之后 CYCCNT 会按 CPU 主频递增。
     */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dwt_cpu_freq_hz = CPU_Freq_mHz * DWT_MHZ_TO_HZ;
    dwt_cpu_freq_hz_ms = dwt_cpu_freq_hz / DWT_MS_PER_SECOND;
    dwt_cpu_freq_hz_us = dwt_cpu_freq_hz / DWT_US_PER_SECOND;

    dwt_sys_time.s = 0U;
    dwt_sys_time.ms = 0U;
    dwt_sys_time.us = 0U;
    dwt_overflow_count = 0U;
    dwt_cyccnt_last = DWT->CYCCNT;
    dwt_cyccnt_64 = 0ULL;
    dwt_is_inited = 1U;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now;
    uint32_t cnt_delta;

    if ((cnt_last == NULL) || (dwt_is_inited == 0U))
    {
        return 0.0f;
    }

    cnt_now = DWT->CYCCNT;

    /*
     * 无符号减法天然支持 32 位回绕：
     * 例如旧值接近 0xFFFFFFFF，新值回到较小值时，结果仍是正确的周期差。
     */
    cnt_delta = cnt_now - *cnt_last;
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return (float)cnt_delta / (float)dwt_cpu_freq_hz;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now;
    uint32_t cnt_delta;

    if ((cnt_last == NULL) || (dwt_is_inited == 0U))
    {
        return 0.0;
    }

    cnt_now = DWT->CYCCNT;
    cnt_delta = cnt_now - *cnt_last;
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return (double)cnt_delta / (double)dwt_cpu_freq_hz;
}

void DWT_SysTimeUpdate(void)
{
    uint32_t cnt_now;
    uint64_t total_cycles;
    uint64_t total_seconds;
    uint64_t cycles_in_current_second;
    uint64_t cycles_in_current_ms;

    if (dwt_is_inited == 0U)
    {
        return;
    }

    DWT_CNT_Update();
    cnt_now = dwt_cyccnt_last;

    /*
     * 把“溢出次数 + 当前 CYCCNT”拼成 64 位周期数。
     * 注意一次溢出的跨度是 2^32，不是 UINT32_MAX。
     */
    total_cycles = (uint64_t)dwt_overflow_count * DWT_CYCCNT_PERIOD + (uint64_t)cnt_now;
    dwt_cyccnt_64 = total_cycles;

    total_seconds = total_cycles / dwt_cpu_freq_hz;
    cycles_in_current_second = total_cycles - total_seconds * dwt_cpu_freq_hz;
    cycles_in_current_ms = cycles_in_current_second % dwt_cpu_freq_hz_ms;

    dwt_sys_time.s = (uint32_t)total_seconds;
    dwt_sys_time.ms = (uint16_t)(cycles_in_current_second / dwt_cpu_freq_hz_ms);
    dwt_sys_time.us = (uint16_t)(cycles_in_current_ms / dwt_cpu_freq_hz_us);
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    return (float)dwt_sys_time.s +
           (float)dwt_sys_time.ms * 0.001f +
           (float)dwt_sys_time.us * 0.000001f;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    return (float)dwt_sys_time.s * 1000.0f +
           (float)dwt_sys_time.ms +
           (float)dwt_sys_time.us * 0.001f;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    return (uint64_t)dwt_sys_time.s * DWT_US_PER_SECOND +
           (uint64_t)dwt_sys_time.ms * DWT_US_PER_MS +
           (uint64_t)dwt_sys_time.us;
}

void DWT_Delay(float Delay)
{
    uint32_t tick_start;
    uint32_t wait_cycles;

    if ((dwt_is_inited == 0U) || (Delay <= 0.0f))
    {
        return;
    }

    tick_start = DWT->CYCCNT;
    wait_cycles = (uint32_t)(Delay * (float)dwt_cpu_freq_hz);

    /*
     * 使用无符号减法判断经过的周期数。
     * 这样即使等待期间 CYCCNT 发生一次回绕，短延时仍然正确。
     */
    while ((uint32_t)(DWT->CYCCNT - tick_start) < wait_cycles)
    {
    }

    DWT_CNT_Update();
}
