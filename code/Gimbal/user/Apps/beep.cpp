#include <cstdint>

#include "Apps/beep.h"

#include "main.h"
#include "Devices/devices_gm6020.h"

namespace
{
constexpr std::int16_t kYawBeepCurrentMa = 2000;
constexpr std::uint32_t kMicrosecondsPerSecond = 1000000U;
constexpr std::uint32_t kMicrosecondsPerMillisecond = 1000U;
constexpr std::uint32_t kHalfCycleFactor = 500000U;
constexpr std::uint32_t kMinHalfPeriodUs = kHalfCycleFactor / YAW_BEEP_MAX_FREQ_HZ;

/**
 * @brief 初始化 DWT 周期计数器
 * @return true=DWT 可用，false=当前平台不支持或启用失败
 */
bool dwt_delay_init(void)
{
    static bool initialized = false;
    static bool available = false;

    if (initialized)
    {
        return available;
    }

    initialized = true;
    SystemCoreClockUpdate();

#if defined(CoreDebug) && defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_CTRL_NOCYCCNT_Msk)
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return false;
    }
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    available = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
#endif

    return available;
}

/**
 * @brief 基于 DWT 的微秒级阻塞延时
 * @param delay_us 延时时长(us)
 * @return true=延时完成，false=DWT 不可用或参数无效
 */
bool dwt_delay_us(std::uint32_t delay_us)
{
    if (delay_us == 0U)
    {
        return true;
    }

    if (!dwt_delay_init() || SystemCoreClock == 0U)
    {
        return false;
    }

    const std::uint64_t cycles = (static_cast<std::uint64_t>(SystemCoreClock) * delay_us) /
                                 static_cast<std::uint64_t>(kMicrosecondsPerSecond);
    if (cycles == 0U || cycles > 0xFFFFFFFFULL)
    {
        return false;
    }

    const std::uint32_t start = DWT->CYCCNT;
    const std::uint32_t wait_cycles = static_cast<std::uint32_t>(cycles);

    while (static_cast<std::uint32_t>(DWT->CYCCNT - start) < wait_cycles)
    {
    }

    return true;
}

/**
 * @brief 下发当前半周期的 yaw 电流
 * @param yaw_current Yaw 目标电流(mA)
 * @return true=发送成功，false=发送失败
 */
bool send_yaw_current(std::int16_t yaw_current)
{
    devices_gimbal_set_currents(yaw_current, 0);
    return devices_gimbal_send();
}

/**
 * @brief 安全停止云台电流输出
 * @return true=停止命令发送成功，false=停止发送失败
 */
bool stop_beep_output(void)
{
    devices_gimbal_stop();
    return devices_gimbal_send();
}
}

extern "C" bool beep_yaw_blocking(std::uint16_t frequency_hz, std::uint32_t duration_ms)
{
    if (frequency_hz < YAW_BEEP_MIN_FREQ_HZ ||
        frequency_hz > YAW_BEEP_MAX_FREQ_HZ ||
        duration_ms == 0U)
    {
        return false;
    }

    const std::uint32_t half_period_us = kHalfCycleFactor / frequency_hz;
    if (half_period_us == 0U || half_period_us < kMinHalfPeriodUs)
    {
        return false;
    }

    if (!dwt_delay_init())
    {
        return false;
    }

    std::uint64_t remaining_us = static_cast<std::uint64_t>(duration_ms) *
                                 static_cast<std::uint64_t>(kMicrosecondsPerMillisecond);
    std::int16_t output_current = kYawBeepCurrentMa;
    bool ok = true;

    while (remaining_us > 0U)
    {
        ok = send_yaw_current(output_current);
        if (!ok)
        {
            break;
        }

        std::uint32_t wait_us = half_period_us;
        if (remaining_us < static_cast<std::uint64_t>(half_period_us))
        {
            wait_us = static_cast<std::uint32_t>(remaining_us);
        }

        ok = dwt_delay_us(wait_us);
        if (!ok)
        {
            break;
        }

        remaining_us -= wait_us;
        output_current = static_cast<std::int16_t>(-output_current);
    }

    const bool stop_ok = stop_beep_output();
    return ok && stop_ok;
}
