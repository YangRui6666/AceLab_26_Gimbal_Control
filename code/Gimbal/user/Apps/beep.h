#ifndef GIMBAL_BEEP_H
#define GIMBAL_BEEP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YAW_BEEP_MIN_FREQ_HZ 50U
#define YAW_BEEP_MAX_FREQ_HZ 1000U

/**
 * @brief 以阻塞方式驱动 yaw 轴电机发声
 * @param frequency_hz 目标声音频率(Hz)
 * @param duration_ms 持续时间(ms)
 * @return true=鸣叫完成且已安全停机，false=参数非法或中途发送失败
 */
bool beep_yaw_blocking(uint16_t frequency_hz, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_BEEP_H
