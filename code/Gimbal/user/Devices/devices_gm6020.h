#ifndef GIMBAL_DEVICES_GM6020_H
#define GIMBAL_DEVICES_GM6020_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    // 反馈数据采用任务层统一格式，避免上层直接依赖底层快照结构。
    float position_deg;
    float velocity_rpm;
    float current_ma;
    float filtered_current_ma;
    int8_t temp;
    uint32_t last_rx_tick;
    bool online;
} devices_gm6020_feedback_t;

// 轮询设备层 CAN 接收，并刷新内部状态缓存。
void devices_gimbal_poll(void);
// 获取 yaw 轴反馈快照。
bool devices_gimbal_get_yaw_feedback(devices_gm6020_feedback_t* out);
// 获取 pitch 轴反馈快照。
bool devices_gimbal_get_pitch_feedback(devices_gm6020_feedback_t* out);
// 设置两轴目标电流，尚未发送。
void devices_gimbal_set_currents(int16_t yaw_current, int16_t pitch_current);
// 发送当前电流命令到 CAN 总线。
bool devices_gimbal_send(void);
// 立即清零电流命令，用于保护场景。
void devices_gimbal_stop(void);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_DEVICES_GM6020_H
