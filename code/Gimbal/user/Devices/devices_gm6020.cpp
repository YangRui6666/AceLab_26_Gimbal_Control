#include "devices_gm6020.h"

#include "motor_manage.hpp"

#include "Config/pid_config.h"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr float kDegreesPerRawCount = 360.0f / 8192.0f;

MotorManager& motorManager()
{
    static MotorManager manager;
    return manager;
}

bool buildFeedback(const MotorSnapshot& snapshot, devices_gm6020_feedback_t* out)
{
    if (out == nullptr)
    {
        return false;
    }

    const uint32_t now_tick = static_cast<uint32_t>(xTaskGetTickCount());

    out->position_deg = static_cast<float>(snapshot.total_angle) * kDegreesPerRawCount;
    out->velocity_rpm = static_cast<float>(snapshot.speed);
    out->current_ma = static_cast<float>(snapshot.current);
    out->filtered_current_ma = snapshot.filtered_current;
    out->temp = snapshot.temp;
    out->last_rx_tick = snapshot.last_rx_tick;
    out->online = snapshot.last_rx_tick != 0U &&
                  (now_tick - snapshot.last_rx_tick) <= pdMS_TO_TICKS(MOTOR_COMM_TIMEOUT_MS);

    return true;
}
}

extern "C" {

void devices_gimbal_poll(void)
{
    motorManager().pollCanRx();
}

bool devices_gimbal_get_yaw_feedback(devices_gm6020_feedback_t* out)
{
    return buildFeedback(motorManager().yawSnapshot(), out);
}

bool devices_gimbal_get_pitch_feedback(devices_gm6020_feedback_t* out)
{
    return buildFeedback(motorManager().pitchSnapshot(), out);
}

void devices_gimbal_set_currents(int16_t yaw_current, int16_t pitch_current)
{
    motorManager().setYawCurrent(yaw_current);
    motorManager().setPitchCurrent(pitch_current);
}

bool devices_gimbal_send(void)
{
    return motorManager().sendCurrentCommands();
}

void devices_gimbal_stop(void)
{
    devices_gimbal_set_currents(0, 0);
}

}
