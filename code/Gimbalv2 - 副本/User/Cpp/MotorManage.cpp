#include "MotorManage.hpp"

#include "bsp_can.h"
#include "gimbal_config.h"
#include "motor_manage_c.h"

namespace {
float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
} // namespace

MotorManage::MotorManage()
    : yaw_(YAW_MOTOR_CAN_ID,
           YAW_MAX_CURRENT_DEFAULT,
           YAW_LIMIT_POS_DEG,
           YAW_LIMIT_NEG_DEG,
           YAW_ENCODER_ZERO_RAW),
      pitch_(PITCH_MOTOR_CAN_ID,
             PITCH_MAX_CURRENT_DEFAULT,
             PITCH_LIMIT_POS_DEG,
             PITCH_LIMIT_NEG_DEG,
             PITCH_ENCODER_ZERO_RAW),
      initialized_(false),
      locked_(false),
      disabled_(false)
{
}

bool MotorManage::init()
{
    yaw_.init();
    pitch_.init();

    yaw_pid_speed_.init(g_yaw_spd_pid_param);
    yaw_pid_location_.init(g_yaw_pos_pid_param);
    pitch_pid_speed_.init(g_pitch_spd_pid_param);
    pitch_pid_location_.init(g_pitch_pos_pid_param);

    initialized_ = true;
    locked_ = false;
    disabled_ = false;
    send_zero_current();
    return true;
}

void MotorManage::update_feedback(uint32_t now_ms)
{
    CanRxFrame frame = {0};
    uint32_t rx_time_ms = 0U;

    if (!initialized_) {
        return;
    }

    if (bsp_can_get_latest(yaw_.can_id(), &frame, &rx_time_ms)) {
        yaw_.update(frame, rx_time_ms);
    }

    if (bsp_can_get_latest(pitch_.can_id(), &frame, &rx_time_ms)) {
        pitch_.update(frame, rx_time_ms);
    }

    (void)now_ms;
}

void MotorManage::set(float yaw_target_deg, float pitch_target_deg)
{
    if ((!initialized_) || disabled_) {
        send_zero_current();
        return;
    }

    const GM6020::State &yaw_state = yaw_.state();
    const GM6020::State &pitch_state = pitch_.state();
    const float clamped_yaw_target =
        clampf_local(yaw_target_deg, YAW_LIMIT_NEG_DEG, YAW_LIMIT_POS_DEG);
    const float clamped_pitch_target =
        clampf_local(pitch_target_deg, PITCH_LIMIT_NEG_DEG, PITCH_LIMIT_POS_DEG);
    float yaw_speed_ref = 0.0f;
    float pitch_speed_ref = 0.0f;
    int16_t yaw_current = 0;
    int16_t pitch_current = 0;

    locked_ = false;
    sync_pid_params();
    yaw_speed_ref = yaw_pid_location_.calc(clamped_yaw_target, yaw_state.angle_deg);
    pitch_speed_ref = pitch_pid_location_.calc(clamped_pitch_target, pitch_state.angle_deg);
    yaw_current = yaw_.clamp_current(yaw_pid_speed_.calc(yaw_speed_ref, yaw_state.speed_dps));
    pitch_current =
        pitch_.clamp_current(pitch_pid_speed_.calc(pitch_speed_ref, pitch_state.speed_dps));
    (void)bsp_can_send_gimbal_currents(yaw_current, pitch_current);
}

void MotorManage::lock()
{
    if (!initialized_) {
        return;
    }

    locked_ = true;
    reset_controllers();
    send_zero_current();
}

void MotorManage::disable()
{
    if (!initialized_) {
        return;
    }

    disabled_ = true;
    locked_ = true;
    reset_controllers();
    send_zero_current();
}

uint32_t MotorManage::check(uint32_t now_ms) const
{
    uint32_t faults = MOTOR_FAULT_NONE;

    if (!yaw_.online(now_ms)) {
        faults |= MOTOR_FAULT_YAW_OFFLINE;
    }

    if (!pitch_.online(now_ms)) {
        faults |= MOTOR_FAULT_PITCH_OFFLINE;
    }

    if (yaw_.limit_error()) {
        faults |= MOTOR_FAULT_YAW_LIMIT;
    }

    if (pitch_.limit_error()) {
        faults |= MOTOR_FAULT_PITCH_LIMIT;
    }

    return faults;
}

void MotorManage::sync_pid_params()
{
    yaw_pid_speed_.set_param(g_yaw_spd_pid_param);
    yaw_pid_location_.set_param(g_yaw_pos_pid_param);
    pitch_pid_speed_.set_param(g_pitch_spd_pid_param);
    pitch_pid_location_.set_param(g_pitch_pos_pid_param);
}

void MotorManage::reset_controllers()
{
    yaw_pid_speed_.reset();
    yaw_pid_location_.reset();
    pitch_pid_speed_.reset();
    pitch_pid_location_.reset();
}

void MotorManage::send_zero_current() const
{
    (void)bsp_can_send_gimbal_currents(0, 0);
}
