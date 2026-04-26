//
// Created by CORE on 2026/4/9.
//

#include <cmath>

#include "MotorManage.h"

#include "bsp_can.h"
#include "cmsis_os2.h"

namespace
{

// Temporary debug bypass. Set to 0 to restore online checks.
#define TEMP_DISABLE_MOTOR_ONLINE_CHECK 1
constexpr uint32_t k_inner_loop_hz = 1000U;
constexpr uint32_t k_outer_loop_hz = 200U;
static_assert(k_outer_loop_hz > 0U, "k_outer_loop_hz must be greater than 0");
static_assert((k_inner_loop_hz % k_outer_loop_hz) == 0U, "k_inner_loop_hz must be divisible by k_outer_loop_hz");
constexpr uint8_t k_outer_loop_divider = static_cast<uint8_t>(k_inner_loop_hz / k_outer_loop_hz);
constexpr float k_inner_dt_default_s = 1.0f / (float)k_inner_loop_hz;
constexpr float k_outer_dt_default_s = 1.0f / (float)k_outer_loop_hz;
constexpr float k_tick_to_s = 0.001f;
constexpr int16_t k_current_cmd_limit = 10000;
constexpr float k_yaw_limit_min_deg = -60.0f;
constexpr float k_yaw_limit_max_deg = 60.0f;
constexpr float k_yaw_limit_margin_deg = 1.0f;
constexpr float k_pitch_zero_abs_deg = 60.0f;
constexpr float k_pitch_limit_min_deg = -10.0f;
constexpr float k_pitch_limit_max_deg = 40.0f;
constexpr float k_planner_pos_snap_deg = 0.01f;
constexpr float k_planner_vel_snap_dps = 0.1f;

constexpr float k_small_track_yaw_pos_kp_scale = 1.35f;
constexpr float k_small_track_pitch_pos_kp_scale = 1.10f;
constexpr float k_small_track_yaw_speed_ki_scale = 0.20f;
constexpr float k_small_track_output_limit_scale = 0.55f;
constexpr float k_small_track_ff_gain_scale = 1.00f;

constexpr float k_spin_yaw_pos_kp_scale = 0.65f;
constexpr float k_spin_pitch_pos_kp_scale = 1.05f;
constexpr float k_spin_yaw_speed_ki_scale = 0.12f;
constexpr float k_spin_output_limit_scale = 0.70f;
constexpr float k_spin_ff_gain_scale = 1.10f;

constexpr float k_lost_track_yaw_pos_kp_scale = 0.75f;
constexpr float k_lost_track_yaw_speed_ki_scale = 0.10f;
constexpr float k_lost_track_output_limit_scale = 0.60f;
constexpr float k_lost_track_ff_gain_scale = 0.90f;

float absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float signf(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }

    if (value < 0.0f)
    {
        return -1.0f;
    }

    return 0.0f;
}

int16_t clamp_current_cmd(float value, int16_t limit)
{
    if (value > (float)limit)
    {
        return limit;
    }

    if (value < -(float)limit)
    {
        return static_cast<int16_t>(-limit);
    }

    if (value >= 0.0f)
    {
        return static_cast<int16_t>(value + 0.5f);
    }

    return static_cast<int16_t>(value - 0.5f);
}

bool yaw_is_pushing_outward(float yaw_joint_deg, int16_t yaw_current_cmd)
{
    if ((yaw_joint_deg >= (k_yaw_limit_max_deg - k_yaw_limit_margin_deg)) &&
        (yaw_current_cmd > 0))
    {
        return true;
    }

    if ((yaw_joint_deg <= (k_yaw_limit_min_deg + k_yaw_limit_margin_deg)) &&
        (yaw_current_cmd < 0))
    {
        return true;
    }

    return false;
}

float compute_measured_dt_s(uint32_t now_tick_ms, uint32_t &last_tick_ms, float default_dt_s)
{
    if (last_tick_ms == 0U)
    {
        last_tick_ms = now_tick_ms;
        return default_dt_s;
    }

    const uint32_t delta_tick_ms = now_tick_ms - last_tick_ms;
    last_tick_ms = now_tick_ms;

    if (delta_tick_ms == 0U)
    {
        return default_dt_s;
    }

    return (float)delta_tick_ms * k_tick_to_s;
}

float get_output_limit_scale(MotorManageAimMode_e aim_mode)
{
    switch (aim_mode)
    {
        case MOTOR_AIM_MODE_SMALL_TRACK:
            return k_small_track_output_limit_scale;

        case MOTOR_AIM_MODE_SPIN_TRACK:
            return k_spin_output_limit_scale;

        case MOTOR_AIM_MODE_TRACK_LOST:
            return k_lost_track_output_limit_scale;

        case MOTOR_AIM_MODE_NONE:
        case MOTOR_AIM_MODE_LARGE_MOVE:
        default:
            return 1.0f;
    }
}

float get_yaw_ff_gain_scale(MotorManageAimMode_e aim_mode)
{
    switch (aim_mode)
    {
        case MOTOR_AIM_MODE_SMALL_TRACK:
            return k_small_track_ff_gain_scale;

        case MOTOR_AIM_MODE_SPIN_TRACK:
            return k_spin_ff_gain_scale;

        case MOTOR_AIM_MODE_TRACK_LOST:
            return k_lost_track_ff_gain_scale;

        case MOTOR_AIM_MODE_NONE:
        case MOTOR_AIM_MODE_LARGE_MOVE:
        default:
            return 1.0f;
    }
}

} // namespace

#ifdef DDBUG_DATA_ON
volatile MotorManageDebugData g_motor_manage_debug = {0};
#endif

volatile MotorManageRuntimeParams g_motor_manage_runtime_params = {
    18.0f, 0.0f, 0.0f,
    30.0f, 5.0f, 0.0f,
    18.0f, 4.0f, 0.0f,
    15.0f, 3.0f, 0.0f,
    180.0f, 720.0f, 50.0f, 0.0f,
    120.0f, 480.0f, 0.08f, 0.0f
};

MotorManage::MotorManage()
    : yaw_(0x206, k_current_cmd_limit, k_yaw_limit_max_deg, k_yaw_limit_min_deg),
      pitch_(0x208, k_current_cmd_limit, k_pitch_limit_max_deg, k_pitch_limit_min_deg),
      yaw_zero_ready_(false),
      yaw_boot_zero_deg_(0.0f),
      outer_loop_divider_count_(0U),
      last_inner_tick_ms_(0U),
      last_outer_tick_ms_(0U),
      yaw_speed_target_cache_(0.0f),
      pitch_speed_target_cache_(0.0f),
      yaw_planner_state_{0.0f, 0.0f, 0.0f, false},
      pitch_planner_state_{0.0f, 0.0f, 0.0f, false},
      yaw_ff_state_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      pitch_ff_state_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      yaw_planner_config_{180.0f, 720.0f, 20.0f},
      pitch_planner_config_{120.0f, 480.0f, 0.08f},
      runtime_params_{18.0f, 0.0f, 0.0f,
                      25.0f, 5.0f, 0.0f,
                      18.0f, 4.0f, 0.0f,
                      15.0f, 3.0f, 0.0f,
                      180.0f, 720.0f, 20.0f, 0.0f,
                      120.0f, 480.0f, 0.08f, 0.0f}
{
    yaw_.init();
    pitch_.init();
    yaw_pid_location_.init(15.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.1f);
    yaw_pid_speed_.init(4.0f, 1.0f, 0.0f, 10000.0f, 10000.0f, 0.1f);
    pitch_pid_location_.init(15.0f, 0.0f, 0.0f, 100.0f, 10000.0f, 0.1f);
    pitch_pid_speed_.init(4.0f, 1.0f, 0.0f, 100.0f, 10000.0f, 0.1f);
    sync_runtime_params_from_global();
}

void MotorManage::update_feedback()
{
    yaw_.update();
    pitch_.update();
}

void MotorManage::send_can_cmd()
{
    uint8_t tx_data[8] = {0};
    const int16_t yaw_current = yaw_.get_target().target_current;
    const int16_t pitch_current = pitch_.get_target().target_current;

    tx_data[2] = (uint8_t)(yaw_current >> 8);
    tx_data[3] = (uint8_t)(yaw_current & 0xFF);
    tx_data[6] = (uint8_t)(pitch_current >> 8);
    tx_data[7] = (uint8_t)(pitch_current & 0xFF);

    bsp_tx(0x1FF, tx_data, sizeof(tx_data));
}

void MotorManage::copy_runtime_params(MotorManageRuntimeParams *dst, const MotorManageRuntimeParams &src)
{
    if (dst == nullptr)
    {
        return;
    }

    dst->yaw_location_kp = src.yaw_location_kp;
    dst->yaw_location_ki = src.yaw_location_ki;
    dst->yaw_location_kd = src.yaw_location_kd;
    dst->yaw_speed_kp = src.yaw_speed_kp;
    dst->yaw_speed_ki = src.yaw_speed_ki;
    dst->yaw_speed_kd = src.yaw_speed_kd;
    dst->pitch_location_kp = src.pitch_location_kp;
    dst->pitch_location_ki = src.pitch_location_ki;
    dst->pitch_location_kd = src.pitch_location_kd;
    dst->pitch_speed_kp = src.pitch_speed_kp;
    dst->pitch_speed_ki = src.pitch_speed_ki;
    dst->pitch_speed_kd = src.pitch_speed_kd;
    dst->yaw_max_vel_dps = src.yaw_max_vel_dps;
    dst->yaw_max_acc_dps2 = src.yaw_max_acc_dps2;
    dst->yaw_k_vel_ff = src.yaw_k_vel_ff;
    dst->yaw_hold_ff = src.yaw_hold_ff;
    dst->pitch_max_vel_dps = src.pitch_max_vel_dps;
    dst->pitch_max_acc_dps2 = src.pitch_max_acc_dps2;
    dst->pitch_k_vel_ff = src.pitch_k_vel_ff;
    dst->pitch_hold_ff = src.pitch_hold_ff;
}

void MotorManage::copy_runtime_params(MotorManageRuntimeParams *dst, const volatile MotorManageRuntimeParams &src)
{
    if (dst == nullptr)
    {
        return;
    }

    dst->yaw_location_kp = src.yaw_location_kp;
    dst->yaw_location_ki = src.yaw_location_ki;
    dst->yaw_location_kd = src.yaw_location_kd;
    dst->yaw_speed_kp = src.yaw_speed_kp;
    dst->yaw_speed_ki = src.yaw_speed_ki;
    dst->yaw_speed_kd = src.yaw_speed_kd;
    dst->pitch_location_kp = src.pitch_location_kp;
    dst->pitch_location_ki = src.pitch_location_ki;
    dst->pitch_location_kd = src.pitch_location_kd;
    dst->pitch_speed_kp = src.pitch_speed_kp;
    dst->pitch_speed_ki = src.pitch_speed_ki;
    dst->pitch_speed_kd = src.pitch_speed_kd;
    dst->yaw_max_vel_dps = src.yaw_max_vel_dps;
    dst->yaw_max_acc_dps2 = src.yaw_max_acc_dps2;
    dst->yaw_k_vel_ff = src.yaw_k_vel_ff;
    dst->yaw_hold_ff = src.yaw_hold_ff;
    dst->pitch_max_vel_dps = src.pitch_max_vel_dps;
    dst->pitch_max_acc_dps2 = src.pitch_max_acc_dps2;
    dst->pitch_k_vel_ff = src.pitch_k_vel_ff;
    dst->pitch_hold_ff = src.pitch_hold_ff;
}

void MotorManage::copy_runtime_params(volatile MotorManageRuntimeParams *dst, const MotorManageRuntimeParams &src)
{
    if (dst == nullptr)
    {
        return;
    }

    dst->yaw_location_kp = src.yaw_location_kp;
    dst->yaw_location_ki = src.yaw_location_ki;
    dst->yaw_location_kd = src.yaw_location_kd;
    dst->yaw_speed_kp = src.yaw_speed_kp;
    dst->yaw_speed_ki = src.yaw_speed_ki;
    dst->yaw_speed_kd = src.yaw_speed_kd;
    dst->pitch_location_kp = src.pitch_location_kp;
    dst->pitch_location_ki = src.pitch_location_ki;
    dst->pitch_location_kd = src.pitch_location_kd;
    dst->pitch_speed_kp = src.pitch_speed_kp;
    dst->pitch_speed_ki = src.pitch_speed_ki;
    dst->pitch_speed_kd = src.pitch_speed_kd;
    dst->yaw_max_vel_dps = src.yaw_max_vel_dps;
    dst->yaw_max_acc_dps2 = src.yaw_max_acc_dps2;
    dst->yaw_k_vel_ff = src.yaw_k_vel_ff;
    dst->yaw_hold_ff = src.yaw_hold_ff;
    dst->pitch_max_vel_dps = src.pitch_max_vel_dps;
    dst->pitch_max_acc_dps2 = src.pitch_max_acc_dps2;
    dst->pitch_k_vel_ff = src.pitch_k_vel_ff;
    dst->pitch_hold_ff = src.pitch_hold_ff;
}

void MotorManage::apply_runtime_params(const MotorManageRuntimeParams &params)
{
    runtime_params_ = params;

    yaw_pid_location_.set_kp(runtime_params_.yaw_location_kp);
    yaw_pid_location_.set_ki(runtime_params_.yaw_location_ki);
    yaw_pid_location_.set_kd(runtime_params_.yaw_location_kd);
    yaw_pid_speed_.set_kp(runtime_params_.yaw_speed_kp);
    yaw_pid_speed_.set_ki(runtime_params_.yaw_speed_ki);
    yaw_pid_speed_.set_kd(runtime_params_.yaw_speed_kd);

    pitch_pid_location_.set_kp(runtime_params_.pitch_location_kp);
    pitch_pid_location_.set_ki(runtime_params_.pitch_location_ki);
    pitch_pid_location_.set_kd(runtime_params_.pitch_location_kd);
    pitch_pid_speed_.set_kp(runtime_params_.pitch_speed_kp);
    pitch_pid_speed_.set_ki(runtime_params_.pitch_speed_ki);
    pitch_pid_speed_.set_kd(runtime_params_.pitch_speed_kd);

    yaw_planner_config_.max_vel_dps = (runtime_params_.yaw_max_vel_dps > 0.0f) ? runtime_params_.yaw_max_vel_dps : 1.0f;
    yaw_planner_config_.max_acc_dps2 = (runtime_params_.yaw_max_acc_dps2 > 0.0f) ? runtime_params_.yaw_max_acc_dps2 : 1.0f;
    yaw_planner_config_.k_vel_ff = runtime_params_.yaw_k_vel_ff;

    pitch_planner_config_.max_vel_dps = (runtime_params_.pitch_max_vel_dps > 0.0f) ? runtime_params_.pitch_max_vel_dps : 1.0f;
    pitch_planner_config_.max_acc_dps2 = (runtime_params_.pitch_max_acc_dps2 > 0.0f) ? runtime_params_.pitch_max_acc_dps2 : 1.0f;
    pitch_planner_config_.k_vel_ff = runtime_params_.pitch_k_vel_ff;
}

void MotorManage::apply_mode_tuning(MotorManageAimMode_e aim_mode)
{
    float yaw_location_kp = runtime_params_.yaw_location_kp;
    float yaw_speed_ki = runtime_params_.yaw_speed_ki;
    float pitch_location_kp = runtime_params_.pitch_location_kp;

    switch (aim_mode)
    {
        case MOTOR_AIM_MODE_SMALL_TRACK:
            yaw_location_kp *= k_small_track_yaw_pos_kp_scale;
            yaw_speed_ki *= k_small_track_yaw_speed_ki_scale;
            pitch_location_kp *= k_small_track_pitch_pos_kp_scale;
            break;

        case MOTOR_AIM_MODE_SPIN_TRACK:
            yaw_location_kp *= k_spin_yaw_pos_kp_scale;
            yaw_speed_ki *= k_spin_yaw_speed_ki_scale;
            pitch_location_kp *= k_spin_pitch_pos_kp_scale;
            break;

        case MOTOR_AIM_MODE_TRACK_LOST:
            yaw_location_kp *= k_lost_track_yaw_pos_kp_scale;
            yaw_speed_ki *= k_lost_track_yaw_speed_ki_scale;
            pitch_location_kp *= k_spin_pitch_pos_kp_scale;
            break;

        case MOTOR_AIM_MODE_NONE:
        case MOTOR_AIM_MODE_LARGE_MOVE:
        default:
            break;
    }

    yaw_pid_location_.set_kp(yaw_location_kp);
    yaw_pid_location_.set_ki(runtime_params_.yaw_location_ki);
    yaw_pid_location_.set_kd(runtime_params_.yaw_location_kd);
    yaw_pid_speed_.set_kp(runtime_params_.yaw_speed_kp);
    yaw_pid_speed_.set_ki(yaw_speed_ki);
    yaw_pid_speed_.set_kd(runtime_params_.yaw_speed_kd);

    pitch_pid_location_.set_kp(pitch_location_kp);
    pitch_pid_location_.set_ki(runtime_params_.pitch_location_ki);
    pitch_pid_location_.set_kd(runtime_params_.pitch_location_kd);
    pitch_pid_speed_.set_kp(runtime_params_.pitch_speed_kp);
    pitch_pid_speed_.set_ki(runtime_params_.pitch_speed_ki);
    pitch_pid_speed_.set_kd(runtime_params_.pitch_speed_kd);
}

void MotorManage::sync_runtime_params_from_global()
{
    MotorManageRuntimeParams params = {0};
    copy_runtime_params(&params, g_motor_manage_runtime_params);
    apply_runtime_params(params);
}

void MotorManage::set_runtime_params(const MotorManageRuntimeParams &params)
{
    MotorManageRuntimeParams global_params = {0};
    copy_runtime_params(&global_params, params);
    copy_runtime_params(&g_motor_manage_runtime_params, global_params);
    apply_runtime_params(global_params);
}

MotorManageRuntimeParams MotorManage::get_runtime_params() const
{
    return runtime_params_;
}

void MotorManage::clear_feedforward_state(FeedforwardAxisState *ff_state)
{
    if (ff_state == nullptr)
    {
        return;
    }

    ff_state->hold_ff = 0.0f;
    ff_state->boot_bias_ff = 0.0f;
    ff_state->vel_ff = 0.0f;
    ff_state->acc_ff = 0.0f;
    ff_state->ff_total = 0.0f;
}

void MotorManage::clear_planner_state(PlannerAxisState *planner_state)
{
    if (planner_state == nullptr)
    {
        return;
    }

    planner_state->planned_pos_deg = 0.0f;
    planner_state->planned_vel_dps = 0.0f;
    planner_state->planned_acc_dps2 = 0.0f;
    planner_state->initialized = false;
}

void MotorManage::sync_planner_state(PlannerAxisState *planner_state, float measured_pos_deg)
{
    if (planner_state == nullptr)
    {
        return;
    }

    planner_state->planned_pos_deg = measured_pos_deg;
    planner_state->planned_vel_dps = 0.0f;
    planner_state->planned_acc_dps2 = 0.0f;
    planner_state->initialized = true;
}

void MotorManage::update_planner_state(PlannerAxisState *planner_state,
                                       float target_pos_deg,
                                       float dt_s,
                                       const PlannerAxisConfig &config)
{
    if ((planner_state == nullptr) || !planner_state->initialized)
    {
        return;
    }

    const float limited_dt_s = (dt_s > 0.0f) ? dt_s : k_inner_dt_default_s;
    const float pos_error_deg = target_pos_deg - planner_state->planned_pos_deg;
    const float abs_pos_error_deg = absf(pos_error_deg);
    const float abs_planned_vel_dps = absf(planner_state->planned_vel_dps);

    if ((abs_pos_error_deg <= k_planner_pos_snap_deg) &&
        (abs_planned_vel_dps <= k_planner_vel_snap_dps))
    {
        planner_state->planned_pos_deg = target_pos_deg;
        planner_state->planned_vel_dps = 0.0f;
        planner_state->planned_acc_dps2 = 0.0f;
        return;
    }

    const float direction = signf(pos_error_deg);
    const float stop_distance_deg =
        (planner_state->planned_vel_dps * planner_state->planned_vel_dps) /
        (2.0f * config.max_acc_dps2);
    float desired_acc_dps2 = 0.0f;

    if ((abs_pos_error_deg <= stop_distance_deg) && (abs_planned_vel_dps > 0.0f))
    {
        desired_acc_dps2 = -signf(planner_state->planned_vel_dps) * config.max_acc_dps2;
    }
    else if (direction != 0.0f)
    {
        const float vel_error_dps = (direction * config.max_vel_dps) - planner_state->planned_vel_dps;

        if (absf(vel_error_dps) <= (config.max_acc_dps2 * limited_dt_s))
        {
            desired_acc_dps2 = vel_error_dps / limited_dt_s;
        }
        else
        {
            desired_acc_dps2 = direction * config.max_acc_dps2;
        }
    }

    float next_vel_dps = planner_state->planned_vel_dps + (desired_acc_dps2 * limited_dt_s);
    if (next_vel_dps > config.max_vel_dps)
    {
        next_vel_dps = config.max_vel_dps;
    }
    else if (next_vel_dps < -config.max_vel_dps)
    {
        next_vel_dps = -config.max_vel_dps;
    }

    if ((planner_state->planned_vel_dps > 0.0f) && (next_vel_dps < 0.0f))
    {
        next_vel_dps = 0.0f;
    }
    else if ((planner_state->planned_vel_dps < 0.0f) && (next_vel_dps > 0.0f))
    {
        next_vel_dps = 0.0f;
    }

    float next_pos_deg = planner_state->planned_pos_deg +
                         (planner_state->planned_vel_dps * limited_dt_s) +
                         (0.5f * desired_acc_dps2 * limited_dt_s * limited_dt_s);
    const float remain_error_deg = target_pos_deg - next_pos_deg;

    if ((direction != 0.0f) && (signf(remain_error_deg) != direction))
    {
        next_pos_deg = target_pos_deg;
        next_vel_dps = 0.0f;
        desired_acc_dps2 = 0.0f;
    }

    planner_state->planned_pos_deg = next_pos_deg;
    planner_state->planned_vel_dps = next_vel_dps;
    planner_state->planned_acc_dps2 = desired_acc_dps2;
}

void MotorManage::set_control_reference(const MotorControlReference &reference,
                                        float yaw_meas_deg,
                                        float pitch_meas_deg)
{
    sync_runtime_params_from_global();
    apply_mode_tuning(reference.aim_mode);

    const auto yaw_state = yaw_.get_state();
    const auto pitch_state = pitch_.get_state();

    const uint32_t ticks = osKernelGetTickCount();
    const float inner_dt_s = compute_measured_dt_s(ticks, last_inner_tick_ms_, k_inner_dt_default_s);

    const bool run_outer_loop = (outer_loop_divider_count_ == 0U);
    outer_loop_divider_count_++;
    if (outer_loop_divider_count_ >= k_outer_loop_divider)
    {
        outer_loop_divider_count_ = 0U;
    }

    float outer_dt_s = k_outer_dt_default_s;
    if (run_outer_loop)
    {
        outer_dt_s = compute_measured_dt_s(ticks, last_outer_tick_ms_, k_outer_dt_default_s);
    }

    if (!yaw_zero_ready_ && yaw_.has_feedback())
    {
        yaw_boot_zero_deg_ = yaw_state.angle_deg;
        yaw_zero_ready_ = true;
        yaw_pid_location_.reset();
        yaw_pid_speed_.reset();
        yaw_speed_target_cache_ = 0.0f;
    }

#if TEMP_DISABLE_MOTOR_ONLINE_CHECK
    const bool yaw_online = true;
    const bool pitch_online = true;
#else
    const bool yaw_online = yaw_.check(ticks);
    const bool pitch_online = pitch_.check(ticks);
#endif

    const float yaw_meas_world_deg = clamp_target_deg(yaw_meas_deg, k_yaw_limit_min_deg, k_yaw_limit_max_deg);
    const float pitch_meas_world_deg = clamp_target_deg(pitch_meas_deg, k_pitch_limit_min_deg, k_pitch_limit_max_deg);

    const float yaw_ref_target_deg = clamp_target_deg(reference.yaw.pos_ref_deg, k_yaw_limit_min_deg, k_yaw_limit_max_deg);
    const float pitch_ref_target_deg = clamp_target_deg(reference.pitch.pos_ref_deg, k_pitch_limit_min_deg, k_pitch_limit_max_deg);

    float yaw_exec_pos = yaw_ref_target_deg;
    float yaw_exec_vel = reference.yaw.vel_ref_dps;
    float yaw_exec_acc = reference.yaw.acc_ref_dps2;
    float pitch_exec_pos = pitch_ref_target_deg;
    float pitch_exec_vel = reference.pitch.vel_ref_dps;
    float pitch_exec_acc = reference.pitch.acc_ref_dps2;

    float yaw_speed_target = yaw_speed_target_cache_;
    float pitch_speed_target = pitch_speed_target_cache_;
    float yaw_current_target = 0.0f;
    float pitch_current_target = 0.0f;
    int16_t yaw_current_cmd = 0;
    int16_t pitch_current_cmd = 0;

    const int16_t current_cmd_limit = clamp_current_cmd((float)k_current_cmd_limit *
                                                            get_output_limit_scale(reference.aim_mode),
                                                        k_current_cmd_limit);

    if (!yaw_zero_ready_ || !yaw_online)
    {
        hold_yaw_axis();
        yaw_exec_pos = yaw_planner_state_.planned_pos_deg;
        yaw_exec_vel = yaw_planner_state_.planned_vel_dps;
        yaw_exec_acc = yaw_planner_state_.planned_acc_dps2;
    }
    else
    {
        const float yaw_joint_deg = yaw_motor_to_joint_deg(yaw_state.angle_deg);

        if (reference.yaw.reference_mode == MOTOR_REFERENCE_MODE_PLANNER)
        {
            if (!yaw_planner_state_.initialized)
            {
                sync_planner_state(&yaw_planner_state_, yaw_meas_world_deg);
            }

            update_planner_state(&yaw_planner_state_, yaw_ref_target_deg, inner_dt_s, yaw_planner_config_);
            yaw_exec_pos = yaw_planner_state_.planned_pos_deg;
            yaw_exec_vel = yaw_planner_state_.planned_vel_dps;
            yaw_exec_acc = yaw_planner_state_.planned_acc_dps2;
        }
        else
        {
            sync_planner_state(&yaw_planner_state_, yaw_meas_world_deg);
        }

        if (run_outer_loop)
        {
            float yaw_speed_correction = yaw_pid_location_.calculate(yaw_exec_pos, yaw_meas_world_deg, outer_dt_s);

            if ((reference.yaw.reference_mode == MOTOR_REFERENCE_MODE_DIRECT) ||
                (reference.aim_mode == MOTOR_AIM_MODE_SMALL_TRACK) ||
                (reference.aim_mode == MOTOR_AIM_MODE_SPIN_TRACK) ||
                (reference.aim_mode == MOTOR_AIM_MODE_TRACK_LOST))
            {
                yaw_speed_correction += yaw_exec_vel;
            }

            yaw_speed_target_cache_ = yaw_speed_correction;
        }

        yaw_speed_target = yaw_speed_target_cache_;
        const float yaw_current_pid = yaw_pid_speed_.calculate(yaw_speed_target, yaw_state.speed_dps, inner_dt_s);
        clear_feedforward_state(&yaw_ff_state_);
        yaw_ff_state_.hold_ff = runtime_params_.yaw_hold_ff;
        yaw_ff_state_.vel_ff =
            (runtime_params_.yaw_k_vel_ff * get_yaw_ff_gain_scale(reference.aim_mode)) * yaw_exec_vel;
        yaw_ff_state_.ff_total = yaw_ff_state_.hold_ff +
                                 yaw_ff_state_.boot_bias_ff +
                                 yaw_ff_state_.vel_ff +
                                 yaw_ff_state_.acc_ff;
        yaw_current_target = yaw_current_pid + yaw_ff_state_.ff_total;
        yaw_current_cmd = clamp_current_cmd(yaw_current_target, current_cmd_limit);

        if (yaw_is_pushing_outward(yaw_joint_deg, yaw_current_cmd))
        {
            hold_yaw_axis();
            yaw_current_target = 0.0f;
            yaw_current_cmd = 0;
        }

        yaw_.set_target_speed_dps(yaw_speed_target);
        yaw_.set_target_current(yaw_current_cmd);
    }

    if (!pitch_online)
    {
        hold_pitch_axis();
        pitch_exec_pos = pitch_planner_state_.planned_pos_deg;
        pitch_exec_vel = pitch_planner_state_.planned_vel_dps;
        pitch_exec_acc = pitch_planner_state_.planned_acc_dps2;
    }
    else
    {
        if (reference.pitch.reference_mode == MOTOR_REFERENCE_MODE_PLANNER)
        {
            if (!pitch_planner_state_.initialized)
            {
                sync_planner_state(&pitch_planner_state_, pitch_meas_world_deg);
            }

            update_planner_state(&pitch_planner_state_, pitch_ref_target_deg, inner_dt_s, pitch_planner_config_);
            pitch_exec_pos = pitch_planner_state_.planned_pos_deg;
            pitch_exec_vel = pitch_planner_state_.planned_vel_dps;
            pitch_exec_acc = pitch_planner_state_.planned_acc_dps2;
        }
        else
        {
            sync_planner_state(&pitch_planner_state_, pitch_meas_world_deg);
        }

        if (run_outer_loop)
        {
            float pitch_speed_correction =
                -pitch_pid_location_.calculate(pitch_exec_pos, pitch_meas_world_deg, outer_dt_s);

            if ((reference.pitch.reference_mode == MOTOR_REFERENCE_MODE_DIRECT) ||
                (reference.aim_mode == MOTOR_AIM_MODE_SMALL_TRACK) ||
                (reference.aim_mode == MOTOR_AIM_MODE_SPIN_TRACK) ||
                (reference.aim_mode == MOTOR_AIM_MODE_TRACK_LOST))
            {
                pitch_speed_correction += (-pitch_exec_vel);
            }

            pitch_speed_target_cache_ = pitch_speed_correction;
        }

        pitch_speed_target = pitch_speed_target_cache_;
        const float pitch_current_pid = pitch_pid_speed_.calculate(pitch_speed_target, pitch_state.speed_dps, inner_dt_s);
        clear_feedforward_state(&pitch_ff_state_);
        pitch_ff_state_.hold_ff = runtime_params_.pitch_hold_ff;
        pitch_ff_state_.vel_ff = runtime_params_.pitch_k_vel_ff * (-pitch_exec_vel);
        pitch_ff_state_.ff_total = pitch_ff_state_.hold_ff +
                                   pitch_ff_state_.boot_bias_ff +
                                   pitch_ff_state_.vel_ff +
                                   pitch_ff_state_.acc_ff;
        pitch_current_target = pitch_current_pid + pitch_ff_state_.ff_total;
        pitch_current_cmd = clamp_current_cmd(pitch_current_target, current_cmd_limit);
        pitch_.set_target_speed_dps(pitch_speed_target);
        pitch_.set_target_current(pitch_current_cmd);
    }

#ifdef DDBUG_DATA_ON
    g_motor_manage_debug.yaw.target_deg = yaw_ref_target_deg;
    g_motor_manage_debug.pitch.target_deg = pitch_ref_target_deg;
    g_motor_manage_debug.yaw.ref_deg = yaw_exec_pos;
    g_motor_manage_debug.pitch.ref_deg = pitch_exec_pos;
    g_motor_manage_debug.yaw.speed_target_dps = yaw_speed_target;
    g_motor_manage_debug.pitch.speed_target_dps = pitch_speed_target;
    g_motor_manage_debug.yaw.ref_speed_dps = yaw_exec_vel;
    g_motor_manage_debug.pitch.ref_speed_dps = pitch_exec_vel;
    g_motor_manage_debug.yaw.ref_acc_dps2 = yaw_exec_acc;
    g_motor_manage_debug.pitch.ref_acc_dps2 = pitch_exec_acc;
    g_motor_manage_debug.yaw.current_pid = yaw_current_target - yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch.current_pid = pitch_current_target - pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw.current_ff = yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch.current_ff = pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw.hold_ff = yaw_ff_state_.hold_ff;
    g_motor_manage_debug.pitch.hold_ff = pitch_ff_state_.hold_ff;
    g_motor_manage_debug.yaw.boot_bias_ff = yaw_ff_state_.boot_bias_ff;
    g_motor_manage_debug.pitch.boot_bias_ff = pitch_ff_state_.boot_bias_ff;
    g_motor_manage_debug.yaw.vel_ff = yaw_ff_state_.vel_ff;
    g_motor_manage_debug.pitch.vel_ff = pitch_ff_state_.vel_ff;
    g_motor_manage_debug.yaw.acc_ff = yaw_ff_state_.acc_ff;
    g_motor_manage_debug.pitch.acc_ff = pitch_ff_state_.acc_ff;
    g_motor_manage_debug.yaw.ff_total = yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch.ff_total = pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw.current_cmd = yaw_.get_target().target_current;
    g_motor_manage_debug.pitch.current_cmd = pitch_.get_target().target_current;
    g_motor_manage_debug.yaw.meas_deg = yaw_meas_world_deg;
    g_motor_manage_debug.pitch.meas_deg = pitch_meas_world_deg;
    g_motor_manage_debug.yaw.meas_speed_dps = yaw_state.speed_dps;
    g_motor_manage_debug.pitch.meas_speed_dps = pitch_state.speed_dps;
    g_motor_manage_debug.yaw.current_meas = yaw_state.current;
    g_motor_manage_debug.pitch.current_meas = pitch_state.current;
    g_motor_manage_debug.yaw_angle_t = yaw_ref_target_deg;
    g_motor_manage_debug.pitch_angle_t = pitch_ref_target_deg;
    g_motor_manage_debug.yaw_planned_angle_t = yaw_exec_pos;
    g_motor_manage_debug.pitch_planned_angle_t = pitch_exec_pos;
    g_motor_manage_debug.yaw_speed_t = yaw_speed_target;
    g_motor_manage_debug.pitch_speed_t = pitch_speed_target;
    g_motor_manage_debug.yaw_planned_speed_t = yaw_exec_vel;
    g_motor_manage_debug.pitch_planned_speed_t = pitch_exec_vel;
    g_motor_manage_debug.yaw_planned_acc_t = yaw_exec_acc;
    g_motor_manage_debug.pitch_planned_acc_t = pitch_exec_acc;
    g_motor_manage_debug.yaw_current_pid = yaw_current_target - yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch_current_pid = pitch_current_target - pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw_current_ff = yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch_current_ff = pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw_hold_ff = yaw_ff_state_.hold_ff;
    g_motor_manage_debug.pitch_hold_ff = pitch_ff_state_.hold_ff;
    g_motor_manage_debug.yaw_boot_bias_ff = yaw_ff_state_.boot_bias_ff;
    g_motor_manage_debug.pitch_boot_bias_ff = pitch_ff_state_.boot_bias_ff;
    g_motor_manage_debug.yaw_vel_ff = yaw_ff_state_.vel_ff;
    g_motor_manage_debug.pitch_vel_ff = pitch_ff_state_.vel_ff;
    g_motor_manage_debug.yaw_acc_ff = yaw_ff_state_.acc_ff;
    g_motor_manage_debug.pitch_acc_ff = pitch_ff_state_.acc_ff;
    g_motor_manage_debug.yaw_ff_total = yaw_ff_state_.ff_total;
    g_motor_manage_debug.pitch_ff_total = pitch_ff_state_.ff_total;
    g_motor_manage_debug.yaw_current_cmd = yaw_.get_target().target_current;
    g_motor_manage_debug.pitch_current_cmd = pitch_.get_target().target_current;
    g_motor_manage_debug.yaw_angle_meas_deg = yaw_meas_world_deg;
    g_motor_manage_debug.pitch_angle_meas_deg = pitch_meas_world_deg;
    g_motor_manage_debug.yaw_speed_meas_dps = yaw_state.speed_dps;
    g_motor_manage_debug.pitch_speed_meas_dps = pitch_state.speed_dps;
    g_motor_manage_debug.yaw_current_meas = yaw_state.current;
    g_motor_manage_debug.pitch_current_meas = pitch_state.current;
    g_motor_manage_debug.dt_s = inner_dt_s;
    g_motor_manage_debug.tick_ms = ticks;
#endif
}

void MotorManage::set_world_target(float yaw_target_deg,
                                   float pitch_target_deg,
                                   float yaw_meas_deg,
                                   float pitch_meas_deg,
                                   bool enable_planner)
{
    MotorControlReference reference = {};

    reference.aim_mode = MOTOR_AIM_MODE_NONE;
    reference.yaw.pos_ref_deg = yaw_target_deg;
    reference.yaw.vel_ref_dps = 0.0f;
    reference.yaw.acc_ref_dps2 = 0.0f;
    reference.yaw.reference_mode = enable_planner ? MOTOR_REFERENCE_MODE_PLANNER : MOTOR_REFERENCE_MODE_DIRECT;
    reference.pitch.pos_ref_deg = pitch_target_deg;
    reference.pitch.vel_ref_dps = 0.0f;
    reference.pitch.acc_ref_dps2 = 0.0f;
    reference.pitch.reference_mode = enable_planner ? MOTOR_REFERENCE_MODE_PLANNER : MOTOR_REFERENCE_MODE_DIRECT;

    set_control_reference(reference, yaw_meas_deg, pitch_meas_deg);
}

void MotorManage::lock()
{
    uint8_t tx_data[8] = {0};

    clear_planner_state(&yaw_planner_state_);
    clear_planner_state(&pitch_planner_state_);
    clear_feedforward_state(&yaw_ff_state_);
    clear_feedforward_state(&pitch_ff_state_);
    hold_yaw_axis();
    hold_pitch_axis();
    bsp_tx(0xFE, tx_data, sizeof(tx_data));
}

GM6020::Target MotorManage::get_yaw_target() const
{
    return yaw_.get_target();
}

GM6020::Target MotorManage::get_pitch_target() const
{
    return pitch_.get_target();
}

float MotorManage::get_yaw_joint_deg() const
{
    if (!yaw_zero_ready_ || !yaw_.has_feedback())
    {
        return 0.0f;
    }

    return yaw_motor_to_joint_deg(yaw_.get_state().angle_deg);
}

float MotorManage::get_pitch_joint_deg() const
{
    if (!pitch_.has_feedback())
    {
        return 0.0f;
    }

    return pitch_motor_to_joint_deg(pitch_.get_state().angle_deg);
}

float MotorManage::clamp_target_deg(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

float MotorManage::yaw_motor_to_joint_deg(float motor_angle_deg) const
{
    return motor_angle_deg - yaw_boot_zero_deg_;
}

float MotorManage::pitch_motor_to_joint_deg(float motor_angle_deg)
{
    return motor_angle_deg - k_pitch_zero_abs_deg;
}

void MotorManage::hold_yaw_axis()
{
    yaw_pid_location_.reset();
    yaw_pid_speed_.reset();
    yaw_speed_target_cache_ = 0.0f;
    clear_planner_state(&yaw_planner_state_);
    clear_feedforward_state(&yaw_ff_state_);
    yaw_.set_target_speed_dps(0.0f);
    yaw_.set_target_current(0);
}

void MotorManage::hold_pitch_axis()
{
    pitch_pid_location_.reset();
    pitch_pid_speed_.reset();
    pitch_speed_target_cache_ = 0.0f;
    clear_planner_state(&pitch_planner_state_);
    clear_feedforward_state(&pitch_ff_state_);
    pitch_.set_target_speed_dps(0.0f);
    pitch_.set_target_current(0);
}
