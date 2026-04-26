//
// Created by Codex on 2026/4/26.
//

#include "debug_tune.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "SEGGER_RTT.h"

namespace
{

constexpr unsigned k_rtt_buffer_index = 0U;
constexpr float k_two_pi = 6.28318530718f;
constexpr uint32_t k_default_sample_period_ms = 10U;
constexpr uint32_t k_min_sample_period_ms = 1U;
constexpr uint32_t k_default_hold_ms = 1000U;
constexpr uint32_t k_default_ramp_ms = 1000U;

DebugTuneState s_tune = {
    false,
    DEBUG_TUNE_MODE_OFF,
    DEBUG_TUNE_AXIS_YAW,
    false,
    0U,
    k_default_sample_period_ms,
    false,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    k_default_hold_ms,
    k_default_ramp_ms
};
bool s_disabled_since_last_check = false;
bool s_sample_header_sent = false;
uint32_t s_last_sample_tick_ms = 0U;

float wrap_elapsed_s(uint32_t now_tick_ms, uint32_t start_tick_ms)
{
    return (float)(now_tick_ms - start_tick_ms) * 0.001f;
}

float step_wave(float amp, uint32_t elapsed_ms, uint32_t hold_ms)
{
    const uint32_t segment_ms = (hold_ms == 0U) ? k_default_hold_ms : hold_ms;
    const uint32_t phase = (elapsed_ms / segment_ms) % 5U;

    switch (phase)
    {
        case 1U:
            return amp;

        case 3U:
            return -amp;

        case 0U:
        case 2U:
        case 4U:
        default:
            return 0.0f;
    }
}

float ramp_wave(float amp, uint32_t elapsed_ms, uint32_t ramp_ms, uint32_t hold_ms)
{
    const uint32_t limited_ramp_ms = (ramp_ms == 0U) ? k_default_ramp_ms : ramp_ms;
    const uint32_t limited_hold_ms = (hold_ms == 0U) ? k_default_hold_ms : hold_ms;
    const uint32_t step_ms = limited_ramp_ms + limited_hold_ms;
    const uint32_t cycle_ms = step_ms * 4U;
    uint32_t phase_ms = (cycle_ms == 0U) ? 0U : (elapsed_ms % cycle_ms);

    if (phase_ms < limited_ramp_ms)
    {
        return amp * ((float)phase_ms / (float)limited_ramp_ms);
    }

    if (phase_ms < step_ms)
    {
        return amp;
    }

    phase_ms -= step_ms;
    if (phase_ms < limited_ramp_ms)
    {
        return amp * (1.0f - ((float)phase_ms / (float)limited_ramp_ms));
    }

    if (phase_ms < step_ms)
    {
        return 0.0f;
    }

    phase_ms -= step_ms;
    if (phase_ms < limited_ramp_ms)
    {
        return -amp * ((float)phase_ms / (float)limited_ramp_ms);
    }

    if (phase_ms < step_ms)
    {
        return -amp;
    }

    phase_ms -= step_ms;
    if (phase_ms < limited_ramp_ms)
    {
        return -amp * (1.0f - ((float)phase_ms / (float)limited_ramp_ms));
    }

    return 0.0f;
}

float sine_wave(float amp, float freq_hz, float elapsed_s)
{
    return amp * sinf(k_two_pi * freq_hz * elapsed_s);
}

void reset_output_state(void)
{
    s_sample_header_sent = false;
    s_last_sample_tick_ms = 0U;
}

void format_fixed(char *buffer, size_t buffer_size, float value, uint32_t decimals)
{
    int32_t scale = 1;
    bool negative = false;
    float abs_value = value;
    int32_t scaled = 0;
    int32_t whole = 0;
    int32_t frac = 0;

    if ((buffer == nullptr) || (buffer_size == 0U))
    {
        return;
    }

    if (value < 0.0f)
    {
        negative = true;
        abs_value = -value;
    }

    for (uint32_t i = 0U; i < decimals; ++i)
    {
        scale *= 10;
    }

    scaled = (int32_t)(abs_value * (float)scale + 0.5f);
    whole = scaled / scale;
    frac = scaled % scale;

    if (decimals == 0U)
    {
        (void)std::snprintf(buffer, buffer_size, "%s%ld", negative ? "-" : "", (long)whole);
        return;
    }

    (void)std::snprintf(buffer,
                        buffer_size,
                        "%s%ld.%0*ld",
                        negative ? "-" : "",
                        (long)whole,
                        (int)decimals,
                        (long)frac);
}

void print_param_line(const char *name, float value)
{
    char text[24] = {0};
    format_fixed(text, sizeof(text), value, 6U);
    SEGGER_RTT_printf(k_rtt_buffer_index, "#PARAM %s=%s\n", name, text);
}

MotorManageRuntimeParams copy_params_from_global(void)
{
    MotorManageRuntimeParams params = {};

    params.yaw_location_kp = g_motor_manage_runtime_params.yaw_location_kp;
    params.yaw_location_ki = g_motor_manage_runtime_params.yaw_location_ki;
    params.yaw_location_kd = g_motor_manage_runtime_params.yaw_location_kd;
    params.yaw_speed_kp = g_motor_manage_runtime_params.yaw_speed_kp;
    params.yaw_speed_ki = g_motor_manage_runtime_params.yaw_speed_ki;
    params.yaw_speed_kd = g_motor_manage_runtime_params.yaw_speed_kd;
    params.pitch_location_kp = g_motor_manage_runtime_params.pitch_location_kp;
    params.pitch_location_ki = g_motor_manage_runtime_params.pitch_location_ki;
    params.pitch_location_kd = g_motor_manage_runtime_params.pitch_location_kd;
    params.pitch_speed_kp = g_motor_manage_runtime_params.pitch_speed_kp;
    params.pitch_speed_ki = g_motor_manage_runtime_params.pitch_speed_ki;
    params.pitch_speed_kd = g_motor_manage_runtime_params.pitch_speed_kd;
    params.yaw_max_vel_dps = g_motor_manage_runtime_params.yaw_max_vel_dps;
    params.yaw_max_acc_dps2 = g_motor_manage_runtime_params.yaw_max_acc_dps2;
    params.yaw_k_vel_ff = g_motor_manage_runtime_params.yaw_k_vel_ff;
    params.yaw_hold_ff = g_motor_manage_runtime_params.yaw_hold_ff;
    params.pitch_max_vel_dps = g_motor_manage_runtime_params.pitch_max_vel_dps;
    params.pitch_max_acc_dps2 = g_motor_manage_runtime_params.pitch_max_acc_dps2;
    params.pitch_k_vel_ff = g_motor_manage_runtime_params.pitch_k_vel_ff;
    params.pitch_hold_ff = g_motor_manage_runtime_params.pitch_hold_ff;

    return params;
}

void copy_params_to_global(const MotorManageRuntimeParams &params)
{
    g_motor_manage_runtime_params.yaw_location_kp = params.yaw_location_kp;
    g_motor_manage_runtime_params.yaw_location_ki = params.yaw_location_ki;
    g_motor_manage_runtime_params.yaw_location_kd = params.yaw_location_kd;
    g_motor_manage_runtime_params.yaw_speed_kp = params.yaw_speed_kp;
    g_motor_manage_runtime_params.yaw_speed_ki = params.yaw_speed_ki;
    g_motor_manage_runtime_params.yaw_speed_kd = params.yaw_speed_kd;
    g_motor_manage_runtime_params.pitch_location_kp = params.pitch_location_kp;
    g_motor_manage_runtime_params.pitch_location_ki = params.pitch_location_ki;
    g_motor_manage_runtime_params.pitch_location_kd = params.pitch_location_kd;
    g_motor_manage_runtime_params.pitch_speed_kp = params.pitch_speed_kp;
    g_motor_manage_runtime_params.pitch_speed_ki = params.pitch_speed_ki;
    g_motor_manage_runtime_params.pitch_speed_kd = params.pitch_speed_kd;
    g_motor_manage_runtime_params.yaw_max_vel_dps = params.yaw_max_vel_dps;
    g_motor_manage_runtime_params.yaw_max_acc_dps2 = params.yaw_max_acc_dps2;
    g_motor_manage_runtime_params.yaw_k_vel_ff = params.yaw_k_vel_ff;
    g_motor_manage_runtime_params.yaw_hold_ff = params.yaw_hold_ff;
    g_motor_manage_runtime_params.pitch_max_vel_dps = params.pitch_max_vel_dps;
    g_motor_manage_runtime_params.pitch_max_acc_dps2 = params.pitch_max_acc_dps2;
    g_motor_manage_runtime_params.pitch_k_vel_ff = params.pitch_k_vel_ff;
    g_motor_manage_runtime_params.pitch_hold_ff = params.pitch_hold_ff;
}

float *find_param(MotorManageRuntimeParams *params, const char *name)
{
    if ((params == nullptr) || (name == nullptr))
    {
        return nullptr;
    }

    if (strcmp(name, "yaw_location_kp") == 0) { return &params->yaw_location_kp; }
    if (strcmp(name, "yaw_location_ki") == 0) { return &params->yaw_location_ki; }
    if (strcmp(name, "yaw_location_kd") == 0) { return &params->yaw_location_kd; }
    if (strcmp(name, "yaw_speed_kp") == 0) { return &params->yaw_speed_kp; }
    if (strcmp(name, "yaw_speed_ki") == 0) { return &params->yaw_speed_ki; }
    if (strcmp(name, "yaw_speed_kd") == 0) { return &params->yaw_speed_kd; }
    if (strcmp(name, "pitch_location_kp") == 0) { return &params->pitch_location_kp; }
    if (strcmp(name, "pitch_location_ki") == 0) { return &params->pitch_location_ki; }
    if (strcmp(name, "pitch_location_kd") == 0) { return &params->pitch_location_kd; }
    if (strcmp(name, "pitch_speed_kp") == 0) { return &params->pitch_speed_kp; }
    if (strcmp(name, "pitch_speed_ki") == 0) { return &params->pitch_speed_ki; }
    if (strcmp(name, "pitch_speed_kd") == 0) { return &params->pitch_speed_kd; }
    if (strcmp(name, "yaw_max_vel_dps") == 0) { return &params->yaw_max_vel_dps; }
    if (strcmp(name, "yaw_max_acc_dps2") == 0) { return &params->yaw_max_acc_dps2; }
    if (strcmp(name, "yaw_k_vel_ff") == 0) { return &params->yaw_k_vel_ff; }
    if (strcmp(name, "yaw_hold_ff") == 0) { return &params->yaw_hold_ff; }
    if (strcmp(name, "pitch_max_vel_dps") == 0) { return &params->pitch_max_vel_dps; }
    if (strcmp(name, "pitch_max_acc_dps2") == 0) { return &params->pitch_max_acc_dps2; }
    if (strcmp(name, "pitch_k_vel_ff") == 0) { return &params->pitch_k_vel_ff; }
    if (strcmp(name, "pitch_hold_ff") == 0) { return &params->pitch_hold_ff; }

    return nullptr;
}

void set_angle_axis_targets(float wave, float *yaw_target, float *pitch_target)
{
    if ((yaw_target == nullptr) || (pitch_target == nullptr))
    {
        return;
    }

    switch (s_tune.axis)
    {
        case DEBUG_TUNE_AXIS_YAW:
            *yaw_target = s_tune.bias + wave;
            *pitch_target = s_tune.other;
            break;

        case DEBUG_TUNE_AXIS_PITCH:
            *yaw_target = s_tune.other;
            *pitch_target = s_tune.bias + wave;
            break;

        case DEBUG_TUNE_AXIS_BOTH:
        default:
            *yaw_target = s_tune.bias + wave;
            *pitch_target = s_tune.bias + wave;
            break;
    }
}

void set_speed_axis_targets(float wave,
                            float current_yaw_deg,
                            float current_pitch_deg,
                            MotorControlReference *reference)
{
    if (reference == nullptr)
    {
        return;
    }

    reference->aim_mode = MOTOR_AIM_MODE_NONE;
    reference->yaw.reference_mode = MOTOR_REFERENCE_MODE_DIRECT;
    reference->pitch.reference_mode = MOTOR_REFERENCE_MODE_DIRECT;
    reference->yaw.pos_ref_deg = current_yaw_deg;
    reference->pitch.pos_ref_deg = current_pitch_deg;
    reference->yaw.acc_ref_dps2 = 0.0f;
    reference->pitch.acc_ref_dps2 = 0.0f;
    reference->yaw.vel_ref_dps = 0.0f;
    reference->pitch.vel_ref_dps = 0.0f;

    if (s_tune.axis == DEBUG_TUNE_AXIS_YAW)
    {
        reference->yaw.vel_ref_dps = wave;
        reference->pitch.pos_ref_deg = s_tune.other;
    }
    else if (s_tune.axis == DEBUG_TUNE_AXIS_PITCH)
    {
        reference->yaw.pos_ref_deg = s_tune.other;
        reference->pitch.vel_ref_dps = wave;
    }
}

} // namespace

bool debug_tune_is_enabled(void)
{
    return s_tune.enabled;
}

bool debug_tune_was_disabled(bool clear_flag)
{
    const bool was_disabled = s_disabled_since_last_check;
    if (clear_flag)
    {
        s_disabled_since_last_check = false;
    }

    return was_disabled;
}

DebugTuneState debug_tune_get_state(void)
{
    return s_tune;
}

void debug_tune_disable(void)
{
    if (s_tune.enabled)
    {
        s_disabled_since_last_check = true;
    }

    s_tune.enabled = false;
    s_tune.mode = DEBUG_TUNE_MODE_OFF;
    reset_output_state();
}

void debug_tune_set_sample(bool enabled, uint32_t period_ms)
{
    s_tune.sample_enabled = enabled;
    s_tune.sample_period_ms = (period_ms < k_min_sample_period_ms) ? k_min_sample_period_ms : period_ms;
    reset_output_state();
}

bool debug_tune_enable_hold(float yaw_deg, float pitch_deg, bool planner_enabled, uint32_t now_tick_ms)
{
    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_HOLD;
    s_tune.axis = DEBUG_TUNE_AXIS_BOTH;
    s_tune.planner_enabled = planner_enabled;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.yaw_deg = yaw_deg;
    s_tune.pitch_deg = pitch_deg;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_step(DebugTuneAxis axis,
                            float amp,
                            uint32_t hold_ms,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms)
{
    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_STEP;
    s_tune.axis = axis;
    s_tune.planner_enabled = planner_enabled;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.amp = amp;
    s_tune.hold_ms = (hold_ms == 0U) ? k_default_hold_ms : hold_ms;
    s_tune.bias = bias;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_sine(DebugTuneAxis axis,
                            float amp,
                            float freq_hz,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms)
{
    if (freq_hz <= 0.0f)
    {
        return false;
    }

    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_SINE;
    s_tune.axis = axis;
    s_tune.planner_enabled = planner_enabled;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.amp = amp;
    s_tune.freq_hz = freq_hz;
    s_tune.bias = bias;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_ramp(DebugTuneAxis axis,
                            float amp,
                            uint32_t ramp_ms,
                            uint32_t hold_ms,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms)
{
    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_RAMP;
    s_tune.axis = axis;
    s_tune.planner_enabled = planner_enabled;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.amp = amp;
    s_tune.ramp_ms = (ramp_ms == 0U) ? k_default_ramp_ms : ramp_ms;
    s_tune.hold_ms = (hold_ms == 0U) ? k_default_hold_ms : hold_ms;
    s_tune.bias = bias;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_speed_hold(DebugTuneAxis axis,
                                  float speed_dps,
                                  float other,
                                  uint32_t now_tick_ms)
{
    if (axis == DEBUG_TUNE_AXIS_BOTH)
    {
        return false;
    }

    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_SPEED_HOLD;
    s_tune.axis = axis;
    s_tune.planner_enabled = false;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.speed_dps = speed_dps;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_speed_step(DebugTuneAxis axis,
                                  float amp_dps,
                                  uint32_t hold_ms,
                                  float other,
                                  uint32_t now_tick_ms)
{
    if (axis == DEBUG_TUNE_AXIS_BOTH)
    {
        return false;
    }

    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_SPEED_STEP;
    s_tune.axis = axis;
    s_tune.planner_enabled = false;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.amp = amp_dps;
    s_tune.hold_ms = (hold_ms == 0U) ? k_default_hold_ms : hold_ms;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_enable_speed_sine(DebugTuneAxis axis,
                                  float amp_dps,
                                  float freq_hz,
                                  float other,
                                  uint32_t now_tick_ms)
{
    if ((axis == DEBUG_TUNE_AXIS_BOTH) || (freq_hz <= 0.0f))
    {
        return false;
    }

    s_tune.enabled = true;
    s_tune.mode = DEBUG_TUNE_MODE_SPEED_SINE;
    s_tune.axis = axis;
    s_tune.planner_enabled = false;
    s_tune.start_tick_ms = now_tick_ms;
    s_tune.amp = amp_dps;
    s_tune.freq_hz = freq_hz;
    s_tune.other = other;
    s_disabled_since_last_check = false;
    reset_output_state();
    return true;
}

bool debug_tune_eval(uint32_t now_tick_ms,
                     float current_yaw_deg,
                     float current_pitch_deg,
                     MotorControlReference *reference)
{
    if ((reference == nullptr) || !s_tune.enabled)
    {
        return false;
    }

    const uint32_t elapsed_ms = now_tick_ms - s_tune.start_tick_ms;
    const float elapsed_s = wrap_elapsed_s(now_tick_ms, s_tune.start_tick_ms);
    float yaw_target = current_yaw_deg;
    float pitch_target = current_pitch_deg;
    float wave = 0.0f;

    switch (s_tune.mode)
    {
        case DEBUG_TUNE_MODE_HOLD:
            yaw_target = s_tune.yaw_deg;
            pitch_target = s_tune.pitch_deg;
            break;

        case DEBUG_TUNE_MODE_STEP:
            wave = step_wave(s_tune.amp, elapsed_ms, s_tune.hold_ms);
            set_angle_axis_targets(wave, &yaw_target, &pitch_target);
            break;

        case DEBUG_TUNE_MODE_SINE:
            wave = sine_wave(s_tune.amp, s_tune.freq_hz, elapsed_s);
            set_angle_axis_targets(wave, &yaw_target, &pitch_target);
            break;

        case DEBUG_TUNE_MODE_RAMP:
            wave = ramp_wave(s_tune.amp, elapsed_ms, s_tune.ramp_ms, s_tune.hold_ms);
            set_angle_axis_targets(wave, &yaw_target, &pitch_target);
            break;

        case DEBUG_TUNE_MODE_SPEED_HOLD:
            set_speed_axis_targets(s_tune.speed_dps, current_yaw_deg, current_pitch_deg, reference);
            return true;

        case DEBUG_TUNE_MODE_SPEED_STEP:
            wave = step_wave(s_tune.amp, elapsed_ms, s_tune.hold_ms);
            set_speed_axis_targets(wave, current_yaw_deg, current_pitch_deg, reference);
            return true;

        case DEBUG_TUNE_MODE_SPEED_SINE:
            wave = sine_wave(s_tune.amp, s_tune.freq_hz, elapsed_s);
            set_speed_axis_targets(wave, current_yaw_deg, current_pitch_deg, reference);
            return true;

        case DEBUG_TUNE_MODE_OFF:
        default:
            return false;
    }

    reference->aim_mode = MOTOR_AIM_MODE_NONE;
    reference->yaw.pos_ref_deg = yaw_target;
    reference->yaw.vel_ref_dps = 0.0f;
    reference->yaw.acc_ref_dps2 = 0.0f;
    reference->yaw.reference_mode = s_tune.planner_enabled ? MOTOR_REFERENCE_MODE_PLANNER : MOTOR_REFERENCE_MODE_DIRECT;
    reference->pitch.pos_ref_deg = pitch_target;
    reference->pitch.vel_ref_dps = 0.0f;
    reference->pitch.acc_ref_dps2 = 0.0f;
    reference->pitch.reference_mode = s_tune.planner_enabled ? MOTOR_REFERENCE_MODE_PLANNER : MOTOR_REFERENCE_MODE_DIRECT;
    return true;
}

bool debug_tune_set_param(const char *name, float value)
{
    MotorManageRuntimeParams params = copy_params_from_global();
    float *param = find_param(&params, name);

    if (param == nullptr)
    {
        return false;
    }

    *param = value;
    copy_params_to_global(params);
    return true;
}

bool debug_tune_get_param(const char *name, float *value)
{
    MotorManageRuntimeParams params = copy_params_from_global();
    float *param = find_param(&params, name);

    if ((param == nullptr) || (value == nullptr))
    {
        return false;
    }

    *value = *param;
    return true;
}

void debug_tune_print_params(void)
{
    const MotorManageRuntimeParams params = copy_params_from_global();

    print_param_line("yaw_location_kp", params.yaw_location_kp);
    print_param_line("yaw_location_ki", params.yaw_location_ki);
    print_param_line("yaw_location_kd", params.yaw_location_kd);
    print_param_line("yaw_speed_kp", params.yaw_speed_kp);
    print_param_line("yaw_speed_ki", params.yaw_speed_ki);
    print_param_line("yaw_speed_kd", params.yaw_speed_kd);
    print_param_line("pitch_location_kp", params.pitch_location_kp);
    print_param_line("pitch_location_ki", params.pitch_location_ki);
    print_param_line("pitch_location_kd", params.pitch_location_kd);
    print_param_line("pitch_speed_kp", params.pitch_speed_kp);
    print_param_line("pitch_speed_ki", params.pitch_speed_ki);
    print_param_line("pitch_speed_kd", params.pitch_speed_kd);
    print_param_line("yaw_max_vel_dps", params.yaw_max_vel_dps);
    print_param_line("yaw_max_acc_dps2", params.yaw_max_acc_dps2);
    print_param_line("yaw_k_vel_ff", params.yaw_k_vel_ff);
    print_param_line("yaw_hold_ff", params.yaw_hold_ff);
    print_param_line("pitch_max_vel_dps", params.pitch_max_vel_dps);
    print_param_line("pitch_max_acc_dps2", params.pitch_max_acc_dps2);
    print_param_line("pitch_k_vel_ff", params.pitch_k_vel_ff);
    print_param_line("pitch_hold_ff", params.pitch_hold_ff);
    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "#STATE enabled=%u mode=%s axis=%s sample=%u rate_ms=%lu\n",
                      s_tune.enabled ? 1U : 0U,
                      debug_tune_mode_name(s_tune.mode),
                      debug_tune_axis_name(s_tune.axis),
                      s_tune.sample_enabled ? 1U : 0U,
                      (unsigned long)s_tune.sample_period_ms);
}

void debug_tune_emit_sample_if_due(uint32_t now_tick_ms)
{
#ifdef DDBUG_DATA_ON
    if (!s_tune.sample_enabled)
    {
        return;
    }

    if ((s_last_sample_tick_ms != 0U) &&
        ((uint32_t)(now_tick_ms - s_last_sample_tick_ms) < s_tune.sample_period_ms))
    {
        return;
    }

    if (!s_sample_header_sent)
    {
        SEGGER_RTT_WriteString(k_rtt_buffer_index,
                               "kind,tick_ms,mode,axis,target_yaw_deg,ref_yaw_deg,meas_yaw_deg,"
                               "target_pitch_deg,ref_pitch_deg,meas_pitch_deg,yaw_speed_target_dps,"
                               "yaw_meas_speed_dps,pitch_speed_target_dps,pitch_meas_speed_dps,"
                               "yaw_current_pid,yaw_current_ff,yaw_current_cmd,yaw_current_meas,"
                               "pitch_current_pid,pitch_current_ff,pitch_current_cmd,pitch_current_meas\n");
        s_sample_header_sent = true;
    }

    const volatile MotorManageDebugData *debug = &g_motor_manage_debug;
    char target_yaw_text[24] = {0};
    char ref_yaw_text[24] = {0};
    char meas_yaw_text[24] = {0};
    char target_pitch_text[24] = {0};
    char ref_pitch_text[24] = {0};
    char meas_pitch_text[24] = {0};
    char yaw_speed_target_text[24] = {0};
    char yaw_meas_speed_text[24] = {0};
    char pitch_speed_target_text[24] = {0};
    char pitch_meas_speed_text[24] = {0};
    char yaw_current_pid_text[24] = {0};
    char yaw_current_ff_text[24] = {0};
    char pitch_current_pid_text[24] = {0};
    char pitch_current_ff_text[24] = {0};

    format_fixed(target_yaw_text, sizeof(target_yaw_text), debug->yaw.target_deg, 6U);
    format_fixed(ref_yaw_text, sizeof(ref_yaw_text), debug->yaw.ref_deg, 6U);
    format_fixed(meas_yaw_text, sizeof(meas_yaw_text), debug->yaw.meas_deg, 6U);
    format_fixed(target_pitch_text, sizeof(target_pitch_text), debug->pitch.target_deg, 6U);
    format_fixed(ref_pitch_text, sizeof(ref_pitch_text), debug->pitch.ref_deg, 6U);
    format_fixed(meas_pitch_text, sizeof(meas_pitch_text), debug->pitch.meas_deg, 6U);
    format_fixed(yaw_speed_target_text, sizeof(yaw_speed_target_text), debug->yaw.speed_target_dps, 6U);
    format_fixed(yaw_meas_speed_text, sizeof(yaw_meas_speed_text), debug->yaw.meas_speed_dps, 6U);
    format_fixed(pitch_speed_target_text, sizeof(pitch_speed_target_text), debug->pitch.speed_target_dps, 6U);
    format_fixed(pitch_meas_speed_text, sizeof(pitch_meas_speed_text), debug->pitch.meas_speed_dps, 6U);
    format_fixed(yaw_current_pid_text, sizeof(yaw_current_pid_text), debug->yaw.current_pid, 6U);
    format_fixed(yaw_current_ff_text, sizeof(yaw_current_ff_text), debug->yaw.current_ff, 6U);
    format_fixed(pitch_current_pid_text, sizeof(pitch_current_pid_text), debug->pitch.current_pid, 6U);
    format_fixed(pitch_current_ff_text, sizeof(pitch_current_ff_text), debug->pitch.current_ff, 6U);

    SEGGER_RTT_printf(k_rtt_buffer_index,
                      "sample,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d,%s,%s,%d,%d\n",
                      (unsigned long)now_tick_ms,
                      debug_tune_mode_name(s_tune.mode),
                      debug_tune_axis_name(s_tune.axis),
                      target_yaw_text,
                      ref_yaw_text,
                      meas_yaw_text,
                      target_pitch_text,
                      ref_pitch_text,
                      meas_pitch_text,
                      yaw_speed_target_text,
                      yaw_meas_speed_text,
                      pitch_speed_target_text,
                      pitch_meas_speed_text,
                      yaw_current_pid_text,
                      yaw_current_ff_text,
                      (int)debug->yaw.current_cmd,
                      (int)debug->yaw.current_meas,
                      pitch_current_pid_text,
                      pitch_current_ff_text,
                      (int)debug->pitch.current_cmd,
                      (int)debug->pitch.current_meas);
    s_last_sample_tick_ms = now_tick_ms;
#else
    (void)now_tick_ms;
#endif
}

const char *debug_tune_mode_name(DebugTuneMode mode)
{
    switch (mode)
    {
        case DEBUG_TUNE_MODE_HOLD:
            return "hold";

        case DEBUG_TUNE_MODE_STEP:
            return "step";

        case DEBUG_TUNE_MODE_SINE:
            return "sine";

        case DEBUG_TUNE_MODE_RAMP:
            return "ramp";

        case DEBUG_TUNE_MODE_SPEED_HOLD:
            return "speed_hold";

        case DEBUG_TUNE_MODE_SPEED_STEP:
            return "speed_step";

        case DEBUG_TUNE_MODE_SPEED_SINE:
            return "speed_sine";

        case DEBUG_TUNE_MODE_OFF:
        default:
            return "off";
    }
}

const char *debug_tune_axis_name(DebugTuneAxis axis)
{
    switch (axis)
    {
        case DEBUG_TUNE_AXIS_PITCH:
            return "pitch";

        case DEBUG_TUNE_AXIS_BOTH:
            return "both";

        case DEBUG_TUNE_AXIS_YAW:
        default:
            return "yaw";
    }
}
