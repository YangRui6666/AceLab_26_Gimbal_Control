//
// Created by Codex on 2026/4/26.
//

#ifndef GIMBAL_UM_DEBUG_TUNE_H
#define GIMBAL_UM_DEBUG_TUNE_H

#include <stdbool.h>
#include <stdint.h>

#include "MotorManage.h"

typedef enum
{
    DEBUG_TUNE_AXIS_YAW = 0,
    DEBUG_TUNE_AXIS_PITCH,
    DEBUG_TUNE_AXIS_BOTH
} DebugTuneAxis;

typedef enum
{
    DEBUG_TUNE_MODE_OFF = 0,
    DEBUG_TUNE_MODE_HOLD,
    DEBUG_TUNE_MODE_STEP,
    DEBUG_TUNE_MODE_SINE,
    DEBUG_TUNE_MODE_RAMP,
    DEBUG_TUNE_MODE_SPEED_HOLD,
    DEBUG_TUNE_MODE_SPEED_STEP,
    DEBUG_TUNE_MODE_SPEED_SINE
} DebugTuneMode;

typedef struct
{
    bool enabled;
    DebugTuneMode mode;
    DebugTuneAxis axis;
    bool planner_enabled;
    uint32_t start_tick_ms;
    uint32_t sample_period_ms;
    bool sample_enabled;

    float yaw_deg;
    float pitch_deg;
    float amp;
    float bias;
    float other;
    float freq_hz;
    float speed_dps;
    uint32_t hold_ms;
    uint32_t ramp_ms;
} DebugTuneState;

bool debug_tune_is_enabled(void);
bool debug_tune_was_disabled(bool clear_flag);
DebugTuneState debug_tune_get_state(void);
void debug_tune_disable(void);
void debug_tune_set_sample(bool enabled, uint32_t period_ms);

bool debug_tune_enable_hold(float yaw_deg, float pitch_deg, bool planner_enabled, uint32_t now_tick_ms);
bool debug_tune_enable_step(DebugTuneAxis axis,
                            float amp,
                            uint32_t hold_ms,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms);
bool debug_tune_enable_sine(DebugTuneAxis axis,
                            float amp,
                            float freq_hz,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms);
bool debug_tune_enable_ramp(DebugTuneAxis axis,
                            float amp,
                            uint32_t ramp_ms,
                            uint32_t hold_ms,
                            float bias,
                            float other,
                            bool planner_enabled,
                            uint32_t now_tick_ms);
bool debug_tune_enable_speed_hold(DebugTuneAxis axis,
                                  float speed_dps,
                                  float other,
                                  uint32_t now_tick_ms);
bool debug_tune_enable_speed_step(DebugTuneAxis axis,
                                  float amp_dps,
                                  uint32_t hold_ms,
                                  float other,
                                  uint32_t now_tick_ms);
bool debug_tune_enable_speed_sine(DebugTuneAxis axis,
                                  float amp_dps,
                                  float freq_hz,
                                  float other,
                                  uint32_t now_tick_ms);

bool debug_tune_eval(uint32_t now_tick_ms,
                     float current_yaw_deg,
                     float current_pitch_deg,
                     MotorControlReference *reference);

bool debug_tune_set_param(const char *name, float value);
bool debug_tune_get_param(const char *name, float *value);
void debug_tune_print_params(void);
void debug_tune_emit_sample_if_due(uint32_t now_tick_ms);

const char *debug_tune_mode_name(DebugTuneMode mode);
const char *debug_tune_axis_name(DebugTuneAxis axis);

#endif // GIMBAL_UM_DEBUG_TUNE_H
