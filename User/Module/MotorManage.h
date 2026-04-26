//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_MOTORMANAGE_H
#define GIMBAL_UM_MOTORMANAGE_H

#include <stdint.h>

#include "PID.h"
#include "device_gm6020.h"

#define DDBUG_DATA_ON

typedef enum
{
    MOTOR_AIM_MODE_NONE = 0,
    MOTOR_AIM_MODE_LARGE_MOVE,
    MOTOR_AIM_MODE_SMALL_TRACK,
    MOTOR_AIM_MODE_SPIN_TRACK,
    MOTOR_AIM_MODE_TRACK_LOST
} MotorManageAimMode_e;

typedef enum
{
    MOTOR_REFERENCE_MODE_DIRECT = 0,
    MOTOR_REFERENCE_MODE_PLANNER
} MotorReferenceMode_e;

typedef struct
{
    float yaw_location_kp;
    float yaw_location_ki;
    float yaw_location_kd;
    float yaw_speed_kp;
    float yaw_speed_ki;
    float yaw_speed_kd;

    float pitch_location_kp;
    float pitch_location_ki;
    float pitch_location_kd;
    float pitch_speed_kp;
    float pitch_speed_ki;
    float pitch_speed_kd;

    float yaw_max_vel_dps;
    float yaw_max_acc_dps2;
    float yaw_k_vel_ff;
    float yaw_hold_ff;

    float pitch_max_vel_dps;
    float pitch_max_acc_dps2;
    float pitch_k_vel_ff;
    float pitch_hold_ff;
} MotorManageRuntimeParams;

typedef struct
{
    float pos_ref_deg;
    float vel_ref_dps;
    float acc_ref_dps2;
    MotorReferenceMode_e reference_mode;
} MotorAxisReference;

typedef struct
{
    MotorManageAimMode_e aim_mode;
    MotorAxisReference yaw;
    MotorAxisReference pitch;
} MotorControlReference;

extern volatile MotorManageRuntimeParams g_motor_manage_runtime_params;

#ifdef DDBUG_DATA_ON
typedef struct
{
    float target_deg;
    float ref_deg;
    float speed_target_dps;
    float ref_speed_dps;
    float ref_acc_dps2;
    float current_pid;
    float current_ff;
    float hold_ff;
    float boot_bias_ff;
    float vel_ff;
    float acc_ff;
    float ff_total;
    int16_t current_cmd;
    float meas_deg;
    float meas_speed_dps;
    int16_t current_meas;
} MotorManageDebugAxisData;

typedef struct
{
    MotorManageDebugAxisData yaw;
    MotorManageDebugAxisData pitch;
    float yaw_angle_t;
    float pitch_angle_t;
    float yaw_planned_angle_t;
    float pitch_planned_angle_t;
    float yaw_speed_t;
    float pitch_speed_t;
    float yaw_planned_speed_t;
    float pitch_planned_speed_t;
    float yaw_planned_acc_t;
    float pitch_planned_acc_t;
    float yaw_current_pid;
    float pitch_current_pid;
    float yaw_current_ff;
    float pitch_current_ff;
    float yaw_hold_ff;
    float pitch_hold_ff;
    float yaw_boot_bias_ff;
    float pitch_boot_bias_ff;
    float yaw_vel_ff;
    float pitch_vel_ff;
    float yaw_acc_ff;
    float pitch_acc_ff;
    float yaw_ff_total;
    float pitch_ff_total;
    int16_t yaw_current_cmd;
    int16_t pitch_current_cmd;
    float yaw_angle_meas_deg;
    float pitch_angle_meas_deg;
    float yaw_speed_meas_dps;
    float pitch_speed_meas_dps;
    int16_t yaw_current_meas;
    int16_t pitch_current_meas;
    float dt_s;
    uint32_t tick_ms;
} MotorManageDebugData;

extern volatile MotorManageDebugData g_motor_manage_debug;
#endif

class MotorManage
{
public:
    MotorManage();

    void update_feedback();
    void send_can_cmd();
    void sync_runtime_params_from_global();
    void set_runtime_params(const MotorManageRuntimeParams &params);
    MotorManageRuntimeParams get_runtime_params() const;
    void set_control_reference(const MotorControlReference &reference,
                               float yaw_meas_deg,
                               float pitch_meas_deg);
    void set_world_target(float yaw_target_deg,
                          float pitch_target_deg,
                          float yaw_meas_deg,
                          float pitch_meas_deg,
                          bool enable_planner);
    void lock();

    GM6020::Target get_yaw_target() const;
    GM6020::Target get_pitch_target() const;
    float get_yaw_joint_deg() const;
    float get_pitch_joint_deg() const;

    GM6020 yaw_;
    GM6020 pitch_;

    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;

private:
    struct PlannerAxisState
    {
        float planned_pos_deg;
        float planned_vel_dps;
        float planned_acc_dps2;
        bool initialized;
    };

    struct FeedforwardAxisState
    {
        float hold_ff;
        float boot_bias_ff;
        float vel_ff;
        float acc_ff;
        float ff_total;
    };

    struct PlannerAxisConfig
    {
        float max_vel_dps;
        float max_acc_dps2;
        float k_vel_ff;
    };

    static void copy_runtime_params(MotorManageRuntimeParams *dst, const MotorManageRuntimeParams &src);
    static void copy_runtime_params(MotorManageRuntimeParams *dst, const volatile MotorManageRuntimeParams &src);
    static void copy_runtime_params(volatile MotorManageRuntimeParams *dst, const MotorManageRuntimeParams &src);
    void apply_runtime_params(const MotorManageRuntimeParams &params);
    void apply_mode_tuning(MotorManageAimMode_e aim_mode);
    static float clamp_target_deg(float value, float min_value, float max_value);
    float yaw_motor_to_joint_deg(float motor_angle_deg) const;
    static float pitch_motor_to_joint_deg(float motor_angle_deg);
    static void clear_feedforward_state(FeedforwardAxisState *ff_state);
    static void clear_planner_state(PlannerAxisState *planner_state);
    static void sync_planner_state(PlannerAxisState *planner_state, float measured_pos_deg);
    static void update_planner_state(PlannerAxisState *planner_state,
                                     float target_pos_deg,
                                     float dt_s,
                                     const PlannerAxisConfig &config);
    void hold_yaw_axis();
    void hold_pitch_axis();

    bool yaw_zero_ready_;
    float yaw_boot_zero_deg_;
    uint8_t outer_loop_divider_count_;
    uint32_t last_inner_tick_ms_;
    uint32_t last_outer_tick_ms_;
    float yaw_speed_target_cache_;
    float pitch_speed_target_cache_;
    PlannerAxisState yaw_planner_state_;
    PlannerAxisState pitch_planner_state_;
    FeedforwardAxisState yaw_ff_state_;
    FeedforwardAxisState pitch_ff_state_;
    PlannerAxisConfig yaw_planner_config_;
    PlannerAxisConfig pitch_planner_config_;
    MotorManageRuntimeParams runtime_params_;
};

#endif // GIMBAL_UM_MOTORMANAGE_H
