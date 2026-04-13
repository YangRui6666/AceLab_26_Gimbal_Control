//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_DEVICE_GM6020_H
#define GIMBAL_UM_DEVICE_GM6020_H
#include "bsp_can.h"

/**
 * @brief       GM6020电机类
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 */
class GM6020
{
public:
    struct State
    {
        float angle_deg;        // 角度，单位：deg
        float speed_dps;        // 角速度，单位：deg/s
        int16_t current;        // 电机反馈电流原始值
        uint16_t encoder_raw;
    };
    struct Target
    {
        int16_t target_current;      // 目标电流原始值
        float target_speed_dps;      // 目标角速度，单位：deg/s
    };

public:
    GM6020(uint16_t can_id, int16_t max_current, float limit_pos, float limit_neg);

    bool init();
    bool update();
    State get_state() const;
    Target get_target() const;
    bool check(uint32_t now_ms) const;

    void set_target_current(int16_t target_current);
    void set_target_speed_dps(float speed_dps);

    uint16_t get_can_id() const { return can_id_; }

private:
    uint16_t can_id_;
    int16_t max_current_;       // 最大电流原始值
    float limit_pos_;          // 正机械角度限制，单位：deg
    float limit_neg_;          // 负机械角度限制，单位：deg

    uint32_t last_rx_time_;     //毫秒
    Target target_;
    State state_;
};


#endif //GIMBAL_UM_DEVICE_GM6020_H
