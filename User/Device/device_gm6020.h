//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_DEVICE_GM6020_H
#define GIMBAL_UM_DEVICE_GM6020_H

#include <stdint.h>

#include "bsp_can.h"

class GM6020
{
public:
    struct State
    {
        float angle_deg;
        float speed_dps;
        int16_t current;
        uint16_t encoder_raw;
    };

    struct Target
    {
        int16_t target_current;
        float target_speed_dps;
    };

    GM6020(uint16_t can_id, int16_t max_current, float limit_pos, float limit_neg);

    bool init();
    bool update();
    State get_state() const;
    Target get_target() const;
    bool check(uint32_t now_ms) const;
    bool has_feedback() const;

    void set_target_current(int16_t target_current);
    void set_target_speed_dps(float speed_dps);

    uint16_t get_can_id() const
    {
        return can_id_;
    }

private:
    uint16_t can_id_;
    int16_t max_current_;
    float limit_pos_;
    float limit_neg_;
    uint32_t last_rx_time_;
    bool has_feedback_;
    Target target_;
    State state_;
};

#endif // GIMBAL_UM_DEVICE_GM6020_H
