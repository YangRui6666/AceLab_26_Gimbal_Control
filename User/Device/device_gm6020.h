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
        //放缩100倍
        int32_t angle_cdeg;    // 角度 0.01°
        int32_t speed_cdps;    //角速度 0.01°/s
        int16_t current;        //电流
        uint16_t encoder_raw;
    };
    struct Target
    {
        
        int32_t target_current;    
        int32_t target_speed_cdps;    
    };

public:
    GM6020(uint16_t can_id, int16_t max_current, int32_t limit_cpos, int32_t limit_cneg);

    bool init();
    bool update();
    State get_state() const;
    Target get_target() const;
    bool check(uint32_t now_ms) const;

    void set_target_current(int32_t target_current);
    void set_target_speed_cdps(int32_t speed_cdps);

    uint16_t get_can_id() const { return can_id_; }

private:
    uint16_t can_id_;
    int16_t max_current_;       //最大电流
    int32_t limit_cpos_;        //正机械角度限制
    int32_t limit_cneg_;        //负机械角度限制

    uint32_t last_rx_time_;     //毫秒
    Target target_;
    State state_;
};


#endif //GIMBAL_UM_DEVICE_GM6020_H