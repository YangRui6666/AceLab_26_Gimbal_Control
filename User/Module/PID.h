//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_PID_H
#define GIMBAL_UM_PID_H
#include <stdint.h>

/**
 * @class PID
 * @brief 位置式 PID 控制器，带积分限幅、输出限幅及微分低通滤波
 */
class PID
{
public:

    PID();


    void init(float kp, float ki, float kd,
              float integral_limit, float output_limit,
              float derivative_filter);


    void reset();

    float calculate(float setpoint, float measurement, float dt_s);

    // --- 在线调参接口 ---
    void set_kp(float kp) { kp_ = kp; }
    void set_ki(float ki) { ki_ = ki; }
    void set_kd(float kd) { kd_ = kd; }
    void set_integral_limit(float limit) { integral_limit_ = limit; }
    void set_output_limit(float limit) { output_limit_ = limit; }

private:
    // 参数
    float kp_;
    float ki_;
    float kd_;
    float integral_limit_;
    float output_limit_;
    float derivative_filter_;   // 0.0 ~ 1.0

    // 内部状态
    float integral_;
    float prev_error_;
    float prev_measurement_;
    float filtered_derivative_;

    bool initialized_;
};

#endif //GIMBAL_UM_PID_H