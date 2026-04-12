//
// Created by CORE on 2026/4/9.
//

#include "PID.h"




/**
 * @brief       Construct a new PID::PID object
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 */
PID::PID()
{

}

/**
 * @brief       初始化 PID 控制器参数
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param integral_limit 
 * @param output_limit 
 * @param derivative_filter 
 */
void PID::init(float kp, float ki, float kd, float integral_limit, float output_limit, float derivative_filter)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integral_limit_ = integral_limit;
    output_limit_ = output_limit;
    derivative_filter_ = derivative_filter;
}

/**
 * @brief       重置清理pid状态
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 */
void PID::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_measurement_ = 0.0f;
    filtered_derivative_ = 0.0f;

}

/**
 * @brief       跑一次pid计算
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 * @param setpoint 目标点
 * @param measurement 设置点
 * @param dt_s 间隔时间
 * @return float 
 */
float PID::calculate(float setpoint, float measurement, float dt_s)
{
    return 0.0f;
}
