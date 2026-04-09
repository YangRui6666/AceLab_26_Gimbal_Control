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
    
}

/**
 * @brief       跑一次pid计算
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 * @param setpoint 
 * @param measurement 
 * @param dt_s 
 * @return float 
 */
float PID::calculate(float setpoint, float measurement, float dt_s)
{
    return 0.0f;
}
