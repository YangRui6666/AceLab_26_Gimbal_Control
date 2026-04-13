//
// Created by CORE on 2026/4/9.
//

#include "PID.h"

namespace
{
float clamp_symmetric(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

float clamp_unit(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}
}

/**
 * @brief       Construct a new PID::PID object
 *
 * @date        2026-04-10
 * @author      Rui.
 *
 */
PID::PID()
    : kp_(0.0f),
      ki_(0.0f),
      kd_(0.0f),
      integral_limit_(0.0f),
      output_limit_(0.0f),
      derivative_filter_(0.0f),
      integral_(0.0f),
      prev_error_(0.0f),
      prev_measurement_(0.0f),
      filtered_derivative_(0.0f),
      initialized_(false)
{
}

/**
 * @brief       Initialize PID controller parameters
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
    derivative_filter_ = clamp_unit(derivative_filter);
    reset();
}

/**
 * @brief       Reset PID runtime state
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
    initialized_ = false;
}

/**
 * @brief       Run one PID step
 *
 * @date        2026-04-10
 * @author      Rui.
 *
 * @param setpoint target value
 * @param measurement measured value
 * @param dt_s timestep in seconds
 * @return float controller output
 */
float PID::calculate(float setpoint, float measurement, float dt_s)
{
    const float error = setpoint - measurement;
    const float filter_alpha = clamp_unit(derivative_filter_);
    const bool first_update = !initialized_;

    if (first_update)
    {
        prev_measurement_ = measurement;
        prev_error_ = error;
        filtered_derivative_ = 0.0f;
        initialized_ = true;
    }

    const float proportional = kp_ * error;

    if (dt_s > 0.0f)
    {
        if (integral_limit_ > 0.0f)
        {
            integral_ += error * dt_s;
            integral_ = clamp_symmetric(integral_, integral_limit_);
        }
        else
        {
            integral_ = 0.0f;
        }
    }

    const float integral = ki_ * integral_;

    float derivative = 0.0f;
    if ((!first_update) && (dt_s > 0.0f))
    {
        const float raw_derivative = -(measurement - prev_measurement_) / dt_s;
        filtered_derivative_ = filter_alpha * filtered_derivative_ +
                               (1.0f - filter_alpha) * raw_derivative;
        derivative = kd_ * filtered_derivative_;
    }

    float output = proportional + integral + derivative;
    output = clamp_symmetric(output, output_limit_);

    prev_error_ = error;
    prev_measurement_ = measurement;

    return output;
}
