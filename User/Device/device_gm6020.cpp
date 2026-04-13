//
// Created by CORE on 2026/4/9.
//
#include <stdint.h>
#include "bsp_can.h"
#include "device_gm6020.h"

/**
 * @brief       Construct a new GM6020::GM6020 object
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 * @param can_id    电机的CAN ID
 * @param max_current 电机允许的最大电流原始值
 * @param limit_cpos   电机正机械角度限制，单位deg
 * @param limit_cneg   电机负机械角度限制，单位deg
 */
GM6020::GM6020(uint16_t can_id, int16_t max_current, float limit_cpos, float limit_cneg)
{
    can_id_ = can_id;
    max_current_ = max_current;
    limit_cpos_ = limit_cpos;
    limit_cneg_ = limit_cneg;
    last_rx_time_ = 0;

    target_.target_current = 0;
    target_.target_speed_dps = 0.0f;
}

/**
 * @brief       电机的初始化函数
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 * @return true
 * @return false
 */
bool GM6020::init()
{
    state_.angle_deg = 0.0f;
    state_.speed_dps = 0.0f;
    state_.current = 0;
    state_.encoder_raw = 0;
    last_rx_time_ = 0;
    //TODO:
    //此处将会和can通讯有关系
    return true;
}

/**
 * @brief       更新电机内部参数
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 * @return true
 * @return false
 */
bool GM6020::update()
{
    CanRxFrame frame;
    const int32_t encoder_range = 8192;
    const float angle_per_turn_deg = 360.0f;

    if (!bsp_can_rx(can_id_, &frame) || frame.dlc != 8U)
    {
        return false;
    }

    const uint16_t raw_encoder   = (uint16_t)((frame.data[0] << 8U) | frame.data[1]);
    const int16_t raw_speed_rpm  = (int16_t)((frame.data[2] << 8U) | frame.data[3]);
    const int16_t raw_current    = (int16_t)((frame.data[4] << 8U) | frame.data[5]);

    if (raw_encoder >= encoder_range)
    {
        return false;
    }

    if (last_rx_time_ == 0U)
    {
        state_.angle_deg = (float)raw_encoder * angle_per_turn_deg / (float)encoder_range;
    }
    else
    {
        int32_t delta_encoder = (int32_t)raw_encoder - (int32_t)state_.encoder_raw;

        if (delta_encoder > encoder_range / 2)
        {
            delta_encoder -= encoder_range;
        }
        else if (delta_encoder < -(encoder_range / 2))
        {
            delta_encoder += encoder_range;
        }

        state_.angle_deg += (float)delta_encoder * angle_per_turn_deg / (float)encoder_range;
    }

    state_.encoder_raw = raw_encoder;
    state_.speed_dps = (float)raw_speed_rpm * 6.0f;
    state_.current = raw_current;
    last_rx_time_ = frame.timestamp_ms;

    return true;
}

/**
 * @brief       获取电机状态快照
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 * @return GM6020::State 电机状态结构体，包含角度、速度、电流和编码器原始值
 */
GM6020::State GM6020::get_state() const
{
    return state_;
}

GM6020::Target GM6020::get_target() const
{
    return target_;
}

/**
 * @brief       检查电机是否离线
 *
 * @date        2026-04-09
 * @author      Rui.
 *
 * @param now_ms 传入当前时间的毫秒数，用于与上次接收时间进行比较
 * @return true
 * @return false
 */
bool GM6020::check(uint32_t now_ms) const
{
    if (now_ms - last_rx_time_ < 500)
    {
        return true;
    }

    return false;
}

void GM6020::set_target_current(int16_t target_current)
{
    if (target_current > max_current_)
    {
        target_.target_current = max_current_;
    }
    else if (target_current < -max_current_)
    {
        target_.target_current = -max_current_;
    }
    else
    {
        target_.target_current = target_current;
    }
}

void GM6020::set_target_speed_dps(float speed_dps)
{
    target_.target_speed_dps = speed_dps;
}
