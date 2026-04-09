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
 * @param max_current 电机允许的最大电流
 * @param limit_cpos    电机正机械角度限制，单位0.01°
 * @param limit_cneg    电机负机械角度限制，单位0.01°
 */
GM6020::GM6020(uint16_t can_id, int16_t max_current, int32_t limit_cpos, int32_t limit_cneg)
{
    can_id_ = can_id;
    max_current_ = max_current;
    limit_cpos_ = limit_cpos;
    limit_cneg_ = limit_cneg;
    last_rx_time_ = 0;
    
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

    state_.angle_cdeg = 0;
    state_.speed_cdps = 0;
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
    //此处将会调用bsp_can_rx函数获取数据，并更新state_和last_rx_time_
    CanRxFrame frame;
    if (bsp_can_rx(can_id_, &frame))
    {
        //TODO:
        // 解析接收到的CAN数据并更新电机状态
        // 这里需要根据具体的CAN数据格式进行解析
        last_rx_time_ = frame.timestamp_ms;
    }

    return false;
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
