//
// Created by CORE on 2026/4/2.
//

#ifndef GIMBAL_MOTOR_MANAGE_HPP
#define GIMBAL_MOTOR_MANAGE_HPP

#include <cstdint>

#include "gm6020.hpp"

/**
 * @brief 电机管理器
 * @details 负责 CAN 接收分发、目标电流打包和双电机状态管理。
 */
class MotorManager
{
public:
    MotorManager();

    /**
     * @brief 处理一帧 CAN 反馈
     * @param std_id 标准帧 ID
     * @param data CAN 数据区
     * @param dlc 数据长度
     * @param tick 时间戳
     * @return true=当前电机已处理该帧，false=不是目标电机或输入非法
     */
    bool onCanFrame(uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint32_t tick);
    /**
     * @brief 轮询 CAN 接收队列
     */
    void pollCanRx();
    /**
     * @brief 发送当前电流指令
     * @return true=发送成功，false=无发送帧或发送失败
     */
    bool sendCurrentCommands() const;
    /**
     * @brief 设置 Yaw 轴目标电流
     * @param current 目标电流(mA)
     */
    void setYawCurrent(int16_t current);
    /**
     * @brief 设置 Pitch 轴目标电流
     * @param current 目标电流(mA)
     */
    void setPitchCurrent(int16_t current);
    /**
     * @brief 获取 Yaw 轴状态快照
     * @return Yaw 轴状态副本
     */
    MotorSnapshot yawSnapshot() const;
    /**
     * @brief 获取 Pitch 轴状态快照
     * @return Pitch 轴状态副本
     */
    MotorSnapshot pitchSnapshot() const;

private:
    Gm6020Motor yaw_motor_;
    Gm6020Motor pitch_motor_;
};

#endif //GIMBAL_MOTOR_MANAGE_HPP
