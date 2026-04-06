//
// Created by CORE on 2026/4/2.
//

#include "motor_manage.hpp"

#include "Bsp/Inc/bsp_can.h"
#include "Config/can_id.h"
#include "Config/pid_config.h"

namespace
{
constexpr MotorConfig kYawConfig = {GIMBAL_YAW_CAN_ID, GIMBAL_YAW_CURRENT_LIMIT};
constexpr MotorConfig kPitchConfig = {GIMBAL_PITCH_CAN_ID, GIMBAL_PITCH_CURRENT_LIMIT};

/**
 * @brief CAN 电流发送缓存
 */
struct TxFrame
{
    uint16_t std_id = 0U;
    int16_t slots[4] = {0, 0, 0, 0};
    bool active = false;
};

/**
 * @brief 将四路电流指令打包成 CAN 8 字节数据
 * @param slots 四路电流槽位
 * @param data 输出 CAN 数据
 */
void packSlots(const int16_t slots[4], uint8_t data[8])
{
    data[0] = static_cast<uint8_t>(slots[0] >> 8);
    data[1] = static_cast<uint8_t>(slots[0] & 0xFF);
    data[2] = static_cast<uint8_t>(slots[1] >> 8);
    data[3] = static_cast<uint8_t>(slots[1] & 0xFF);
    data[4] = static_cast<uint8_t>(slots[2] >> 8);
    data[5] = static_cast<uint8_t>(slots[2] & 0xFF);
    data[6] = static_cast<uint8_t>(slots[3] >> 8);
    data[7] = static_cast<uint8_t>(slots[3] & 0xFF);
}

/**
 * @brief 将单个电机映射到对应的 CAN 发送帧
 * @param motor 电机对象
 * @param group1 group1 发送帧
 * @param group2 group2 发送帧
 */
void assignMotorToFrame(const Gm6020Motor& motor, TxFrame& group1, TxFrame& group2)
{
    const uint8_t slot = motor.txSlot();
    TxFrame* frame = nullptr;

    if (slot < 1U || slot > 4U)
    {
        return;
    }

    if (motor.txStdId() == CAN_STDID_DJI_GROUP1)
    {
        frame = &group1;
    }
    else if (motor.txStdId() == CAN_STDID_DJI_GROUP2)
    {
        frame = &group2;
    }
    else
    {
        return;
    }

    frame->active = true;
    frame->slots[slot - 1U] = motor.targetCurrent();
}
}

/**
 * @brief 构造电机管理器并绑定默认配置
 */
MotorManager::MotorManager()
    : yaw_motor_(kYawConfig),
      pitch_motor_(kPitchConfig)
{
}

/**
 * @brief 处理一帧 CAN 反馈数据
 * @param std_id 标准帧 ID
 * @param data CAN 数据区
 * @param dlc 数据长度
 * @param tick 接收时间戳
 * @return true=帧被当前电机处理，false=不是目标电机或输入非法
 */
bool MotorManager::onCanFrame(uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint32_t tick)
{
    if (data == nullptr || dlc != 8U)
    {
        return false;
    }

    if (yaw_motor_.acceptsStdId(std_id))
    {
        return yaw_motor_.updateFromCanPayload(data, tick);
    }

    if (pitch_motor_.acceptsStdId(std_id))
    {
        return pitch_motor_.updateFromCanPayload(data, tick);
    }

    return false;
}

/**
 * @brief 轮询 CAN 接收队列并分发给对应电机
 */
void MotorManager::pollCanRx()
{
    can_rx_msg_t rx_msg;

    while (bsp_can_pop_rx(&rx_msg))
    {
        (void)onCanFrame(rx_msg.std_id, rx_msg.data, rx_msg.dlc, static_cast<uint32_t>(rx_msg.timestamp));
    }
}

/**
 * @brief 发送当前两轴电流指令
 * @return true=至少发送了一帧且发送成功，false=无数据或发送失败
 */
bool MotorManager::sendCurrentCommands() const
{
    TxFrame group1 = {CAN_STDID_DJI_GROUP1, {0, 0, 0, 0}, false};
    TxFrame group2 = {CAN_STDID_DJI_GROUP2, {0, 0, 0, 0}, false};
    bool ok = true;
    bool sent = false;

    assignMotorToFrame(yaw_motor_, group1, group2);
    assignMotorToFrame(pitch_motor_, group1, group2);

    if (group1.active)
    {
        uint8_t data[8];
        packSlots(group1.slots, data);
        ok = bsp_can_send_std(group1.std_id, data, 8U) && ok;
        sent = true;
    }

    if (group2.active)
    {
        uint8_t data[8];
        packSlots(group2.slots, data);
        ok = bsp_can_send_std(group2.std_id, data, 8U) && ok;
        sent = true;
    }

    return sent && ok;
}

/**
 * @brief 设置 Yaw 轴目标电流
 * @param current 目标电流(mA)
 */
void MotorManager::setYawCurrent(int16_t current)
{
    yaw_motor_.setTargetCurrent(current);
}

/**
 * @brief 设置 Pitch 轴目标电流
 * @param current 目标电流(mA)
 */
void MotorManager::setPitchCurrent(int16_t current)
{
    pitch_motor_.setTargetCurrent(current);
}

/**
 * @brief 获取 Yaw 轴状态快照
 * @return Yaw 轴状态副本
 */
MotorSnapshot MotorManager::yawSnapshot() const
{
    return yaw_motor_.snapshot();
}

/**
 * @brief 获取 Pitch 轴状态快照
 * @return Pitch 轴状态副本
 */
MotorSnapshot MotorManager::pitchSnapshot() const
{
    return pitch_motor_.snapshot();
}
