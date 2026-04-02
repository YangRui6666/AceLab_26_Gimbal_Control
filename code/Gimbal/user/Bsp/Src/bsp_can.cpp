//
// Created by CORE on 2026/4/1.
//
#include <cstdint>

// 1. 逻辑名字：这个电机是 Yaw 还是 Pitch
enum class MotorId
{
    Yaw,
    Pitch
};

// 2. 固定配置：这些通常创建后就不怎么改
struct MotorConfig
{
    uint16_t rx_id;    // 反馈ID，比如 0x204
    uint8_t tx_slot;   // 发控制帧时所在槽位，比如第1个电机
};

// 3. 运行状态：这些会随着反馈不断变化
struct MotorState
{
    int16_t pos = 0;
    int16_t speed = 0;
};

// 4. 电机类：把“配置 + 状态 + 操作”放在一起
class Gm6020Motor
{
public:
    // 构造函数：创建对象时，顺手把内部成员初始化好
    Gm6020Motor(MotorId id, MotorConfig cfg)
        :
        id_(id),
        cfg_(cfg)
    {
    }

    // 更新反馈
    void updateFeedback(int16_t pos, int16_t speed)
    {
        state_.pos = pos;
        state_.speed = speed;
    }

    // 读取当前位置
    int16_t getPosition()
    {
        return state_.pos;
    }

    // 读取反馈ID
    uint16_t getRxId()
    {
        return cfg_.rx_id;
    }

private:
    MotorId id_;       // 这个对象自己的逻辑身份
    MotorConfig cfg_;  // 这个对象自己的固定配置
    MotorState state_; // 这个对象自己的运行状态
};


// 5. 使用示例
int main()
{
    Gm6020Motor yaw_motor(MotorId::Yaw, {0x204, 1});

    yaw_motor.updateFeedback(1234, 56);

    int16_t pos = yaw_motor.getPosition();
    uint16_t rx_id = yaw_motor.getRxId();

    return 0;
}
