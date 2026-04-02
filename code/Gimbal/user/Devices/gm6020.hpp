//
// Created by CORE on 2026/4/2.
//

#ifndef GIMBAL_GM6020_HPP
#define GIMBAL_GM6020_HPP

#include <cstdint>

struct MotorConfig
{
    uint16_t rx_id = 0;
    uint8_t tx_slot = 0;
    int16_t current_limit = 0;
};

struct MotorFeedback
{
    int16_t pos = 0;
    int16_t speed = 0;
    int16_t current = 0;
    int8_t temp = 0;
    uint32_t tick = 0;
};

struct MotorSnapshot
{
    int32_t total_angle = 0;
    int16_t speed = 0;
    int16_t current = 0;
    int8_t temp = 0;
    uint32_t last_rx_tick = 0;
};

class Gm6020Motor
{
public:
    Gm6020Motor(MotorConfig config);

    void updateFromFeedback(const MotorFeedback& fb);
    void setTargetCurrent(int16_t current);
    MotorSnapshot snapshot() const;

    uint16_t rxId() const;
    uint8_t txSlot() const;
    int16_t targetCurrent() const;

private:
    void unwrap(int16_t pos);

private:
    MotorConfig config_;
    MotorSnapshot snapshot_;

    int16_t target_current_ = 0;
    int16_t last_pos_ = 0;
    int16_t turn_count_ = 0;
};

#endif //GIMBAL_GM6020_HPP
