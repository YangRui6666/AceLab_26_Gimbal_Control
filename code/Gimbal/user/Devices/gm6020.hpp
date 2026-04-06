//
// Created by CORE on 2026/4/2.
//

#ifndef GIMBAL_GM6020_HPP
#define GIMBAL_GM6020_HPP

#include <cstdint>

struct MotorConfig
{
    uint8_t can_id = 0;
    int16_t current_limit = 0;
};

struct MotorFeedback
{
    int16_t raw_angle = 0;
    int16_t speed = 0;
    int16_t current = 0;
    int8_t temp = 0;
    uint32_t tick = 0;
};

struct MotorSnapshot
{
    int16_t raw_angle = 0;
    int32_t total_angle = 0;
    int16_t speed = 0;
    int16_t current = 0;
    float filtered_current = 0.0f;
    int8_t temp = 0;
    uint32_t last_rx_tick = 0;
};

class Gm6020Motor
{
public:
    explicit Gm6020Motor(MotorConfig config);

    bool acceptsStdId(uint16_t std_id) const;
    bool updateFromCanPayload(const uint8_t data[8], uint32_t tick);
    void updateFromFeedback(const MotorFeedback& fb);
    void setTargetCurrent(int16_t current);
    MotorSnapshot snapshot() const;

    uint8_t canId() const;
    uint16_t rxStdId() const;
    uint16_t txStdId() const;
    uint8_t txSlot() const;
    int16_t targetCurrent() const;

private:
    void unwrap(int16_t pos);
    static bool isValidCanId(uint8_t can_id);

private:
    MotorConfig config_;
    MotorSnapshot snapshot_;

    int16_t target_current_ = 0;
    int16_t last_pos_ = 0;
    int16_t turn_count_ = 0;
};

#endif //GIMBAL_GM6020_HPP
