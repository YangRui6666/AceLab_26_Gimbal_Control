#ifndef USER_CPP_GM6020_HPP
#define USER_CPP_GM6020_HPP

#include "gimbal_types.h"

class GM6020 {
public:
    struct State {
        float angle_deg;
        float speed_dps;
        int16_t current;
        uint16_t encoder_raw;
    };

    GM6020(uint16_t can_id,
           float max_current,
           float limit_pos_deg,
           float limit_neg_deg,
           uint16_t zero_encoder_raw);

    void init();
    void update(const CanRxFrame &frame, uint32_t now_ms);
    bool online(uint32_t now_ms) const;
    bool limit_error() const;
    int16_t clamp_current(float current) const;

    uint16_t can_id() const;
    const State &state() const;

private:
    static float normalize_deg(float angle_deg);
    float encoder_to_deg(uint16_t encoder_raw) const;

    uint16_t can_id_;
    float max_current_;
    float limit_pos_deg_;
    float limit_neg_deg_;
    uint16_t zero_encoder_raw_;
    uint32_t last_rx_time_ms_;
    State state_;
    bool initialized_;
};

#endif
