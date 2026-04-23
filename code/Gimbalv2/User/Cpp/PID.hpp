#ifndef USER_CPP_PID_HPP
#define USER_CPP_PID_HPP

#include "gimbal_types.h"

class PID {
public:
    PID();

    void init(const PIDParam_t &param);
    void reset();
    void set_param(const PIDParam_t &param);
    float calc(float ref, float fdb);

    const PIDParam_t &param() const;

private:
    PIDParam_t param_;
    float err_;
    float last_err_;
    float integral_;
};

#endif
