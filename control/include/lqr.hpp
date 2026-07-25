#pragma once

#include "leg_config.hpp"

namespace wbr::control
{

struct lqr_output
{
    float left_wheel = 0.0f;
    float right_wheel = 0.0f;
    float left_hip = 0.0f;
    float right_hip = 0.0f;
    bool valid = false;
};

class LQR
{
public:
    explicit LQR(const lqr_config& cfg) : cfg_(cfg) {}

    lqr_output solve(float left_leg_len, float right_leg_len, bool offground,
                     const float observed[10], const float reference[10]) const;

private:
    lqr_config cfg_{};
};

} // namespace wbr::control
