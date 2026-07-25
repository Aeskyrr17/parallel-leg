#include "lqr.hpp"

#include "leg_math.hpp"
#include "lqr_coeffs.hpp"

#include <cmath>

namespace wbr::control
{
namespace
{

bool finite_vector(const float* values, int count)
{
    if (values == nullptr)
    {
        return false;
    }
    for (int i = 0; i < count; ++i)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

bool offground_coefficient(int index)
{
    return (index >= 24 && index < 28) || (index >= 34 && index < 38);
}

} // namespace

lqr_output LQR::solve(float left_leg_len, float right_leg_len, bool offground,
                      const float observed[10], const float reference[10]) const
{
    lqr_output output{};
    if (!std::isfinite(left_leg_len) || !std::isfinite(right_leg_len) ||
        !finite_vector(observed, 10) || !finite_vector(reference, 10))
    {
        return output;
    }

    if (!std::isfinite(cfg_.leg_len_resolution) || cfg_.leg_len_resolution <= 0.0f)
    {
        return output;
    }
    const float left = std::round(clamp(left_leg_len, cfg_.min_leg_len, cfg_.max_leg_len) /
                                  cfg_.leg_len_resolution) *
                       cfg_.leg_len_resolution;
    const float right = std::round(clamp(right_leg_len, cfg_.min_leg_len, cfg_.max_leg_len) /
                                   cfg_.leg_len_resolution) *
                        cfg_.leg_len_resolution;
    const float polynomial[6] = {
        1.0f, left, right, left * left, left * right, right * right,
    };

    float gain[40] = {};
    for (int row = 0; row < 40; ++row)
    {
        if (offground && !offground_coefficient(row))
        {
            continue;
        }
        for (int coefficient = 0; coefficient < 6; ++coefficient)
        {
            gain[row] += k_lqr_coefficients[row][coefficient] * polynomial[coefficient];
        }
    }

    float error[10] = {};
    for (int state = 0; state < 10; ++state)
    {
        error[state] = observed[state] - reference[state];
    }

    float torque[4] = {};
    for (int actuator = 0; actuator < 4; ++actuator)
    {
        for (int state = 0; state < 10; ++state)
        {
            torque[actuator] += gain[actuator * 10 + state] * error[state];
        }
    }
    if (!finite_vector(torque, 4))
    {
        return output;
    }

    output.left_wheel = torque[0];
    output.right_wheel = torque[1];
    output.left_hip = torque[2];
    output.right_hip = torque[3];
    output.valid = true;
    return output;
}

} // namespace wbr::control
