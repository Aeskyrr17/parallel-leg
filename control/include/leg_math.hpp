#pragma once

#include <algorithm>

namespace wbr::control
{

inline constexpr float k_pi = 3.14159265358979f;
inline constexpr float k_two_pi = 6.28318530717959f;
inline constexpr float k_gravity = 9.78f;

inline float clamp(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

inline float loop_clamp(float value, float min_value, float max_value)
{
    const float range = max_value - min_value;
    if (range <= 0.0f)
    {
        return min_value;
    }

    while (value > max_value)
    {
        value -= range;
    }
    while (value < min_value)
    {
        value += range;
    }
    return value;
}

class slope
{
public:
    slope(float value, float path) : value_(value), inc_path_(path), dec_path_(path) {}

    void reset(float value = 0.0f) { value_ = value; }

    void set_path(float path)
    {
        inc_path_ = path;
        dec_path_ = path;
    }

    void set_asymmetric(float increment_path, float decrement_path)
    {
        inc_path_ = increment_path;
        dec_path_ = decrement_path;
    }

    float update(float target)
    {
        const float delta = target - value_;
        if ((delta >= 0.0f && delta < inc_path_) || (delta < 0.0f && delta > -dec_path_))
        {
            value_ = target;
        }
        else if (target < value_)
        {
            value_ -= dec_path_;
        }
        else
        {
            value_ += inc_path_;
        }
        return value_;
    }

    float value() const { return value_; }

private:
    float value_ = 0.0f;

    // Angle increment per phi_control() call, in rad/call.
    float inc_path_ = 0.0f;
    float dec_path_ = 0.0f;
};

} // namespace wbr::control
