#pragma once

namespace wbr::control
{

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

private:
    float value_ = 0.0f;
    float inc_path_ = 0.0f;
    float dec_path_ = 0.0f;
};

} // namespace wbr::control
