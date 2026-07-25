#pragma once

namespace app
{

class slope
{
public:
    explicit constexpr slope(float initial_value = 0.0f, float path = 0.0f) noexcept
        : initial_value_(initial_value),
          value_(initial_value),
          path_(absolute(path))
    {
    }

    constexpr float update(float target) noexcept
    {
        const float delta = target - value_;

        if (delta > path_)
        {
            value_ += path_;
            reached_ = false;
        }
        else if (delta < -path_)
        {
            value_ -= path_;
            reached_ = false;
        }
        else
        {
            value_ = target;
            reached_ = true;
        }

        return value_;
    }

    constexpr void set_path(float path) noexcept
    {
        path_ = absolute(path);
        reached_ = false;
    }

    [[nodiscard]] constexpr bool reached() const noexcept
    {
        return reached_;
    }

    constexpr void reset() noexcept
    {
        value_ = initial_value_;
        reached_ = false;
    }

    constexpr void reset(float value) noexcept
    {
        value_ = value;
        reached_ = false;
    }

    [[nodiscard]] constexpr float value() const noexcept
    {
        return value_;
    }

    [[nodiscard]] constexpr float path() const noexcept
    {
        return path_;
    }

private:
    static constexpr float absolute(float value) noexcept
    {
        return value < 0.0f ? -value : value;
    }

    float initial_value_ = 0.0f;
    float value_ = 0.0f;
    float path_ = 0.0f;
    bool reached_ = false;
};

} // namespace app
