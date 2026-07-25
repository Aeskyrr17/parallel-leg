#pragma once

#include "leg_config.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace app
{

class lqr
{
public:
    static constexpr std::size_t state_dimension = 10U;
    static constexpr std::size_t output_dimension = 4U;

    // State order:
    // [position, velocity, yaw, yaw velocity,
    //  left leg angle, left leg angular velocity,
    //  right leg angle, right leg angular velocity,
    //  pitch, pitch angular velocity]
    using state = std::array<float, state_dimension>;

    enum class gain_mode
    {
        normal,
        off_ground,
    };

    struct output
    {
        float left_wheel_torque_nm = 0.0f;
        float right_wheel_torque_nm = 0.0f;
        float left_leg_torque_nm = 0.0f;
        float right_leg_torque_nm = 0.0f;
        bool valid = false;
    };

    [[nodiscard]] bool refresh_gain(float left_leg_length_m,
                                    float right_leg_length_m,
                                    gain_mode mode) noexcept
    {
        gains_.fill(0.0f);
        gains_valid_ = false;

        if (!finite(left_leg_length_m) || !finite(right_leg_length_m))
        {
            return false;
        }

        left_leg_length_m = normalize_leg_length(left_leg_length_m);
        right_leg_length_m = normalize_leg_length(right_leg_length_m);

        for (std::size_t index = 0; index < gains_.size(); ++index)
        {
            const bool enabled =
                mode == gain_mode::normal ||
                (index >= 24U && index < 28U) ||
                (index >= 34U && index < 38U);

            if (enabled)
            {
                gains_[index] = evaluate(coefficients_[index],
                                         left_leg_length_m,
                                         right_leg_length_m);
                if (!finite(gains_[index]))
                {
                    gains_.fill(0.0f);
                    return false;
                }
            }
        }

        left_leg_length_m_ = left_leg_length_m;
        right_leg_length_m_ = right_leg_length_m;
        gain_mode_ = mode;
        gains_valid_ = true;
        return true;
    }

    [[nodiscard]] output calculate(const state& observed,
                                   const state& reference) const noexcept
    {
        if (!gains_valid_ || !finite_state(observed) || !finite_state(reference))
        {
            return {};
        }

        std::array<float, output_dimension> raw_output{};

        for (std::size_t row = 0; row < output_dimension; ++row)
        {
            float sum = 0.0f;
            for (std::size_t column = 0; column < state_dimension; ++column)
            {
                const float error = observed[column] - reference[column];
                sum += gains_[row * state_dimension + column] * error;
            }
            if (!finite(sum))
            {
                return {};
            }
            raw_output[row] = sum;
        }

        return {
            raw_output[0],
            raw_output[1],
            raw_output[2],
            raw_output[3],
            true,
        };
    }

    [[nodiscard]] output calculate(const state& observed,
                                   const state& reference,
                                   float left_leg_length_m,
                                   float right_leg_length_m,
                                   gain_mode mode) noexcept
    {
        if (!refresh_gain(left_leg_length_m, right_leg_length_m, mode))
        {
            return {};
        }
        return static_cast<const lqr&>(*this).calculate(observed, reference);
    }

    [[nodiscard]] bool gain_valid() const noexcept
    {
        return gains_valid_;
    }

    [[nodiscard]] float left_leg_length_m() const noexcept
    {
        return left_leg_length_m_;
    }

    [[nodiscard]] float right_leg_length_m() const noexcept
    {
        return right_leg_length_m_;
    }

    [[nodiscard]] gain_mode mode() const noexcept
    {
        return gain_mode_;
    }

private:
    using coefficient_row = std::array<float, 6U>;

    // Each gain is fitted as:
    // a0 + a1*Ll + a2*Lr + a3*Ll^2 + a4*Ll*Lr + a5*Lr^2.
    inline static constexpr std::array<coefficient_row, 40U> coefficients_ = {{
        {{4.608627f, 15.693424f, -13.130824f, -21.350456f, 8.333527f, 9.805292f}},
        {{9.110602f, 17.461721f, -27.260268f, -30.953716f, 26.639835f, 19.261991f}},
        {{10.828982f, -20.828436f, 9.119085f, 23.722050f, -5.121706f, -10.209400f}},
        {{5.226723f, -11.298301f, 5.497872f, 12.161874f, -2.919920f, -6.265625f}},
        {{17.885435f, 85.839879f, -25.351072f, -62.062095f, -1.729210f, 29.523098f}},
        {{1.435492f, 11.559065f, -4.077824f, -0.697298f, -2.022590f, 4.677714f}},
        {{7.372722f, -19.221705f, 41.994916f, 20.129481f, -29.003084f, -40.189048f}},
        {{0.819483f, -0.719209f, 4.629012f, -1.669292f, 3.512847f, -4.187260f}},
        {{42.433682f, -88.566003f, -46.637988f, 72.381677f, 43.486075f, 34.086246f}},
        {{4.883122f, -7.081486f, -7.887498f, 2.582041f, 8.285827f, 6.029934f}},
        {{4.608627f, -13.130824f, 15.693424f, 9.805292f, 8.333527f, -21.350456f}},
        {{9.110602f, -27.260268f, 17.461721f, 19.261991f, 26.639835f, -30.953716f}},
        {{-10.828982f, -9.119085f, 20.828436f, 10.209400f, 5.121706f, -23.722050f}},
        {{-5.226723f, -5.497872f, 11.298301f, 6.265625f, 2.919920f, -12.161874f}},
        {{7.372722f, 41.994916f, -19.221705f, -40.189048f, -29.003084f, 20.129481f}},
        {{0.819483f, 4.629012f, -0.719209f, -4.187260f, 3.512847f, -1.669292f}},
        {{17.885435f, -25.351072f, 85.839879f, 29.523098f, -1.729210f, -62.062095f}},
        {{1.435492f, -4.077824f, 11.559065f, 4.677714f, -2.022590f, -0.697298f}},
        {{42.433682f, -46.637988f, -88.566003f, 34.086246f, 43.486075f, 72.381677f}},
        {{4.883122f, -7.887498f, -7.081486f, 6.029934f, 8.285827f, 2.582041f}},
        {{-4.031337f, -10.255458f, 21.441367f, 22.874263f, -15.083320f, -18.125448f}},
        {{-7.076496f, -13.661244f, 38.117600f, 34.274141f, -30.532341f, -32.116578f}},
        {{8.167806f, 17.783977f, 11.977512f, -28.665862f, 12.643648f, -14.834448f}},
        {{3.923688f, 10.463738f, 6.448669f, -15.824739f, 8.427218f, -7.477878f}},
        {{-35.715486f, 4.556768f, -10.243641f, -21.424780f, -76.650504f, 23.754794f}},
        {{-2.612315f, -3.176328f, 1.471048f, -2.492183f, -5.018671f, -1.022546f}},
        {{3.877345f, 34.432313f, 51.860924f, -51.918628f, 74.287588f, -30.259659f}},
        {{0.296078f, 2.164369f, 2.737478f, -1.214197f, 2.979120f, 4.928525f}},
        {{83.329589f, 175.157405f, -75.631021f, -200.204595f, -16.852611f, 80.842920f}},
        {{5.177305f, 15.833920f, -5.779582f, -14.023466f, -4.543931f, 5.394145f}},
        {{-4.031337f, 21.441367f, -10.255458f, -18.125448f, -15.083320f, 22.874263f}},
        {{-7.076496f, 38.117600f, -13.661244f, -32.116578f, -30.532341f, 34.274141f}},
        {{-8.167806f, -11.977512f, -17.783977f, 14.834448f, -12.643648f, 28.665862f}},
        {{-3.923688f, -6.448669f, -10.463738f, 7.477878f, -8.427218f, 15.824739f}},
        {{3.877345f, 51.860924f, 34.432313f, -30.259659f, 74.287588f, -51.918628f}},
        {{0.296078f, 2.737478f, 2.164369f, 4.928525f, 2.979120f, -1.214197f}},
        {{-35.715486f, -10.243641f, 4.556768f, 23.754794f, -76.650504f, -21.424780f}},
        {{-2.612315f, 1.471048f, -3.176328f, -1.022546f, -5.018671f, -2.492183f}},
        {{83.329589f, -75.631021f, 175.157405f, 80.842920f, -16.852611f, -200.204595f}},
        {{5.177305f, -5.779582f, 15.833920f, 5.394145f, -4.543931f, -14.023466f}},
    }};

    static bool finite(float value) noexcept
    {
        return std::isfinite(value);
    }

    static bool finite_state(const state& values) noexcept
    {
        for (const float value : values)
        {
            if (!finite(value))
            {
                return false;
            }
        }
        return true;
    }

    static float normalize_leg_length(float length_m) noexcept
    {
        if (length_m < leg_config::min_control_leg_length_m)
        {
            length_m = leg_config::min_control_leg_length_m;
        }
        else if (length_m > leg_config::max_control_leg_length_m)
        {
            length_m = leg_config::max_control_leg_length_m;
        }

        return std::round(length_m / leg_config::leg_length_resolution_m) *
               leg_config::leg_length_resolution_m;
    }

    static float evaluate(const coefficient_row& coefficients,
                          float left_leg_length_m,
                          float right_leg_length_m) noexcept
    {
        return coefficients[0] +
               coefficients[1] * left_leg_length_m +
               coefficients[2] * right_leg_length_m +
               coefficients[3] * left_leg_length_m * left_leg_length_m +
               coefficients[4] * left_leg_length_m * right_leg_length_m +
               coefficients[5] * right_leg_length_m * right_leg_length_m;
    }

    std::array<float, output_dimension * state_dimension> gains_{};
    float left_leg_length_m_ = 0.0f;
    float right_leg_length_m_ = 0.0f;
    gain_mode gain_mode_ = gain_mode::normal;
    bool gains_valid_ = false;
};

} // namespace app
