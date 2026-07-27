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

    // Generated from application/leg/Matlab/lqr_config.m during the CMake build.
    // Each gain is fitted as:
    // a0 + a1*Ll + a2*Lr + a3*Ll^2 + a4*Ll*Lr + a5*Lr^2.
    inline static constexpr std::array<coefficient_row, 40U> coefficients_ = {{
        // LQR_COEFFICIENTS_BEGIN
        {{4.657705f, 14.989574f, -12.648837f, -20.093806f, 8.410805f, 8.735656f}},
        {{8.641232f, 16.364162f, -24.630605f, -28.237190f, 24.715490f, 15.957632f}},
        {{10.953624f, -20.587584f, 9.282895f, 23.948229f, -6.129848f, -10.434514f}},
        {{5.217407f, -11.086567f, 5.538724f, 12.231319f, -3.403548f, -6.352334f}},
        {{17.952792f, 80.324371f, -25.938148f, -62.237936f, 5.547333f, 29.279033f}},
        {{1.412288f, 10.801680f, -4.013013f, -0.965600f, -1.100053f, 4.391813f}},
        {{8.044722f, -21.227749f, 37.549116f, 24.865230f, -29.623541f, -38.227984f}},
        {{0.842091f, -0.997068f, 4.417606f, -0.992234f, 3.107375f, -4.311425f}},
        {{41.165286f, -86.776501f, -46.345970f, 72.729316f, 40.193746f, 36.221659f}},
        {{4.642264f, -7.063799f, -7.598472f, 2.897386f, 7.988268f, 5.883209f}},
        {{4.657705f, -12.648837f, 14.989574f, 8.735656f, 8.410805f, -20.093806f}},
        {{8.641232f, -24.630605f, 16.364162f, 15.957632f, 24.715490f, -28.237190f}},
        {{-10.953624f, -9.282895f, 20.587584f, 10.434514f, 6.129848f, -23.948229f}},
        {{-5.217407f, -5.538724f, 11.086567f, 6.352334f, 3.403548f, -12.231319f}},
        {{8.044722f, 37.549116f, -21.227749f, -38.227984f, -29.623541f, 24.865230f}},
        {{0.842091f, 4.417606f, -0.997068f, -4.311425f, 3.107375f, -0.992234f}},
        {{17.952792f, -25.938148f, 80.324371f, 29.279033f, 5.547333f, -62.237936f}},
        {{1.412288f, -4.013013f, 10.801680f, 4.391813f, -1.100053f, -0.965600f}},
        {{41.165286f, -46.345970f, -86.776501f, 36.221659f, 40.193746f, 72.729316f}},
        {{4.642264f, -7.598472f, -7.063799f, 5.883209f, 7.988268f, 2.897386f}},
        {{-3.937064f, -9.807571f, 21.160727f, 20.529496f, -14.562366f, -17.023045f}},
        {{-6.499352f, -13.230700f, 35.609251f, 30.072647f, -27.399463f, -28.567268f}},
        {{7.714712f, 17.250544f, 12.895461f, -27.737550f, 13.805919f, -16.173789f}},
        {{3.648089f, 10.170194f, 6.876369f, -15.313304f, 8.955077f, -8.116825f}},
        {{-35.640498f, 6.610396f, -13.384292f, -24.098979f, -78.058192f, 28.750778f}},
        {{-2.608222f, -2.784134f, 1.039148f, -3.205214f, -5.249105f, -0.277831f}},
        {{4.215735f, 36.520370f, 52.053702f, -56.736094f, 74.497730f, -30.499260f}},
        {{0.301066f, 2.574418f, 2.898154f, -2.170876f, 3.082133f, 4.928620f}},
        {{84.727655f, 173.502090f, -81.157873f, -199.323213f, -12.629416f, 86.292874f}},
        {{5.249689f, 16.311704f, -6.569867f, -14.905312f, -4.081023f, 6.243218f}},
        {{-3.937064f, 21.160727f, -9.807571f, -17.023045f, -14.562366f, 20.529496f}},
        {{-6.499352f, 35.609251f, -13.230700f, -28.567268f, -27.399463f, 30.072647f}},
        {{-7.714712f, -12.895461f, -17.250544f, 16.173789f, -13.805919f, 27.737550f}},
        {{-3.648089f, -6.876369f, -10.170194f, 8.116825f, -8.955077f, 15.313304f}},
        {{4.215735f, 52.053702f, 36.520370f, -30.499260f, 74.497730f, -56.736094f}},
        {{0.301066f, 2.898154f, 2.574418f, 4.928620f, 3.082133f, -2.170876f}},
        {{-35.640498f, -13.384292f, 6.610396f, 28.750778f, -78.058192f, -24.098979f}},
        {{-2.608222f, 1.039148f, -2.784134f, -0.277831f, -5.249105f, -3.205214f}},
        {{84.727655f, -81.157873f, 173.502090f, 86.292874f, -12.629416f, -199.323213f}},
        {{5.249689f, -6.569867f, 16.311704f, 6.243218f, -4.081023f, -14.905312f}},
        // LQR_COEFFICIENTS_END
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
