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
        {{6.049058f, 21.813883f, -18.370352f, -28.917019f, 8.391463f, 16.255714f}},
        {{8.292844f, 19.814885f, -25.120587f, -28.649813f, 16.179320f, 22.251442f}},
        {{8.092852f, -20.312692f, 7.866363f, 22.992915f, -8.235795f, -8.893175f}},
        {{3.015487f, -8.911481f, 3.704102f, 9.592122f, -4.108455f, -4.136561f}},
        {{11.897232f, 81.699206f, -25.636236f, -53.708096f, 9.272312f, 29.877297f}},
        {{1.207778f, 10.875558f, -4.058000f, 0.133601f, -1.416694f, 5.143919f}},
        {{5.977357f, -16.425984f, 34.516774f, 18.030474f, -35.615886f, -33.001750f}},
        {{0.697333f, -0.487203f, 4.142715f, -1.282652f, 1.994504f, -3.259342f}},
        {{25.718977f, -47.713249f, -31.313133f, 30.149004f, 32.077499f, 24.966148f}},
        {{3.924360f, -5.304012f, -6.702178f, 1.169167f, 6.716831f, 5.732238f}},
        {{6.049058f, -18.370352f, 21.813883f, 16.255714f, 8.391463f, -28.917019f}},
        {{8.292844f, -25.120587f, 19.814885f, 22.251442f, 16.179320f, -28.649813f}},
        {{-8.092852f, -7.866363f, 20.312692f, 8.893175f, 8.235795f, -22.992915f}},
        {{-3.015487f, -3.704102f, 8.911481f, 4.136561f, 4.108455f, -9.592122f}},
        {{5.977357f, 34.516774f, -16.425984f, -33.001750f, -35.615886f, 18.030474f}},
        {{0.697333f, 4.142715f, -0.487203f, -3.259342f, 1.994504f, -1.282652f}},
        {{11.897232f, -25.636236f, 81.699206f, 29.877297f, 9.272312f, -53.708096f}},
        {{1.207778f, -4.058000f, 10.875558f, 5.143919f, -1.416694f, 0.133601f}},
        {{25.718977f, -31.313133f, -47.713249f, 24.966148f, 32.077499f, 30.149004f}},
        {{3.924360f, -6.702178f, -5.304012f, 5.732238f, 6.716831f, 1.169167f}},
        {{-5.603986f, -3.912854f, 18.900186f, 20.345057f, -18.058300f, -15.755667f}},
        {{-6.708852f, -4.323107f, 25.082562f, 22.265844f, -23.771438f, -20.896944f}},
        {{6.219262f, 17.396302f, 11.439560f, -27.732426f, 11.993926f, -13.924486f}},
        {{2.244625f, 8.853683f, 5.405246f, -13.231563f, 6.490357f, -6.288022f}},
        {{-22.681578f, -31.257407f, -9.067959f, 32.522925f, -70.301690f, 21.466591f}},
        {{-2.179580f, -3.701567f, 0.696251f, 0.894926f, -6.139587f, 0.118375f}},
        {{-0.143453f, 34.982574f, 49.586528f, -52.571039f, 64.218506f, -35.197624f}},
        {{0.059232f, 2.885699f, 2.069124f, -2.907910f, 4.730778f, 2.511473f}},
        {{43.129535f, 102.947149f, -41.117450f, -110.593869f, -13.341218f, 41.577568f}},
        {{3.666312f, 13.780877f, -4.775360f, -12.398881f, -3.746050f, 4.480438f}},
        {{-5.603986f, 18.900186f, -3.912854f, -15.755667f, -18.058300f, 20.345057f}},
        {{-6.708852f, 25.082562f, -4.323107f, -20.896944f, -23.771438f, 22.265844f}},
        {{-6.219262f, -11.439560f, -17.396302f, 13.924486f, -11.993926f, 27.732426f}},
        {{-2.244625f, -5.405246f, -8.853683f, 6.288022f, -6.490357f, 13.231563f}},
        {{-0.143453f, 49.586528f, 34.982574f, -35.197624f, 64.218506f, -52.571039f}},
        {{0.059232f, 2.069124f, 2.885699f, 2.511473f, 4.730778f, -2.907910f}},
        {{-22.681578f, -9.067959f, -31.257407f, 21.466591f, -70.301690f, 32.522925f}},
        {{-2.179580f, 0.696251f, -3.701567f, 0.118375f, -6.139587f, 0.894926f}},
        {{43.129535f, -41.117450f, 102.947149f, 41.577568f, -13.341218f, -110.593869f}},
        {{3.666312f, -4.775360f, 13.780877f, 4.480438f, -3.746050f, -12.398881f}},
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
