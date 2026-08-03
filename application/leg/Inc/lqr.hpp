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
        {{5.701349f, 19.465086f, -16.316833f, -25.559643f, 7.190538f, 14.470422f}},
        {{8.193372f, 18.730124f, -23.692387f, -26.801342f, 15.060321f, 21.004850f}},
        {{8.214936f, -19.002612f, 8.099752f, 21.612418f, -7.475587f, -9.087901f}},
        {{3.682796f, -9.737744f, 4.351313f, 10.634334f, -4.268989f, -4.834255f}},
        {{11.876546f, 81.153771f, -26.116310f, -52.703160f, 8.895829f, 30.341798f}},
        {{1.198935f, 10.803039f, -4.004302f, 0.238683f, -1.349460f, 5.052658f}},
        {{6.123664f, -16.954893f, 36.473468f, 19.161976f, -36.845741f, -33.779740f}},
        {{0.705835f, -0.587017f, 4.356632f, -1.022896f, 1.695884f, -3.145731f}},
        {{25.834796f, -46.910262f, -32.231077f, 29.315977f, 32.874312f, 25.139086f}},
        {{3.929219f, -5.286746f, -6.693746f, 1.200910f, 6.759999f, 5.617180f}},
        {{5.701349f, -16.316833f, 19.465086f, 14.470422f, 7.190538f, -25.559643f}},
        {{8.193372f, -23.692387f, 18.730124f, 21.004850f, 15.060321f, -26.801342f}},
        {{-8.214936f, -8.099752f, 19.002612f, 9.087901f, 7.475587f, -21.612418f}},
        {{-3.682796f, -4.351313f, 9.737744f, 4.834255f, 4.268989f, -10.634334f}},
        {{6.123664f, 36.473468f, -16.954893f, -33.779740f, -36.845741f, 19.161976f}},
        {{0.705835f, 4.356632f, -0.587017f, -3.145731f, 1.695884f, -1.022896f}},
        {{11.876546f, -26.116310f, 81.153771f, 30.341798f, 8.895829f, -52.703160f}},
        {{1.198935f, -4.004302f, 10.803039f, 5.052658f, -1.349460f, 0.238683f}},
        {{25.834796f, -32.231077f, -46.910262f, 25.139086f, 32.874312f, 29.315977f}},
        {{3.929219f, -6.693746f, -5.286746f, 5.617180f, 6.759999f, 1.200910f}},
        {{-5.146113f, -5.263896f, 18.941392f, 20.155483f, -16.677596f, -15.737569f}},
        {{-6.504678f, -6.132684f, 26.041510f, 23.403352f, -23.081391f, -21.669264f}},
        {{5.899169f, 15.998696f, 11.103123f, -26.083572f, 12.785173f, -13.526858f}},
        {{2.594100f, 9.285595f, 5.992497f, -14.276582f, 7.767241f, -7.031282f}},
        {{-22.635260f, -33.120219f, -9.547232f, 32.888651f, -77.257783f, 22.741074f}},
        {{-2.176559f, -3.804735f, 0.696336f, 0.419436f, -6.421121f, 0.131501f}},
        {{0.037242f, 34.875541f, 51.162892f, -53.793374f, 71.491923f, -35.288156f}},
        {{0.092724f, 2.768718f, 2.158643f, -2.846172f, 5.028713f, 2.951546f}},
        {{43.306005f, 103.695368f, -42.845287f, -110.454730f, -12.390963f, 41.876081f}},
        {{3.698229f, 13.705295f, -4.875154f, -12.162904f, -3.591887f, 4.341117f}},
        {{-5.146113f, 18.941392f, -5.263896f, -15.737569f, -16.677596f, 20.155483f}},
        {{-6.504678f, 26.041510f, -6.132684f, -21.669264f, -23.081391f, 23.403352f}},
        {{-5.899169f, -11.103123f, -15.998696f, 13.526858f, -12.785173f, 26.083572f}},
        {{-2.594100f, -5.992497f, -9.285595f, 7.031282f, -7.767241f, 14.276582f}},
        {{0.037242f, 51.162892f, 34.875541f, -35.288156f, 71.491923f, -53.793374f}},
        {{0.092724f, 2.158643f, 2.768718f, 2.951546f, 5.028713f, -2.846172f}},
        {{-22.635260f, -9.547232f, -33.120219f, 22.741074f, -77.257783f, 32.888651f}},
        {{-2.176559f, 0.696336f, -3.804735f, 0.131501f, -6.421121f, 0.419436f}},
        {{43.306005f, -42.845287f, 103.695368f, 41.876081f, -12.390963f, -110.454730f}},
        {{3.698229f, -4.875154f, 13.705295f, 4.341117f, -3.591887f, -12.162904f}},
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
