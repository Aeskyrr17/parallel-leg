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
        {{5.397366f, 18.508129f, -15.439348f, -24.249155f, 6.742476f, 13.709837f}},
        {{7.955198f, 18.306733f, -23.022919f, -26.170157f, 14.554048f, 20.416844f}},
        {{8.215414f, -18.964868f, 8.059510f, 21.651438f, -7.596835f, -9.003278f}},
        {{3.683063f, -9.710602f, 4.322747f, 10.648399f, -4.335287f, -4.780587f}},
        {{11.786557f, 80.645730f, -25.704011f, -52.926879f, 9.432224f, 29.744280f}},
        {{1.186343f, 10.668978f, -3.924575f, 0.168761f, -1.252419f, 4.939652f}},
        {{6.038180f, -16.925411f, 36.324450f, 19.490983f, -37.681426f, -33.534622f}},
        {{0.694087f, -0.601277f, 4.312079f, -0.935094f, 1.487480f, -3.106619f}},
        {{25.960625f, -47.323754f, -31.967376f, 29.642944f, 32.782012f, 24.804104f}},
        {{3.831356f, -5.099563f, -6.540603f, 1.034985f, 6.651394f, 5.482820f}},
        {{5.397366f, -15.439348f, 18.508129f, 13.709837f, 6.742476f, -24.249155f}},
        {{7.955198f, -23.022919f, 18.306733f, 20.416844f, 14.554048f, -26.170157f}},
        {{-8.215414f, -8.059510f, 18.964868f, 9.003278f, 7.596835f, -21.651438f}},
        {{-3.683063f, -4.322747f, 9.710602f, 4.780587f, 4.335287f, -10.648399f}},
        {{6.038180f, 36.324450f, -16.925411f, -33.534622f, -37.681426f, 19.490983f}},
        {{0.694087f, 4.312079f, -0.601277f, -3.106619f, 1.487480f, -0.935094f}},
        {{11.786557f, -25.704011f, 80.645730f, 29.744280f, 9.432224f, -52.926879f}},
        {{1.186343f, -3.924575f, 10.668978f, 4.939652f, -1.252419f, 0.168761f}},
        {{25.960625f, -31.967376f, -47.323754f, 24.804104f, 32.782012f, 29.642944f}},
        {{3.831356f, -6.540603f, -5.099563f, 5.482820f, 6.651394f, 1.034985f}},
        {{-4.961289f, -5.024032f, 18.168774f, 19.309948f, -16.139585f, -14.926763f}},
        {{-6.457789f, -5.887615f, 25.644713f, 22.936393f, -22.972077f, -21.143767f}},
        {{5.898438f, 15.928774f, 11.177274f, -26.138153f, 12.907685f, -13.599898f}},
        {{2.593753f, 9.260017f, 6.020108f, -14.313010f, 7.823414f, -7.053549f}},
        {{-22.751587f, -32.693614f, -9.684624f, 32.788415f, -77.642336f, 22.980849f}},
        {{-2.183013f, -3.751041f, 0.657495f, 0.399794f, -6.486844f, 0.203159f}},
        {{-0.084636f, 34.817703f, 51.543755f, -54.062312f, 71.838284f, -35.655570f}},
        {{0.085213f, 2.786303f, 2.162581f, -2.914838f, 5.078960f, 2.946893f}},
        {{43.038825f, 105.825591f, -43.785434f, -113.082781f, -12.021326f, 42.606724f}},
        {{3.440293f, 13.506285f, -4.849828f, -11.896791f, -3.557819f, 4.306390f}},
        {{-4.961289f, 18.168774f, -5.024032f, -14.926763f, -16.139585f, 19.309948f}},
        {{-6.457789f, 25.644713f, -5.887615f, -21.143767f, -22.972077f, 22.936393f}},
        {{-5.898438f, -11.177274f, -15.928774f, 13.599898f, -12.907685f, 26.138153f}},
        {{-2.593753f, -6.020108f, -9.260017f, 7.053549f, -7.823414f, 14.313010f}},
        {{-0.084636f, 51.543755f, 34.817703f, -35.655570f, 71.838284f, -54.062312f}},
        {{0.085213f, 2.162581f, 2.786303f, 2.946893f, 5.078960f, -2.914838f}},
        {{-22.751587f, -9.684624f, -32.693614f, 22.980849f, -77.642336f, 32.788415f}},
        {{-2.183013f, 0.657495f, -3.751041f, 0.203159f, -6.486844f, 0.399794f}},
        {{43.038825f, -43.785434f, 105.825591f, 42.606724f, -12.021326f, -113.082781f}},
        {{3.440293f, -4.849828f, 13.506285f, 4.306390f, -3.557819f, -11.896791f}},
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
