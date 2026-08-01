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
        {{5.987434f, 16.761150f, -15.591339f, -22.395994f, 6.899557f, 14.037829f}},
        {{9.578144f, 17.132573f, -26.148765f, -24.349994f, 13.761032f, 24.877827f}},
        {{9.578169f, -30.769193f, 3.795568f, 42.466937f, -20.319860f, -4.638336f}},
        {{4.733611f, -17.242325f, 1.815269f, 23.456627f, -12.300716f, -2.135136f}},
        {{14.666602f, 107.673777f, -18.937049f, -92.156476f, 67.722953f, 17.355681f}},
        {{1.317035f, 13.126933f, -3.526414f, -1.825002f, 4.055511f, 4.250631f}},
        {{9.355876f, -43.313693f, 21.021243f, 76.226133f, -97.582680f, -26.830267f}},
        {{0.911649f, -3.281577f, 4.154690f, 4.695655f, -4.643143f, -4.222425f}},
        {{24.285915f, -62.295104f, -17.379267f, 46.716578f, 26.735281f, 18.816681f}},
        {{3.828061f, -8.268637f, -4.398613f, 3.996634f, 6.187558f, 4.765986f}},
        {{5.987434f, -15.591339f, 16.761150f, 14.037829f, 6.899557f, -22.395994f}},
        {{9.578144f, -26.148765f, 17.132573f, 24.877827f, 13.761032f, -24.349994f}},
        {{-9.578169f, -3.795568f, 30.769193f, 4.638336f, 20.319860f, -42.466937f}},
        {{-4.733611f, -1.815269f, 17.242325f, 2.135136f, 12.300716f, -23.456627f}},
        {{9.355876f, 21.021243f, -43.313693f, -26.830267f, -97.582680f, 76.226133f}},
        {{0.911649f, 4.154690f, -3.281577f, -4.222425f, -4.643143f, 4.695655f}},
        {{14.666602f, -18.937049f, 107.673777f, 17.355681f, 67.722953f, -92.156476f}},
        {{1.317035f, -3.526414f, 13.126933f, 4.250631f, 4.055511f, -1.825002f}},
        {{24.285915f, -17.379267f, -62.295104f, 18.816681f, 26.735281f, 46.716578f}},
        {{3.828061f, -4.398613f, -8.268637f, 4.765986f, 6.187558f, 3.996634f}},
        {{-1.405312f, 2.152052f, 1.602635f, 1.068834f, -5.552146f, 1.265088f}},
        {{-1.958455f, 2.604190f, 3.386316f, 1.329274f, -8.068716f, 0.528635f}},
        {{4.559523f, 9.807166f, 10.306107f, -16.219954f, 1.948709f, -11.680135f}},
        {{2.338780f, 6.277422f, 6.087691f, -9.735165f, 1.187750f, -6.625956f}},
        {{-11.529125f, -39.860958f, -20.419757f, 39.043628f, -33.279192f, 30.202293f}},
        {{-1.112367f, -2.521499f, -1.891236f, 0.410382f, -3.675659f, 3.095707f}},
        {{0.914367f, 27.062526f, 50.856785f, -44.764043f, 26.087148f, -35.525177f}},
        {{0.240229f, 2.962569f, 2.452770f, -4.438957f, 2.568001f, 1.359812f}},
        {{32.935816f, 58.307152f, -37.134531f, -67.549669f, 3.311495f, 35.885992f}},
        {{3.534636f, 9.940947f, -6.306619f, -10.648804f, -0.085017f, 6.094687f}},
        {{-1.405312f, 1.602635f, 2.152052f, 1.265088f, -5.552146f, 1.068834f}},
        {{-1.958455f, 3.386316f, 2.604190f, 0.528635f, -8.068716f, 1.329274f}},
        {{-4.559523f, -10.306107f, -9.807166f, 11.680135f, -1.948709f, 16.219954f}},
        {{-2.338780f, -6.087691f, -6.277422f, 6.625956f, -1.187750f, 9.735165f}},
        {{0.914367f, 50.856785f, 27.062526f, -35.525177f, 26.087148f, -44.764043f}},
        {{0.240229f, 2.452770f, 2.962569f, 1.359812f, 2.568001f, -4.438957f}},
        {{-11.529125f, -20.419757f, -39.860958f, 30.202293f, -33.279192f, 39.043628f}},
        {{-1.112367f, -1.891236f, -2.521499f, 3.095707f, -3.675659f, 0.410382f}},
        {{32.935816f, -37.134531f, 58.307152f, 35.885992f, 3.311495f, -67.549669f}},
        {{3.534636f, -6.306619f, 9.940947f, 6.094687f, -0.085017f, -10.648804f}},
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
