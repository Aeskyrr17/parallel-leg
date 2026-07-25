#pragma once

#include "leg_config.hpp"
#include "leg_messages.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace app
{

struct odometry_input
{
    std::array<float, 4> quaternion_wxyz{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> acceleration_body_mps2{};
    float yaw_rad = 0.0f;
    float left_wheel_velocity_rad_s = 0.0f;
    float right_wheel_velocity_rad_s = 0.0f;
    std::uint32_t tick = 0U;
    bool off_ground = false;
};

class odometry
{
public:
    odometry() noexcept
    {
        reset();
    }

    [[nodiscard]] leg_messages::odometry update(const odometry_input& input) noexcept
    {
        if (input.off_ground ||
            !all_finite(input.quaternion_wxyz) ||
            !all_finite(input.acceleration_body_mps2) ||
            !std::isfinite(input.yaw_rad) ||
            !std::isfinite(input.left_wheel_velocity_rad_s) ||
            !std::isfinite(input.right_wheel_velocity_rad_s))
        {
            return invalidate(input.tick);
        }

        float w = input.quaternion_wxyz[0];
        float x = input.quaternion_wxyz[1];
        float y = input.quaternion_wxyz[2];
        float z = input.quaternion_wxyz[3];
        const float norm_squared = w * w + x * x + y * y + z * z;
        if (!std::isfinite(norm_squared) || norm_squared < 1.0e-8f)
        {
            return invalidate(input.tick);
        }

        const float inverse_norm = 1.0f / std::sqrt(norm_squared);
        w *= inverse_norm;
        x *= inverse_norm;
        y *= inverse_norm;
        z *= inverse_norm;

        const float ax = input.acceleration_body_mps2[0];
        const float ay = input.acceleration_body_mps2[1];
        const float az = input.acceleration_body_mps2[2];
        const vector3 acceleration_world{
            (1.0f - 2.0f * (y * y + z * z)) * ax +
                2.0f * (x * y - w * z) * ay +
                2.0f * (x * z + w * y) * az,
            2.0f * (x * y + w * z) * ax +
                (1.0f - 2.0f * (x * x + z * z)) * ay +
                2.0f * (y * z - w * x) * az,
            2.0f * (x * z - w * y) * ax +
                2.0f * (y * z + w * x) * ay +
                (1.0f - 2.0f * (x * x + y * y)) * az,
        };

        const float left_velocity =
            input.left_wheel_velocity_rad_s *
            static_cast<float>(leg_config::left_wheel_direction) *
            leg_config::wheel_radius_m;
        const float right_velocity =
            input.right_wheel_velocity_rad_s *
            static_cast<float>(leg_config::right_wheel_direction) *
            leg_config::wheel_radius_m;
        const float wheel_velocity = 0.5f * (left_velocity + right_velocity);
        const float forward_acceleration =
            acceleration_world[0] * std::cos(input.yaw_rad) +
            acceleration_world[1] * std::sin(input.yaw_rad);

        if (!all_finite(acceleration_world) ||
            !std::isfinite(wheel_velocity) ||
            !std::isfinite(forward_acceleration) ||
            !update_filter(wheel_velocity, forward_acceleration))
        {
            return invalidate(input.tick);
        }

        return {
            state_[0],
            state_[1],
            acceleration_world[2],
            input.tick,
            true,
        };
    }

    void reset() noexcept
    {
        state_ = {};
        covariance_ = {
            10.0f, 0.0f, 0.0f,
            10.0f, 0.0f,
            10.0f,
        };
    }

private:
    using vector3 = std::array<float, 3>;
    using covariance6 = std::array<float, 6>;

    static constexpr float dt_s = 0.001f;
    static constexpr float process_noise = 5.0f;
    static constexpr float wheel_velocity_noise = 0.1f;
    static constexpr float acceleration_noise = 50.0f;

    template <std::size_t size>
    static bool all_finite(const std::array<float, size>& values) noexcept
    {
        for (const float value : values)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        return true;
    }

    leg_messages::odometry invalidate(std::uint32_t tick) noexcept
    {
        reset();
        leg_messages::odometry output{};
        output.tick = tick;
        return output;
    }

    bool update_filter(float wheel_velocity, float acceleration) noexcept
    {
        constexpr float dt2 = dt_s * dt_s;
        constexpr float dt3 = dt2 * dt_s;
        constexpr float dt4 = dt3 * dt_s;
        const covariance6 p = covariance_;
        state_[0] += dt_s * state_[1] + 0.5f * dt2 * state_[2];
        state_[1] += dt_s * state_[2];
        covariance_ = {
            p[0] + 2.0f * dt_s * p[1] + dt2 * (p[2] + p[3]) +
                dt3 * p[4] + 0.25f * dt4 * p[5] +
                dt3 / 20.0f * process_noise,
            p[1] + dt_s * (p[2] + p[3]) + 1.5f * dt2 * p[4] +
                0.5f * dt3 * p[5] + dt4 / 8.0f * process_noise,
            p[2] + dt_s * p[4] + 0.5f * dt2 * p[5] +
                dt3 / 6.0f * process_noise,
            p[3] + 2.0f * dt_s * p[4] + dt2 * p[5] +
                dt3 / 3.0f * process_noise,
            p[4] + dt_s * p[5] + dt2 / 2.0f * process_noise,
            p[5] + dt_s * process_noise,
        };
        return correct(wheel_velocity, 1U, wheel_velocity_noise) &&
               correct(acceleration, 2U, acceleration_noise);
    }

    static constexpr std::size_t covariance_index(std::size_t row,
                                                   std::size_t column) noexcept
    {
        if (row > column)
        {
            const std::size_t temporary = row;
            row = column;
            column = temporary;
        }
        return row == 0U ? column : (row == 1U ? column + 2U : 5U);
    }

    bool correct(float measurement,
                 std::size_t observed_state,
                 float measurement_noise) noexcept
    {
        const covariance6 previous_covariance = covariance_;
        const float innovation_variance =
            previous_covariance[covariance_index(observed_state,
                                                 observed_state)] +
            measurement_noise;
        if (!std::isfinite(innovation_variance) ||
            std::fabs(innovation_variance) < 1.0e-12f)
        {
            return false;
        }

        const float error = measurement - state_[observed_state];
        for (std::size_t row = 0; row < 3U; ++row)
        {
            const float gain =
                previous_covariance[covariance_index(row, observed_state)] /
                innovation_variance;
            state_[row] += gain * error;
            for (std::size_t column = row; column < 3U; ++column)
            {
                const std::size_t index = covariance_index(row, column);
                covariance_[index] =
                    previous_covariance[index] -
                    gain *
                        previous_covariance[covariance_index(observed_state,
                                                             column)];
            }
        }
        return all_finite(state_) && all_finite(covariance_);
    }

    vector3 state_{};
    covariance6 covariance_{};
};

} // namespace app
