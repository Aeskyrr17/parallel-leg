#pragma once

#include "msgs.hpp"

#include <cmath>

namespace wbr::control
{

struct odometry_config
{
    // Legacy wbr_2026 process-noise scale qq.
    float process_noise = 5.0f;

    // Measurement noise values used directly in R.
    float wheel_velocity_noise = 0.1f;
    float acceleration_noise = 50.0f;

    float initial_variance = 10.0f;

    float innovation_epsilon = 1.0e-9f;
};

inline const odometry_config k_default_odometry{};

struct odometry_input
{
    // Body-to-world unit quaternion in [w, x, y, z] order.
    // The caller must provide a normalized quaternion.
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // Body-frame accelerometer specific force [m/s^2].
    // Gravity has not been removed.
    float acceleration[3] = {};

    // Chassis longitudinal velocity from the two wheels, in m/s.
    float wheel_velocity = 0.0f;

    // Chassis yaw in rad.
    float yaw = 0.0f;
    float dt = 0.0f;
};

class Odometry
{
public:
    explicit Odometry(const odometry_config& cfg);

    bool update(const odometry_input& input);
    const odometry_state& state() const { return state_; }

    void reset();

private:
    static void quaternion_product(const float left[4], const float right[4], float result[4]);
    bool update_filter(float velocity, float acceleration, float dt);

    odometry_config cfg_{};
    odometry_state state_{};

    // State is [longitudinal position, velocity, acceleration].
    float estimate_[3] = {};
    float covariance_[9] = {};
};

inline Odometry::Odometry(const odometry_config& cfg) : cfg_(cfg)
{
    reset();
}

inline bool Odometry::update(const odometry_input& input)
{
    const float acceleration_quaternion[4] = {
        0.0f,
        input.acceleration[0],
        input.acceleration[1],
        input.acceleration[2],
    };
    const float quaternion_conjugate[4] = {
        input.quaternion[0],
        -input.quaternion[1],
        -input.quaternion[2],
        -input.quaternion[3],
    };

    float temporary[4] = {};
    float world_acceleration[4] = {};
    quaternion_product(input.quaternion, acceleration_quaternion, temporary);
    quaternion_product(temporary, quaternion_conjugate, world_acceleration);

    const float horizontal_acceleration = std::sqrt(world_acceleration[1] * world_acceleration[1] +
                                                    world_acceleration[2] * world_acceleration[2]);
    const float acceleration_angle = std::atan2(world_acceleration[2], world_acceleration[1]);
    const float forward_acceleration =
        horizontal_acceleration * std::cos(acceleration_angle - input.yaw);

    if (!update_filter(input.wheel_velocity, forward_acceleration, input.dt))
    {
        return false;
    }

    state_.x = estimate_[0];
    state_.v = estimate_[1];
    state_.a_z = world_acceleration[3];
    return true;
}

inline void Odometry::reset()
{
    state_ = {};
    estimate_[0] = 0.0f;
    estimate_[1] = 0.0f;
    estimate_[2] = 0.0f;

    for (float& value : covariance_)
    {
        value = 0.0f;
    }
    covariance_[0] = cfg_.initial_variance;
    covariance_[4] = cfg_.initial_variance;
    covariance_[8] = cfg_.initial_variance;
}

inline void Odometry::quaternion_product(const float left[4], const float right[4],
                                         float result[4])
{
    result[0] = left[0] * right[0] - left[1] * right[1] - left[2] * right[2] -
                left[3] * right[3];
    result[1] = left[0] * right[1] + left[1] * right[0] + left[2] * right[3] -
                left[3] * right[2];
    result[2] = left[0] * right[2] - left[1] * right[3] + left[2] * right[0] +
                left[3] * right[1];
    result[3] = left[0] * right[3] + left[1] * right[2] - left[2] * right[1] +
                left[3] * right[0];
}

inline bool Odometry::update_filter(float velocity, float acceleration, float dt)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    const float transition[9] = {
        1.0f, dt, 0.5f * dt2, 0.0f, 1.0f, dt, 0.0f, 0.0f, 1.0f,
    };
    const float q = cfg_.process_noise;

    // Intentionally preserves the legacy wbr_2026 Q matrix.
    // Q(0,0) uses dt^3 rather than replacing it with a textbook dt^5 term.
    const float process_noise[9] = {
        dt3 * q / 20.0f, dt4 * q / 8.0f, dt3 * q / 6.0f, dt4 * q / 8.0f, dt3 * q / 3.0f,
        dt2 * q / 2.0f,  dt3 * q / 6.0f, dt2 * q / 2.0f, dt * q,
    };

    float predicted_estimate[3] = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            predicted_estimate[row] += transition[row * 3 + column] * estimate_[column];
        }
    }

    float transition_covariance[9] = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int inner = 0; inner < 3; ++inner)
            {
                transition_covariance[row * 3 + column] +=
                    transition[row * 3 + inner] * covariance_[inner * 3 + column];
            }
        }
    }

    float predicted_covariance[9] = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int inner = 0; inner < 3; ++inner)
            {
                predicted_covariance[row * 3 + column] +=
                    transition_covariance[row * 3 + inner] * transition[column * 3 + inner];
            }
            predicted_covariance[row * 3 + column] += process_noise[row * 3 + column];
        }
    }

    const float s00 = predicted_covariance[4] + cfg_.wheel_velocity_noise;
    const float s01 = predicted_covariance[5];
    const float s10 = predicted_covariance[7];
    const float s11 = predicted_covariance[8] + cfg_.acceleration_noise;
    const float determinant = s00 * s11 - s01 * s10;
    if (std::fabs(determinant) <= cfg_.innovation_epsilon)
    {
        return false;
    }

    const float inverse_innovation[4] = {
        s11 / determinant,
        -s01 / determinant,
        -s10 / determinant,
        s00 / determinant,
    };
    float gain[6] = {};
    for (int row = 0; row < 3; ++row)
    {
        const float velocity_covariance = predicted_covariance[row * 3 + 1];
        const float acceleration_covariance = predicted_covariance[row * 3 + 2];
        gain[row * 2] = velocity_covariance * inverse_innovation[0] +
                        acceleration_covariance * inverse_innovation[2];
        gain[row * 2 + 1] = velocity_covariance * inverse_innovation[1] +
                            acceleration_covariance * inverse_innovation[3];
    }

    const float innovation[2] = {
        velocity - predicted_estimate[1],
        acceleration - predicted_estimate[2],
    };
    float next_estimate[3] = {};
    for (int row = 0; row < 3; ++row)
    {
        next_estimate[row] = predicted_estimate[row] + gain[row * 2] * innovation[0] +
                             gain[row * 2 + 1] * innovation[1];
    }

    float next_covariance[9] = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            next_covariance[row * 3 + column] =
                predicted_covariance[row * 3 + column] -
                gain[row * 2] * predicted_covariance[3 + column] -
                gain[row * 2 + 1] * predicted_covariance[6 + column];
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        estimate_[i] = next_estimate[i];
    }
    for (int i = 0; i < 9; ++i)
    {
        covariance_[i] = next_covariance[i];
    }
    return true;
}

} // namespace wbr::control
