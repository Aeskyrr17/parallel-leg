#include "odometry.hpp"

#include <cmath>

namespace wbr::control
{
namespace
{

bool finite_array(const float* values, int count)
{
    if (values == nullptr)
    {
        return false;
    }

    for (int i = 0; i < count; ++i)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

} // namespace

Odometry::Odometry(const chassis_config& cfg) : cfg_(cfg)
{
    reset();
}

bool Odometry::update(const odometry_input& input)
{
    state_.valid = false;
    if (!valid_input(input))
    {
        return false;
    }

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

    if (!finite_array(world_acceleration, 4))
    {
        return false;
    }

    const float horizontal_acceleration = std::sqrt(world_acceleration[1] * world_acceleration[1] +
                                                    world_acceleration[2] * world_acceleration[2]);
    const float acceleration_angle = std::atan2(world_acceleration[2], world_acceleration[1]);
    const float forward_acceleration =
        horizontal_acceleration * std::cos(acceleration_angle - input.yaw);

    if (!std::isfinite(forward_acceleration) ||
        !update_filter(input.wheel_velocity, forward_acceleration, input.dt))
    {
        return false;
    }

    state_.x = estimate_[0];
    state_.v = estimate_[1];
    state_.a_z = world_acceleration[3];
    state_.valid = std::isfinite(state_.x) && std::isfinite(state_.v) && std::isfinite(state_.a_z);
    return state_.valid;
}

void Odometry::reset()
{
    state_ = {};
    estimate_[0] = 0.0f;
    estimate_[1] = 0.0f;
    estimate_[2] = 0.0f;

    for (float& value : covariance_)
    {
        value = 0.0f;
    }
    covariance_[0] = cfg_.odometry.initial_variance;
    covariance_[4] = cfg_.odometry.initial_variance;
    covariance_[8] = cfg_.odometry.initial_variance;
}

void Odometry::quaternion_product(const float left[4], const float right[4], float result[4])
{
    result[0] = left[0] * right[0] - left[1] * right[1] - left[2] * right[2] - left[3] * right[3];
    result[1] = left[0] * right[1] + left[1] * right[0] + left[2] * right[3] - left[3] * right[2];
    result[2] = left[0] * right[2] - left[1] * right[3] + left[2] * right[0] + left[3] * right[1];
    result[3] = left[0] * right[3] + left[1] * right[2] - left[2] * right[1] + left[3] * right[0];
}

bool Odometry::valid_input(const odometry_input& input) const
{
    if (!input.valid || !finite_array(input.quaternion, 4) ||
        !finite_array(input.acceleration, 3) || !std::isfinite(input.wheel_velocity) ||
        !std::isfinite(input.yaw) || !std::isfinite(input.dt) || input.dt < cfg_.runtime.min_dt_s ||
        input.dt > cfg_.runtime.max_dt_s)
    {
        return false;
    }

    const float norm_sq =
        input.quaternion[0] * input.quaternion[0] + input.quaternion[1] * input.quaternion[1] +
        input.quaternion[2] * input.quaternion[2] + input.quaternion[3] * input.quaternion[3];
    return std::isfinite(norm_sq) && norm_sq > cfg_.odometry.quaternion_norm_epsilon;
}

bool Odometry::update_filter(float velocity, float acceleration, float dt)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    const float transition[9] = {
        1.0f, dt, 0.5f * dt2, 0.0f, 1.0f, dt, 0.0f, 0.0f, 1.0f,
    };
    const float q = cfg_.odometry.process_noise;
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

    const float s00 = predicted_covariance[4] + cfg_.odometry.wheel_velocity_noise;
    const float s01 = predicted_covariance[5];
    const float s10 = predicted_covariance[7];
    const float s11 = predicted_covariance[8] + cfg_.odometry.acceleration_noise;
    const float determinant = s00 * s11 - s01 * s10;
    if (!std::isfinite(determinant) || std::fabs(determinant) <= cfg_.odometry.innovation_epsilon)
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

    if (!finite_array(next_estimate, 3) || !finite_array(next_covariance, 9))
    {
        return false;
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
