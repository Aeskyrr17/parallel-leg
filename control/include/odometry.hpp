#pragma once

#include "msgs.hpp"

namespace wbr::control
{

struct odometry_config
{
    float process_noise = 5.0f;
    float wheel_velocity_noise = 0.1f;
    float acceleration_noise = 50.0f;
    float initial_variance = 10.0f;

    float min_dt = 1.0e-5f;
    float max_dt = 5.0e-2f;
    float quaternion_norm_epsilon = 1.0e-9f;
    float innovation_epsilon = 1.0e-9f;
};

struct odometry_input
{
    // Body-to-world attitude quaternion in [w, x, y, z] order.
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // Body-frame acceleration in m/s^2.
    float acceleration[3] = {};

    // Chassis longitudinal velocity from the two wheels, in m/s.
    float wheel_velocity = 0.0f;

    // Chassis yaw in rad.
    float yaw = 0.0f;
    float dt = 0.001f;

    bool valid = false;
};

class odometry
{
public:
    explicit odometry(const odometry_config& cfg = {});

    bool update(const odometry_input& input);
    const odometry_state& state() const { return state_; }

    void reset();

private:
    static void quaternion_product(const float left[4], const float right[4], float result[4]);
    bool valid_input(const odometry_input& input) const;
    bool update_filter(float velocity, float acceleration, float dt);

    odometry_config cfg_{};
    odometry_state state_{};

    // State is [longitudinal position, velocity, acceleration].
    float estimate_[3] = {};
    float covariance_[9] = {};
};

} // namespace wbr::control
