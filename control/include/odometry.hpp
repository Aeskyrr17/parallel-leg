#pragma once

#include "leg_config.hpp"
#include "msgs.hpp"

namespace wbr::control
{

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
    float dt = 0.0f;

    bool valid = false;
};

class Odometry
{
public:
    explicit Odometry(const chassis_config& cfg);

    bool update(const odometry_input& input);
    const odometry_state& state() const { return state_; }

    void reset();

private:
    static void quaternion_product(const float left[4], const float right[4], float result[4]);
    bool valid_input(const odometry_input& input) const;
    bool update_filter(float velocity, float acceleration, float dt);

    chassis_config cfg_{};
    odometry_state state_{};

    // State is [longitudinal position, velocity, acceleration].
    float estimate_[3] = {};
    float covariance_[9] = {};
};

} // namespace wbr::control
