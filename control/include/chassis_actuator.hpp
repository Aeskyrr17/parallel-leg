#pragma once

#include "chassis.hpp"
#include "control_config.hpp"
#include "leg.hpp"
#include "lkmotors.hpp"

namespace wbr
{

struct actuator_command
{
    float left_joint1 = 0.0f;
    float left_joint4 = 0.0f;
    float right_joint1 = 0.0f;
    float right_joint4 = 0.0f;
    float left_wheel = 0.0f;
    float right_wheel = 0.0f;
};

struct chassis_motor_handles
{
    motors::lk8016& left_joint1;
    motors::lk8016& left_joint4;
    motors::lk8016& right_joint1;
    motors::lk8016& right_joint4;
    motors::lk9025& left_wheel;
    motors::lk9025& right_wheel;
};

class chassis_actuator
{
public:
    chassis_actuator(const actuator_config& cfg, const leg_solver& left_leg,
                     const leg_solver& right_leg, chassis_motor_handles motors);

    joint_state left_joint_state(const motors::feedback& joint1, const motors::feedback& joint4) const;
    joint_state right_joint_state(const motors::feedback& joint1, const motors::feedback& joint4) const;
    float wheel_velocity(const motors::feedback& left, const motors::feedback& right, float wheel_radius) const;

    bool resolve(const chassis_output& output, actuator_command& command) const;
    void write(const actuator_command& command);

private:
    static joint_state make_joint_state(const motors::feedback& joint1,
                                        const motors::feedback& joint4,
                                        const leg_dir& direction);

    const actuator_config& cfg_;
    const leg_solver& left_leg_;
    const leg_solver& right_leg_;
    chassis_motor_handles motors_;
};

} // namespace wbr
