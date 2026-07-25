#pragma once

#include "motor.hpp"

#include <cstdint>

namespace wbr::control
{

enum class command_mode : std::uint8_t
{
    relax = 0,
    normal,
    spin,
};

enum class jump_command : std::uint8_t
{
    none = 0,
    prepare,
    execute,
};

struct chassis_command
{
    // Longitudinal position/velocity references, in m and m/s.
    float x = 0.0f;
    float v = 0.0f;

    // Base yaw rate and closed-loop yaw correction, in rad/s.
    float w = 0.0f;
    float dyaw = 0.0f;

    // Leg length and roll references, in m and rad.
    float len = 0.0f;
    float roll = 0.0f;

    command_mode mode = command_mode::relax;
    jump_command jump = jump_command::none;
    bool valid = false;
};

struct odometry_state
{
    // Longitudinal position, velocity, and world vertical acceleration.
    float x = 0.0f;
    float v = 0.0f;
    float a_z = 0.0f;

    bool valid = false;
};

struct motor_feedback_frame
{
    motors::feedback left_joint1{};
    motors::feedback left_joint4{};
    motors::feedback right_joint1{};
    motors::feedback right_joint4{};
    motors::feedback left_wheel{};
    motors::feedback right_wheel{};

    bool left_joint1_valid = false;
    bool left_joint4_valid = false;
    bool right_joint1_valid = false;
    bool right_joint4_valid = false;
    bool left_wheel_valid = false;
    bool right_wheel_valid = false;
    bool valid = false;
};

} // namespace wbr::control
