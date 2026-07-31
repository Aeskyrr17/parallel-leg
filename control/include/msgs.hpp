#pragma once

#include "motor.hpp"

#include <cstdint>

namespace wbr
{

enum class command_mode : std::uint8_t
{
    relax = 0,
    normal,
    spin,
    jump,
};

enum class jump_command : std::uint8_t
{
    none = 0,
    extending,
    inair,
    landing,
};

struct chassis_command
{
    // Longitudinal position/velocity references, in m and m/s.
    float x = 0.0f;
    float v = 0.0f;

    // Base yaw rate and closed-loop yaw correction, in rad/s.
    float w = 0.0f;
    float dyaw = 0.0f;

    // Final continuous-yaw references, in rad and rad/s.
    float yaw = 0.0f;
    float yaw_rate = 0.0f;

    // Leg length and roll references, in m and rad.
    float len = 0.0f;
    float roll = 0.0f;

    command_mode mode = command_mode::relax;
    jump_command jump = jump_command::none;
    bool valid = false;
};

struct odometry_state
{
    float x = 0.0f;
    float v = 0.0f;

    float a_z = 0.0f; // m/s^2, world z specific force; stationary is about +g
};

struct chassis_feedback
{
    float x = 0.0f;
    float v = 0.0f;
    float total_yaw = 0.0f;
};

struct motor_fdb_frame
{
    motors::feedback left_joint1{};
    motors::feedback left_joint4{};
    motors::feedback right_joint1{};
    motors::feedback right_joint4{};
    motors::feedback left_wheel{};
    motors::feedback right_wheel{};
};

} // namespace wbr
