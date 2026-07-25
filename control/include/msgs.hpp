#pragma once

#include "leg_types.hpp"

#include <cstdint>

namespace wbr::control
{

enum class command_mode : std::uint8_t
{
    relax = 0,
    normal,
    spin,
};

enum class command_event : std::uint8_t
{
    none = 0,
    prepare_jump,
    start_jump,
    gostair,
};

enum class command_action : std::uint8_t
{
    none = 0,
    prepare_jump,
    execute_jump,
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
    command_action action = command_action::none;
    command_event event = command_event::none;
    bool valid = false;
};

struct function_feedback
{
    float odometry_x = 0.0f;
};

struct odometry_state
{
    // Longitudinal position, velocity, and world vertical acceleration.
    float x = 0.0f;
    float v = 0.0f;
    float a_z = 0.0f;

    bool valid = false;
};

struct attitude_state
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float total_yaw = 0.0f;
    float gyro_r = 0.0f;
    float gyro_p = 0.0f;
    float gyro_y = 0.0f;
    float acceleration[3] = {};
    bool valid = false;
};

struct motor_feedback_frame
{
    motor_feedback_sample left_joint1{};
    motor_feedback_sample left_joint4{};
    motor_feedback_sample right_joint1{};
    motor_feedback_sample right_joint4{};
    motor_feedback_sample left_wheel{};
    motor_feedback_sample right_wheel{};
    bool valid = false;
};

struct power_state
{
    // Reserved insertion point for referee/supercapacitor limiting.
    float torque_scale = 1.0f;
    bool valid = false;
};

struct health_state
{
    bool motors_online = false;
    bool attitude_fresh = false;
    bool command_fresh = false;
    bool valid = false;
};

} // namespace wbr::control
