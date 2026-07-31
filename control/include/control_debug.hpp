#pragma once

#include "chassis.hpp"
#include "msgs.hpp"
#include <cstdint>

namespace wbr
{

struct leg_debug_t
{
    float motor1_position = 0.0f;
    float motor4_position = 0.0f;
    float motor1_velocity = 0.0f;
    float motor4_velocity = 0.0f;

    float phi = 0.0f;
    float dphi = 0.0f;
    float alpha = 0.0f;
    float dalpha = 0.0f;

    float length = 0.0f;
    float length_velocity = 0.0f;

    float force_fdb = 0.0f;
    float torque_fdb = 0.0f;
    float force_ref = 0.0f;
    float torque_ref = 0.0f;
    float support_force = 0.0f;
    float spring_force = 0.0f;

    bool valid = false;
};

struct control_debug_t
{
    std::uint32_t cycle_count = 0U;
    float dt = 0.0f;

    float pitch = 0.0f;
    float dpitch = 0.0f;
    float roll = 0.0f;
    float droll = 0.0f;
    float yaw = 0.0f;
    float dyaw = 0.0f;

    float odometry_x = 0.0f;
    float odometry_v = 0.0f;
    float acceleration_z = 0.0f;

    float command_x = 0.0f;
    float command_v = 0.0f;
    float command_yaw = 0.0f;
    float command_yaw_rate = 0.0f;
    float command_leg_length = 0.0f;

    float wheel_torque_left_ref = 0.0f;
    float wheel_torque_right_ref = 0.0f;

    motors::feedback wheel_left_fdb{};
    motors::feedback wheel_right_fdb{};

    leg_debug_t left_leg{};
    leg_debug_t right_leg{};

    // left joint1, left joint4, right joint1, right joint4, left wheel, right wheel
    float motor_torque[6]{};

    chassis_state state = chassis_state::RELAX;
    jump_stage jump = jump_stage::DONT_JUMP;
    bool input_valid = false;
    bool output_valid = false;
    bool actuation_enabled = false;
};

extern control_debug_t control_debug_data;

} // namespace wbr
