#pragma once

#include <cstdint>
#include <type_traits>

namespace app::leg_messages
{

enum class jump_state : std::uint8_t
{
    idle = 0,
    starting,
    extending,
    airborne,
    landing,
};

struct command
{
    float speed_mps = 0.0f;
    float yaw_rate_rad_s = 0.0f;
    float position_m = 0.0f;
    float leg_length_m = 0.0f;
    float roll_rad = 0.0f;
    jump_state jump_status = jump_state::idle;
    bool enabled = false;
    bool spin_mode = false;
    std::uint32_t tick = 0U;
    bool valid = false;
};

struct solver_feedback
{
    float left_leg_length_m = 0.0f;
    float right_leg_length_m = 0.0f;
    // LQR alpha = VMC leg angle - pi/2 + body pitch.
    float left_leg_angle_rad = 0.0f;
    float right_leg_angle_rad = 0.0f;
    float left_leg_length_velocity_mps = 0.0f;
    float right_leg_length_velocity_mps = 0.0f;
    // LQR alpha_dot = VMC leg angular velocity + body pitch rate.
    float left_leg_angular_velocity_rad_s = 0.0f;
    float right_leg_angular_velocity_rad_s = 0.0f;
    float support_force_n = 0.0f;
    std::uint32_t tick = 0U;
    bool valid = false;
};

struct control_target
{
    float left_leg_force_n = 0.0f;
    float right_leg_force_n = 0.0f;
    float left_leg_torque_nm = 0.0f;
    float right_leg_torque_nm = 0.0f;
    float left_wheel_torque_nm = 0.0f;
    float right_wheel_torque_nm = 0.0f;
    std::uint32_t tick = 0U;
    bool valid = false;
};

struct motor_torque
{
    struct value
    {
        float target = 0.0f;
        float fdb = 0.0f;
    };

    value left_joint_1{};
    value left_joint_4{};
    value right_joint_1{};
    value right_joint_4{};
    value left_wheel{};
    value right_wheel{};
};

struct odometry
{
    float position_m = 0.0f;
    float velocity_mps = 0.0f;
    float vertical_acceleration_mps2 = 0.0f;
    std::uint32_t tick = 0U;
    bool valid = false;
};

static_assert(std::is_trivially_copyable_v<command>);
static_assert(std::is_trivially_copyable_v<solver_feedback>);
static_assert(std::is_trivially_copyable_v<control_target>);
static_assert(std::is_trivially_copyable_v<motor_torque>);
static_assert(std::is_trivially_copyable_v<odometry>);

} // namespace app::leg_messages
