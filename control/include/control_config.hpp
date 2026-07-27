#pragma once

#include "pid.hpp"

#include <cstdint>

namespace wbr::control
{

struct command_config
{
    // Full-stick scales: m/s, rad/s.
    float vel_scale = 2.5f;
    float yaw_scale = 1.5f;
    float spin_rate = 3.0f;

    // (m/s)/s, (rad/s)/s.
    float vel_slope_rate = 5.0f;
    float yaw_slope_rate = 10.0f;

    // m/s, m.
    float manual_len_rate = 0.8f;
    float min_len = 0.14f;
    float max_len = 0.34f;
    float normal_len = 0.17f;

    float stationary_vel = 0.002f;
};

struct runtime_config
{
    float dt = 0.001f;

    std::uint32_t control_thread_priority = 5U;
    bool actuation_enabled = false;
};

struct leg_dir
{
    // Motor-to-mechanical coordinate direction. Values must be +1 or -1.
    float joint1_dir = 1.0f;
    float joint4_dir = 1.0f;
};

struct motor_dir_config
{
    leg_dir left_leg{-1.0f, -1.0f};
    leg_dir right_leg{1.0f, 1.0f};

    // Command direction from chassis convention to motor convention.
    float left_wheel_dir = 1.0f;
    float right_wheel_dir = -1.0f;
};

struct joint_offset_config
{
    // LK8016 raw encoder zero points.
    std::uint16_t left_joint4 = 0x11FFU;
    std::uint16_t left_joint1 = 0x25FFU;
    std::uint16_t right_joint4 = 0xC6C3U;
    std::uint16_t right_joint1 = 0x8419U;
};

struct solver_numerics_config
{
    float singularity_epsilon = 1.0e-5f;
    float spring_singularity_epsilon = 5.0e-2f;
};

struct lqr_config
{
    // m.
    float min_leg_len = 0.15f;
    float max_leg_len = 0.35f;
    float leg_len_resolution = 0.01f;
};

struct state_config
{
    // wbr_2026 Normal control values: N, m.
    float offground_support_force = 20.0f;
    float leg_len_bias = 0.03f;
};

struct leg_config
{
    ::control::pid len_pid{5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f,
                           ::control::pid_mode::delta};
    float max_hip_tau = 40.0f;

    // wbr_2026 five-bar geometry [m].
    float l1 = 0.150f;
    float l2 = 0.270f;
    float motor_distance = 0.150f;

    bool has_spring = false;
    // N, m, m, rad.
    float spring_force = 0.0f;
    float spring_offset_1 = 0.0f;
    float spring_offset_2 = 0.0f;
    float spring_angle = 0.0f;

    solver_numerics_config numerics{};
};

struct chassis_config
{
    ::control::pid roll_pid{0.5f, 0.0f, -0.5f, 3.0f, 0.0f,
                            ::control::pid_mode::position};
    float max_wheel_tau = 15.0f;

    command_config cmd{};
    runtime_config runtime{};

    float wheel_radius = 0.077f;
    float wheel_side_mass = 1.41f;
    float gravity = 9.78f;

    motor_dir_config motor_dir{};
    joint_offset_config joint_offset{};
    lqr_config lqr{};
    state_config state{};
};

struct control_config
{
    leg_config leg{};
    chassis_config chassis{};
};

inline const control_config k_default_control{};

} // namespace wbr::control
