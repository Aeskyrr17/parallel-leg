#pragma once

#include "pid.hpp"

#include <cstdint>

namespace wbr
{

struct command_config
{
    // Full-stick scales: m/s, rad/s.
    float vel_scale = 2.0f;
    float yaw_scale = 4.0f;
    float spin_rate = 5.0f;

    // (m/s)/s, (rad/s)/s.
    float vel_slope_rate = 3.0f;
    float yaw_slope_rate = 10.0f;

    // m/s, m.
    float manual_len_rate = 0.2f;
    float min_len = 0.14f;
    float max_len = 0.34f;
    float normal_len = 0.23f;
    float prepare_len = 0.20f;

    float stationary_vel = 0.05f;
    float stationary_vel_error = 0.05f;
    std::uint8_t stop_confirm_ticks = 5U;
};

struct runtime_config
{
    float dt = 0.001f;

    std::uint32_t control_thread_priority = 5U;
    std::uint32_t function_thread_priority = 6U;
    bool actuation_enabled = true;
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

struct actuator_config
{
    motor_dir_config motor_dir{};
    float max_hip_tau = 40.0f;
    float max_wheel_tau = 15.0f;
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
    float min_leg_len = 0.14f;
    float max_leg_len = 0.35f;
    float leg_len_resolution = 0.01f;
};

struct state_config
{
    // wbr_2026 Normal control values: N, m.
    float offground_support_force = 10.0f;
};

struct jump_config
{
    // wbr_2026 V0.1 jump values: m, N.
    float prepare_len = 0.20f;
    float extend_len = 0.370f;
    float retract_len = 0.140f;
    float offground_len = 0.300f;

    float extend_done_len = 0.340f;
    float retract_done_len = 0.145f;
    float support_force = 30.0f;

    float extend_force = 450.0f;
    float retract_force = -200.0f;

    float landing_len = 0.26f;

    // 实际腿长比 landing_len 短多少，才认为发生了压缩。
    float landing_compression = 0.01f;

    // 腿长收缩速度阈值，单位 m/s。
    float landing_dlen_threshold = 0.03f;

    // 1 kHz 下连续确认 10 ms。
    std::uint16_t landing_confirm_ticks = 10;

};

struct leg_config
{
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

struct leg_control_config
{
    ::control::pid len_pid{4000.0f, 0.0f, -200.0f, 200.0f, 0.0f,
                           ::control::pid_mode::delta};
};

struct chassis_config
{
    leg_control_config leg_control{};
    leg_control_config jump_retract_control{
        ::control::pid{5000.0f, 0.0f, -200.0f, 200.0f, 0.0f,
                       ::control::pid_mode::delta}};
    ::control::pid roll_pid{0.3f, 0.0f, -0.015f, 0.05f, 0.0f,
                             ::control::pid_mode::delta};
    actuator_config actuator{};

    command_config cmd{};
    runtime_config runtime{};

    float wheel_radius = 0.077f;
    float wheel_side_mass = 1.41f;
    float gravity = 9.78f;

    joint_offset_config joint_offset{};
    lqr_config lqr{};
    state_config state{};
    jump_config jump{};

    //在relax状态下读到的roll和pitch角度偏差
    float roll_offset = 0.002;
    float pitch_offset = -0.015;
};

inline const leg_config k_default_leg{};
inline const chassis_config k_default_chassis{};

} // namespace wbr
