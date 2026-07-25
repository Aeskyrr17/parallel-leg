#pragma once

#include "pid.hpp"

#include <cstdint>

namespace wbr::control
{

struct pid_params
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float max_out = 0.0f;
    float max_i_out = 0.0f;
    ::control::pid_mode mode = ::control::pid_mode::position;
};

struct leg_pid_config
{
    // Active wbr_2026 real-robot values. Length output is N and derivative input
    // is m/s.
    pid_params len{5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f, ::control::pid_mode::delta};

    // MuJoCo-derived phi controller. Output is N*m and derivative input is rad/s.
    // Its use remains gated until the special-state targets are validated.
    pid_params phi{0.7f, 0.0f, 1.4f, 20.0f, 0.005f, ::control::pid_mode::delta};
    float phi_slope_step = 0.005f;
};

struct leg_calibration
{
    // Motor feedback zero positions, in rad.
    float joint1_zero_rad = 0.0f;
    float joint4_zero_rad = 0.0f;

    // Motor-to-mechanical coordinate direction. Calibrated values must be +1 or
    // -1.
    float joint1_direction = 1.0f;
    float joint4_direction = 1.0f;
};

struct motor_calibration_config
{
    // Raw LK encoder offsets copied from the active wbr_2026 robot.
    std::uint16_t left_joint4_offset = 0x11FFU;
    std::uint16_t left_joint1_offset = 0x25FFU;
    std::uint16_t right_joint4_offset = 0xC6C3U;
    std::uint16_t right_joint1_offset = 0x8419U;

    leg_calibration left_leg{0.0f, 0.0f, -1.0f, -1.0f};
    leg_calibration right_leg{0.0f, 0.0f, 1.0f, 1.0f};

    // Command direction from chassis convention to motor convention.
    float left_wheel_direction = 1.0f;
    float right_wheel_direction = -1.0f;
};

struct command_config
{
    // Remote-axis scales at full stick: m/s and rad/s.
    float velocity_scale = 2.5f;
    float yaw_scale = 1.5f;
    float spin_rate = 3.0f;

    // Command slew rates: (m/s)/s and (rad/s)/s. Values preserve the old
    // 0.005 and 0.01 per-update increments at the nominal 1 kHz cycle.
    float velocity_slope_rate = 5.0f;
    float yaw_slope_rate = 10.0f;

    // Manual leg-length adjustment rate and limits: m/s and m.
    float manual_len_rate = 0.8f;
    float min_len = 0.14f;
    float max_len = 0.34f;
    float normal_len = 0.17f;

    float stationary_velocity = 0.002f;
};

struct odometry_config
{
    float process_noise = 5.0f;
    float wheel_velocity_noise = 0.1f;
    float acceleration_noise = 50.0f;
    float initial_variance = 10.0f;

    float quaternion_norm_epsilon = 1.0e-9f;
    float innovation_epsilon = 1.0e-9f;
};

struct lqr_config
{
    // Range of the verified SJTU_MODEL polynomial gain fit, in m.
    float min_leg_len = 0.15f;
    float max_leg_len = 0.35f;
    float leg_len_resolution = 0.01f;
};

struct fsm_pid_config
{
    // Active wbr_2026 SJTU_MODEL roll PID.
    pid_params roll{0.5f, 0.0f, -0.5f, 3.0f, 0.0f, ::control::pid_mode::position};
};

struct jump_config
{
    // Active wbr_2026 jump force and geometry thresholds: N and m.
    float extend_force = 400.0f;
    float retract_force = -200.0f;
    float extend_reached_len = 0.32f;
    float retract_reached_len = 0.14f;
    float landing_support_force = 20.0f;

    // New safety guard in seconds. This is not a wbr_2026 tuning value and must
    // be validated.
    float action_timeout_s = 1.5f;
};

struct fsm_guards
{
    float offground_support_force = 20.0f;
    float offground_leg_len = 0.27f;
    float leg_len_bias = 0.03f;

    // Preserve the old immediate contact transition by default while expressing
    // time in seconds.
    float contact_confirm_s = 0.0f;

    // No validated old-real-robot thresholds/targets exist for these states yet.
    bool recover_enabled = false;
    bool flatten_enabled = false;
    bool neutral_enabled = false;
    bool gostair_enabled = false;
};

struct solver_numerics_config
{
    // Geometry is in m; sine terms are dimensionless.
    float singularity_epsilon = 1.0e-5f;
    float spring_singularity_epsilon = 5.0e-2f;

    // State labels retained for future FSM entry guards.
    float neutral_alpha_rad = 0.6f;
    float flat_phi_min_rad = 2.0f;
    float flat_phi_max_rad = 3.1f;
};

struct runtime_config
{
    // Control timing and input freshness, in s.
    float nominal_dt_s = 0.001f;
    float min_dt_s = 1.0e-5f;
    float max_dt_s = 5.0e-2f;
    float ahrs_max_age_s = 5.0e-3f;
    float command_max_age_s = 5.0e-3f;
    float alive_check_period_s = 1.0e-2f;
    float attitude_quaternion_norm_min = 0.8f;
    float attitude_quaternion_norm_max = 1.2f;

    // ThreadX application priorities. AHRS/remoter keep their module defaults.
    std::uint32_t control_thread_priority = 5U;
    std::uint32_t input_thread_priority = 7U;

    // Keep false until geometry, shaft units, zero positions, and signs are
    // validated together.
    bool actuation_enabled = false;
};

struct chassis_config
{
    // Current migrated five-bar lengths and operating bounds, in m.
    // They conflict with wbr_2026's motor-distance model and remain a hardware
    // blocker.
    float l1 = 0.220f;
    float l2 = 0.260f;
    float lmin = 0.16f;
    float lmax = 0.36f;

    // Chassis/wheel values used by the active old real-robot control.
    float wheel_radius = 0.077f;
    float wheel_side_mass = 1.41f;
    float gravity = 9.78f;

    // Spring model: N, m, m, rad.
    float fspring = 450.0f;
    float dspring1 = 0.03f;
    float dspring2 = 0.05f;
    float ang_spring = 0.2164f;

    // alpha_eq = c0 + c1 * len + c2 * len^2, result in rad.
    float alpha_eq_coeff[3] = {0.280918f, -1.101757f, 1.232768f};

    // Actuator limits in N*m.
    float max_hip_torque = 40.0f;
    float max_wheel_torque = 15.0f;

    leg_pid_config leg_pid{};
    motor_calibration_config motor_calibration{};
    command_config command{};
    odometry_config odometry{};
    lqr_config lqr{};
    fsm_pid_config fsm_pid{};
    fsm_guards fsm{};
    jump_config jump{};
    solver_numerics_config numerics{};
    runtime_config runtime{};
};

inline constexpr chassis_config k_default_chassis{};

} // namespace wbr::control
