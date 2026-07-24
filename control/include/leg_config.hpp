#pragma once

#include "pid.hpp"

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
    // MuJoCo starting values; validate before enabling hardware.
    // len output is N; derivative input is m/s.
    pid_params len{6000.0f, 0.0f, -1500.0f, 100.0f, 0.0f, ::control::pid_mode::delta};

    // Output is N*m; derivative input is rad/s.
    pid_params phi{0.7f, 0.0f, 1.4f, 20.0f, 0.005f, ::control::pid_mode::delta};
};

struct leg_calibration
{
    // Motor feedback zero positions, in rad.
    float joint1_zero_rad = 0.0f;
    float joint4_zero_rad = 0.0f;

    // Motor-to-mechanical coordinate direction. Calibrated values must be +1 or -1.
    float joint1_direction = 1.0f;
    float joint4_direction = 1.0f;
};

struct chassis_config
{
    // MuJoCo five-bar link lengths and operating bounds, in m.
    // These are not a substitute for real mechanism measurements.
    float l1 = 0.220f;
    float l2 = 0.260f;
    float lmin = 0.16f;
    float lmax = 0.36f;

    // Wheel-side equivalent mass used by the support-force model, in kg.
    float mwheel = 0.21f;

    // Spring model: N, m, m, rad.
    float fspring = 450.0f;
    float dspring1 = 0.03f;
    float dspring2 = 0.05f;
    float ang_spring = 0.2164f;

    // alpha_eq = c0 + c1 * len + c2 * len^2, result in rad.
    float alpha_eq_coeff[3] = {0.280918f, -1.101757f, 1.232768f};

    // Absolute joint torque limit, in N*m.
    float thip_max = 40.0f;

    leg_pid_config leg_pid{};
};

inline constexpr chassis_config k_default_chassis{};

} // namespace wbr::control
