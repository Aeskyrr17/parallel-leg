#pragma once

#include <cstddef>
#include <cstdint>

namespace app::leg_config
{

namespace feature
{
// Temporarily disabled during chassis bring-up. Keep the implementation in
// place so each feature can be restored independently after basic control is
// verified.
inline constexpr bool jump = true;
inline constexpr bool off_ground_detection = true;
} // namespace feature

// Mechanical geometry, in metres.
inline constexpr float vmc_link_1_length_m = 0.150f;
inline constexpr float vmc_link_2_length_m = 0.270f;
inline constexpr float motor_distance_m = 0.150f;
inline constexpr float half_motor_distance_m = motor_distance_m / 2.0f;

// Mechanical zero points, in raw encoder counts.
inline constexpr std::uint16_t left_joint_4_offset = 0x11FFU;
inline constexpr std::uint16_t left_joint_1_offset = 0x25FFU;
inline constexpr std::uint16_t right_joint_4_offset = 0xC6C3U;
inline constexpr std::uint16_t right_joint_1_offset = 0x8419U;
inline constexpr std::uint16_t left_wheel_offset = 0U;
inline constexpr std::uint16_t right_wheel_offset = 0U;

inline constexpr float wheel_radius_m = 0.077f;
inline constexpr float wheel_mass_kg = 1.41f;
inline constexpr float pitch_zero_offset_rad = -0.02f;
inline constexpr float normal_leg_angle_reference_rad = -0.016f;

// Leg-length targets and adjustment steps, in metres.
inline constexpr float min_control_leg_length_m = 0.15f;
inline constexpr float max_control_leg_length_m = 0.35f;
inline constexpr float leg_length_resolution_m = 0.01f;
inline constexpr float normal_leg_length_m = 0.20f;
inline constexpr float normal_leg_step_m = 0.001f;

namespace jump
{
inline constexpr float ready_leg_length_m = 0.180f;
inline constexpr float start_leg_length_m = 0.360f;
inline constexpr float air_leg_length_m = 0.150f;
inline constexpr float leg_step_m = 0.008f;
inline constexpr float extended_threshold_m = 0.370f;
inline constexpr float retracted_threshold_m = 0.150f;
inline constexpr float wheel_fade_start_length_m = 0.25f;
inline constexpr float wheel_off_length_m = 0.30f;
inline constexpr float extending_force_n = 400.0f;
} // namespace jump

namespace command
{
inline constexpr float max_speed_mps = 2.5f;
inline constexpr float max_yaw_rate_rad_s = 1.75f;
inline constexpr float spin_yaw_rate_rad_s = 5.0f;
inline constexpr float speed_step_mps = 0.004f;
inline constexpr float yaw_rate_step_rad_s = 0.01f;
inline constexpr float manual_leg_step_m = 0.0005f;
inline constexpr float speed_deadband_mps = 0.005f;
inline constexpr float yaw_rate_deadband_rad_s = 0.0001f;
inline constexpr float position_lock_velocity_error_mps = 0.03f;
inline constexpr float hold_release_stick_deadband = 0.05f;
} // namespace command

// Output torque limits, in N*m.
inline constexpr float max_joint_torque_nm = 40.0f;
inline constexpr float max_wheel_torque_nm = 15.0f;

// PID defaults. FreeMASTER can tune the live controller values; change the
// power-on defaults only here.
namespace pid
{
namespace leg_length
{
inline constexpr float kp = 2000.0f;
inline constexpr float ki = 0.0f;
inline constexpr float kd = -200.0f;
inline constexpr float max_out = 200.0f;
inline constexpr float max_iout = 0.0f;
} // namespace leg_length

namespace roll
{
inline constexpr float kp = 0.5f;
inline constexpr float ki = 0.0f;
inline constexpr float kd = 0.5f;
inline constexpr float max_out = 3.0f;
inline constexpr float max_iout = 0.0f;
} // namespace roll
} // namespace pid

// Signs used to map motor feedback/output into the robot coordinate system.
inline constexpr std::int8_t left_joint_4_direction = -1;
inline constexpr std::int8_t left_joint_1_direction = -1;
inline constexpr std::int8_t right_joint_4_direction = 1;
inline constexpr std::int8_t right_joint_1_direction = 1;
inline constexpr std::int8_t left_wheel_direction = 1;
inline constexpr std::int8_t right_wheel_direction = -1;

// The robot is considered off the ground below this total support force.
inline constexpr float off_ground_force_threshold_n = 52.5f;

namespace solver_thread
{
inline constexpr std::uint32_t period_ticks = 1U;
inline constexpr std::uint32_t priority = 5U;
inline constexpr std::size_t stack_size = 4096U;
} // namespace solver_thread

namespace control_thread
{
inline constexpr std::uint32_t period_ticks = 1U;
inline constexpr float period_s = 0.001f;
inline constexpr std::uint32_t priority = 6U;
inline constexpr std::size_t stack_size = 4096U;
} // namespace control_thread

namespace control_task_thread
{
inline constexpr std::uint32_t period_ticks = 1U;
inline constexpr float period_s = 0.001f;
inline constexpr std::uint32_t priority = 7U;
inline constexpr std::size_t stack_size = 2048U;
} // namespace control_task_thread

} // namespace app::leg_config
