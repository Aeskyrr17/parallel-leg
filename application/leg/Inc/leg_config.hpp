#pragma once

#include <cstddef>
#include <cstdint>

namespace app::leg_config
{

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

// Leg-length targets and adjustment steps, in metres.
inline constexpr float min_control_leg_length_m = 0.15f;
inline constexpr float max_control_leg_length_m = 0.35f;
inline constexpr float leg_length_resolution_m = 0.01f;
inline constexpr float normal_leg_length_m = 0.17f;
inline constexpr float jump_start_leg_length_m = 0.370f;
inline constexpr float jump_air_leg_length_m = 0.140f;
inline constexpr float normal_leg_step_m = 0.001f;
inline constexpr float jump_leg_step_m = 0.007f;

namespace command
{
inline constexpr float max_speed_mps = 2.5f;
inline constexpr float max_yaw_rate_rad_s = 1.5f;
inline constexpr float spin_yaw_rate_rad_s = 3.0f;
inline constexpr float speed_step_mps = 0.005f;
inline constexpr float yaw_rate_step_rad_s = 0.01f;
inline constexpr float manual_leg_step_m = 0.0008f;
inline constexpr float speed_deadband_mps = 0.002f;
inline constexpr float yaw_rate_deadband_rad_s = 0.0001f;
inline constexpr float jump_extended_threshold_m = 0.32f;
inline constexpr float jump_retracted_threshold_m = 0.14f;
} // namespace command

// Output torque limits, in N*m.
inline constexpr float max_joint_torque_nm = 40.0f;
inline constexpr float max_wheel_torque_nm = 15.0f;

// Signs used to map motor feedback/output into the robot coordinate system.
inline constexpr std::int8_t left_joint_4_direction = -1;
inline constexpr std::int8_t left_joint_1_direction = -1;
inline constexpr std::int8_t right_joint_4_direction = 1;
inline constexpr std::int8_t right_joint_1_direction = 1;
inline constexpr std::int8_t left_wheel_direction = 1;
inline constexpr std::int8_t right_wheel_direction = -1;

// The robot is considered off the ground below this total support force.
inline constexpr float off_ground_force_threshold_n = 20.0f;

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
