#pragma once

#include "ahrs.hpp"
#include "leg_messages.hpp"
#include "pid.hpp"
#include "types.hpp"

#include <cstdint>

#if defined(__GNUC__)
#define LEG_DEBUG_INLINE inline __attribute__((used))
#else
#define LEG_DEBUG_INLINE inline
#endif

// C-linkage keeps the names simple in FreeMASTER and Cortex-Debug:
// leg_debug_remoter, leg_debug_imu, leg_debug_command, ...
extern "C"
{
LEG_DEBUG_INLINE ::remoter::state leg_debug_remoter{};
LEG_DEBUG_INLINE ::ahrs::message leg_debug_imu{};
LEG_DEBUG_INLINE app::leg_messages::command leg_debug_command{};
LEG_DEBUG_INLINE app::leg_messages::solver_feedback leg_debug_solver_feedback{};
LEG_DEBUG_INLINE app::leg_messages::control_target leg_debug_control_target{};
LEG_DEBUG_INLINE app::leg_messages::odometry leg_debug_odometry{};

// These are the real controllers used by pendulum_task. FreeMASTER writes
// their members directly, e.g. left_leg_length_pid.kp.
LEG_DEBUG_INLINE control::pid left_leg_length_pid(
    4000.0f, 0.0f, -120.0f, 200.0f, 0.0f,
    control::pid_mode::delta);
LEG_DEBUG_INLINE control::pid right_leg_length_pid(
    4000.0f, 0.0f, -120.0f, 200.0f, 0.0f,
    control::pid_mode::delta);
LEG_DEBUG_INLINE control::pid roll_pid(0.5f, 0.0f, -0.5f, 3.0f, 0.0f);

// Final torque targets and motor feedback, grouped for external monitoring.
LEG_DEBUG_INLINE app::leg_messages::motor_torque leg_debug_motor_torque{};

LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_control_heartbeat = 0U;
LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_pendulum_heartbeat = 0U;
LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_solver_heartbeat = 0U;
}

#undef LEG_DEBUG_INLINE
