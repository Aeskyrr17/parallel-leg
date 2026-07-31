#pragma once

#include "ahrs.hpp"
#include "leg_messages.hpp"
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

LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_control_heartbeat = 0U;
LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_pendulum_heartbeat = 0U;
LEG_DEBUG_INLINE volatile std::uint32_t leg_debug_solver_heartbeat = 0U;
}

#undef LEG_DEBUG_INLINE
