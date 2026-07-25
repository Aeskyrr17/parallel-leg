#pragma once

#include "chassis.hpp"

#include <cstdint>

namespace wbr::control
{

struct control_task_telemetry
{
    bool initialization_attempted = false;
    bool dwt_initialized = false;
    bool ahrs_initialized = false;
    bool remoter_initialized = false;
    bool subscribers_ready = false;
    bool snapshot_ready = false;
    bool motors_registered = false;
    bool input_thread_started = false;
    bool control_thread_started = false;
    bool thread_started = false;

    bool ahrs_fresh = false;
    bool remoter_fresh = false;
    bool motors_online = false;
    bool motor_snapshot_valid = false;
    bool odometry_valid = false;
    bool left_leg_valid = false;
    bool right_leg_valid = false;
    bool chassis_valid = false;
    bool offground = false;

    // All control requests are calculated, but nonzero output stays disabled
    // until real-mechanism geometry/calibration is validated.
    bool actuation_enabled = false;
    bool outputs_relaxed = true;

    std::uint32_t cycle_count = 0;
    std::uint32_t input_cycle_count = 0;
    std::uint32_t invalid_cycle_count = 0;
    std::uint32_t alive_check_count = 0;
    std::uint32_t snapshot_contention_count = 0;

    float dt = 0.0f;
    float wheel_velocity = 0.0f;
    float support_force = 0.0f;
    float state_elapsed_s = 0.0f;

    chassis_state state = chassis_state::RELAX;
    jump_stage jump = jump_stage::DONT;
    odometry_state odometry{};
    chassis_command command{};
    chassis_output request{};
    motor_feedback_frame motor_feedback{};
    link_state left_link{};
    link_state right_link{};
};

bool start_control_task() noexcept;

} // namespace wbr::control

extern "C" wbr::control::control_task_telemetry wbr_control_task_telemetry;
