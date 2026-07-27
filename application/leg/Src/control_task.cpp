#include "leg_tasks.hpp"

#include "constrain.hpp"
#include "msg.hpp"
#include "tx_api.h"

#include <cmath>

namespace app
{
namespace
{

float clamp_stick(float value) noexcept
{
    return math::clamp(value, -1.0f, 1.0f);
}

bool dr16_online(const ::remoter::state& remote) noexcept
{
    return !remote.offline &&
           remote.active_source == ::remoter::source::dr16;
}

} // namespace

command_interpreter::command_interpreter() noexcept
    : speed_ramp_(0.0f, leg_config::command::speed_step_mps),
      yaw_rate_ramp_(0.0f, leg_config::command::yaw_rate_step_rad_s),
      leg_length_ramp_(leg_config::normal_leg_length_m,
                       leg_config::normal_leg_step_m)
{
}

void command_interpreter::reset() noexcept
{
    speed_ramp_.reset(0.0f);
    yaw_rate_ramp_.reset(0.0f);
    leg_length_ramp_.reset(leg_config::normal_leg_length_m);
    leg_length_ramp_.set_path(leg_config::normal_leg_step_m);
    manual_leg_length_m_ = leg_config::normal_leg_length_m;
    position_target_m_ = 0.0f;
    jump_state_ = leg_messages::jump_state::idle;
    holding_position_ = false;
}

leg_messages::command command_interpreter::stop_command(
    const leg_messages::odometry& odometry,
    std::uint32_t tick) noexcept
{
    speed_ramp_.reset(0.0f);
    yaw_rate_ramp_.reset(0.0f);
    leg_length_ramp_.reset(leg_config::normal_leg_length_m);
    leg_length_ramp_.set_path(leg_config::normal_leg_step_m);
    manual_leg_length_m_ = leg_config::normal_leg_length_m;
    jump_state_ = leg_messages::jump_state::idle;

    leg_messages::command command{};
    command.leg_length_m = leg_config::normal_leg_length_m;
    command.jump_status = jump_state_;
    command.tick = tick;
    command.enabled = false;
    command.valid = true;
    update_position(command, odometry, true);
    return command;
}

void command_interpreter::apply_drive_input(const ::remoter::state& remote,
                                             leg_messages::command& command) noexcept
{
    float speed_target = clamp_stick(remote.left_y) * leg_config::command::max_speed_mps;
    float yaw_rate_target =
        -clamp_stick(remote.left_x) * leg_config::command::max_yaw_rate_rad_s;

    if (std::fabs(speed_target) < leg_config::command::speed_deadband_mps)
    {
        speed_target = 0.0f;
    }
    if (std::fabs(yaw_rate_target) < leg_config::command::yaw_rate_deadband_rad_s)
    {
        yaw_rate_target = 0.0f;
    }

    command.speed_mps = speed_ramp_.update(speed_target);
    command.yaw_rate_rad_s = yaw_rate_ramp_.update(yaw_rate_target);
}

void command_interpreter::update_position(leg_messages::command& command,
                                          const leg_messages::odometry& odometry,
                                          bool force_hold) noexcept
{
    const bool hold = force_hold ||
                      std::fabs(command.speed_mps) <
                          leg_config::command::speed_deadband_mps;

    if (odometry.valid)
    {
        if (hold)
        {
            if (!holding_position_)
            {
                position_target_m_ = odometry.position_m;
                holding_position_ = true;
            }
        }
        else
        {
            position_target_m_ = odometry.position_m +
                                 command.speed_mps *
                                     leg_config::control_task_thread::period_s;
            holding_position_ = false;
        }
    }

    command.position_m = position_target_m_;
}

void command_interpreter::begin_jump_stage(leg_messages::jump_state stage) noexcept
{
    jump_state_ = stage;
    if (stage == leg_messages::jump_state::extending ||
        stage == leg_messages::jump_state::airborne)
    {
        leg_length_ramp_.set_path(leg_config::jump_leg_step_m);
    }
    else
    {
        leg_length_ramp_.set_path(leg_config::normal_leg_step_m);
    }
}

void command_interpreter::apply_spin_mode(
    leg_messages::command& command) noexcept
{
    speed_ramp_.reset(0.0f);
    yaw_rate_ramp_.reset(0.0f);
    begin_jump_stage(leg_messages::jump_state::idle);
    manual_leg_length_m_ = leg_config::normal_leg_length_m;

    command.spin_mode = true;
    command.yaw_rate_rad_s = leg_config::command::spin_yaw_rate_rad_s;
    command.leg_length_m =
        leg_length_ramp_.update(leg_config::normal_leg_length_m);
}

void command_interpreter::apply_manual_mode(
    const ::remoter::state& remote,
    leg_messages::command& command) noexcept
{
    if (jump_state_ != leg_messages::jump_state::idle)
    {
        leg_length_ramp_.reset(leg_config::normal_leg_length_m);
        manual_leg_length_m_ = leg_config::normal_leg_length_m;
    }

    jump_state_ = leg_messages::jump_state::idle;
    apply_drive_input(remote, command);

    manual_leg_length_m_ +=
        clamp_stick(remote.right_y) * leg_config::command::manual_leg_step_m;
    manual_leg_length_m_ =
        math::clamp(manual_leg_length_m_,
                    leg_config::min_control_leg_length_m,
                    leg_config::max_control_leg_length_m);
    leg_length_ramp_.reset(manual_leg_length_m_);
    command.leg_length_m = manual_leg_length_m_;
}

void command_interpreter::apply_jump_ready_mode(
    const ::remoter::state& remote,
    leg_messages::command& command) noexcept
{
    apply_drive_input(remote, command);
    manual_leg_length_m_ = leg_config::normal_leg_length_m;
    begin_jump_stage(leg_messages::jump_state::starting);
    command.leg_length_m =
        leg_length_ramp_.update(leg_config::normal_leg_length_m);
}

void command_interpreter::apply_jump_mode(
    const leg_messages::solver_feedback& solver,
    leg_messages::command& command) noexcept
{
    speed_ramp_.reset(0.0f);
    yaw_rate_ramp_.reset(0.0f);

    if (solver.valid)
    {
        const float average_leg_length =
            0.5f * (solver.left_leg_length_m + solver.right_leg_length_m);

        if (jump_state_ == leg_messages::jump_state::extending &&
            average_leg_length > leg_config::command::jump_extended_threshold_m)
        {
            begin_jump_stage(leg_messages::jump_state::airborne);
        }
        else if (jump_state_ == leg_messages::jump_state::airborne &&
                 average_leg_length <
                     leg_config::command::jump_retracted_threshold_m)
        {
            begin_jump_stage(leg_messages::jump_state::landing);
        }
        else if (jump_state_ == leg_messages::jump_state::landing &&
                 solver.support_force_n >
                     leg_config::off_ground_force_threshold_n)
        {
            begin_jump_stage(leg_messages::jump_state::idle);
            leg_length_ramp_.reset(leg_config::normal_leg_length_m);
        }
    }

    switch (jump_state_)
    {
    case leg_messages::jump_state::starting:
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::normal_leg_length_m);
        if (leg_length_ramp_.reached())
        {
            begin_jump_stage(leg_messages::jump_state::extending);
        }
        break;
    case leg_messages::jump_state::extending:
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::jump_start_leg_length_m);
        break;
    case leg_messages::jump_state::airborne:
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::jump_air_leg_length_m);
        break;
    case leg_messages::jump_state::landing:
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::normal_leg_length_m);
        break;
    default:
        command.leg_length_m = leg_config::normal_leg_length_m;
        break;
    }
}

leg_messages::command command_interpreter::update(
    const ::remoter::state& remote,
    const leg_messages::solver_feedback& solver,
    const leg_messages::odometry& odometry,
    std::uint32_t tick) noexcept
{
    if (!dr16_online(remote))
    {
        return stop_command(odometry, tick);
    }

    leg_messages::command command{};
    command.tick = tick;
    command.enabled = true;
    command.valid = true;
    bool hold_position = false;

    // 左拨杆：下停机，中正常控制，上小陀螺。
    switch (remote.left_sw)
    {
    case ::remoter::sw_state::low:
        return stop_command(odometry, tick);
    case ::remoter::sw_state::up:
        apply_spin_mode(command);
        hold_position = true;
        break;
    case ::remoter::sw_state::mid:
        // 右拨杆：下手动控制，中跳跃准备，上执行跳跃。
        switch (remote.right_sw)
        {
        case ::remoter::sw_state::low:
            apply_manual_mode(remote, command);
            break;
        case ::remoter::sw_state::mid:
            apply_jump_ready_mode(remote, command);
            break;
        case ::remoter::sw_state::up:
            apply_jump_mode(solver, command);
            hold_position = true;
            break;
        default:
            return stop_command(odometry, tick);
        }
        break;
    default:
        return stop_command(odometry, tick);
    }

    command.jump_status = jump_state_;
    update_position(command, odometry, hold_position);
    return command;
}

namespace control_task
{

[[noreturn]] void run() noexcept
{
    auto* command_topic = msg::create<leg_messages::command>();
    auto remoter_sub = msg::subscribe<::remoter::state>();
    auto solver_sub = msg::subscribe<leg_messages::solver_feedback>();
    auto odometry_sub = msg::subscribe<leg_messages::odometry>();
    command_interpreter interpreter;

    ::remoter::state remote{};
    leg_messages::solver_feedback solver{};
    leg_messages::odometry odometry{};

    for (;;)
    {
        (void)msg::read(remoter_sub, remote);
        (void)msg::read(solver_sub, solver);
        (void)msg::read(odometry_sub, odometry);

        const auto tick = static_cast<std::uint32_t>(tx_time_get());
        const leg_messages::command command =
            interpreter.update(remote, solver, odometry, tick);
        (void)msg::publish(command_topic, command);

        tx_thread_sleep(leg_config::control_task_thread::period_ticks);
    }
}

} // namespace control_task

} // namespace app
