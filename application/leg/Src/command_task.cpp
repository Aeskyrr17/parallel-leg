#include "leg_tasks.hpp"

#include <cmath>

namespace app
{
namespace
{

float clamp(float value, float minimum, float maximum) noexcept
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

float clamp_stick(float value) noexcept
{
    return clamp(value, -1.0f, 1.0f);
}

bool remote_input_valid(const ::remoter::state& remote) noexcept
{
    return !remote.offline &&
           remote.active_source != ::remoter::source::none &&
           remote.update_count != 0U &&
           std::isfinite(remote.left_x) &&
           std::isfinite(remote.left_y) &&
           std::isfinite(remote.right_x) &&
           std::isfinite(remote.right_y);
}

bool solver_feedback_valid(const leg_messages::solver_feedback& solver) noexcept
{
    return solver.valid &&
           std::isfinite(solver.left_leg_length_m) &&
           std::isfinite(solver.right_leg_length_m) &&
           std::isfinite(solver.support_force_n);
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

void command_interpreter::apply_manual_motion(const ::remoter::state& remote,
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
                                     leg_config::command_thread::period_s;
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

leg_messages::command command_interpreter::update(
    const ::remoter::state& remote,
    const leg_messages::solver_feedback& solver,
    const leg_messages::odometry& odometry,
    std::uint32_t tick) noexcept
{
    if (!remote_input_valid(remote) || remote.left_sw == ::remoter::sw_state::low)
    {
        return stop_command(odometry, tick);
    }

    leg_messages::command command{};
    command.tick = tick;
    command.enabled = true;
    command.valid = true;

    if (remote.left_sw == ::remoter::sw_state::up)
    {
        speed_ramp_.reset(0.0f);
        yaw_rate_ramp_.reset(0.0f);
        begin_jump_stage(leg_messages::jump_state::idle);
        manual_leg_length_m_ = leg_config::normal_leg_length_m;
        command.spin_mode = true;
        command.yaw_rate_rad_s = leg_config::command::spin_yaw_rate_rad_s;
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::normal_leg_length_m);
        command.jump_status = jump_state_;
        update_position(command, odometry, true);
        return command;
    }

    if (remote.left_sw != ::remoter::sw_state::mid)
    {
        return stop_command(odometry, tick);
    }

    if (remote.right_sw == ::remoter::sw_state::low)
    {
        if (jump_state_ != leg_messages::jump_state::idle)
        {
            leg_length_ramp_.reset(leg_config::normal_leg_length_m);
            manual_leg_length_m_ = leg_config::normal_leg_length_m;
        }
        jump_state_ = leg_messages::jump_state::idle;
        apply_manual_motion(remote, command);
        manual_leg_length_m_ += clamp_stick(remote.right_y) *
                                leg_config::command::manual_leg_step_m;
        manual_leg_length_m_ = clamp(manual_leg_length_m_,
                                     leg_config::min_control_leg_length_m,
                                     leg_config::max_control_leg_length_m);
        leg_length_ramp_.reset(manual_leg_length_m_);
        command.leg_length_m = manual_leg_length_m_;
    }
    else if (remote.right_sw == ::remoter::sw_state::mid)
    {
        apply_manual_motion(remote, command);
        manual_leg_length_m_ = leg_config::normal_leg_length_m;
        begin_jump_stage(leg_messages::jump_state::starting);
        command.leg_length_m =
            leg_length_ramp_.update(leg_config::normal_leg_length_m);
    }
    else if (remote.right_sw == ::remoter::sw_state::up)
    {
        speed_ramp_.reset(0.0f);
        yaw_rate_ramp_.reset(0.0f);

        if (solver_feedback_valid(solver))
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
    else
    {
        return stop_command(odometry, tick);
    }

    command.jump_status = jump_state_;
    update_position(command, odometry, remote.right_sw == ::remoter::sw_state::up);
    return command;
}

command_task& command_task::instance() noexcept
{
    static command_task task;
    return task;
}

bool command_task::prepare() noexcept
{
    if (prepared_)
    {
        return true;
    }

    command_topic_ = msg::create<leg_messages::command>();
    remoter_sub_ = msg::subscribe<::remoter::state>();
    solver_sub_ = msg::subscribe<leg_messages::solver_feedback>();
    odometry_sub_ = msg::subscribe<leg_messages::odometry>();
    if (command_topic_ == nullptr || !remoter_sub_.valid() ||
        !solver_sub_.valid() || !odometry_sub_.valid())
    {
        return false;
    }

    interpreter_.reset();
    if (tx_thread_create(&thread_, const_cast<CHAR*>("leg_command"), thread_entry, 0U,
                         stack_, sizeof(stack_), leg_config::command_thread::priority,
                         leg_config::command_thread::priority, TX_NO_TIME_SLICE,
                         TX_DONT_START) != TX_SUCCESS)
    {
        return false;
    }

    prepared_ = true;
    return true;
}

bool command_task::start() noexcept
{
    if (started_)
    {
        return true;
    }
    if (!prepared_ || tx_thread_resume(&thread_) != TX_SUCCESS)
    {
        return false;
    }

    started_ = true;
    return true;
}

void command_task::thread_entry(ULONG /*input*/)
{
    instance().run();
}

void command_task::run() noexcept
{
    ::remoter::state remote{};
    leg_messages::solver_feedback solver{};
    leg_messages::odometry odometry{};

    for (;;)
    {
        (void)msg::read(remoter_sub_, remote);
        (void)msg::read(solver_sub_, solver);
        (void)msg::read(odometry_sub_, odometry);

        const auto tick = static_cast<std::uint32_t>(tx_time_get());
        const leg_messages::command command =
            interpreter_.update(remote, solver, odometry, tick);
        (void)msg::publish(command_topic_, command);

        tx_thread_sleep(leg_config::command_thread::period_ticks);
    }
}

} // namespace app
