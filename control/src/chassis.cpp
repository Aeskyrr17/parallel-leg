#include "chassis.hpp"

#include <algorithm>
#include <cmath>

namespace wbr::control
{
namespace
{

void clear_vector(float values[10])
{
    std::fill(values, values + 10, 0.0f);
}

} // namespace

ChassisController::ChassisController(const chassis_config& cfg)
    : cfg_(cfg), lqr_(cfg.lqr),
      roll_pd_(cfg.fsm_pid.roll.kp, cfg.fsm_pid.roll.ki, cfg.fsm_pid.roll.kd,
               cfg.fsm_pid.roll.max_out, cfg.fsm_pid.roll.max_i_out, cfg.fsm_pid.roll.mode)
{
    reset();
}

fsm_output ChassisController::step(const fsm_input& input)
{
    const bool inputs_valid = valid_input(input);
    if (inputs_valid)
    {
        state_elapsed_s_ += input.dt;
        if (state_ == chassis_state::JUMP && jump_stage_ != jump_stage::DONT)
        {
            jump_elapsed_s_ += input.dt;
        }

        if (input.command.mode == command_mode::relax)
        {
            enter_state(chassis_state::RELAX, input);
        }
        else if (state_ == chassis_state::RELAX)
        {
            enter_state(requested_motion_state(input.command), input);
        }
        else if (state_ == chassis_state::NORMAL || state_ == chassis_state::SPIN)
        {
            if (support_force(input) < cfg_.fsm.offground_support_force)
            {
                enter_state(chassis_state::OFFGROUND, input);
            }
            else if (state_ == chassis_state::NORMAL &&
                     input.command.event == command_event::prepare_jump)
            {
                enter_state(chassis_state::JUMP, input);
            }
            else
            {
                enter_state(requested_motion_state(input.command), input);
            }
        }
    }
    else
    {
        enter_state(chassis_state::RELAX, input);
    }

    fsm_output output{};
    switch (state_)
    {
    case chassis_state::RELAX:
        input.lpendulum.reset();
        input.rpendulum.reset();
        roll_pd_.clear();
        output.reset_odometry = true;
        output.relax = true;
        output.valid = true;
        break;

    case chassis_state::RECOVER:
        // Target phi, kick torque, IMU guards, and exit thresholds are not
        // validated in the active old real-robot branch. Safe fallback below.
        break;

    case chassis_state::FLATTEN:
        // Rotation direction, target phi, and wheel assistance remain unconfigured.
        break;

    case chassis_state::NEUTRAL:
        // Length/phi targets and alpha transition guards remain unconfigured.
        break;

    case chassis_state::NORMAL:
        output = run_lqr(input, false, true);
        break;

    case chassis_state::OFFGROUND:
        if (support_force(input) >= cfg_.fsm.offground_support_force &&
            state_elapsed_s_ >= cfg_.fsm.contact_confirm_s)
        {
            enter_state(requested_motion_state(input.command), input);
            output = run_lqr(input, false, true);
        }
        else
        {
            output = run_lqr(input, true, false);
            output.left_wheel_torque = 0.0f;
            output.right_wheel_torque = 0.0f;
            output.reset_odometry = true;
        }
        break;

    case chassis_state::SPIN:
        // The old active build uses the same gain table with a spin yaw reference.
        output = run_lqr(input, false, true);
        break;

    case chassis_state::GOSTAIR:
        // STAIRUP was disabled and the current robot has no TOF input chain.
        break;

    case chassis_state::JUMP:
    {
        if (jump_stage_ != jump_stage::DONT && jump_elapsed_s_ > cfg_.jump.action_timeout_s)
        {
            enter_state(chassis_state::RELAX, input);
            break;
        }

        const float mean_len = 0.5f * (input.lpendulum.link().len + input.rpendulum.link().len);
        const bool sensed_offground = support_force(input) < cfg_.fsm.offground_support_force;
        switch (jump_stage_)
        {
        case jump_stage::DONT:
            if (sensed_offground)
            {
                output = run_lqr(input, true, false);
                output.left_wheel_torque = 0.0f;
                output.right_wheel_torque = 0.0f;
                output.reset_odometry = true;
                break;
            }
            if (input.command.action == command_action::none)
            {
                enter_state(requested_motion_state(input.command), input);
                output = run_lqr(input, false, true);
                break;
            }
            if (input.command.event == command_event::start_jump ||
                input.command.action == command_action::execute_jump)
            {
                enter_jump_stage(jump_stage::EXTENDING);
            }
            else
            {
                output = run_lqr(input, false, true);
                break;
            }
            [[fallthrough]];

        case jump_stage::EXTENDING:
            output = run_lqr(input, sensed_offground, !sensed_offground);
            if (sensed_offground)
            {
                output.left_wheel_torque = 0.0f;
                output.right_wheel_torque = 0.0f;
                output.reset_odometry = true;
            }
            else
            {
                output.left_leg_force.f = cfg_.jump.extend_force;
                output.right_leg_force.f = cfg_.jump.extend_force;
            }
            if (mean_len > cfg_.jump.extend_reached_len)
            {
                enter_jump_stage(jump_stage::INAIR);
            }
            break;

        case jump_stage::INAIR:
            output = run_lqr(input, true, false);
            if (!sensed_offground)
            {
                output.left_leg_force.f = cfg_.jump.retract_force;
                output.right_leg_force.f = cfg_.jump.retract_force;
            }
            output.left_wheel_torque = 0.0f;
            output.right_wheel_torque = 0.0f;
            output.reset_odometry = true;
            if (mean_len < cfg_.jump.retract_reached_len)
            {
                enter_jump_stage(jump_stage::LANDING);
            }
            break;

        case jump_stage::LANDING:
            output = run_lqr(input, sensed_offground, !sensed_offground);
            if (sensed_offground)
            {
                output.left_wheel_torque = 0.0f;
                output.right_wheel_torque = 0.0f;
                output.reset_odometry = true;
            }
            if (support_force(input) > cfg_.jump.landing_support_force)
            {
                enter_state(chassis_state::NORMAL, input);
            }
            break;
        }
        break;
    }
    }

    if (!output.valid || !finite_output(output))
    {
        enter_state(chassis_state::RELAX, input);
        input.lpendulum.reset();
        input.rpendulum.reset();
        roll_pd_.clear();
        output = {};
        output.reset_odometry = true;
        output.relax = true;
        output.valid = true;
    }

    output.state = state_;
    output.jump = jump_stage_;
    output.reset_odometry = output.reset_odometry || reset_odometry_pending_;
    output.reset_command = output.reset_command || reset_command_pending_;
    reset_odometry_pending_ = false;
    reset_command_pending_ = false;
    return output;
}

void ChassisController::reset()
{
    state_ = chassis_state::RELAX;
    jump_stage_ = jump_stage::DONT;
    state_elapsed_s_ = 0.0f;
    jump_elapsed_s_ = 0.0f;
    reset_odometry_pending_ = true;
    reset_command_pending_ = true;
    roll_pd_.clear();
    clear_vector(observed_);
    clear_vector(reference_);
}

fsm_output ChassisController::run_lqr(const fsm_input& input, bool offground, bool apply_roll)
{
    fsm_output output{};
    fill_observed(input);

    float left_len_reference = input.command.len + cfg_.fsm.leg_len_bias;
    float right_len_reference = left_len_reference;
    if (offground)
    {
        left_len_reference = cfg_.fsm.offground_leg_len;
        right_len_reference = cfg_.fsm.offground_leg_len;
        fill_offground_reference(input);
    }
    else
    {
        if (apply_roll)
        {
            update_roll(input);
            left_len_reference += roll_pd_.result;
            right_len_reference -= roll_pd_.result;
        }
        fill_normal_reference(input);
    }

    output.left_leg_force.f = input.lpendulum.len_control(left_len_reference);
    output.right_leg_force.f = input.rpendulum.len_control(right_len_reference);

    const lqr_output lqr = lqr_.solve(input.lpendulum.link().len, input.rpendulum.link().len,
                                      offground, observed_, reference_);
    if (!lqr.valid)
    {
        return {};
    }

    output.left_leg_force.tp = lqr.left_hip;
    output.right_leg_force.tp = lqr.right_hip;
    if (!offground)
    {
        output.left_wheel_torque = lqr.left_wheel;
        output.right_wheel_torque = lqr.right_wheel;
    }
    output.relax = false;
    output.valid = finite_output(output);
    return output;
}

void ChassisController::fill_observed(const fsm_input& input)
{
    observed_[0] = input.odometry.x;
    observed_[1] = input.odometry.v;
    observed_[2] = input.attitude.total_yaw;
    observed_[3] = input.attitude.gyro_y;
    observed_[4] = input.lpendulum.link().alpha;
    observed_[5] = input.lpendulum.link().dalpha;
    observed_[6] = input.rpendulum.link().alpha;
    observed_[7] = input.rpendulum.link().dalpha;
    observed_[8] = input.attitude.pitch;
    observed_[9] = input.attitude.gyro_p;
}

void ChassisController::fill_normal_reference(const fsm_input& input)
{
    reference_[0] = input.command.x;
    reference_[1] = input.command.v;
    reference_[2] = input.attitude.total_yaw + input.command.dyaw * input.dt;
    reference_[3] = input.command.w + input.command.dyaw;
    for (int index = 4; index < 10; ++index)
    {
        reference_[index] = 0.0f;
    }
}

void ChassisController::fill_offground_reference(const fsm_input& input)
{
    reference_[0] = observed_[0];
    reference_[1] = observed_[1];
    reference_[2] = observed_[2];
    reference_[3] = observed_[3];
    reference_[4] = input.attitude.pitch;
    reference_[5] = 0.0f;
    reference_[6] = input.attitude.pitch;
    reference_[7] = 0.0f;
    reference_[8] = input.attitude.pitch;
    reference_[9] = input.attitude.gyro_p;
}

void ChassisController::update_roll(const fsm_input& input)
{
    // Position-mode PID preserves the active old implementation; its velocity
    // argument is ignored.
    roll_pd_.ref = input.command.roll;
    roll_pd_.fdb = input.attitude.roll;
    roll_pd_.update(input.attitude.gyro_r);
}

bool ChassisController::valid_input(const fsm_input& input) const
{
    return input.valid && input.command.valid && input.odometry.valid && input.health.valid &&
           input.health.motors_online && input.health.attitude_fresh &&
           input.health.command_fresh && input.lpendulum.link().valid &&
           input.rpendulum.link().valid && std::isfinite(input.dt) &&
           input.dt >= cfg_.runtime.min_dt_s && input.dt <= cfg_.runtime.max_dt_s &&
           std::isfinite(input.odometry.x) && std::isfinite(input.odometry.v) &&
           std::isfinite(input.attitude.pitch) && std::isfinite(input.attitude.roll) &&
           std::isfinite(input.attitude.total_yaw) && std::isfinite(input.attitude.gyro_r) &&
           std::isfinite(input.attitude.gyro_p) && std::isfinite(input.attitude.gyro_y);
}

bool ChassisController::finite_output(const fsm_output& output)
{
    return std::isfinite(output.left_leg_force.f) && std::isfinite(output.left_leg_force.tp) &&
           std::isfinite(output.right_leg_force.f) && std::isfinite(output.right_leg_force.tp) &&
           std::isfinite(output.left_wheel_torque) && std::isfinite(output.right_wheel_torque);
}

float ChassisController::support_force(const fsm_input& input) const
{
    return input.lpendulum.link().n + input.rpendulum.link().n;
}

chassis_state ChassisController::requested_motion_state(const chassis_command& command) const
{
    return command.mode == command_mode::spin ? chassis_state::SPIN : chassis_state::NORMAL;
}

void ChassisController::enter_state(chassis_state next, const fsm_input& input)
{
    if (next == state_)
    {
        return;
    }

    state_ = next;
    state_elapsed_s_ = 0.0f;
    roll_pd_.clear();
    input.lpendulum.reset_control();
    input.rpendulum.reset_control();
    clear_vector(observed_);
    clear_vector(reference_);

    if (next != chassis_state::JUMP)
    {
        jump_stage_ = jump_stage::DONT;
        jump_elapsed_s_ = 0.0f;
    }
    if (next == chassis_state::RELAX)
    {
        reset_odometry_pending_ = true;
        reset_command_pending_ = true;
    }
}

void ChassisController::enter_jump_stage(jump_stage next)
{
    jump_stage_ = next;
    state_elapsed_s_ = 0.0f;
    if (next == jump_stage::EXTENDING)
    {
        jump_elapsed_s_ = 0.0f;
    }
}

} // namespace wbr::control
