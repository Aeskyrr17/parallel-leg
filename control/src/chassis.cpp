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
    : cfg_(cfg), lqr_(cfg.lqr), roll_pd_(cfg.roll_pid)
{
    reset();
}

chassis_output ChassisController::step(const chassis_command& command,
                                       const odometry_state& odometry,
                                       const ahrs::message& attitude, Pendulum& lpendulum,
                                       Pendulum& rpendulum, float dt, bool control_valid)
{
    const bool inputs_valid =
        valid_input(command, odometry, attitude, lpendulum, rpendulum, dt, control_valid);
    if (inputs_valid)
    {
        state_elapsed_s_ += dt;
        if (state_ == chassis_state::JUMP && jump_stage_ != jump_stage::DONT)
        {
            jump_elapsed_s_ += dt;
        }
    }

    if (!inputs_valid || command.mode == command_mode::relax)
    {
        enter_state(chassis_state::RELAX, lpendulum, rpendulum);
    }

    chassis_output output{};
    switch (state_)
    {
    case chassis_state::RELAX:
        if (!inputs_valid || command.mode == command_mode::relax)
        {
            // wbr_2026: !cmd.move -> six zero-current commands and odometry reset.
            lpendulum.reset();
            rpendulum.reset();
            roll_pd_.clear();
            output.reset_odometry = true;
            output.relax = true;
            output.valid = true;
            break;
        }

        enter_state(requested_motion_state(command), lpendulum, rpendulum);
        output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
        break;

    case chassis_state::NORMAL:
        if (support_force(lpendulum, rpendulum) < cfg_.state.offground_support_force)
        {
            enter_state(chassis_state::OFFGROUND, lpendulum, rpendulum);
            output = offground_control(odometry, attitude, lpendulum, rpendulum);
        }
        else if (command.jump == jump_command::prepare)
        {
            enter_state(chassis_state::JUMP, lpendulum, rpendulum);
            output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
        }
        else
        {
            enter_state(requested_motion_state(command), lpendulum, rpendulum);
            output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
        }
        break;

    case chassis_state::SPIN:
        if (support_force(lpendulum, rpendulum) < cfg_.state.offground_support_force)
        {
            enter_state(chassis_state::OFFGROUND, lpendulum, rpendulum);
            output = offground_control(odometry, attitude, lpendulum, rpendulum);
        }
        else
        {
            // wbr_2026 uses the normal LQR table with command.w = 3 rad/s.
            enter_state(requested_motion_state(command), lpendulum, rpendulum);
            output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
        }
        break;

    case chassis_state::OFFGROUND:
        if (support_force(lpendulum, rpendulum) >= cfg_.state.offground_support_force &&
            state_elapsed_s_ >= cfg_.state.contact_confirm_s)
        {
            enter_state(requested_motion_state(command), lpendulum, rpendulum);
            output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
        }
        else
        {
            output = offground_control(odometry, attitude, lpendulum, rpendulum);
        }
        break;

    case chassis_state::JUMP:
    {
        if (jump_stage_ != jump_stage::DONT &&
            jump_elapsed_s_ > cfg_.jump.action_timeout_s)
        {
            enter_state(chassis_state::RELAX, lpendulum, rpendulum);
            break;
        }

        const float mean_len = 0.5f * (lpendulum.link().len + rpendulum.link().len);
        const float sensed_support_force = support_force(lpendulum, rpendulum);
        const bool sensed_offground =
            sensed_support_force < cfg_.state.offground_support_force;

        switch (jump_stage_)
        {
        case jump_stage::DONT:
            if (sensed_offground)
            {
                output = offground_control(odometry, attitude, lpendulum, rpendulum);
                break;
            }
            if (command.jump == jump_command::none)
            {
                enter_state(requested_motion_state(command), lpendulum, rpendulum);
                output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
                break;
            }
            if (command.jump != jump_command::execute)
            {
                // Prepared: hold the normal leg length until the right switch moves up.
                output = normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
                break;
            }
            enter_jump_stage(jump_stage::EXTENDING);
            [[fallthrough]];

        case jump_stage::EXTENDING:
            output = sensed_offground
                         ? offground_control(odometry, attitude, lpendulum, rpendulum)
                         : normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
            if (!sensed_offground)
            {
                // wbr_2026 cmd.extending: Fl = Fr = 400 N.
                output.left_target.F = cfg_.jump.extend_force;
                output.right_target.F = cfg_.jump.extend_force;
            }
            if (mean_len > cfg_.jump.extend_reached_len)
            {
                enter_jump_stage(jump_stage::INAIR);
            }
            break;

        case jump_stage::INAIR:
            output = offground_control(odometry, attitude, lpendulum, rpendulum);
            if (!sensed_offground)
            {
                // wbr_2026 only applies -200 N in its contacted branch.
                output.left_target.F = cfg_.jump.retract_force;
                output.right_target.F = cfg_.jump.retract_force;
            }
            if (mean_len < cfg_.jump.retract_reached_len)
            {
                enter_jump_stage(jump_stage::LANDING);
            }
            break;

        case jump_stage::LANDING:
            output = sensed_offground
                         ? offground_control(odometry, attitude, lpendulum, rpendulum)
                         : normal_control(command, odometry, attitude, lpendulum, rpendulum, dt);
            if (sensed_support_force > cfg_.jump.landing_support_force)
            {
                enter_state(chassis_state::NORMAL, lpendulum, rpendulum);
            }
            break;
        }
        break;
    }
    }

    if (!output.valid || !finite_output(output))
    {
        enter_state(chassis_state::RELAX, lpendulum, rpendulum);
        lpendulum.reset();
        rpendulum.reset();
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

chassis_output ChassisController::normal_control(const chassis_command& command,
                                                 const odometry_state& odometry,
                                                 const ahrs::message& attitude,
                                                 Pendulum& lpendulum, Pendulum& rpendulum,
                                                 float dt)
{
    chassis_output output{};
    fill_observed(odometry, attitude, lpendulum, rpendulum);

    roll_pd_.ref = command.roll;
    roll_pd_.fdb = attitude.roll;
    roll_pd_.update(attitude.gyro_r);

    const float base_leg_len = command.len + cfg_.state.leg_len_bias;
    output.left_target.F = lpendulum.len_control(base_leg_len + roll_pd_.result);
    output.right_target.F = rpendulum.len_control(base_leg_len - roll_pd_.result);

    clear_vector(reference_);
    reference_[0] = command.x;
    reference_[1] = command.v;
    reference_[2] = attitude.total_yaw + command.dyaw * dt;
    reference_[3] = command.w + command.dyaw;

    const lqr_output lqr =
        lqr_.solve(lpendulum.link().len, rpendulum.link().len, false, observed_, reference_);
    if (!lqr.valid)
    {
        return {};
    }

    output.left_target.Tp = lqr.left_hip;
    output.right_target.Tp = lqr.right_hip;
    output.left_wheel_torque = lqr.left_wheel;
    output.right_wheel_torque = lqr.right_wheel;
    output.relax = false;
    output.valid = finite_output(output);
    return output;
}

chassis_output ChassisController::offground_control(const odometry_state& odometry,
                                                    const ahrs::message& attitude,
                                                    Pendulum& lpendulum, Pendulum& rpendulum)
{
    chassis_output output{};
    fill_observed(odometry, attitude, lpendulum, rpendulum);

    output.left_target.F = lpendulum.len_control(cfg_.state.offground_leg_len);
    output.right_target.F = rpendulum.len_control(cfg_.state.offground_leg_len);

    reference_[0] = observed_[0];
    reference_[1] = observed_[1];
    reference_[2] = observed_[2];
    reference_[3] = observed_[3];
    reference_[4] = attitude.pitch;
    reference_[5] = 0.0f;
    reference_[6] = attitude.pitch;
    reference_[7] = 0.0f;
    reference_[8] = attitude.pitch;
    reference_[9] = attitude.gyro_p;

    const lqr_output lqr =
        lqr_.solve(lpendulum.link().len, rpendulum.link().len, true, observed_, reference_);
    if (!lqr.valid)
    {
        return {};
    }

    output.left_target.Tp = lqr.left_hip;
    output.right_target.Tp = lqr.right_hip;
    output.left_wheel_torque = 0.0f;
    output.right_wheel_torque = 0.0f;
    output.reset_odometry = true;
    output.relax = false;
    output.valid = finite_output(output);
    return output;
}

void ChassisController::fill_observed(const odometry_state& odometry,
                                      const ahrs::message& attitude,
                                      const Pendulum& lpendulum, const Pendulum& rpendulum)
{
    observed_[0] = odometry.x;
    observed_[1] = odometry.v;
    observed_[2] = attitude.total_yaw;
    observed_[3] = attitude.gyro_y;
    observed_[4] = lpendulum.link().alpha;
    observed_[5] = lpendulum.link().dalpha;
    observed_[6] = rpendulum.link().alpha;
    observed_[7] = rpendulum.link().dalpha;
    observed_[8] = attitude.pitch;
    observed_[9] = attitude.gyro_p;
}

bool ChassisController::valid_input(const chassis_command& command,
                                    const odometry_state& odometry,
                                    const ahrs::message& attitude,
                                    const Pendulum& lpendulum, const Pendulum& rpendulum,
                                    float dt, bool control_valid) const
{
    return control_valid && command.valid && odometry.valid && lpendulum.link().valid &&
           rpendulum.link().valid && std::isfinite(dt) && dt >= cfg_.runtime.min_dt_s &&
           dt <= cfg_.runtime.max_dt_s && std::isfinite(odometry.x) &&
           std::isfinite(odometry.v) && std::isfinite(attitude.pitch) &&
           std::isfinite(attitude.roll) && std::isfinite(attitude.total_yaw) &&
           std::isfinite(attitude.gyro_r) && std::isfinite(attitude.gyro_p) &&
           std::isfinite(attitude.gyro_y);
}

bool ChassisController::finite_output(const chassis_output& output)
{
    return std::isfinite(output.left_target.F) && std::isfinite(output.left_target.Tp) &&
           std::isfinite(output.right_target.F) && std::isfinite(output.right_target.Tp) &&
           std::isfinite(output.left_wheel_torque) &&
           std::isfinite(output.right_wheel_torque);
}

float ChassisController::support_force(const Pendulum& lpendulum, const Pendulum& rpendulum)
{
    return lpendulum.link().N + rpendulum.link().N;
}

chassis_state
ChassisController::requested_motion_state(const chassis_command& command) const
{
    return command.mode == command_mode::spin ? chassis_state::SPIN : chassis_state::NORMAL;
}

void ChassisController::enter_state(chassis_state next, Pendulum& lpendulum,
                                    Pendulum& rpendulum)
{
    if (next == state_)
    {
        return;
    }

    state_ = next;
    state_elapsed_s_ = 0.0f;
    roll_pd_.clear();
    lpendulum.reset_control();
    rpendulum.reset_control();
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
