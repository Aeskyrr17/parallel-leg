#include "function.hpp"

#include <cmath>

namespace wbr::control
{

Function::Function(const command_config& cfg)
    : cfg_(cfg), yaw_updater_(0.0f, 0.0f), velocity_updater_(0.0f, 0.0f)
{
    reset();
}

const chassis_command& Function::update(const remoter::state& remote, float odometry_x, float dt)
{
    const bool dt_valid = std::isfinite(dt) && dt > 0.0f;
    if (dt_valid)
    {
        yaw_updater_.set_path(cfg_.yaw_slope_rate * dt);
        velocity_updater_.set_path(cfg_.velocity_slope_rate * dt);
    }
    const bool transition = previous_switches_valid_ && !transition_cooldown_ &&
                            is_transition(previous_control_, remote.left_sw);

    if (remote.offline || !valid_remote(remote) || !dt_valid)
    {
        command_ = {};
        command_.len = cfg_.normal_len;
    }
    else if (!transition)
    {
        command_.valid = true;
        switch (remote.left_sw)
        {
        case remoter::sw_state::low:
            command_.mode = command_mode::relax;
            command_.jump = jump_command::none;
            command_.v = 0.0f;
            command_.w = 0.0f;
            command_.len = cfg_.normal_len;
            command_.dyaw = 0.0f;
            break;

        case remoter::sw_state::mid:
            command_.mode = command_mode::normal;
            switch (remote.right_sw)
            {
            case remoter::sw_state::low:
                command_.jump = jump_command::none;
                command_.dyaw = yaw_updater_.update(-remote.right_x * cfg_.yaw_scale);
                command_.v = velocity_updater_.update(remote.left_y * cfg_.velocity_scale);
                command_.roll = 0.0f;
                command_.len = clamp(command_.len + remote.left_x * cfg_.manual_len_rate * dt,
                                     cfg_.min_len, cfg_.max_len);
                command_.w = 0.0f;
                break;

            case remoter::sw_state::mid:
                command_.jump = jump_command::prepare;
                command_.len = cfg_.normal_len;
                command_.v = velocity_updater_.update(remote.left_y * cfg_.velocity_scale);
                command_.dyaw = yaw_updater_.update(-remote.right_x * cfg_.yaw_scale);
                command_.roll = 0.0f;
                command_.w = 0.0f;
                break;

            case remoter::sw_state::up:
                command_.jump = jump_command::execute;
                command_.v = 0.0f;
                command_.dyaw = 0.0f;
                command_.w = 0.0f;
                break;
            }
            break;

        case remoter::sw_state::up:
            command_.mode = command_mode::spin;
            command_.jump = jump_command::none;
            command_.w = cfg_.spin_rate;
            command_.dyaw = 0.0f;
            command_.v = 0.0f;
            break;
        }
    }

    const bool spin = command_.mode == command_mode::spin && !transition;
    update_position(spin, odometry_x, dt);

    if (!remote.offline)
    {
        previous_control_ = remote.left_sw;
        previous_switches_valid_ = true;
        transition_cooldown_ = transition;
    }

    return command_;
}

void Function::reset()
{
    command_ = {};
    command_.len = cfg_.normal_len;

    yaw_updater_.reset();
    velocity_updater_.reset();

    maintained_x_ = 0.0f;
    previous_control_ = remoter::sw_state::low;
    maintaining_x_ = false;
    previous_switches_valid_ = false;
    transition_cooldown_ = false;
}

bool Function::is_transition(remoter::sw_state previous, remoter::sw_state current)
{
    return (previous == remoter::sw_state::low && current == remoter::sw_state::mid) ||
           (previous == remoter::sw_state::mid && current == remoter::sw_state::low) ||
           (previous == remoter::sw_state::mid && current == remoter::sw_state::up) ||
           (previous == remoter::sw_state::up && current == remoter::sw_state::mid);
}

bool Function::valid_remote(const remoter::state& remote) const
{
    return std::isfinite(remote.left_x) && std::isfinite(remote.left_y) &&
           std::isfinite(remote.right_x);
}

void Function::update_position(bool spin, float odometry_x, float dt)
{
    if (!std::isfinite(odometry_x) || !std::isfinite(dt) || dt <= 0.0f)
    {
        command_.valid = false;
        command_.mode = command_mode::relax;
        return;
    }

    if (std::fabs(command_.v) < cfg_.stationary_velocity || spin)
    {
        if (!maintaining_x_)
        {
            maintained_x_ = odometry_x;
            maintaining_x_ = true;
        }
        command_.x = maintained_x_;
        return;
    }

    maintaining_x_ = false;
    command_.x = odometry_x + command_.v * dt;
}

} // namespace wbr::control
