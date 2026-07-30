#include "function.hpp"

#include "constrain.hpp"

#include <cmath>

namespace wbr
{

Function::Function(const command_config& cfg)
    : cfg_(cfg), yaw_updater_(0.0f, 0.0f), vel_updater_(0.0f, 0.0f)
{
    reset();
}

const chassis_command& Function::update(const remoter::state& remote, float odom_x, float odom_v,
                                        float total_yaw, float dt)
{
    yaw_updater_.set_path(cfg_.yaw_slope_rate * dt);
    vel_updater_.set_path(cfg_.vel_slope_rate * dt);
    const bool transition = prev_sw_valid_ && !transition_cooldown_ &&
                            is_transition(prev_ctrl_, remote.left_sw);

    if (remote.offline)
    {
        cmd_ = {};
        cmd_.len = cfg_.normal_len;
    }
    else if (!transition)
    {
        cmd_.valid = true;
        switch (remote.left_sw)
        {
        case remoter::sw_state::low:
            cmd_.mode = command_mode::relax;
            cmd_.jump = jump_command::none;
            cmd_.v = 0.0f;
            cmd_.w = 0.0f;
            cmd_.len = cfg_.normal_len;
            cmd_.dyaw = 0.0f;
            break;

        case remoter::sw_state::mid:
            cmd_.mode = command_mode::normal;
            switch (remote.right_sw)
            {
            case remoter::sw_state::low:
                cmd_.mode = command_mode::normal;
                cmd_.jump = jump_command::none;
                cmd_.dyaw = yaw_updater_.update(-remote.right_x * cfg_.yaw_scale);
                cmd_.v = vel_updater_.update(remote.left_y * cfg_.vel_scale);
                cmd_.roll = 0.0f;
                cmd_.len =
                    math::clamp(cmd_.len + remote.left_x * cfg_.manual_len_rate * dt,
                                cfg_.min_len, cfg_.max_len);
                cmd_.w = 0.0f;
                break;

            case remoter::sw_state::mid:
                cmd_.mode = command_mode::jump;
                cmd_.jump = jump_command::none;
                cmd_.len = cfg_.normal_len;
                cmd_.v = vel_updater_.update(remote.left_y * cfg_.vel_scale);
                cmd_.dyaw = yaw_updater_.update(-remote.right_x * cfg_.yaw_scale);
                cmd_.roll = 0.0f;
                cmd_.w = 0.0f;
                break;

            case remoter::sw_state::up:
                cmd_.mode = command_mode::jump;
                cmd_.jump = jump_command::extending;
                cmd_.v = 0.0f;
                cmd_.dyaw = 0.0f;
                cmd_.w = 0.0f;
                break;
            }
            break;

        case remoter::sw_state::up:
            cmd_.mode = command_mode::spin;
            cmd_.jump = jump_command::none;
            cmd_.w = cfg_.spin_rate;
            cmd_.dyaw = 0.0f;
            cmd_.v = 0.0f;
            break;
        }
    }

    const bool spin = cmd_.mode == command_mode::spin && !transition;
    update_position(spin, odom_x, odom_v, dt);
    cmd_.yaw_rate = cmd_.w + cmd_.dyaw;
    update_yaw(total_yaw, dt);

    if (!remote.offline)
    {
        prev_ctrl_ = remote.left_sw;
        prev_sw_valid_ = true;
        transition_cooldown_ = transition;
    }

    return cmd_;
}

void Function::reset()
{
    cmd_ = {};
    cmd_.len = cfg_.normal_len;

    yaw_updater_.reset();
    vel_updater_.reset();

    maintained_x_ = 0.0f;
    maintained_yaw_ = 0.0f;
    prev_ctrl_ = remoter::sw_state::low;
    maintaining_x_ = false;
    maintaining_yaw_ = false;
    prev_sw_valid_ = false;
    transition_cooldown_ = false;
}

bool Function::is_transition(remoter::sw_state prev, remoter::sw_state current)
{
    return (prev == remoter::sw_state::low && current == remoter::sw_state::mid) ||
           (prev == remoter::sw_state::mid && current == remoter::sw_state::low) ||
           (prev == remoter::sw_state::mid && current == remoter::sw_state::up) ||
           (prev == remoter::sw_state::up && current == remoter::sw_state::mid);
}

void Function::update_position(bool spin, float odom_x, float odom_v, float dt)
{
    const bool stopped = std::fabs(cmd_.v) < cfg_.stationary_vel &&
                         std::fabs(cmd_.v - odom_v) < cfg_.stationary_vel_error;
    if (stopped || spin)
    {
        if (!maintaining_x_)
        {
            maintained_x_ = odom_x;
            maintaining_x_ = true;
            cmd_.v = 0.0f;
        }
        cmd_.x = maintained_x_;
        return;
    }

    maintaining_x_ = false;
    cmd_.x = odom_x + cmd_.v * dt;
}

void Function::reset_position(float x)
{
    maintained_x_ = x;
    cmd_.x = x;
    maintaining_x_ = true;
}

void Function::reset_yaw(float total_yaw)
{
    maintained_yaw_ = total_yaw;
    cmd_.yaw = total_yaw;
    maintaining_yaw_ = true;
}

void Function::update_yaw(float total_yaw, float dt)
{
    if (cmd_.yaw_rate != 0.0f)
    {
        maintaining_yaw_ = false;
        cmd_.yaw = total_yaw + cmd_.yaw_rate * dt;
        return;
    }

    if (!maintaining_yaw_)
    {
        maintained_yaw_ = total_yaw;
        maintaining_yaw_ = true;
    }
    cmd_.yaw = maintained_yaw_;
}

} // namespace wbr
