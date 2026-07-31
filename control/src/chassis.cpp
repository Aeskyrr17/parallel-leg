#include "chassis.hpp"

#include "constrain.hpp"

namespace wbr
{

ChassisController::ChassisController(const chassis_config& cfg)
    : cfg_(cfg),
      lqr_(cfg.lqr),
      roll_pd_(cfg.roll_pid)
{
    reset();
}

chassis_output ChassisController::step(const chassis_context& ctx)
{
    chassis_output out{};

    if (!ctx.control_ok || ctx.cmd.mode == command_mode::relax)
    {
        const bool entering_relax = (state_ != chassis_state::RELAX);

        transition_to(chassis_state::RELAX);
        transition_jump_to(jump_stage::DONT_JUMP);

        if (entering_relax)
        {
            roll_pd_.clear();
        }
        step_relax(ctx, out);
    }
    else
    {
        switch (state_)
        {
        case chassis_state::RELAX:
            step_relax(ctx, out);
            break;

        case chassis_state::NORMAL:
            step_normal(ctx, out);
            break;

        case chassis_state::OFFGROUND:
            step_offground(ctx, out);
            break;

        case chassis_state::SPIN:
            step_spin(ctx, out);
            break;

        case chassis_state::JUMP:
            step_jump(ctx, out);
            break;
        }
    }

    return out;
}

void ChassisController::reset()
{
    state_ = chassis_state::RELAX;
    jump_stage_ = jump_stage::DONT_JUMP;

    roll_pd_.clear();
}

void ChassisController::transition_to(chassis_state next)
{
    if (state_ == next) {return;}
    state_ = next;
}

void ChassisController::transition_jump_to(jump_stage next)
{
    if (jump_stage_ == next) {return;}
    jump_stage_ = next;
}

void ChassisController::step_relax(const chassis_context& ctx, chassis_output& out)
{
    out = {};
    out.relax = true;
    out.reset_odom = true;

    // ctx.left.reset();
    // ctx.right.reset();

    if (!ctx.control_ok)
    {
        return;
    }

    if (ctx.cmd.mode == command_mode::normal)
    {
        transition_to(chassis_state::NORMAL);
    }
    else if (ctx.cmd.mode == command_mode::spin)
    {
        transition_to(chassis_state::SPIN);
    }
    else if (ctx.cmd.mode == command_mode::jump)
    {
        transition_to(chassis_state::NORMAL);
    }
}

void ChassisController::step_normal(const chassis_context& ctx, chassis_output& out)
{
    const link_state& left = ctx.left.link();
    const link_state& right = ctx.right.link();
    const lqr_state obs = build_obs(ctx);
    const lqr_state ref = build_normal_ref(ctx);
    const lqr_output lqr = lqr_.solve(left.len, right.len, false, obs, ref);

    roll_pd_.ref = ctx.cmd.roll;
    roll_pd_.fdb = ctx.ins.roll;
    roll_pd_.update(ctx.ins.gyro_r);

    out = {};
    const float len_ref = ctx.cmd.len + cfg_.state.leg_len_bias;
    out.left_target.F = ctx.left.len_control(len_ref + roll_pd_.result) - left.Fs;
    out.right_target.F = ctx.right.len_control(len_ref - roll_pd_.result) - right.Fs;
    // out.left_target.F = ctx.left.len_control(len_ref);
    // out.right_target.F = ctx.right.len_control(len_ref);
    out.left_target.Tp = lqr.tau_l_l;
    out.right_target.Tp = lqr.tau_l_r;
    out.tau_w_l = lqr.tau_w_l;
    out.tau_w_r = lqr.tau_w_r;
    // out.tau_w_l = 0.0;
    // out.tau_w_r = 0.0;
    out.relax = false;

    if (ctx.cmd.mode == command_mode::spin)
    {
        transition_to(chassis_state::SPIN);
    }

    if (ctx.cmd.mode == command_mode::jump)
    {
        if (ctx.cmd.jump == jump_command::Prepared)
        {
            transition_jump_to(jump_stage::START_JUMP);
        }
        else if (ctx.cmd.jump == jump_command::Jump &&
                 jump_stage_ == jump_stage::START_JUMP)
        {
            transition_jump_to(jump_stage::EXTEND_LEGS);
            transition_to(chassis_state::JUMP);
        }
    }
    else
    {
        transition_jump_to(jump_stage::DONT_JUMP);
    }

    // if (left.N + right.N < cfg_.state.offground_support_force)
    // {
    //     transition_to(chassis_state::OFFGROUND);
    // }
}

void ChassisController::step_offground(const chassis_context& ctx, chassis_output& out)
{
    const lqr_state obs = build_obs(ctx);

    const lqr_state ref = build_offground_ref(ctx);

    (void)obs;
    (void)ref;

    out = {};
    out.relax = true;
}

void ChassisController::step_spin(const chassis_context& ctx, chassis_output& out)
{
    step_normal(ctx, out);
    if (state_ == chassis_state::SPIN && ctx.cmd.mode != command_mode::spin)
    {
        transition_to(chassis_state::NORMAL);
    }
}

void ChassisController::step_jump(const chassis_context& ctx, chassis_output& out)
{
    if (ctx.cmd.mode != command_mode::jump || ctx.cmd.jump != jump_command::Jump ||
        jump_stage_ == jump_stage::DONT_JUMP)
    {
        transition_jump_to(jump_stage::DONT_JUMP);
        transition_to(chassis_state::NORMAL);
        step_normal(ctx, out);
        return;
    }

    const link_state& left = ctx.left.link();
    const link_state& right = ctx.right.link();
    const float support_force = left.N + right.N;
    const bool offground = support_force < cfg_.jump.support_force;

    const lqr_state obs = build_obs(ctx);
    const lqr_state ref = offground ? build_offground_ref(ctx) : build_normal_ref(ctx);
    const bool use_offground_lqr = offground || jump_stage_ == jump_stage::IN_AIR;
    const lqr_output lqr =
        lqr_.solve(left.len, right.len, use_offground_lqr, obs, ref);

    roll_pd_.ref = ctx.cmd.roll;
    roll_pd_.fdb = ctx.ins.roll;
    roll_pd_.update(ctx.ins.gyro_r);

    float command_len = cfg_.cmd.normal_len;
    if (jump_stage_ == jump_stage::EXTEND_LEGS)
    {
        command_len = cfg_.jump.extend_len;
    }
    else if (jump_stage_ == jump_stage::IN_AIR)
    {
        command_len = cfg_.jump.retract_len;
    }

    out = {};
    if (offground)
    {
        out.left_target.F = ctx.left.len_control(cfg_.jump.offground_len) - left.Fs;
        out.right_target.F = ctx.right.len_control(cfg_.jump.offground_len) - right.Fs;
        out.reset_odom = true;
    }
    else
    {
        const float len_ref = command_len + cfg_.state.leg_len_bias;
        out.left_target.F = ctx.left.len_control(len_ref + roll_pd_.result) - left.Fs;
        out.right_target.F = ctx.right.len_control(len_ref - roll_pd_.result) - right.Fs;

        if (jump_stage_ == jump_stage::EXTEND_LEGS)
        {
            out.left_target.F = cfg_.jump.extend_force;
            out.right_target.F = cfg_.jump.extend_force;
        }
        else if (jump_stage_ == jump_stage::IN_AIR)
        {
            out.left_target.F = cfg_.jump.retract_force;
            out.right_target.F = cfg_.jump.retract_force;
        }
    }

    out.left_target.Tp = lqr.tau_l_l;
    out.right_target.Tp = lqr.tau_l_r;
    out.tau_w_l = lqr.tau_w_l;
    out.tau_w_r = lqr.tau_w_r;
    out.relax = false;

    const float average_len = 0.5f * (left.len + right.len);
    if (jump_stage_ == jump_stage::EXTEND_LEGS && average_len > cfg_.jump.extend_done_len)
    {
        transition_jump_to(jump_stage::IN_AIR);
    }
    else if (jump_stage_ == jump_stage::IN_AIR && average_len < cfg_.jump.retract_done_len)
    {
        transition_jump_to(jump_stage::LANDING);
    }
    else if (jump_stage_ == jump_stage::LANDING && support_force > cfg_.jump.support_force)
    {
        transition_jump_to(jump_stage::DONT_JUMP);
        transition_to(chassis_state::NORMAL);
    }
}

lqr_state ChassisController::build_obs(const chassis_context& ctx) const
{
    lqr_state state{};

    state.x = ctx.odom.x;
    state.dx = ctx.odom.v;

    state.phi = ctx.ins.total_yaw;
    state.dphi = ctx.ins.gyro_y;

    state.theta_l_l = ctx.left.link().alpha;
    state.dtheta_l_l = ctx.left.link().dalpha;

    state.theta_l_r = ctx.right.link().alpha;
    state.dtheta_l_r = ctx.right.link().dalpha;

    state.theta_b = ctx.ins.pitch;
    state.dtheta_b = ctx.ins.gyro_p;

    return state;
}

lqr_state ChassisController::build_normal_ref(const chassis_context& ctx) const
{
    lqr_state ref{};

    ref.x = ctx.cmd.x;
    ref.dx = ctx.cmd.v;
    ref.phi = ctx.cmd.yaw;
    ref.dphi = ctx.cmd.yaw_rate;

    ref.theta_l_l = 0.0f;
    ref.dtheta_l_l = 0.0f;
    ref.theta_l_r = 0.0f;
    ref.dtheta_l_r = 0.0f;

    ref.theta_b = cfg_.pitch_offset;
    // ref.theta_b = 0.0f;
    ref.dtheta_b = 0.0f;

    return ref;
}

lqr_state ChassisController::build_offground_ref(const chassis_context& ctx) const
{
    const lqr_state obs = build_obs(ctx);

    lqr_state ref{};

    ref.x = obs.x;
    ref.dx = obs.dx;
    ref.phi = obs.phi;
    ref.dphi = obs.dphi;

    ref.theta_l_l = obs.theta_b;
    ref.dtheta_l_l = 0.0f;
    ref.theta_l_r = obs.theta_b;
    ref.dtheta_l_r = 0.0f;

    ref.theta_b = obs.theta_b;
    ref.dtheta_b = obs.dtheta_b;

    return ref;
}

} // namespace wbr
