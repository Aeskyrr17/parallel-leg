#include "chassis.hpp"

#include "constrain.hpp"

namespace wbr
{

volatile control_tuning g_control_tuning{};

ChassisController::ChassisController(const chassis_config& cfg)
    : cfg_(cfg),
      lqr_(cfg.lqr),
      left_len_pid_(cfg.leg_control.len_pid),
      right_len_pid_(cfg.leg_control.len_pid),
      left_jump_retract_pid_(cfg.jump_retract_control.len_pid),
      right_jump_retract_pid_(cfg.jump_retract_control.len_pid),
      roll_pd_(cfg.roll_pid)
{
    g_control_tuning.leg_len.kp = cfg.leg_control.len_pid.kp;
    g_control_tuning.leg_len.ki = cfg.leg_control.len_pid.ki;
    g_control_tuning.leg_len.kd = cfg.leg_control.len_pid.kd;

    g_control_tuning.roll.kp = cfg.roll_pid.kp;
    g_control_tuning.roll.ki = cfg.roll_pid.ki;
    g_control_tuning.roll.kd = cfg.roll_pid.kd;

    reset();
}

chassis_output ChassisController::step(const chassis_context& ctx)
{
    sync_tuning();

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
    landing_support_count_ = 0U;

    left_len_pid_.clear();
    right_len_pid_.clear();
    roll_pd_.clear();
}

void ChassisController::sync_tuning()
{
    const float leg_kp = g_control_tuning.leg_len.kp;
    const float leg_ki = g_control_tuning.leg_len.ki;
    const float leg_kd = g_control_tuning.leg_len.kd;

    left_len_pid_.tune(leg_kp, leg_ki, leg_kd);
    right_len_pid_.tune(leg_kp, leg_ki, leg_kd);

    const float roll_kp = g_control_tuning.roll.kp;
    const float roll_ki = g_control_tuning.roll.ki;
    const float roll_kd = g_control_tuning.roll.kd;

    roll_pd_.tune(roll_kp, roll_ki, roll_kd);
}

float ChassisController::len_control(::control::pid& pid, const link_state& leg,
                                     float reference)
{
    if (!leg.valid)
    {
        return 0.0f;
    }

    pid.ref = reference;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
    return pid.result;
}

float ChassisController::roll_control(const chassis_context& ctx)
{
    roll_pd_.ref = ctx.cmd.roll;
    roll_pd_.fdb = ctx.ins.roll;
    roll_pd_.update(ctx.ins.gyro_r);
    return roll_pd_.result;
}

void ChassisController::transition_to(chassis_state next)
{
    if (state_ == next) {return;}
    state_ = next;
}

void ChassisController::transition_jump_to(jump_stage next)
{
    if (jump_stage_ == next) {return;}

    if (next == jump_stage::IN_AIR)
    {
        left_jump_retract_pid_.clear();
        right_jump_retract_pid_.clear();
    }

    jump_stage_ = next;
    landing_support_count_ = 0U;
}

void ChassisController::step_relax(const chassis_context& ctx, chassis_output& out)
{
    out = {};
    out.relax = true;
    out.reset_odom = true;

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
    const link_state& left = ctx.left;
    const link_state& right = ctx.right;
    const lqr_state obs = build_obs(ctx);
    const lqr_state ref = build_normal_ref(ctx);
    const lqr_output lqr = lqr_.solve(left.len, right.len, false, obs, ref);

    const float roll_pd_result = roll_control(ctx);

    out = {};
    const float len_ref = ctx.cmd.len;
    out.left_target.F = len_control(left_len_pid_, left, len_ref + roll_pd_result) ;
    out.right_target.F = len_control(right_len_pid_, right, len_ref - roll_pd_result) ;
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
    if (ctx.cmd.mode != command_mode::jump ||
        ctx.cmd.jump != jump_command::Jump ||
        jump_stage_ == jump_stage::DONT_JUMP)
    {
        transition_jump_to(jump_stage::DONT_JUMP);
        transition_to(chassis_state::NORMAL);
        step_normal(ctx, out);
        return;
    }

    const link_state& left = ctx.left;
    const link_state& right = ctx.right;

    const float support_force = left.N + right.N;
    const float average_len = 0.5f * (left.len + right.len);

    const jump_stage current_stage = jump_stage_;

    const bool offground = support_force < cfg_.jump.support_force;
    const bool use_offground_lqr = offground || current_stage == jump_stage::IN_AIR || current_stage == jump_stage::LANDING;

    const lqr_state obs = build_obs(ctx);
    const lqr_state ref = use_offground_lqr ? build_offground_ref(ctx) : build_normal_ref(ctx);
    const lqr_output lqr =lqr_.solve(left.len,right.len,use_offground_lqr, obs, ref);

    const float roll_pd_result = roll_control(ctx);

    out = {};

    switch (current_stage)
    {
    case jump_stage::EXTEND_LEGS:
    {
        //起跳阶段直接输出恒定伸腿力
        out.left_target.F = cfg_.jump.extend_force;
        out.right_target.F = cfg_.jump.extend_force;

        //只根据腿长判断是否进入inair
        if (average_len > cfg_.jump.extend_done_len)
        {
            transition_jump_to(jump_stage::IN_AIR);
        }

        break;
    }

    case jump_stage::IN_AIR:
    {
        // 主动收腿到 retract_len
        const float command_len = cfg_.jump.retract_len;
        out.left_target.F = len_control(left_jump_retract_pid_,left, command_len + roll_pd_result);
        out.right_target.F =len_control( right_jump_retract_pid_, right, command_len - roll_pd_result);
        out.reset_odom = true;
        // 收腿阶段根据腿长判断是否进入landing
        if (average_len < cfg_.jump.retract_done_len)
        {
            transition_jump_to(jump_stage::LANDING);
        }

        break;
    }

    // case jump_stage::LANDING:
    // {
    //     // 落地准备，主动伸腿到 landing_len
    //     const float command_len = cfg_.jump.landing_len;
    //     out.left_target.F =len_control(left_len_pid_,left,command_len + roll_pd_result);
    //     out.right_target.F =len_control(right_len_pid_,right,command_len - roll_pd_result);
    //     out.reset_odom = true;

    //     // 支撑力恢复后认为落地
    //     if (support_force > cfg_.jump.support_force)
    //     {
    //         ++landing_support_count_;
    //         if (landing_support_count_ > 10U)
    //         {
    //             transition_jump_to(jump_stage::DONT_JUMP);
    //             transition_to(chassis_state::NORMAL);
    //         }
    //     }
    //     else
    //     {
    //         landing_support_count_ = 0U;
    //     }

    //     break;
    // }
    case jump_stage::LANDING:
    {
        const float command_len = cfg_.jump.landing_len;

        out.left_target.F = len_control(left_len_pid_, left, command_len + roll_pd_result);
        out.right_target.F = len_control(right_len_pid_, right, command_len - roll_pd_result);
        out.reset_odom = true;

        const float average_dlen = 0.5f * (left.dlen + right.dlen);
        const bool compressed = average_len < command_len - cfg_.jump.landing_compression;
        const bool compressing = average_dlen < -cfg_.jump.landing_dlen_threshold;

        landing_support_count_ = compressed && compressing
                                    ? landing_support_count_ + 1
                                    : 0;

        if (landing_support_count_ >= cfg_.jump.landing_confirm_ticks)
        {
            transition_jump_to(jump_stage::DONT_JUMP);
            transition_to(chassis_state::NORMAL);
        }


        break;
    }

    case jump_stage::START_JUMP:
    case jump_stage::DONT_JUMP:
    default:
    {
        // 正常流程中，进入 step_jump() 时不应处于这些阶段。
        transition_jump_to(jump_stage::DONT_JUMP);
        transition_to(chassis_state::NORMAL);
        step_normal(ctx, out);
        return;
    }
    }

    // 公用输出
    out.left_target.Tp = lqr.tau_l_l;
    out.right_target.Tp = lqr.tau_l_r;

    out.tau_w_l = lqr.tau_w_l;
    out.tau_w_r = lqr.tau_w_r;

    out.relax = false;
}

lqr_state ChassisController::build_obs(const chassis_context& ctx) const
{
    lqr_state state{};

    state.x = ctx.odom.x;
    state.dx = ctx.odom.v;

    state.phi = ctx.ins.total_yaw;
    state.dphi = ctx.ins.gyro_y;

    state.theta_l_l = ctx.left.alpha;
    state.dtheta_l_l = ctx.left.dalpha;

    state.theta_l_r = ctx.right.alpha;
    state.dtheta_l_r = ctx.right.dalpha;

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
