#include "chassis.hpp"

namespace wbr::control
{

ChassisController::ChassisController(const chassis_config& cfg)
    : lqr_(cfg.lqr),
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
        transition_jump_to(jump_stage::DONT);

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
    jump_stage_ = jump_stage::DONT;

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

    (void)ctx;
}

void ChassisController::step_normal(const chassis_context& ctx, chassis_output& out)
{
    const lqr_state obs = build_obs(ctx);

    const lqr_state ref = build_normal_ref(ctx);

    (void)obs;
    (void)ref;

    out = {};
    out.relax = true;
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
    const lqr_state obs = build_obs(ctx);

    const lqr_state ref = build_spin_ref(ctx);

    (void)obs;
    (void)ref;

    out = {};
    out.relax = true;
}

void ChassisController::step_jump(const chassis_context& ctx, chassis_output& out)
{
    (void)ctx;

    out = {};
    out.relax = true;
}

lqr_state ChassisController::build_obs(const chassis_context& ctx) const
{
    lqr_state state{};

    state.x = ctx.odom.x;
    state.dx = ctx.odom.v;

    state.phi = ctx.ins.yaw;
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

    ref.dtheta_l_l = 0.0f;
    ref.dtheta_l_r = 0.0f;

    ref.theta_b = 0.0f;
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

    return ref;
}

lqr_state ChassisController::build_spin_ref(const chassis_context& ctx) const
{
    lqr_state ref{};

    ref.x = ctx.cmd.x;
    ref.dx = ctx.cmd.v;

    ref.dtheta_l_l = 0.0f;
    ref.dtheta_l_r = 0.0f;

    ref.theta_b = 0.0f;
    ref.dtheta_b = 0.0f;

    return ref;
}

} // namespace wbr::control
