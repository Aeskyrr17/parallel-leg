#pragma once

#include "ahrs.hpp"
#include "control_config.hpp"
#include "leg_types.hpp"
#include "lqr.hpp"
#include "msgs.hpp"
#include "pid.hpp"

#include <cstdint>

namespace wbr
{

enum class chassis_state : std::uint8_t
{
    RELAX = 0,
    NORMAL,
    OFFGROUND,
    SPIN,
    JUMP,
};

enum class jump_stage : std::uint8_t
{
    DONT_JUMP = 0,
    START_JUMP,
    EXTEND_LEGS,
    IN_AIR,
    LANDING,
};

struct chassis_context
{
    const ahrs::message& ins;
    const chassis_command& cmd;
    const odometry_state& odom;

    const link_state& left;
    const link_state& right;

    bool control_ok = false;
};

struct chassis_output
{
    virtual_force left_target{};
    virtual_force right_target{};

    float tau_w_l = 0.0f;
    float tau_w_r = 0.0f;

    bool relax = true;
    bool reset_odom = false;
};

class ChassisController
{
public:
    explicit ChassisController(const chassis_config& cfg);

    chassis_output step(const chassis_context& ctx);
    chassis_state state() const { return state_; }
    jump_stage jump_state() const { return jump_stage_; }

    ::control::pid& left_len_pid_for_tuning() noexcept { return left_control_.len_pid; }
    ::control::pid& right_len_pid_for_tuning() noexcept { return right_control_.len_pid; }
    ::control::pid& roll_pid_for_tuning() noexcept { return roll_pd_; }

private:
    struct leg_control_state
    {
        explicit leg_control_state(const leg_control_config& cfg) : len_pid(cfg.len_pid) {}

        ::control::pid len_pid;
    };

    void reset();
    static float len_control(leg_control_state& control, const link_state& leg,
                             float reference);
    float roll_control(const chassis_context& ctx);
    void transition_to(chassis_state next);
    void transition_jump_to(jump_stage next);

    void step_relax(const chassis_context& ctx, chassis_output& out);
    void step_normal(const chassis_context& ctx, chassis_output& out);
    void step_offground(const chassis_context& ctx, chassis_output& out);
    void step_spin(const chassis_context& ctx, chassis_output& out);
    void step_jump(const chassis_context& ctx, chassis_output& out);

    lqr_state build_obs(const chassis_context& ctx) const;

    lqr_state build_normal_ref(const chassis_context& ctx) const;
    lqr_state build_offground_ref(const chassis_context& ctx) const;

private:
    const chassis_config& cfg_;
    LQR lqr_;
    leg_control_state left_control_;
    leg_control_state right_control_;
    leg_control_state left_jump_retract_control_;
    leg_control_state right_jump_retract_control_;

    ::control::pid roll_pd_;

    chassis_state state_ = chassis_state::RELAX;
    jump_stage jump_stage_ = jump_stage::DONT_JUMP;
    std::uint8_t landing_support_count_ = 0U;
};

} // namespace wbr
