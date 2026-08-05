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

struct pid_tuning
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct control_tuning
{
    pid_tuning leg_len{};
    pid_tuning roll{};
};

extern volatile control_tuning g_control_tuning;

class ChassisController
{
public:
    explicit ChassisController(const chassis_config& cfg);

    chassis_output step(const chassis_context& ctx);
    chassis_state state() const { return state_; }
    jump_stage jump_state() const { return jump_stage_; }
    const lqr_diagnostics& lqr_debug() const { return lqr_debug_; }

private:
    void reset();
    void sync_tuning();
    static float len_control(::control::pid& pid, const link_state& leg,
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
    lqr_diagnostics lqr_debug_{};
    ::control::pid left_len_pid_;
    ::control::pid right_len_pid_;
    ::control::pid left_jump_retract_pid_;
    ::control::pid right_jump_retract_pid_;

    ::control::pid roll_pd_;

    chassis_state state_ = chassis_state::RELAX;
    jump_stage jump_stage_ = jump_stage::DONT_JUMP;
    std::uint8_t landing_support_count_ = 0U;
};

} // namespace wbr
