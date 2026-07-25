#pragma once

#include "ahrs.hpp"
#include "lqr.hpp"
#include "msgs.hpp"
#include "pendulum.hpp"
#include "pid.hpp"

#include <cstdint>

namespace wbr::control
{

enum class chassis_state : std::uint8_t
{
    RELAX = 0,
    NORMAL,
    OFFGROUND,
    SPIN,
    GOSTAIR,
    JUMP,
};

enum class jump_stage : std::uint8_t
{
    DONT = 0,
    EXTENDING,
    INAIR,
    LANDING,
};

struct fsm_input
{
    const chassis_command& command;
    const odometry_state& odometry;
    const ahrs::message& attitude;
    Pendulum& lpendulum;
    Pendulum& rpendulum;
    health_state health{};
    power_state power{};
    float dt = 0.0f;
    bool valid = false;
};

struct fsm_output
{
    link_force left_leg_force{};
    link_force right_leg_force{};
    float left_wheel_torque = 0.0f;
    float right_wheel_torque = 0.0f;

    chassis_state state = chassis_state::RELAX;
    jump_stage jump = jump_stage::DONT;

    bool reset_odometry = false;
    bool reset_command = false;
    bool relax = true;
    bool valid = false;
};

class ChassisController
{
public:
    explicit ChassisController(const chassis_config& cfg);

    fsm_output step(const fsm_input& input);
    void reset();

    chassis_state state() const { return state_; }
    jump_stage jump_state() const { return jump_stage_; }
    float state_elapsed_s() const { return state_elapsed_s_; }

private:
    fsm_output run_lqr(const fsm_input& input, bool offground, bool apply_roll);
    void fill_observed(const fsm_input& input);
    void fill_normal_reference(const fsm_input& input);
    void fill_offground_reference(const fsm_input& input);
    void update_roll(const fsm_input& input);

    bool valid_input(const fsm_input& input) const;
    static bool finite_output(const fsm_output& output);
    float support_force(const fsm_input& input) const;
    chassis_state requested_motion_state(const chassis_command& command) const;
    void enter_state(chassis_state next, const fsm_input& input);
    void enter_jump_stage(jump_stage next);

    chassis_config cfg_{};
    LQR lqr_;
    ::control::pid roll_pd_;

    chassis_state state_ = chassis_state::RELAX;
    jump_stage jump_stage_ = jump_stage::DONT;
    float state_elapsed_s_ = 0.0f;
    float jump_elapsed_s_ = 0.0f;
    bool reset_odometry_pending_ = true;
    bool reset_command_pending_ = true;

    float observed_[10] = {};
    float reference_[10] = {};
};

} // namespace wbr::control
