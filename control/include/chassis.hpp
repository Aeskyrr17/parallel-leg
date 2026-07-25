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
    JUMP,
};

enum class jump_stage : std::uint8_t
{
    DONT = 0,
    EXTENDING,
    INAIR,
    LANDING,
};

struct chassis_output
{
    virtual_force left_leg_force{};
    virtual_force right_leg_force{};
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

    chassis_output step(const chassis_command& command, const odometry_state& odometry,
                        const ahrs::message& attitude, Pendulum& lpendulum, Pendulum& rpendulum,
                        float dt, bool control_valid);

    float state_elapsed_s() const { return state_elapsed_s_; }

private:
    void reset();

    chassis_output normal_control(const chassis_command& command,
                                  const odometry_state& odometry,
                                  const ahrs::message& attitude, Pendulum& lpendulum,
                                  Pendulum& rpendulum, float dt);
    chassis_output offground_control(const odometry_state& odometry,
                                     const ahrs::message& attitude, Pendulum& lpendulum,
                                     Pendulum& rpendulum);
    void fill_observed(const odometry_state& odometry, const ahrs::message& attitude,
                       const Pendulum& lpendulum, const Pendulum& rpendulum);

    bool valid_input(const chassis_command& command, const odometry_state& odometry,
                     const ahrs::message& attitude, const Pendulum& lpendulum,
                     const Pendulum& rpendulum, float dt, bool control_valid) const;
    static bool finite_output(const chassis_output& output);
    static float support_force(const Pendulum& lpendulum, const Pendulum& rpendulum);
    chassis_state requested_motion_state(const chassis_command& command) const;
    void enter_state(chassis_state next, Pendulum& lpendulum, Pendulum& rpendulum);
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
