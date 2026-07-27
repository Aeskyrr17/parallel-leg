#include "leg_tasks.hpp"

#include "ahrs.hpp"
#include "lqr.hpp"
#include "msg.hpp"
#include "pid.hpp"
#include "tx_api.h"

namespace app
{
namespace
{

constexpr float grounded_leg_length_bias_m = 0.03f;
constexpr float off_ground_leg_length_m = 0.27f;
constexpr float jump_extending_force_n = 400.0f;
constexpr float jump_airborne_force_n = -200.0f;

} // namespace

namespace pendulum_task
{

[[noreturn]] void run() noexcept
{
    auto* control_target_topic = msg::create<leg_messages::control_target>();
    auto ahrs_sub = msg::subscribe<::ahrs::message>();
    auto command_sub = msg::subscribe<leg_messages::command>();
    auto solver_sub = msg::subscribe<leg_messages::solver_feedback>();
    auto odometry_sub = msg::subscribe<leg_messages::odometry>();

    // Repository PID: kp, ki, kd, max output, max integral output, mode.
    control::pid left_leg_length_pid(
        4000.0f, 0.0f, -120.0f, 200.0f, 0.0f,
        control::pid_mode::delta);
    control::pid right_leg_length_pid(
        4000.0f, 0.0f, -120.0f, 200.0f, 0.0f,
        control::pid_mode::delta);
    control::pid roll_pid(0.5f, 0.0f, -0.5f, 3.0f, 0.0f);
    lqr lqr_controller;

    ::ahrs::message attitude{};
    leg_messages::command command{};
    leg_messages::solver_feedback solver{};
    leg_messages::odometry odometry{};
    bool ahrs_received = false;

    for (;;)
    {
        if (msg::read(ahrs_sub, attitude) == types::status::ok)
        {
            ahrs_received = true;
        }
        (void)msg::read(command_sub, command);
        (void)msg::read(solver_sub, solver);
        (void)msg::read(odometry_sub, odometry);

        const auto tick = static_cast<std::uint32_t>(tx_time_get());
        leg_messages::control_target target{};
        target.tick = tick;

        const bool disabled = command.valid && !command.enabled;
        const bool inputs_ready = ahrs_received && command.valid && solver.valid && odometry.valid;

        if (disabled || !inputs_ready)
        {
            left_leg_length_pid.clear();
            right_leg_length_pid.clear();
            roll_pid.clear();
            target.valid = disabled;
        }
        else
        {
            const bool off_ground =
                solver.support_force_n < leg_config::off_ground_force_threshold_n;
            const bool airborne =
                command.jump_status == leg_messages::jump_state::airborne;

            const lqr::state observed{
                odometry.position_m,
                odometry.velocity_mps,
                attitude.total_yaw,
                attitude.gyro_y,
                solver.left_leg_angle_rad,
                solver.left_leg_angular_velocity_rad_s,
                solver.right_leg_angle_rad,
                solver.right_leg_angular_velocity_rad_s,
                attitude.pitch,
                attitude.gyro_p,
            };
            lqr::state reference{};

            roll_pid.ref = command.roll_rad;
            roll_pid.fdb = attitude.roll;
            roll_pid.update();

            if (off_ground)
            {
                left_leg_length_pid.ref = off_ground_leg_length_m;
                left_leg_length_pid.fdb = solver.left_leg_length_m;
                left_leg_length_pid.update(
                    solver.left_leg_length_velocity_mps);

                right_leg_length_pid.ref = off_ground_leg_length_m;
                right_leg_length_pid.fdb = solver.right_leg_length_m;
                right_leg_length_pid.update(
                    solver.right_leg_length_velocity_mps);

                target.left_leg_force_n = left_leg_length_pid.result;
                target.right_leg_force_n = right_leg_length_pid.result;

                reference[0] = observed[0];
                reference[1] = observed[1];
                reference[2] = observed[2];
                reference[3] = observed[3];
                reference[4] = observed[8];
                reference[6] = observed[8];
                reference[8] = observed[8];
                reference[9] = observed[9];
            }
            else
            {
                const float base_length =
                    command.leg_length_m + grounded_leg_length_bias_m;

                left_leg_length_pid.ref = base_length + roll_pid.result;
                left_leg_length_pid.fdb = solver.left_leg_length_m;
                left_leg_length_pid.update(
                    solver.left_leg_length_velocity_mps);

                right_leg_length_pid.ref = base_length - roll_pid.result;
                right_leg_length_pid.fdb = solver.right_leg_length_m;
                right_leg_length_pid.update(
                    solver.right_leg_length_velocity_mps);

                target.left_leg_force_n = left_leg_length_pid.result;
                target.right_leg_force_n = right_leg_length_pid.result;

                if (command.jump_status ==
                    leg_messages::jump_state::extending)
                {
                    target.left_leg_force_n = jump_extending_force_n;
                    target.right_leg_force_n = jump_extending_force_n;
                }
                else if (airborne)
                {
                    target.left_leg_force_n = jump_airborne_force_n;
                    target.right_leg_force_n = jump_airborne_force_n;
                }

                reference[0] = command.position_m;
                reference[1] = command.speed_mps;
                reference[2] =
                    attitude.total_yaw +
                    (command.spin_mode
                         ? 0.0f
                         : command.yaw_rate_rad_s * leg_config::control_thread::period_s);
                reference[3] = command.yaw_rate_rad_s;
            }

            const lqr::output lqr_output =
                lqr_controller.calculate(
                    observed,
                    reference,
                    solver.left_leg_length_m,
                    solver.right_leg_length_m,
                    (off_ground || airborne)
                        ? lqr::gain_mode::off_ground
                        : lqr::gain_mode::normal);

            if (lqr_output.valid)
            {
                target.left_leg_torque_nm =
                    lqr_output.left_leg_torque_nm;
                target.right_leg_torque_nm =
                    lqr_output.right_leg_torque_nm;
                if (!off_ground)
                {
                    target.left_wheel_torque_nm =
                        lqr_output.left_wheel_torque_nm;
                    target.right_wheel_torque_nm =
                        lqr_output.right_wheel_torque_nm;
                }
                target.valid = true;
            }
            else
            {
                target = {};
                target.tick = tick;
                left_leg_length_pid.clear();
                right_leg_length_pid.clear();
                roll_pid.clear();
            }
        }

        (void)msg::publish(control_target_topic, target);
        tx_thread_sleep(leg_config::control_thread::period_ticks);
    }
}

} // namespace pendulum_task

} // namespace app
