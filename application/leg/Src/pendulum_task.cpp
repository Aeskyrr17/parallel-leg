#include "leg_tasks.hpp"

#include "ahrs.hpp"
#include "constrain.hpp"
#include "leg_debug.hpp"
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

    lqr lqr_controller;
    control::pid right_leg_pid(
        app::leg_config::pid::leg_length::kp,
        app::leg_config::pid::leg_length::ki,
        app::leg_config::pid::leg_length::kd,
        app::leg_config::pid::leg_length::max_out,
        app::leg_config::pid::leg_length::max_iout,
        control::pid_mode::delta);

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

        // leg_pid is the single FreeMASTER tuning entry. Keep the right-leg
        // controller's independent state while sharing the same live gains.
        right_leg_pid.tune(leg_pid.kp, leg_pid.ki, leg_pid.kd);
        right_leg_pid.max_out = leg_pid.max_out;
        right_leg_pid.max_iout = leg_pid.max_iout;

        const bool disabled = command.valid && !command.enabled;
        const bool inputs_ready = ahrs_received && command.valid && solver.valid && odometry.valid;

        if (disabled || !inputs_ready)
        {
            leg_pid.clear();
            right_leg_pid.clear();
            roll_pid.clear();
            target.valid = disabled;
        }
        else
        {
            const bool off_ground =
                leg_config::feature::off_ground_detection &&
                solver.support_force_n < leg_config::off_ground_force_threshold_n;
            const bool extending =
                leg_config::feature::jump &&
                command.jump_status == leg_messages::jump_state::extending;
            const bool airborne =
                leg_config::feature::jump &&
                command.jump_status == leg_messages::jump_state::airborne;
            const bool landing =
                leg_config::feature::jump &&
                command.jump_status == leg_messages::jump_state::landing;
            const bool air_control = off_ground || airborne || landing;
            const float corrected_pitch =
                attitude.pitch - leg_config::pitch_zero_offset_rad;

            const lqr::state observed{
                odometry.position_m,
                odometry.velocity_mps,
                attitude.total_yaw,
                attitude.gyro_y,
                solver.left_leg_angle_rad,
                solver.left_leg_angular_velocity_rad_s,
                solver.right_leg_angle_rad,
                solver.right_leg_angular_velocity_rad_s,
                corrected_pitch,
                attitude.gyro_p,
            };
            lqr::state reference{};

            roll_pid.ref = command.roll_rad;
            roll_pid.fdb = attitude.roll;
            roll_pid.update();

            if ((off_ground || landing) && !extending && !airborne)
            {
                leg_pid.ref = off_ground_leg_length_m;
                leg_pid.fdb = solver.left_leg_length_m;
                leg_pid.update(solver.left_leg_length_velocity_mps);

                right_leg_pid.ref = off_ground_leg_length_m;
                right_leg_pid.fdb = solver.right_leg_length_m;
                right_leg_pid.update(solver.right_leg_length_velocity_mps);

                target.left_leg_force_n = leg_pid.result;
                target.right_leg_force_n = right_leg_pid.result;

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
                const float base_length = command.leg_length_m +
                                          (airborne ? 0.0f : grounded_leg_length_bias_m);
                const float roll_offset = airborne ? 0.0f : roll_pid.result;

                leg_pid.ref = base_length + roll_offset;
                leg_pid.fdb = solver.left_leg_length_m;
                leg_pid.update(solver.left_leg_length_velocity_mps);

                right_leg_pid.ref = base_length - roll_offset;
                right_leg_pid.fdb = solver.right_leg_length_m;
                right_leg_pid.update(solver.right_leg_length_velocity_mps);

                target.left_leg_force_n = leg_pid.result;
                target.right_leg_force_n = right_leg_pid.result;

                if (extending)
                {
                    target.left_leg_force_n = leg_config::jump::extending_force_n;
                    target.right_leg_force_n = leg_config::jump::extending_force_n;
                }

                reference[0] = command.position_m;
                reference[1] = command.speed_mps;
                reference[2] = command.yaw_rad;
                reference[3] = command.yaw_rate_rad_s;
                reference[4] = leg_config::normal_leg_angle_reference_rad;
                reference[6] = leg_config::normal_leg_angle_reference_rad;
            }

            const lqr::output lqr_output =
                lqr_controller.calculate(
                    observed,
                    reference,
                    solver.left_leg_length_m,
                    solver.right_leg_length_m,
                    air_control
                        ? lqr::gain_mode::off_ground
                        : lqr::gain_mode::normal);

            float wheel_scale = 1.0f;
            if (extending)
            {
                const float average_leg_length =
                    0.5f * (solver.left_leg_length_m + solver.right_leg_length_m);
                wheel_scale = math::clamp(
                    (leg_config::jump::wheel_off_length_m - average_leg_length) /
                        (leg_config::jump::wheel_off_length_m -
                         leg_config::jump::wheel_fade_start_length_m),
                    0.0f,
                    1.0f);
            }

            if (lqr_output.valid)
            {
                target.left_leg_torque_nm = lqr_output.left_leg_torque_nm;
                target.right_leg_torque_nm = lqr_output.right_leg_torque_nm;
                if (!air_control)
                {
                    target.left_wheel_torque_nm = lqr_output.left_wheel_torque_nm * wheel_scale;
                    target.right_wheel_torque_nm = lqr_output.right_wheel_torque_nm * wheel_scale;
                }
                target.valid = true;
            }
            else
            {
                target = {};
                target.tick = tick;
                leg_pid.clear();
                right_leg_pid.clear();
                roll_pid.clear();
            }
        }

        (void)msg::publish(control_target_topic, target);
        leg_debug_imu = attitude;
        leg_debug_control_target = target;
        ++leg_debug_pendulum_heartbeat;
        tx_thread_sleep(leg_config::control_thread::period_ticks);
    }
}

} // namespace pendulum_task

} // namespace app
