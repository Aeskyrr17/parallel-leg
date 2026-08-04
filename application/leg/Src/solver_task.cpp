#include "leg_tasks.hpp"

#include "ahrs.hpp"
#include "constants.hpp"
#include "constrain.hpp"
#include "leg_debug.hpp"
#include "leg_system.hpp"
#include "lkmotorhandler.hpp"
#include "msg.hpp"
#include "odometry.hpp"
#include "tx_api.h"
#include "vmc.hpp"

#include <cmath>

namespace app
{
namespace
{

constexpr std::uint32_t motor_feedback_check_period_ticks = 100U;

float directed(float value, std::int8_t direction) noexcept
{
    return value * static_cast<float>(direction);
}

bool control_target_usable(
    const leg_messages::control_target& target) noexcept
{
    return target.valid &&
           std::isfinite(target.left_leg_force_n) &&
           std::isfinite(target.right_leg_force_n) &&
           std::isfinite(target.left_leg_torque_nm) &&
           std::isfinite(target.right_leg_torque_nm) &&
           std::isfinite(target.left_wheel_torque_nm) &&
           std::isfinite(target.right_wheel_torque_nm);
}

template <typename Motor>
float limit_motor_torque(float requested_torque,
                         float configured_limit,
                         const Motor& motor) noexcept
{
    const float motor_limit =
        static_cast<float>(motor.max_current) * motor.torque_constant;
    const float limit =
        motor_limit < configured_limit ? motor_limit : configured_limit;
    return math::limit_abs(requested_torque, limit);
}

float estimate_support_force(const leg_force& measured_force,
                             float leg_length,
                             float leg_angle,
                             float length_velocity_change,
                             float vertical_acceleration) noexcept
{
    const float projected_force =
        measured_force.axial_force_n * std::cos(leg_angle) +
        measured_force.torque_nm / leg_length * std::sin(leg_angle);
    return projected_force +
           leg_config::wheel_mass_kg *
               (vertical_acceleration - length_velocity_change * std::cos(leg_angle));
}

void relax_all(leg_system& robot) noexcept
{
    robot.left_joint_4().relax();
    robot.left_joint_1().relax();
    robot.right_joint_4().relax();
    robot.right_joint_1().relax();
    robot.left_wheel().relax();
    robot.right_wheel().relax();
}

} // namespace

namespace solver_task
{

[[noreturn]] void run() noexcept
{
    auto* solver_feedback_topic = msg::create<leg_messages::solver_feedback>();
    auto* odometry_topic = msg::create<leg_messages::odometry>();
    auto ahrs_sub = msg::subscribe<::ahrs::message>();
    auto control_target_sub = msg::subscribe<leg_messages::control_target>();

    auto& robot = leg_system::instance();
    auto& left_joint_4 = robot.left_joint_4();
    auto& left_joint_1 = robot.left_joint_1();
    auto& right_joint_4 = robot.right_joint_4();
    auto& right_joint_1 = robot.right_joint_1();
    auto& left_wheel = robot.left_wheel();
    auto& right_wheel = robot.right_wheel();
    auto& motor_handler = motors::lkmotorhandler::instance();

    vmc left_vmc;
    vmc right_vmc;
    odometry odometry_filter;

    ::ahrs::message attitude{};
    leg_messages::control_target control_target{};
    bool ahrs_received = false;
    bool motors_online = motor_handler.alive_check();
    std::uint32_t last_motor_check_tick = static_cast<std::uint32_t>(tx_time_get());
    float previous_left_length_velocity = 0.0f;
    float previous_right_length_velocity = 0.0f;

    for (;;)
    {
        if (msg::read(ahrs_sub, attitude) == types::status::ok)
        {
            ahrs_received = true;
        }
        (void)msg::read(control_target_sub, control_target);

        const auto tick = static_cast<std::uint32_t>(tx_time_get());
        if ((tick - last_motor_check_tick) >= motor_feedback_check_period_ticks)
        {
            motors_online = motor_handler.alive_check();
            last_motor_check_tick = tick;
        }

        leg_messages::solver_feedback feedback{};
        leg_messages::odometry odometry_data{};
        feedback.tick = tick;
        odometry_data.tick = tick;

        if (ahrs_received && motors_online)
        {
            const auto& left_joint_4_feedback = left_joint_4.get_feedback();
            const auto& left_joint_1_feedback = left_joint_1.get_feedback();
            const auto& right_joint_4_feedback = right_joint_4.get_feedback();
            const auto& right_joint_1_feedback = right_joint_1.get_feedback();

            const bool vmc_valid =
                left_vmc.resolve(
                    math::pi +
                        directed(
                            left_joint_1_feedback.position,
                            leg_config::left_joint_1_direction),
                    directed(
                        left_joint_4_feedback.position,
                        leg_config::left_joint_4_direction)) &&
                right_vmc.resolve(
                    math::pi +
                        directed(
                            right_joint_1_feedback.position,
                            leg_config::right_joint_1_direction),
                    directed(
                        right_joint_4_feedback.position,
                        leg_config::right_joint_4_direction));

            if (vmc_valid)
            {
                const leg_velocity left_velocity =
                    left_vmc.joint_velocity_to_leg_velocity({
                        directed(
                            left_joint_1_feedback.velocity,
                            leg_config::left_joint_1_direction),
                        directed(
                            left_joint_4_feedback.velocity,
                            leg_config::left_joint_4_direction),
                    });
                const leg_velocity right_velocity =
                    right_vmc.joint_velocity_to_leg_velocity({
                        directed(
                            right_joint_1_feedback.velocity,
                            leg_config::right_joint_1_direction),
                        directed(
                            right_joint_4_feedback.velocity,
                            leg_config::right_joint_4_direction),
                    });

                odometry_input odometry_sample{};
                odometry_sample.quaternion_wxyz = {
                    attitude.quaternion[0],
                    attitude.quaternion[1],
                    attitude.quaternion[2],
                    attitude.quaternion[3],
                };
                odometry_sample.acceleration_body_mps2 = {
                    attitude.accel[0],
                    attitude.accel[1],
                    attitude.accel[2],
                };
                odometry_sample.yaw_rad = attitude.yaw;
                odometry_sample.left_wheel_velocity_rad_s =
                    left_wheel.get_feedback().velocity;
                odometry_sample.right_wheel_velocity_rad_s =
                    right_wheel.get_feedback().velocity;
                odometry_sample.tick = tick;
                odometry_data = odometry_filter.update(odometry_sample);

                if (odometry_data.valid)
                {
                    const float corrected_pitch =
                        attitude.pitch - leg_config::pitch_zero_offset_rad;
                    feedback.left_leg_length_m =
                        left_vmc.leg_length_m();
                    feedback.right_leg_length_m =
                        right_vmc.leg_length_m();
                    feedback.left_leg_angle_rad =
                        left_vmc.leg_angle_rad() -
                        0.5f * math::pi + corrected_pitch;
                    feedback.right_leg_angle_rad =
                        right_vmc.leg_angle_rad() -
                        0.5f * math::pi + corrected_pitch;
                    feedback.left_leg_length_velocity_mps =
                        left_velocity.length_mps;
                    feedback.right_leg_length_velocity_mps =
                        right_velocity.length_mps;
                    feedback.left_leg_angular_velocity_rad_s =
                        left_velocity.angle_rad_s + attitude.gyro_p;
                    feedback.right_leg_angular_velocity_rad_s =
                        right_velocity.angle_rad_s + attitude.gyro_p;

                    const leg_force left_measured_force =
                        left_vmc.joint_torque_to_force({
                            directed(
                                left_joint_1_feedback.torque,
                                leg_config::left_joint_1_direction),
                            directed(
                                left_joint_4_feedback.torque,
                                leg_config::left_joint_4_direction),
                        });
                    const leg_force right_measured_force =
                        right_vmc.joint_torque_to_force({
                            directed(
                                right_joint_1_feedback.torque,
                                leg_config::right_joint_1_direction),
                            directed(
                                right_joint_4_feedback.torque,
                                leg_config::right_joint_4_direction),
                        });

                    const float left_length_velocity_change =
                        feedback.left_leg_length_velocity_mps -
                        previous_left_length_velocity;
                    const float right_length_velocity_change =
                        feedback.right_leg_length_velocity_mps -
                        previous_right_length_velocity;

                    const float left_support_force =
                        estimate_support_force(
                            left_measured_force,
                            feedback.left_leg_length_m,
                            feedback.left_leg_angle_rad,
                            left_length_velocity_change,
                            odometry_data.vertical_acceleration_mps2);
                    const float right_support_force =
                        estimate_support_force(
                            right_measured_force,
                            feedback.right_leg_length_m,
                            feedback.right_leg_angle_rad,
                            right_length_velocity_change,
                            odometry_data.vertical_acceleration_mps2);
                    feedback.support_force_n = left_support_force + right_support_force;
                    feedback.yaw_rad = attitude.total_yaw;
                    feedback.valid = true;

                    previous_left_length_velocity =
                        feedback.left_leg_length_velocity_mps;
                    previous_right_length_velocity =
                        feedback.right_leg_length_velocity_mps;

                    if (leg_config::feature::off_ground_detection &&
                        feedback.support_force_n <
                        leg_config::off_ground_force_threshold_n)
                    {
                        odometry_filter.reset();
                    }
                }
            }
        }

        if (!feedback.valid)
        {
            previous_left_length_velocity = 0.0f;
            previous_right_length_velocity = 0.0f;
            odometry_filter.reset();
            control_target.valid = false;
        }

        (void)msg::publish(solver_feedback_topic, feedback);
        (void)msg::publish(odometry_topic, odometry_data);

        float left_joint_1_torque_nm = 0.0f;
        float left_joint_4_torque_nm = 0.0f;
        float right_joint_1_torque_nm = 0.0f;
        float right_joint_4_torque_nm = 0.0f;
        float left_wheel_torque_nm = 0.0f;
        float right_wheel_torque_nm = 0.0f;

        if (feedback.valid && control_target_usable(control_target))
        {
            const joint_torque left_torque =
                left_vmc.force_to_joint_torque({
                    control_target.left_leg_force_n,
                    control_target.left_leg_torque_nm,
                });
            const joint_torque right_torque =
                right_vmc.force_to_joint_torque({
                    control_target.right_leg_force_n,
                    control_target.right_leg_torque_nm,
                });

            left_joint_1_torque_nm = directed(
                limit_motor_torque(
                    left_torque.joint_1_nm,
                    leg_config::max_joint_torque_nm,
                    left_joint_1),
                leg_config::left_joint_1_direction);
            left_joint_4_torque_nm = directed(
                limit_motor_torque(
                    left_torque.joint_4_nm,
                    leg_config::max_joint_torque_nm,
                    left_joint_4),
                leg_config::left_joint_4_direction);
            right_joint_1_torque_nm = directed(
                limit_motor_torque(
                    right_torque.joint_1_nm,
                    leg_config::max_joint_torque_nm,
                    right_joint_1),
                leg_config::right_joint_1_direction);
            right_joint_4_torque_nm = directed(
                limit_motor_torque(
                    right_torque.joint_4_nm,
                    leg_config::max_joint_torque_nm,
                    right_joint_4),
                leg_config::right_joint_4_direction);
            left_wheel_torque_nm = directed(
                limit_motor_torque(
                    control_target.left_wheel_torque_nm,
                    leg_config::max_wheel_torque_nm,
                    left_wheel),
                leg_config::left_wheel_direction);
            right_wheel_torque_nm = directed(
                limit_motor_torque(
                    control_target.right_wheel_torque_nm,
                    leg_config::max_wheel_torque_nm,
                    right_wheel),
                leg_config::right_wheel_direction);

            left_joint_1.set_torque(left_joint_1_torque_nm);
            left_joint_4.set_torque(left_joint_4_torque_nm);
            right_joint_1.set_torque(right_joint_1_torque_nm);
            right_joint_4.set_torque(right_joint_4_torque_nm);
            left_wheel.set_torque(left_wheel_torque_nm);
            right_wheel.set_torque(right_wheel_torque_nm);
        }
        else
        {
            relax_all(robot);
        }

        leg_debug_motor_torque = {
            {left_joint_1_torque_nm, left_joint_1.get_feedback().torque},
            {left_joint_4_torque_nm, left_joint_4.get_feedback().torque},
            {right_joint_1_torque_nm, right_joint_1.get_feedback().torque},
            {right_joint_4_torque_nm, right_joint_4.get_feedback().torque},
            {left_wheel_torque_nm, left_wheel.get_feedback().torque},
            {right_wheel_torque_nm, right_wheel.get_feedback().torque},
        };
        leg_debug_solver_feedback = feedback;
        leg_debug_odometry = odometry_data;
        ++leg_debug_solver_heartbeat;

        motor_handler.send_control();
        tx_thread_sleep(leg_config::solver_thread::period_ticks);
    }
}

} // namespace solver_task

} // namespace app
