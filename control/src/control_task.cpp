#include "control_task.hpp"

#include "ahrs.hpp"
#include "chassis.hpp"
#include "chassis_actuator.hpp"
#include "control_config.hpp"
#include "control_debug.hpp"
#include "function_task.hpp"
#include "leg.hpp"
#include "lkmotorhandler.hpp"
#include "lkmotors.hpp"
#include "msg.hpp"
#include "odometry.hpp"
#include "remoter.hpp"
#include "robot_config.hpp"
#include "tx_api.h"

#include <cstddef>
#include <cstdint>

namespace wbr
{
control_debug_t control_debug_data{};

namespace
{

const leg_config& leg_cfg = k_default_leg;
const chassis_config& chassis_cfg = k_default_chassis;

motors::lk8016 left_joint4{robot::motors::ljoint4};
motors::lk8016 left_joint1{robot::motors::ljoint1};
motors::lk8016 right_joint4{robot::motors::rjoint4};
motors::lk8016 right_joint1{robot::motors::rjoint1};
motors::lk9025 left_wheel{robot::motors::lwheel};
motors::lk9025 right_wheel{robot::motors::rwheel};

leg_solver left_leg{leg_cfg};
leg_solver right_leg{leg_cfg};
Odometry odom{k_default_odometry};
ChassisController chassis{chassis_cfg};
chassis_actuator actuator{chassis_cfg.actuator, left_leg, right_leg,
                        {left_joint1, left_joint4, right_joint1, right_joint4,
                                left_wheel, right_wheel}};

TX_THREAD control_thread{};
alignas(8) std::uint8_t control_stack[6144]{};

msg::subscriber ahrs_sub{};
msg::subscriber command_sub{};
msg::topic* feedback_topic = nullptr;
bool tasks_started = false;

#define WBR_DEBUG
#ifdef WBR_DEBUG
void update_control_debug(
    float dt, const ahrs::message& attitude, const motor_fdb_frame& motor_fdb,
    const chassis_command& cmd, const chassis_output& out, bool input_ok,
    bool output_ok);
#endif

motor_fdb_frame read_motor_fdb()
{
    motor_fdb_frame fdb{};
    fdb.left_joint1 = left_joint1.get_feedback();
    fdb.left_joint4 = left_joint4.get_feedback();
    fdb.right_joint1 = right_joint1.get_feedback();
    fdb.right_joint4 = right_joint4.get_feedback();
    fdb.left_wheel = left_wheel.get_feedback();
    fdb.right_wheel = right_wheel.get_feedback();
    return fdb;
}

void relax_motors()
{
    left_joint1.relax();
    left_joint4.relax();
    right_joint1.relax();
    right_joint4.relax();
    left_wheel.relax();
    right_wheel.relax();
}

bool register_motors()
{
    relax_motors();

    left_joint4.offset = static_cast<float>(chassis_cfg.joint_offset.left_joint4);
    left_joint1.offset = static_cast<float>(chassis_cfg.joint_offset.left_joint1);
    right_joint4.offset = static_cast<float>(chassis_cfg.joint_offset.right_joint4);
    right_joint1.offset = static_cast<float>(chassis_cfg.joint_offset.right_joint1);

    auto& handler = motors::lkmotorhandler::instance();
    const bool left_joint4_ok = handler.register_motor(left_joint4);
    const bool left_joint1_ok = handler.register_motor(left_joint1);
    const bool right_joint4_ok = handler.register_motor(right_joint4);
    const bool right_joint1_ok = handler.register_motor(right_joint1);
    const bool left_wheel_ok = handler.register_motor(left_wheel);
    const bool right_wheel_ok = handler.register_motor(right_wheel);
    return left_joint4_ok && left_joint1_ok && right_joint4_ok && right_joint1_ok &&
           left_wheel_ok && right_wheel_ok;
}

void control_entry(ULONG /*arg*/)
{
    const float dt = chassis_cfg.runtime.dt;
    ahrs::message attitude{};
    chassis_command cmd{};
    cmd.mode = command_mode::relax;
    cmd.valid = false;
    auto& handler = motors::lkmotorhandler::instance();

    for (;;)
    {
        (void)handler.alive_check();

        (void)msg::read(ahrs_sub, attitude);
        (void)msg::read(command_sub, cmd);
        const motor_fdb_frame motor_fdb = read_motor_fdb();

        odometry_input odom_input{};
        odom_input.quaternion[0] = attitude.quaternion[0];
        odom_input.quaternion[1] = attitude.quaternion[1];
        odom_input.quaternion[2] = attitude.quaternion[2];
        odom_input.quaternion[3] = attitude.quaternion[3];
        odom_input.acceleration[0] = attitude.accel[0];
        odom_input.acceleration[1] = attitude.accel[1];
        odom_input.acceleration[2] = attitude.accel[2];
        odom_input.wheel_velocity = actuator.wheel_velocity(
            motor_fdb.left_wheel, motor_fdb.right_wheel, chassis_cfg.wheel_radius);
        odom_input.yaw = attitude.yaw;
        odom_input.dt = dt;
        const bool odom_ok = odom.update(odom_input);

        const joint_state left_joint = actuator.left_joint_state(motor_fdb.left_joint1, motor_fdb.left_joint4);
        const joint_state right_joint = actuator.right_joint_state(motor_fdb.right_joint1, motor_fdb.right_joint4);
        left_leg.solve(left_joint, attitude.pitch, attitude.gyro_p, odom.state().a_z, dt,
                       chassis_cfg.wheel_side_mass, chassis_cfg.gravity);
        right_leg.solve(right_joint, attitude.pitch, attitude.gyro_p, odom.state().a_z, dt,
                        chassis_cfg.wheel_side_mass, chassis_cfg.gravity);

        const bool control_ok = cmd.valid && odom_ok && left_leg.state().valid && right_leg.state().valid;

        const chassis_context ctx{
            attitude,
            cmd,
            odom.state(),
            left_leg.state(),
            right_leg.state(),
            control_ok,
        };
        const chassis_output out = chassis.step(ctx);

        actuator_command command{};
        const bool actuator_ok = actuator.resolve(out, command);
        const bool enable_output = chassis_cfg.runtime.actuation_enabled && control_ok && !out.relax && actuator_ok;
        if (!enable_output)
        {
            command = {};
        }
        actuator.write(command);

#ifdef WBR_DEBUG
        update_control_debug(dt, attitude, motor_fdb, cmd, out, control_ok, enable_output);
#endif

        handler.send_control();

        if (out.reset_odom)
        {
            odom.reset();
        }

        const chassis_feedback feedback{
            odom.state().x,
            odom.state().v,
            attitude.total_yaw,
        };
        (void)msg::publish(feedback_topic, feedback);

        tx_thread_sleep(1);
    }
}

#ifdef WBR_DEBUG
void update_control_debug(float dt, const ahrs::message& attitude,
                          const motor_fdb_frame& motor_fdb,
                          const chassis_command& cmd, const chassis_output& out,
                          bool input_ok, bool output_ok)
{
    auto& debug = ::wbr::control_debug_data;
    ++debug.cycle_count;
    debug.dt = dt;

    debug.pitch = attitude.pitch;
    debug.dpitch = attitude.gyro_p;
    debug.roll = attitude.roll;
    debug.droll = attitude.gyro_r;
    debug.yaw = attitude.yaw;
    debug.dyaw = attitude.gyro_y;

    const odometry_state& odom_state = odom.state();
    debug.odometry_x = odom_state.x;
    debug.odometry_v = odom_state.v;
    debug.acceleration_z = odom_state.a_z;

    debug.command_x = cmd.x;
    debug.command_v = cmd.v;
    debug.command_yaw = cmd.yaw;
    debug.command_yaw_rate = cmd.yaw_rate;
    debug.command_leg_length = cmd.len;
    debug.wheel_torque_left_ref = out.tau_w_l;
    debug.wheel_torque_right_ref = out.tau_w_r;

    const link_state& left_link = left_leg.state();
    debug.left_leg.motor1_position = motor_fdb.left_joint1.position;
    debug.left_leg.motor4_position = motor_fdb.left_joint4.position;
    debug.left_leg.motor1_velocity = motor_fdb.left_joint1.velocity;
    debug.left_leg.motor4_velocity = motor_fdb.left_joint4.velocity;
    debug.left_leg.phi = left_link.phi;
    debug.left_leg.dphi = left_link.dphi;
    debug.left_leg.alpha = left_link.alpha;
    debug.left_leg.dalpha = left_link.dalpha;
    debug.left_leg.length = left_link.len;
    debug.left_leg.length_velocity = left_link.dlen;
    debug.left_leg.force_fdb = left_link.fdb.F;
    debug.left_leg.torque_fdb = left_link.fdb.Tp;
    debug.left_leg.force_ref = out.left_target.F;
    debug.left_leg.torque_ref = out.left_target.Tp;
    debug.left_leg.support_force = left_link.N;
    debug.left_leg.spring_force = left_link.Fs;
    debug.left_leg.valid = left_link.valid;

    debug.wheel_left_fdb = motor_fdb.left_wheel;
    debug.wheel_right_fdb = motor_fdb.right_wheel;

    const link_state& right_link = right_leg.state();
    debug.right_leg.motor1_position = motor_fdb.right_joint1.position;
    debug.right_leg.motor4_position = motor_fdb.right_joint4.position;
    debug.right_leg.motor1_velocity = motor_fdb.right_joint1.velocity;
    debug.right_leg.motor4_velocity = motor_fdb.right_joint4.velocity;
    debug.right_leg.phi = right_link.phi;
    debug.right_leg.dphi = right_link.dphi;
    debug.right_leg.alpha = right_link.alpha;
    debug.right_leg.dalpha = right_link.dalpha;
    debug.right_leg.length = right_link.len;
    debug.right_leg.length_velocity = right_link.dlen;
    debug.right_leg.force_fdb = right_link.fdb.F;
    debug.right_leg.torque_fdb = right_link.fdb.Tp;
    debug.right_leg.force_ref = out.right_target.F;
    debug.right_leg.torque_ref = out.right_target.Tp;
    debug.right_leg.support_force = right_link.N;
    debug.right_leg.spring_force = right_link.Fs;
    debug.right_leg.valid = right_link.valid;

    debug.motor_torque[0] = left_joint1.cmd.torque;
    debug.motor_torque[1] = left_joint4.cmd.torque;
    debug.motor_torque[2] = right_joint1.cmd.torque;
    debug.motor_torque[3] = right_joint4.cmd.torque;
    debug.motor_torque[4] = left_wheel.cmd.torque;
    debug.motor_torque[5] = right_wheel.cmd.torque;

    debug.state = chassis.state();
    debug.jump = chassis.jump_state();
    debug.input_valid = input_ok;
    debug.output_valid = output_ok;
    debug.actuation_enabled = chassis_cfg.runtime.actuation_enabled;
}
#endif

} // namespace

bool start_control_task() noexcept
{
    if (tasks_started)
    {
        return true;
    }

    if (!ahrs::service::instance().init())
    {
        return false;
    }
    if (!remoter::service::instance().init())
    {
        return false;
    }

    ahrs_sub = msg::subscribe<ahrs::message>();
    command_sub = msg::subscribe<chassis_command>();
    feedback_topic = msg::create<chassis_feedback>();
    if (!ahrs_sub.valid() || !command_sub.valid() || feedback_topic == nullptr)
    {
        return false;
    }

    if (!register_motors())
    {
        return false;
    }
    if (!start_function_task())
    {
        return false;
    }

    const UINT status = tx_thread_create(
        &control_thread, const_cast<CHAR*>("wbr_control"), control_entry, 0U, control_stack,
        sizeof(control_stack), chassis_cfg.runtime.control_thread_priority,
        chassis_cfg.runtime.control_thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START);
    tasks_started = status == TX_SUCCESS;
    return tasks_started;
}

} // namespace wbr

struct control_tuning_handles
{
    ::control::pid* left_len_pid;
    ::control::pid* right_len_pid;
    ::control::pid* roll_pid;
};

control_tuning_handles g_control_tuning{
    &wbr::chassis.left_len_pid_for_tuning(),
    &wbr::chassis.right_len_pid_for_tuning(),
    &wbr::chassis.roll_pid_for_tuning(),
};
