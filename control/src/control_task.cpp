#include "control_task.hpp"

#include "ahrs.hpp"
#include "chassis.hpp"
#include "constrain.hpp"
#include "control_config.hpp"
#include "control_debug.hpp"
#include "function.hpp"
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

namespace wbr::control
{
control_debug_t control_debug_data{};

namespace
{

const leg_config& leg_cfg = k_default_control.leg;
const chassis_config& chassis_cfg = k_default_control.chassis;

motors::lk8016 left_joint4{robot::motors::ljoint4};
motors::lk8016 left_joint1{robot::motors::ljoint1};
motors::lk8016 right_joint4{robot::motors::rjoint4};
motors::lk8016 right_joint1{robot::motors::rjoint1};
motors::lk9025 left_wheel{robot::motors::lwheel};
motors::lk9025 right_wheel{robot::motors::rwheel};

leg_controller left_leg{left_joint1, left_joint4, chassis_cfg.motor_dir.left_leg, leg_cfg};
leg_controller right_leg{right_joint1, right_joint4, chassis_cfg.motor_dir.right_leg, leg_cfg};
Function function{chassis_cfg.cmd};
Odometry odom{k_default_odometry};
ChassisController chassis{chassis_cfg};

TX_THREAD control_thread{};
alignas(8) std::uint8_t control_stack[6144]{};

msg::subscriber ahrs_sub{};
msg::subscriber remote_sub{};
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

bool write_actuators(const chassis_output& out)
{
    float lj1_tau = 0.0f;
    float lj4_tau = 0.0f;
    float rj1_tau = 0.0f;
    float rj4_tau = 0.0f;
    if (!left_leg.resolve_torque(out.left_target, lj1_tau, lj4_tau) ||
        !right_leg.resolve_torque(out.right_target, rj1_tau, rj4_tau))
    {
        return false;
    }

    left_leg.write_torque(lj1_tau, lj4_tau);
    right_leg.write_torque(rj1_tau, rj4_tau);

    left_wheel.set_torque(chassis_cfg.motor_dir.left_wheel_dir *
                          math::limit_abs(out.tau_w_l, chassis_cfg.max_wheel_tau));
    right_wheel.set_torque(chassis_cfg.motor_dir.right_wheel_dir *
                           math::limit_abs(out.tau_w_r, chassis_cfg.max_wheel_tau));
    return true;
}

bool register_motors()
{
    relax_motors();

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
    remoter::state remote{};
    auto& handler = motors::lkmotorhandler::instance();

    for (;;)
    {
        (void)handler.alive_check();

        (void)msg::read(ahrs_sub, attitude);
        (void)msg::read(remote_sub, remote);
        const motor_fdb_frame motor_fdb = read_motor_fdb();

        const chassis_command cmd = function.update(remote, odom.state().x, dt);

        odometry_input odom_input{};
        odom_input.quaternion[0] = attitude.quaternion[0];
        odom_input.quaternion[1] = attitude.quaternion[1];
        odom_input.quaternion[2] = attitude.quaternion[2];
        odom_input.quaternion[3] = attitude.quaternion[3];
        odom_input.acceleration[0] = attitude.accel[0];
        odom_input.acceleration[1] = attitude.accel[1];
        odom_input.acceleration[2] = attitude.accel[2];
        odom_input.wheel_velocity = 0.5f * (chassis_cfg.motor_dir.left_wheel_dir * motor_fdb.left_wheel.velocity +
                                            chassis_cfg.motor_dir.right_wheel_dir * motor_fdb.right_wheel.velocity) * chassis_cfg.wheel_radius;
        odom_input.yaw = attitude.yaw;
        odom_input.dt = dt;
        const bool odom_ok = odom.update(odom_input);

        left_leg.solve(motor_fdb.left_joint1, motor_fdb.left_joint4, attitude.pitch,
                       attitude.gyro_p, odom.state().a_z, dt,
                       chassis_cfg.wheel_side_mass, chassis_cfg.gravity);
        right_leg.solve(motor_fdb.right_joint1, motor_fdb.right_joint4, attitude.pitch,
                         attitude.gyro_p, odom.state().a_z, dt,
                         chassis_cfg.wheel_side_mass, chassis_cfg.gravity);

        const bool control_ok = cmd.valid && odom_ok && left_leg.link().valid && right_leg.link().valid;

        const chassis_context ctx{
            attitude,
            cmd,
            odom.state(),
            left_leg,
            right_leg,
            control_ok,
        };
        const chassis_output out = chassis.step(ctx);

        bool written = false;
        if (chassis_cfg.runtime.actuation_enabled && control_ok && !out.relax)
        {
            written = write_actuators(out);
        }
        if (!written)
        {
            relax_motors();
        }

#ifdef WBR_DEBUG
        update_control_debug(dt, attitude, motor_fdb, cmd, out, control_ok, written);
#endif

        handler.send_control();
        tx_thread_sleep(1);
    }
}

#ifdef WBR_DEBUG
void update_control_debug(float dt, const ahrs::message& attitude,
                          const motor_fdb_frame& motor_fdb,
                          const chassis_command& cmd, const chassis_output& out,
                          bool input_ok, bool output_ok)
{
    auto& debug = ::wbr::control::control_debug_data;
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
    debug.command_yaw_rate = cmd.dyaw;
    debug.command_leg_length = cmd.len;
    debug.wheel_torque_left_ref = out.tau_w_l;
    debug.wheel_torque_right_ref = out.tau_w_r;

    const link_state& left_link = left_leg.link();
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

    const link_state& right_link = right_leg.link();
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
    remote_sub = msg::subscribe<remoter::state>();
    if (!ahrs_sub.valid() || !remote_sub.valid())
    {
        return false;
    }

    if (!register_motors())
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

} // namespace wbr::control
