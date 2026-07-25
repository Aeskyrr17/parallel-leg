#include "control_task.hpp"

#include "ahrs.hpp"
#include "bsp_dwt.hpp"
#include "function.hpp"
#include "lkmotorhandler.hpp"
#include "lkmotors.hpp"
#include "msg.hpp"
#include "odometry.hpp"
#include "pendulum.hpp"
#include "remoter.hpp"
#include "robot_config.hpp"
#include "tx_api.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

extern "C"
{
    wbr::control::control_task_telemetry wbr_control_task_telemetry{};
}

namespace wbr::control
{
namespace
{

// Static ThreadX resources are platform capacity, not controller tuning
// parameters.
constexpr std::size_t k_input_stack_size = 2048U;
constexpr std::size_t k_control_stack_size = 6144U;

constexpr chassis_config chassis_cfg = k_default_chassis;

motors::lk8016 left_joint4{robot::motors::ljoint4};
motors::lk8016 left_joint1{robot::motors::ljoint1};
motors::lk8016 right_joint4{robot::motors::rjoint4};
motors::lk8016 right_joint1{robot::motors::rjoint1};
motors::lk9025 left_wheel{robot::motors::lwheel};
motors::lk9025 right_wheel{robot::motors::rwheel};

Pendulum lpendulum{left_joint1, left_joint4, chassis_cfg.motor_calibration.left_leg, chassis_cfg};
Pendulum rpendulum{right_joint1, right_joint4, chassis_cfg.motor_calibration.right_leg,
                   chassis_cfg};
Function function{chassis_cfg.command};
Odometry odometry{chassis_cfg};
ChassisController chassis{chassis_cfg};

TX_THREAD input_thread{};
TX_THREAD control_thread{};
TX_MUTEX input_snapshot_lock{};
alignas(8) std::uint8_t input_stack[k_input_stack_size]{};
alignas(8) std::uint8_t control_stack[k_control_stack_size]{};

msg::subscriber input_ahrs_sub{};
msg::subscriber input_remote_sub{};

struct async_input_snapshot
{
    ahrs::message attitude{};
    remoter::state remote{};
    ULONG attitude_tick = 0U;
    ULONG remote_tick = 0U;
    bool attitude_received = false;
    bool remote_received = false;
};

async_input_snapshot shared_input{};
bool tasks_started = false;

ULONG seconds_to_ticks(float seconds)
{
    if (!std::isfinite(seconds) || seconds <= 0.0f)
    {
        return 0U;
    }
    const float ticks = seconds * static_cast<float>(TX_TIMER_TICKS_PER_SECOND);
    return std::max<ULONG>(1U, static_cast<ULONG>(std::ceil(ticks)));
}

float task_dt(std::uint32_t& last_cycle_count)
{
    const float measured_dt = bsp::dwt::delta_s(&last_cycle_count);
    if (!std::isfinite(measured_dt) || measured_dt < chassis_cfg.runtime.min_dt_s ||
        measured_dt > chassis_cfg.runtime.max_dt_s)
    {
        return chassis_cfg.runtime.nominal_dt_s;
    }
    return measured_dt;
}

void finish_period(ULONG started_at)
{
    const ULONG period_ticks = seconds_to_ticks(chassis_cfg.runtime.nominal_dt_s);
    const ULONG elapsed_ticks = tx_time_get() - started_at;
    if (elapsed_ticks < period_ticks)
    {
        tx_thread_sleep(period_ticks - elapsed_ticks);
    }
    else
    {
        tx_thread_relinquish();
    }
}

bool input_fresh(bool received, ULONG last_tick, ULONG now, float max_age_s)
{
    return received && (now - last_tick) <= seconds_to_ticks(max_age_s);
}

bool finite_ahrs(const ahrs::message& data)
{
    const float norm_sq =
        data.quaternion[0] * data.quaternion[0] + data.quaternion[1] * data.quaternion[1] +
        data.quaternion[2] * data.quaternion[2] + data.quaternion[3] * data.quaternion[3];
    if (!std::isfinite(norm_sq) || norm_sq < chassis_cfg.runtime.attitude_quaternion_norm_min ||
        norm_sq > chassis_cfg.runtime.attitude_quaternion_norm_max || !std::isfinite(data.yaw) ||
        !std::isfinite(data.pitch) || !std::isfinite(data.roll) || !std::isfinite(data.total_yaw) ||
        !std::isfinite(data.gyro_r) || !std::isfinite(data.gyro_p) || !std::isfinite(data.gyro_y))
    {
        return false;
    }

    for (float acceleration : data.accel)
    {
        if (!std::isfinite(acceleration))
        {
            return false;
        }
    }
    return true;
}

bool capture_motor(const motors::api& motor, motors::feedback& snapshot)
{
    snapshot = motor.get_feedback();
    return motor.status() == motors::state::online && snapshot.error_code == 0U &&
           std::isfinite(snapshot.position) && std::isfinite(snapshot.velocity) &&
           std::isfinite(snapshot.torque);
}

motor_feedback_frame capture_motor_feedback()
{
    // CAN feedback is written from ISR context. Keep all six copies inside one
    // short interrupt-disabled section so this cycle never mixes partial frames.
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    motor_feedback_frame frame{};
    frame.left_joint1_valid = capture_motor(left_joint1, frame.left_joint1);
    frame.left_joint4_valid = capture_motor(left_joint4, frame.left_joint4);
    frame.right_joint1_valid = capture_motor(right_joint1, frame.right_joint1);
    frame.right_joint4_valid = capture_motor(right_joint4, frame.right_joint4);
    frame.left_wheel_valid = capture_motor(left_wheel, frame.left_wheel);
    frame.right_wheel_valid = capture_motor(right_wheel, frame.right_wheel);
    TX_RESTORE

    frame.valid = frame.left_joint1_valid && frame.left_joint4_valid && frame.right_joint1_valid &&
                  frame.right_joint4_valid && frame.left_wheel_valid && frame.right_wheel_valid;
    return frame;
}

float wheel_velocity(const motor_feedback_frame& feedback)
{
    if (!feedback.left_wheel_valid || !feedback.right_wheel_valid)
    {
        return 0.0f;
    }

    return 0.5f *
           (chassis_cfg.motor_calibration.left_wheel_direction * feedback.left_wheel.velocity +
            chassis_cfg.motor_calibration.right_wheel_direction * feedback.right_wheel.velocity) *
           chassis_cfg.wheel_radius;
}

void relax_motor_outputs()
{
    left_joint1.relax();
    left_joint4.relax();
    right_joint1.relax();
    right_joint4.relax();
    left_wheel.relax();
    right_wheel.relax();
}

float wheel_torque_limit(const motors::lk9025& wheel)
{
    const float driver_limit =
        static_cast<float>(wheel.max_current) * std::fabs(wheel.torque_constant);
    return std::isfinite(driver_limit) ? clamp(driver_limit, 0.0f, chassis_cfg.max_wheel_torque)
                                       : 0.0f;
}

bool write_actuator_commands(const fsm_output& request, const power_state& power)
{
    if (!request.valid || request.relax)
    {
        return false;
    }

    float left_joint1_torque = 0.0f;
    float left_joint4_torque = 0.0f;
    float right_joint1_torque = 0.0f;
    float right_joint4_torque = 0.0f;
    if (!lpendulum.resolve_torque(request.left_leg_force, left_joint1_torque, left_joint4_torque) ||
        !rpendulum.resolve_torque(request.right_leg_force, right_joint1_torque,
                                  right_joint4_torque))
    {
        return false;
    }

    // Power/referee limiting is inserted here, after VMC and before the six
    // motor command buffers. No power model is invented in this migration.
    const float torque_scale = power.valid && std::isfinite(power.torque_scale)
                                   ? clamp(power.torque_scale, 0.0f, 1.0f)
                                   : 1.0f;
    left_joint1_torque *= torque_scale;
    left_joint4_torque *= torque_scale;
    right_joint1_torque *= torque_scale;
    right_joint4_torque *= torque_scale;

    lpendulum.write_torque(left_joint1_torque, left_joint4_torque);
    rpendulum.write_torque(right_joint1_torque, right_joint4_torque);

    const float left_limit = wheel_torque_limit(left_wheel);
    const float right_limit = wheel_torque_limit(right_wheel);
    left_wheel.set_torque(chassis_cfg.motor_calibration.left_wheel_direction *
                          clamp(request.left_wheel_torque * torque_scale, -left_limit, left_limit));
    right_wheel.set_torque(
        chassis_cfg.motor_calibration.right_wheel_direction *
        clamp(request.right_wheel_torque * torque_scale, -right_limit, right_limit));
    return true;
}

bool register_motors()
{
    left_joint4.offset = static_cast<float>(chassis_cfg.motor_calibration.left_joint4_offset);
    left_joint1.offset = static_cast<float>(chassis_cfg.motor_calibration.left_joint1_offset);
    right_joint4.offset = static_cast<float>(chassis_cfg.motor_calibration.right_joint4_offset);
    right_joint1.offset = static_cast<float>(chassis_cfg.motor_calibration.right_joint1_offset);
    relax_motor_outputs();

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

bool read_async_input(async_input_snapshot& destination)
{
    if (tx_mutex_get(&input_snapshot_lock, TX_NO_WAIT) != TX_SUCCESS)
    {
        return false;
    }
    destination = shared_input;
    tx_mutex_put(&input_snapshot_lock);
    return true;
}

void input_entry(ULONG /*arg*/)
{
    TX_SEMAPHORE* const ahrs_heartbeat = ahrs::service::instance().heartbeat_sem();
    async_input_snapshot local{};

    for (;;)
    {
        if (tx_semaphore_get(ahrs_heartbeat, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            tx_thread_relinquish();
            continue;
        }

        const ULONG now = tx_time_get();
        if (msg::read(input_ahrs_sub, local.attitude) == types::status::ok)
        {
            local.attitude_received = true;
            local.attitude_tick = now;
        }
        if (msg::read(input_remote_sub, local.remote) == types::status::ok)
        {
            local.remote_received = true;
            local.remote_tick = now;
        }

        if (tx_mutex_get(&input_snapshot_lock, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            shared_input = local;
            tx_mutex_put(&input_snapshot_lock);
        }
        ++wbr_control_task_telemetry.input_cycle_count;
    }
}

void control_entry(ULONG /*arg*/)
{
    std::uint32_t last_cycle_count = 0U;
    (void)bsp::dwt::delta_s(&last_cycle_count);
    ULONG last_alive_check_tick = 0U;
    async_input_snapshot async_input{};

    for (;;)
    {
        const ULONG started_at = tx_time_get();
        const float dt = task_dt(last_cycle_count);
        auto& telemetry = wbr_control_task_telemetry;
        auto& handler = motors::lkmotorhandler::instance();

        if (telemetry.alive_check_count == 0U ||
            (started_at - last_alive_check_tick) >=
                seconds_to_ticks(chassis_cfg.runtime.alive_check_period_s))
        {
            telemetry.motors_online = handler.alive_check();
            last_alive_check_tick = started_at;
            ++telemetry.alive_check_count;
        }

        const motor_feedback_frame motor_feedback = capture_motor_feedback();
        if (!read_async_input(async_input))
        {
            ++telemetry.snapshot_contention_count;
        }

        const bool ahrs_fresh =
            input_fresh(async_input.attitude_received, async_input.attitude_tick, started_at,
                        chassis_cfg.runtime.ahrs_max_age_s);
        const bool remote_fresh = input_fresh(async_input.remote_received, async_input.remote_tick,
                                              started_at, chassis_cfg.runtime.command_max_age_s);
        const ahrs::message& attitude = async_input.attitude;
        const bool attitude_valid = ahrs_fresh && finite_ahrs(attitude);

        remoter::state safe_remote = async_input.remote;
        if (!remote_fresh)
        {
            safe_remote.offline = true;
        }
        const function_feedback command_feedback{odometry.state().x};
        const chassis_command command = function.update(safe_remote, command_feedback, dt);

        const float chassis_wheel_velocity = wheel_velocity(motor_feedback);
        odometry_input odometry_sample{};
        for (int index = 0; index < 4; ++index)
        {
            odometry_sample.quaternion[index] = attitude.quaternion[index];
        }
        for (int index = 0; index < 3; ++index)
        {
            odometry_sample.acceleration[index] = attitude.accel[index];
        }
        odometry_sample.wheel_velocity = chassis_wheel_velocity;
        odometry_sample.yaw = attitude.yaw;
        odometry_sample.dt = dt;
        odometry_sample.valid =
            attitude_valid && motor_feedback.left_wheel_valid && motor_feedback.right_wheel_valid;
        const bool odometry_valid = odometry.update(odometry_sample);

        lpendulum.solve(motor_feedback.left_joint1, motor_feedback.left_joint4,
                        motor_feedback.left_joint1_valid && motor_feedback.left_joint4_valid,
                        attitude.pitch, attitude.gyro_p, odometry.state().a_z, dt);
        rpendulum.solve(motor_feedback.right_joint1, motor_feedback.right_joint4,
                        motor_feedback.right_joint1_valid && motor_feedback.right_joint4_valid,
                        attitude.pitch, attitude.gyro_p, odometry.state().a_z, dt);

        health_state health{};
        health.motors_online = telemetry.motors_online && motor_feedback.valid;
        health.attitude_fresh = attitude_valid;
        health.command_fresh = remote_fresh && command.valid;
        health.valid = health.motors_online && health.attitude_fresh && health.command_fresh &&
                       odometry_valid && lpendulum.link().valid && rpendulum.link().valid;

        power_state power{};
        fsm_input input{
            command, odometry.state(), attitude, lpendulum, rpendulum, health, power,
            dt,      health.valid,
        };
        const fsm_output request = chassis.step(input);

        const bool output_written = chassis_cfg.runtime.actuation_enabled && request.valid &&
                                    !request.relax && health.valid &&
                                    write_actuator_commands(request, power);
        if (!output_written)
        {
            relax_motor_outputs();
        }

        // This is the sole application-level LK batch commit.
        handler.send_control();

        if (request.reset_odometry)
        {
            odometry.reset();
        }
        if (request.reset_command)
        {
            function.reset();
        }

        const float support_force = lpendulum.link().valid && rpendulum.link().valid
                                        ? lpendulum.link().n + rpendulum.link().n
                                        : 0.0f;
        telemetry.ahrs_fresh = attitude_valid;
        telemetry.remoter_fresh = remote_fresh;
        telemetry.motor_snapshot_valid = motor_feedback.valid;
        telemetry.odometry_valid = odometry_valid;
        telemetry.left_leg_valid = lpendulum.link().valid;
        telemetry.right_leg_valid = rpendulum.link().valid;
        telemetry.fsm_valid = request.valid;
        telemetry.offground =
            request.state == chassis_state::OFFGROUND ||
            (request.state == chassis_state::JUMP && request.jump == jump_stage::INAIR);
        telemetry.actuation_enabled = chassis_cfg.runtime.actuation_enabled;
        telemetry.outputs_relaxed = !output_written;
        telemetry.dt = dt;
        telemetry.wheel_velocity = chassis_wheel_velocity;
        telemetry.support_force = support_force;
        telemetry.state_elapsed_s = chassis.state_elapsed_s();
        telemetry.state = request.state;
        telemetry.jump = request.jump;
        telemetry.odometry = odometry.state();
        telemetry.command = command;
        telemetry.request = request;
        telemetry.motor_feedback = motor_feedback;
        telemetry.left_link = lpendulum.link();
        telemetry.right_link = rpendulum.link();
        if (!health.valid || !request.valid)
        {
            ++telemetry.invalid_cycle_count;
        }
        ++telemetry.cycle_count;

        finish_period(started_at);
    }
}

bool create_input_snapshot()
{
    if (tx_mutex_create(&input_snapshot_lock, const_cast<CHAR*>("wbr_input"), TX_NO_INHERIT) !=
        TX_SUCCESS)
    {
        return false;
    }

    input_ahrs_sub = msg::subscribe<ahrs::message>();
    input_remote_sub = msg::subscribe<remoter::state>();
    return input_ahrs_sub.valid() && input_remote_sub.valid();
}

bool create_control_threads(control_task_telemetry& telemetry)
{
    const UINT input_status = tx_thread_create(
        &input_thread, const_cast<CHAR*>("wbr_input"), input_entry, 0U, input_stack,
        sizeof(input_stack), chassis_cfg.runtime.input_thread_priority,
        chassis_cfg.runtime.input_thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START);
    telemetry.input_thread_started = input_status == TX_SUCCESS;
    if (!telemetry.input_thread_started)
    {
        return false;
    }

    const UINT control_status = tx_thread_create(
        &control_thread, const_cast<CHAR*>("wbr_control"), control_entry, 0U, control_stack,
        sizeof(control_stack), chassis_cfg.runtime.control_thread_priority,
        chassis_cfg.runtime.control_thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START);
    telemetry.control_thread_started = control_status == TX_SUCCESS;
    return telemetry.control_thread_started;
}

} // namespace

bool start_control_task() noexcept
{
    auto& telemetry = wbr_control_task_telemetry;
    if (tasks_started)
    {
        return true;
    }

    telemetry = {};
    telemetry.initialization_attempted = true;
    telemetry.outputs_relaxed = true;

    telemetry.dwt_initialized = bsp::dwt::initialized() || bsp::dwt::init() == types::status::ok;
    if (!telemetry.dwt_initialized)
    {
        return false;
    }

    // AHRS and remoter own their low-level threads and use module defaults.
    telemetry.ahrs_initialized = ahrs::service::instance().init();
    if (!telemetry.ahrs_initialized)
    {
        return false;
    }
    telemetry.remoter_initialized = remoter::service::instance().init();
    if (!telemetry.remoter_initialized)
    {
        return false;
    }

    telemetry.subscribers_ready = create_input_snapshot();
    telemetry.snapshot_ready = telemetry.subscribers_ready;
    if (!telemetry.subscribers_ready)
    {
        return false;
    }

    telemetry.motors_registered = register_motors();
    if (!telemetry.motors_registered)
    {
        return false;
    }

    telemetry.thread_started = create_control_threads(telemetry);
    tasks_started = telemetry.thread_started;
    return tasks_started;
}

} // namespace wbr::control
