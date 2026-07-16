#include "motor_demo.hpp"

#include "demo_debug.hpp"
#include "djimotorhandler.hpp"
#include "djimotors.hpp"
#include "dmmotorhandler.hpp"
#include "dmmotors.hpp"
#include "robot_config.hpp"
#include "tx_api.h"

#include <cstdint>
#include <type_traits>

namespace demo::motor
{
namespace
{

enum stage : std::uint32_t
{
    dji_registered = 1U << 0U,
    dm_registered = 1U << 1U,
    control_thread_started = 1U << 2U,
    dm_enable_requested = 1U << 3U,
    control_tx_running = 1U << 4U,
    dji_online = 1U << 5U,
    dm_online = 1U << 6U,
    dm_enabled = 1U << 7U,
};

enum failure : std::uint32_t
{
    dji_register_failed = 1U << 0U,
    dm_register_failed = 1U << 1U,
    thread_create_failed = 1U << 2U,
    dji_offline_timeout = 1U << 3U,
    dm_offline_timeout = 1U << 4U,
    dm_enable_timeout = 1U << 5U,
};

constexpr ULONG control_period_ticks = 2U;
constexpr ULONG online_timeout_ticks = 1000U;
constexpr ULONG alive_check_period_ticks = 100U;
constexpr bool require_dji_online = false;
constexpr int16_t dji_test_current = 3000;
constexpr float dm_mit_position = 0.0f;
constexpr float dm_mit_velocity = 0.0f;
constexpr float dm_mit_kp = 0.0f;
constexpr float dm_mit_kd = 0.0f;
constexpr float dm_mit_torque = 0.2f;

template <robot::motors::model Model>
struct dji_motor_type;

template <>
struct dji_motor_type<robot::motors::model::dji_m2006>
{
    using type = motors::m2006;
};

template <>
struct dji_motor_type<robot::motors::model::dji_m3508>
{
    using type = motors::m3508;
};

template <>
struct dji_motor_type<robot::motors::model::dji_gm6020>
{
    using type = motors::gm6020;
};

template <>
struct dji_motor_type<robot::motors::model::dji_xroll>
{
    using type = motors::xroll;
};

template <robot::motors::model Model>
struct dm_motor_type;

template <>
struct dm_motor_type<robot::motors::model::dm_dm4310>
{
    using type = motors::dm4310;
};

template <>
struct dm_motor_type<robot::motors::model::dm_dm8009p>
{
    using type = motors::dm8009p;
};

using motor1_type = typename dji_motor_type<robot::motors::motor1_model>::type;
using motor2_type = typename dm_motor_type<robot::motors::motor2_model>::type;

static_assert(std::is_base_of_v<motors::djimotor, motor1_type>,
              "motor demo motor1 must be a DJI motor model");
static_assert(std::is_base_of_v<motors::dmmotor, motor2_type>,
              "motor demo motor2 must be a DM motor model");

motor1_type motor1{robot::motors::motor1};
motor2_type motor2{robot::motors::motor2};

TX_THREAD control_thread{};
alignas(8) std::uint8_t control_stack[1024]{};
bool registered = false;
bool thread_started = false;
ULONG last_alive_check_tick = 0U;
uint32_t dji_last_alive = 0U;
uint32_t dm_last_alive = 0U;
bool dji_recently_online = false;
bool dm_recently_online = false;
bool dm_enable_latched = false;

void set_dm_mit_command() noexcept
{
    motor2.set_mit(dm_mit_position, dm_mit_velocity, dm_mit_kp, dm_mit_kd, dm_mit_torque);
}

void sync_debug(std::uint32_t stages, std::uint32_t send_count, bool timed_out) noexcept
{
    auto& state = demo::debug::debug_instance.motor_unit;
    const bool dji_has_feedback = motor1.alive > 0U;
    const bool dm_has_feedback = motor2.alive > 0U;
    const bool dji_is_online = dji_recently_online || dji_has_feedback;
    const bool dm_is_online = dm_recently_online || dm_has_feedback;
    const bool dm_is_enabled = dm_enable_latched ||
                               motor2.raw.err == motors::dmmotor::error::enable;

    if (dji_is_online)
    {
        stages |= dji_online;
    }
    if (dm_is_online)
    {
        stages |= dm_online;
    }
    if (dm_is_enabled)
    {
        stages |= dm_enabled;
    }

    state.stage_mask = stages;
    state.last_step = stages;
    state.total_count = send_count;
    state.observed_count = motor1.alive + motor2.alive;
    state.value_a = static_cast<float>(motor2.last_tx_id);
    state.value_b = static_cast<float>(motor2.fdb.velocity);
    state.value_c = static_cast<float>(motor2.fdb.error_code);

    state.failure_mask = 0U;
    if ((stages & dji_registered) == 0U)
    {
        state.failure_mask |= dji_register_failed;
    }
    if ((stages & dm_registered) == 0U)
    {
        state.failure_mask |= dm_register_failed;
    }
    if ((stages & control_thread_started) == 0U)
    {
        state.failure_mask |= thread_create_failed;
    }
    if (require_dji_online && timed_out && !dji_is_online)
    {
        state.failure_mask |= dji_offline_timeout;
    }
    if (timed_out && !dm_is_online)
    {
        state.failure_mask |= dm_offline_timeout;
    }
    if (timed_out && dm_is_online && !dm_is_enabled)
    {
        state.failure_mask |= dm_enable_timeout;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    const bool dji_requirement_met = !require_dji_online || dji_is_online;
    state.passed = (state.failure_mask == 0U) && dji_requirement_met && dm_is_online && dm_is_enabled;
    state.passed_count = state.passed ? state.total_count : 0U;
}

void control_entry(ULONG started_at_arg)
{
    std::uint32_t send_count = 0U;
    std::uint32_t stages = dji_registered | dm_registered | control_thread_started;
    const ULONG started_at = started_at_arg;
    auto& dji_handler = motors::djimotorhandler::instance();
    auto& dm_handler = motors::dmmotorhandler::instance();
    stages |= dm_enable_requested;
    dm_handler.enable_block(&motor2);
    for (;;)
    {

        motor1.set_current(dji_test_current);
        dji_handler.send_control();

        set_dm_mit_command();
        dm_handler.send_control();
        ++send_count;
        stages |= control_tx_running;

        if ((tx_time_get() - last_alive_check_tick) >= alive_check_period_ticks)
        {
            dji_recently_online = motor1.alive != dji_last_alive;
            dm_recently_online = motor2.alive != dm_last_alive;
            dji_last_alive = motor1.alive;
            dm_last_alive = motor2.alive;
            last_alive_check_tick = tx_time_get();
        }
        sync_debug(stages, send_count, (tx_time_get() - started_at) > online_timeout_ticks);

        tx_thread_sleep(control_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.motor_unit;
    state = {};
    state.started = true;

    if (!registered)
    {
        std::uint32_t stages = 0U;
        if (motors::djimotorhandler::instance().register_motor(motor1))
        {
            stages |= dji_registered;
        }
        if (motors::dmmotorhandler::instance().register_motor(motor2))
        {
            stages |= dm_registered;
        }
        registered = (stages & (dji_registered | dm_registered)) == (dji_registered | dm_registered);
        sync_debug(stages, 0U, false);
        if (!registered)
        {
            return;
        }
    }

    if (!thread_started)
    {
        const UINT status = tx_thread_create(&control_thread, const_cast<CHAR*>("motor_demo"),
                                             control_entry, tx_time_get(),
                                             control_stack, sizeof(control_stack),
                                             6U, 6U, TX_NO_TIME_SLICE, TX_AUTO_START);
        if (status != TX_SUCCESS)
        {
            state.failure_mask |= thread_create_failed;
            state.failed_count = 1U;
            return;
        }
        thread_started = true;
    }

    sync_debug(dji_registered | dm_registered | control_thread_started, 0U, false);
}

} // namespace demo::motor
