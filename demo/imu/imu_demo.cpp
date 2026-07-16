#include "imu_demo.hpp"

#include "ahrs.hpp"
#include "config.hpp"
#include "demo_debug.hpp"
#include "msg.hpp"

#include "tx_api.h"

#include <cmath>
#include <cstring>

namespace demo::imu
{
namespace
{

enum stage : std::uint32_t
{
    service_initialized = 1U << 0U,
    subscriber_created = 1U << 1U,
    monitor_thread_started = 1U << 2U,
    data_received = 1U << 3U,
    quaternion_ok = 1U << 4U,
};

enum failure : std::uint32_t
{
    service_init_failed = 1U << 0U,
    subscribe_failed = 1U << 1U,
    thread_create_failed = 1U << 2U,
    data_timeout = 1U << 3U,
    quaternion_invalid = 1U << 4U,
};

constexpr std::uint32_t pass_stage_mask =
    service_initialized | subscriber_created | monitor_thread_started | data_received | quaternion_ok;

constexpr ULONG monitor_period_ticks = 50;
constexpr std::uint32_t warmup_ticks = 12000;

TX_THREAD monitor_thread{};
alignas(8) std::uint8_t monitor_stack[1024]{};
bool monitor_started = false;
msg::subscriber ahrs_sub{};
ULONG started_at = 0;
std::uint32_t observed_count = 0;

bool quaternion_valid(const ahrs::message& data) noexcept
{
    const float norm =
        data.quaternion[0] * data.quaternion[0] +
        data.quaternion[1] * data.quaternion[1] +
        data.quaternion[2] * data.quaternion[2] +
        data.quaternion[3] * data.quaternion[3];
    return std::isfinite(norm) && norm > 0.8f && norm < 1.2f;
}

void sync_debug(const ahrs::message& data, std::uint32_t stages, bool timed_out) noexcept
{
    auto& state = demo::debug::debug_instance.imu_unit;
    const bool data_seen = (stages & data_received) != 0U;
    const bool valid_quaternion = data_seen && quaternion_valid(data);
    if (valid_quaternion)
    {
        stages |= quaternion_ok;
    }

    state.stage_mask = stages;
    state.last_step = stages;
    state.observed_count = observed_count;
    state.value_a = data.yaw;
    state.value_b = data.pitch;
    state.value_c = data.roll;
    state.ahrs_solved = data_seen;
    std::memcpy(state.quaternion, data.quaternion, sizeof(state.quaternion));
    state.yaw = data.yaw;
    state.pitch = data.pitch;
    state.roll = data.roll;
    state.total_yaw = data.total_yaw;

    state.failure_mask = 0;
    if ((stages & service_initialized) == 0U)
    {
        state.failure_mask |= service_init_failed;
    }
    if ((stages & subscriber_created) == 0U)
    {
        state.failure_mask |= subscribe_failed;
    }
    if ((stages & monitor_thread_started) == 0U)
    {
        state.failure_mask |= thread_create_failed;
    }
    if (timed_out && !data_seen)
    {
        state.failure_mask |= data_timeout;
    }
    if (data_seen && !valid_quaternion)
    {
        state.failure_mask |= quaternion_invalid;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    state.passed = (stages & pass_stage_mask) == pass_stage_mask && state.failure_mask == 0U;
    state.passed_count = state.passed ? state.total_count : 0U;
}

void monitor_entry(ULONG /*arg*/)
{
    ahrs::message data{};
    std::uint32_t stages = service_initialized | subscriber_created | monitor_thread_started;
    for (;;)
    {
        if (msg::read(ahrs_sub, data) == types::status::ok)
        {
            stages |= data_received;
            ++observed_count;
        }
        sync_debug(data, stages, (tx_time_get() - started_at) > warmup_ticks);
        tx_thread_sleep(monitor_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.imu_unit;
    state = {};
    state.started = true;
    state.total_count = 5U;
    observed_count = 0;
    started_at = tx_time_get();

    ahrs::config cfg{};
    cfg.imu_offset_x = params::ahrs::imu_offset_x;
    cfg.imu_thread_priority = params::ahrs::imu_thread_priority;
    cfg.temp_thread_priority = params::ahrs::temp_thread_priority;
    cfg.target_temp = params::ahrs::target_temp;
    cfg.temperature_control_enabled = true;

    if (!ahrs::service::instance().init(cfg))
    {
        state.failure_mask = service_init_failed;
        state.failed_count = 1U;
        state.passed = false;
        return;
    }

    ahrs_sub = msg::subscribe<ahrs::message>();
    if (!ahrs_sub.valid())
    {
        state.failure_mask = subscribe_failed;
        state.failed_count = 1U;
        return;
    }

    if (!monitor_started)
    {
        if (tx_thread_create(&monitor_thread, const_cast<CHAR*>("imu_unit_mon"), monitor_entry, 0,
                             monitor_stack, sizeof(monitor_stack), cfg.imu_thread_priority + 1U,
                             cfg.imu_thread_priority + 1U, TX_NO_TIME_SLICE, TX_AUTO_START) == TX_SUCCESS)
        {
            monitor_started = true;
        }
        else
        {
            state.failure_mask = thread_create_failed;
            state.failed_count = 1U;
            return;
        }
    }

    sync_debug({}, service_initialized | subscriber_created | monitor_thread_started, false);
}

} // namespace demo::imu
