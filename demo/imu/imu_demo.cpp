#include "imu_demo.hpp"

#include "ahrs.hpp"
#include "config.hpp"
#include "demo_debug.hpp"
#include "runtime_monitor.hpp"

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
    resources_created = 1U << 1U,
    imu_configured = 1U << 2U,
    chip_id_ok = 1U << 3U,
    temperature_ready = 1U << 4U,
    calibrated = 1U << 5U,
    read_ok = 1U << 6U,
    solved = 1U << 7U,
};

enum failure : std::uint32_t
{
    service_init_failed = 1U << 0U,
    chip_id_failed = 1U << 1U,
    temperature_not_ready = 1U << 2U,
    calibrate_failed = 1U << 3U,
    read_failed = 1U << 4U,
    solve_failed = 1U << 5U,
    quaternion_invalid = 1U << 6U,
};

constexpr std::uint32_t pass_stage_mask =
    service_initialized | resources_created | imu_configured | chip_id_ok |
    temperature_ready | calibrated | read_ok | solved;

constexpr ULONG monitor_period_ticks = 50;
constexpr std::uint32_t warmup_ticks = 12000;

TX_THREAD monitor_thread{};
alignas(8) std::uint8_t monitor_stack[1024]{};
bool monitor_started = false;
runtime::monitor monitor_runtime{};

bool quaternion_valid(const ahrs::telemetry& diag) noexcept
{
    const float norm =
        diag.quaternion[0] * diag.quaternion[0] +
        diag.quaternion[1] * diag.quaternion[1] +
        diag.quaternion[2] * diag.quaternion[2] +
        diag.quaternion[3] * diag.quaternion[3];
    return std::isfinite(norm) && norm > 0.8f && norm < 1.2f;
}

void sync_debug(const ahrs::telemetry& diag, bool timed_out) noexcept
{
    auto& state = demo::debug::debug_instance.imu_unit;
    std::uint32_t stages = 0;
    if (diag.initialized)
    {
        stages |= service_initialized;
    }
    if (diag.resources_created)
    {
        stages |= resources_created;
    }
    if (diag.imu_configured)
    {
        stages |= imu_configured;
    }
    if (diag.accel_chip_ok && diag.gyro_chip_ok)
    {
        stages |= chip_id_ok;
    }
    if (diag.temperature_ready)
    {
        stages |= temperature_ready;
    }
    if (diag.calibrated)
    {
        stages |= calibrated;
    }
    if (diag.read_ok)
    {
        stages |= read_ok;
    }
    if (diag.solved)
    {
        stages |= solved;
    }

    state.stage_mask = stages;
    state.observed_count = diag.update_count;
    state.value_a = diag.temperature;
    state.value_b = diag.accel_norm;
    state.value_c = diag.gyro_norm;
    state.ahrs_solved = diag.solved;
    state.ahrs_dt_s = diag.dt_s;
    state.ahrs_dt_min_s = diag.dt_min_s;
    state.ahrs_dt_max_s = diag.dt_max_s;
    state.ahrs_loop_runtime_us = diag.loop_runtime_us;
    state.ahrs_loop_runtime_max_us = diag.loop_runtime_max_us;
    state.ahrs_loop_runtime_avg_us = diag.loop_runtime_avg_us;
    state.ahrs_loop_runtime_overruns = diag.loop_runtime_overruns;
    std::memcpy(state.quaternion, diag.quaternion, sizeof(state.quaternion));
    state.yaw = diag.yaw;
    state.pitch = diag.pitch;
    state.roll = diag.roll;
    state.total_yaw = diag.total_yaw;
    state.passed = (stages & pass_stage_mask) == pass_stage_mask && quaternion_valid(diag);

    state.failure_mask = 0;
    if (!diag.initialized)
    {
        state.failure_mask |= service_init_failed;
    }
    if (diag.imu_configured && !(diag.accel_chip_ok && diag.gyro_chip_ok))
    {
        state.failure_mask |= chip_id_failed;
    }
    if (timed_out && !diag.temperature_ready)
    {
        state.failure_mask |= temperature_not_ready;
    }
    if (diag.temperature_ready && !diag.calibrated)
    {
        state.failure_mask |= calibrate_failed;
    }
    if (diag.calibrated && !diag.read_ok)
    {
        state.failure_mask |= read_failed;
    }
    if (diag.read_ok && !diag.solved)
    {
        state.failure_mask |= solve_failed;
    }
    if (diag.solved && !quaternion_valid(diag))
    {
        state.failure_mask |= quaternion_invalid;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    state.passed_count = state.passed ? state.total_count : 0U;
    state.last_step = stages;
}

void monitor_entry(ULONG /*arg*/)
{
    const ULONG started_at = tx_time_get();
    for (;;)
    {
        monitor_runtime.begin();
        const auto& diag = ahrs::service::instance().diagnostics();
        const bool timed_out = (tx_time_get() - started_at) > warmup_ticks;
        sync_debug(diag, timed_out);
        monitor_runtime.end();
        const auto& stats = monitor_runtime.stats();
        auto& state = demo::debug::debug_instance.imu_unit;
        state.monitor_runtime_us = stats.last_us;
        state.monitor_runtime_max_us = stats.max_us;
        state.monitor_runtime_avg_us = stats.average_us();
        tx_thread_sleep(monitor_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.imu_unit;
    state = {};
    state.started = true;
    state.total_count = 8U;

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

    if (!monitor_started)
    {
        if (tx_thread_create(&monitor_thread, const_cast<CHAR*>("imu_unit_mon"), monitor_entry, 0,
                             monitor_stack, sizeof(monitor_stack), cfg.imu_thread_priority + 1U,
                             cfg.imu_thread_priority + 1U, TX_NO_TIME_SLICE, TX_AUTO_START) == TX_SUCCESS)
        {
            monitor_started = true;
        }
    }

    sync_debug(ahrs::service::instance().diagnostics(), false);
}

} // namespace demo::imu
