#include "remoter_demo.hpp"

#include "config.hpp"
#include "demo_debug.hpp"
#include "msg.hpp"
#include "remoter.hpp"
#include "tx_api.h"

#include <cstdint>
#include <cstring>

namespace demo::remoter
{
namespace
{

enum stage : std::uint32_t
{
    service_initialized = 1U << 0U,
    subscriber_created = 1U << 1U,
    monitor_thread_started = 1U << 2U,
    data_received = 1U << 3U,
    remoter_online = 1U << 4U,
};

enum failure : std::uint32_t
{
    service_init_failed = 1U << 0U,
    subscribe_failed = 1U << 1U,
    thread_create_failed = 1U << 2U,
    online_timeout = 1U << 3U,
};

constexpr ULONG monitor_period_ticks = 5U;
constexpr ULONG online_timeout_ticks = 500U;

TX_THREAD monitor_thread{};
alignas(8) std::uint8_t monitor_stack[768]{};
bool monitor_started = false;
msg::subscriber remoter_sub{};
ULONG started_at = 0;

std::uint16_t key_bits(const ::remoter::key_state& key) noexcept
{
    std::uint16_t bits = 0;
    std::memcpy(&bits, &key, sizeof(bits));
    return bits;
}

void sync_debug(const ::remoter::state& data, std::uint32_t stages, bool timed_out) noexcept
{
    auto& state = demo::debug::debug_instance.remoter_unit;
    if (!data.offline)
    {
        stages |= remoter_online;
    }
    if (data.update_count > 0U)
    {
        stages |= data_received;
    }

    state.stage_mask = stages;
    state.last_step = stages;
    state.offline = data.offline;
    state.source = static_cast<std::uint32_t>(data.active_source);
    state.left_sw = static_cast<std::uint32_t>(data.left_sw);
    state.right_sw = static_cast<std::uint32_t>(data.right_sw);
    state.last_left_sw = static_cast<std::uint32_t>(data.last_left_sw);
    state.last_right_sw = static_cast<std::uint32_t>(data.last_right_sw);
    state.key_bits = key_bits(data.key);
    state.last_key_bits = key_bits(data.last_key);
    state.update_count = data.update_count;
    state.right_x = data.right_x;
    state.right_y = data.right_y;
    state.left_x = data.left_x;
    state.left_y = data.left_y;
    state.mouse_x = data.mouse_x;
    state.mouse_y = data.mouse_y;

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
    if (timed_out && data.offline)
    {
        state.failure_mask |= online_timeout;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    state.passed = state.failure_mask == 0U && !data.offline;
    state.passed_count = state.passed ? state.total_count : 0U;
}

void monitor_entry(ULONG /*arg*/)
{
    ::remoter::state data{};
    std::uint32_t stages = service_initialized | subscriber_created | monitor_thread_started;
    for (;;)
    {
        if (msg::read(remoter_sub, data) == types::status::ok)
        {
            stages |= data_received;
        }
        sync_debug(data, stages, (tx_time_get() - started_at) > online_timeout_ticks);
        tx_thread_sleep(monitor_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.remoter_unit;
    state = {};
    state.started = true;
    state.total_count = 5U;
    started_at = tx_time_get();

    ::remoter::config cfg{};
    cfg.dr16.thread_priority = params::remoter::thread_priority;
    cfg.dr16.rx_timeout_ticks = params::remoter::rx_timeout_ticks;
    cfg.vt03.thread_priority = params::remoter::thread_priority;
    cfg.thread_priority = params::remoter::thread_priority + 1U;

    if (!::remoter::service::instance().init(cfg))
    {
        state.failure_mask = service_init_failed;
        state.failed_count = 1U;
        return;
    }

    remoter_sub = msg::subscribe<::remoter::state>();
    if (!remoter_sub.valid())
    {
        state.failure_mask = subscribe_failed;
        state.failed_count = 1U;
        return;
    }

    if (!monitor_started)
    {
        const UINT status = tx_thread_create(&monitor_thread, const_cast<CHAR*>("remoter_demo"),
                                             monitor_entry, 0, monitor_stack,
                                             sizeof(monitor_stack), cfg.thread_priority + 1U,
                                             cfg.thread_priority + 1U, TX_NO_TIME_SLICE,
                                             TX_AUTO_START);
        if (status != TX_SUCCESS)
        {
            state.failure_mask = thread_create_failed;
            state.failed_count = 1U;
            return;
        }
        monitor_started = true;
    }

    sync_debug({}, service_initialized | subscriber_created | monitor_thread_started, false);
}

} // namespace demo::remoter
