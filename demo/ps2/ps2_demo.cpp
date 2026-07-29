#include "ps2_demo.hpp"

#include "config.hpp"
#include "demo_debug.hpp"
#include "msg.hpp"
#include "remoter.hpp"
#include "tx_api.h"

#include <cstdint>

namespace demo::ps2
{
namespace
{

enum stage : std::uint32_t
{
    source_enabled = 1U << 0U,
    service_initialized = 1U << 1U,
    raw_subscriber_created = 1U << 2U,
    generic_subscriber_created = 1U << 3U,
    monitor_thread_started = 1U << 4U,
    raw_data_received = 1U << 5U,
    connected_frame_received = 1U << 6U,
    generic_data_received = 1U << 7U,
};

enum failure : std::uint32_t
{
    source_not_enabled = 1U << 0U,
    service_init_failed = 1U << 1U,
    raw_subscribe_failed = 1U << 2U,
    generic_subscribe_failed = 1U << 3U,
    thread_create_failed = 1U << 4U,
    receiver_data_timeout = 1U << 5U,
    receiver_offline = 1U << 6U,
};

constexpr ULONG monitor_period_ticks = 5U;
constexpr ULONG receiver_startup_timeout_ticks = 3000U;

TX_THREAD monitor_thread{};
alignas(8) std::uint8_t monitor_stack[768]{};
bool monitor_started = false;
msg::subscriber raw_sub{};
msg::subscriber generic_sub{};
ULONG started_at = 0;

::remoter::ps2_state latest_raw{};
::remoter::state latest_generic{};
std::uint32_t stages = 0;
std::uint32_t raw_update_count = 0;
std::uint32_t connected_frame_count = 0;
std::uint32_t remote_disconnected_count = 0;
std::uint32_t receiver_offline_count = 0;
std::uint32_t previous_frame_tick = 0;
std::uint32_t last_frame_period_ticks = 0;
std::uint32_t min_frame_period_ticks = 0;
std::uint32_t max_frame_period_ticks = 0;
std::uint16_t last_pressed = 0;
std::uint16_t last_released = 0;
std::uint16_t pressed_seen_mask = 0;
std::uint16_t released_seen_mask = 0;
std::uint32_t press_event_count = 0;
std::uint32_t release_event_count = 0;
::remoter::ps2_link_state previous_link = ::remoter::ps2_link_state::receiver_offline;
bool link_seen = false;

void record_raw_update(const ::remoter::ps2_state& data) noexcept
{
    latest_raw = data;
    ++raw_update_count;
    stages |= raw_data_received;

    if (!link_seen || data.link != previous_link)
    {
        if (data.link == ::remoter::ps2_link_state::remote_disconnected)
        {
            ++remote_disconnected_count;
        }
        else if (data.link == ::remoter::ps2_link_state::receiver_offline)
        {
            ++receiver_offline_count;
        }
        previous_link = data.link;
        link_seen = true;
    }

    if (data.link != ::remoter::ps2_link_state::connected)
    {
        return;
    }

    stages |= connected_frame_received;
    ++connected_frame_count;
    if (data.pressed != 0U)
    {
        last_pressed = data.pressed;
        pressed_seen_mask |= data.pressed;
        ++press_event_count;
    }
    if (data.released != 0U)
    {
        last_released = data.released;
        released_seen_mask |= data.released;
        ++release_event_count;
    }
    if (previous_frame_tick != 0U)
    {
        last_frame_period_ticks = data.last_update_tick - previous_frame_tick;
        if (min_frame_period_ticks == 0U || last_frame_period_ticks < min_frame_period_ticks)
        {
            min_frame_period_ticks = last_frame_period_ticks;
        }
        if (last_frame_period_ticks > max_frame_period_ticks)
        {
            max_frame_period_ticks = last_frame_period_ticks;
        }
    }
    previous_frame_tick = data.last_update_tick;
}

void sync_debug(bool startup_timed_out) noexcept
{
    auto& state = demo::debug::debug_instance.ps2_unit;
    state.stage_mask = stages;
    state.last_step = stages;
    state.observed_count = raw_update_count;
    state.generic_offline = latest_generic.offline;
    state.link = static_cast<std::uint32_t>(latest_raw.link);
    state.generic_source = static_cast<std::uint32_t>(latest_generic.active_source);
    state.generic_update_count = latest_generic.update_count;
    state.raw_update_count = raw_update_count;
    state.connected_frame_count = connected_frame_count;
    state.remote_disconnected_count = remote_disconnected_count;
    state.receiver_offline_count = receiver_offline_count;
    state.last_update_tick = latest_raw.last_update_tick;
    state.last_frame_period_ticks = last_frame_period_ticks;
    state.min_frame_period_ticks = min_frame_period_ticks;
    state.max_frame_period_ticks = max_frame_period_ticks;

    state.buttons = latest_raw.buttons;
    state.pressed = latest_raw.pressed;
    state.released = latest_raw.released;
    state.last_pressed = last_pressed;
    state.last_released = last_released;
    state.pressed_seen_mask = pressed_seen_mask;
    state.released_seen_mask = released_seen_mask;
    state.press_event_count = press_event_count;
    state.release_event_count = release_event_count;
    state.raw_left_x = latest_raw.left_x;
    state.raw_left_y = latest_raw.left_y;
    state.raw_right_x = latest_raw.right_x;
    state.raw_right_y = latest_raw.right_y;
    state.left_x = latest_generic.left_x;
    state.left_y = latest_generic.left_y;
    state.right_x = latest_generic.right_x;
    state.right_y = latest_generic.right_y;

    state.failure_mask = 0;
    if ((stages & source_enabled) == 0U)
    {
        state.failure_mask |= source_not_enabled;
    }
    if ((stages & service_initialized) == 0U)
    {
        state.failure_mask |= service_init_failed;
    }
    if ((stages & raw_subscriber_created) == 0U)
    {
        state.failure_mask |= raw_subscribe_failed;
    }
    if ((stages & generic_subscriber_created) == 0U)
    {
        state.failure_mask |= generic_subscribe_failed;
    }
    if ((stages & monitor_thread_started) == 0U)
    {
        state.failure_mask |= thread_create_failed;
    }
    if (startup_timed_out && (stages & raw_data_received) == 0U)
    {
        state.failure_mask |= receiver_data_timeout;
    }
    if (startup_timed_out && latest_raw.link == ::remoter::ps2_link_state::receiver_offline)
    {
        state.failure_mask |= receiver_offline;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    state.passed =
        state.failure_mask == 0U &&
        latest_raw.link == ::remoter::ps2_link_state::connected &&
        !latest_generic.offline &&
        latest_generic.active_source == ::remoter::source::ps2;
    state.passed_count = state.passed ? state.total_count : 0U;
}

void monitor_entry(ULONG /*arg*/)
{
    for (;;)
    {
        ::remoter::ps2_state raw{};
        if (msg::read(raw_sub, raw) == types::status::ok)
        {
            record_raw_update(raw);
        }

        ::remoter::state generic{};
        if (msg::read(generic_sub, generic) == types::status::ok)
        {
            latest_generic = generic;
            stages |= generic_data_received;
        }

        sync_debug((tx_time_get() - started_at) > receiver_startup_timeout_ticks);
        tx_thread_sleep(monitor_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.ps2_unit;
    if (monitor_started)
    {
        state.started = true;
        return;
    }

    state = {};
    state.started = true;
    state.total_count = 8U;
    started_at = tx_time_get();
    latest_raw = {};
    latest_generic = {};
    stages = 0;
    raw_update_count = 0;
    connected_frame_count = 0;
    remote_disconnected_count = 0;
    receiver_offline_count = 0;
    previous_frame_tick = 0;
    last_frame_period_ticks = 0;
    min_frame_period_ticks = 0;
    max_frame_period_ticks = 0;
    last_pressed = 0;
    last_released = 0;
    pressed_seen_mask = 0;
    released_seen_mask = 0;
    press_event_count = 0;
    release_event_count = 0;
    previous_link = ::remoter::ps2_link_state::receiver_offline;
    link_seen = false;

    if (!::config::feature::enable_ps2)
    {
        sync_debug(false);
        return;
    }
    stages = source_enabled;

    ::remoter::config cfg{};
    cfg.ps2.thread_priority = params::remoter::thread_priority;
    cfg.ps2.receiver_offline_timeout_ticks = params::remoter::ps2_offline_timeout_ticks;
    cfg.ps2.frame_timeout_ticks = params::remoter::ps2_frame_timeout_ticks;
    cfg.ps2.deadzone = params::remoter::ps2_deadzone;
    cfg.thread_priority = params::remoter::thread_priority + 1U;

    if (!::remoter::service::instance().init(cfg))
    {
        sync_debug(false);
        return;
    }
    stages |= service_initialized;

    raw_sub = msg::subscribe<::remoter::ps2_state>();
    if (!raw_sub.valid())
    {
        sync_debug(false);
        return;
    }
    stages |= raw_subscriber_created;

    generic_sub = msg::subscribe<::remoter::state>();
    if (!generic_sub.valid())
    {
        sync_debug(false);
        return;
    }
    stages |= generic_subscriber_created;

    const UINT status = tx_thread_create(&monitor_thread, const_cast<CHAR*>("ps2_demo"),
                                         monitor_entry, 0, monitor_stack,
                                         sizeof(monitor_stack), cfg.thread_priority + 1U,
                                         cfg.thread_priority + 1U, TX_NO_TIME_SLICE,
                                         TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        sync_debug(false);
        return;
    }

    monitor_started = true;
    stages |= monitor_thread_started;
    sync_debug(false);
}

} // namespace demo::ps2
