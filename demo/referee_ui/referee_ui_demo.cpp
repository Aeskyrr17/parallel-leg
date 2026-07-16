#include "referee_ui_demo.hpp"

#include "config.hpp"
#include "demo_debug.hpp"
#include "msg.hpp"
#include "referee.hpp"
#include "tx_api.h"
#include "ui.hpp"

#include <cstdio>
#include <cstdint>

namespace demo::referee_ui
{
namespace
{

enum stage : std::uint32_t
{
    referee_initialized = 1U << 0U,
    ui_initialized = 1U << 1U,
    subscriber_created = 1U << 2U,
    monitor_thread_started = 1U << 3U,
    frame_received = 1U << 4U,
    ui_updated = 1U << 5U,
};

enum failure : std::uint32_t
{
    referee_init_failed = 1U << 0U,
    ui_init_failed = 1U << 1U,
    subscribe_failed = 1U << 2U,
    thread_create_failed = 1U << 3U,
};

constexpr ULONG monitor_period_ticks = 20U;
constexpr std::uint16_t sender_id = RobotId::RedInfantry_3;
constexpr std::uint16_t receiver_id = RobotId::ClientRedInfantry_3;

TX_THREAD monitor_thread{};
alignas(8) std::uint8_t monitor_stack[1024]{};
bool monitor_started = false;
bool objects_created = false;
msg::subscriber referee_sub{};
int8_t hp_text_id = -1;
int8_t heat_text_id = -1;
char hp_text[30]{};
char heat_text[30]{};

void create_objects()
{
    auto& cv = ui::canvas::instance();
    cv.set_sender_receiver_id(sender_id, receiver_id);
    cv.remove_all();
    hp_text_id = cv.create_string(2, ui::object_color::green, 1, 80, 760, 18, hp_text);
    heat_text_id = cv.create_string(2, ui::object_color::cyan, 1, 80, 720, 18, heat_text);
    cv.create_rect(2, ui::object_color::yellow, 1, 60, 700, 360, 790);
    objects_created = hp_text_id >= 0 && heat_text_id >= 0;
}

void sync_debug(const referee::data& data, std::uint32_t stages) noexcept
{
    auto& state = demo::debug::debug_instance.referee_ui;
    if (data.update_count > 0U)
    {
        stages |= frame_received;
    }
    if (objects_created)
    {
        stages |= ui_updated;
    }

    state.stage_mask = stages;
    state.last_step = stages;
    state.referee_online = data.online;
    state.ui_initialized = objects_created;
    state.referee_update_count = data.update_count;
    state.current_hp = data.robot_status.current_HP;
    state.max_hp = data.robot_status.maximum_HP;
    state.heat_now = data.heat_now;
    state.power_buffer = data.power_buffer;

    state.failure_mask = 0;
    if ((stages & referee_initialized) == 0U)
    {
        state.failure_mask |= referee_init_failed;
    }
    if ((stages & ui_initialized) == 0U)
    {
        state.failure_mask |= ui_init_failed;
    }
    if ((stages & subscriber_created) == 0U)
    {
        state.failure_mask |= subscribe_failed;
    }
    if ((stages & monitor_thread_started) == 0U)
    {
        state.failure_mask |= thread_create_failed;
    }

    state.failed_count = state.failure_mask == 0U ? 0U : 1U;
    state.passed = state.failure_mask == 0U && data.online;
    state.passed_count = state.passed ? state.total_count : 0U;
}

void monitor_entry(ULONG /*arg*/)
{
    referee::data data{};
    std::uint32_t stages =
        referee_initialized | ui_initialized | subscriber_created | monitor_thread_started;

    for (;;)
    {
        if (msg::read(referee_sub, data) == types::status::ok)
        {
            stages |= frame_received;
        }

        std::snprintf(hp_text, sizeof(hp_text), "HP %u/%u",
                      data.robot_status.current_HP, data.robot_status.maximum_HP);
        std::snprintf(heat_text, sizeof(heat_text), "Heat %u Buf %u",
                      data.heat_now, data.power_buffer);

        auto& cv = ui::canvas::instance();
        if (objects_created)
        {
            cv.set_string_changed(hp_text_id);
            cv.set_string_changed(heat_text_id);
        }
        cv.update();
        stages |= ui_updated;

        sync_debug(data, stages);
        tx_thread_sleep(monitor_period_ticks);
    }
}

} // namespace

void run() noexcept
{
    auto& state = demo::debug::debug_instance.referee_ui;
    state = {};
    state.started = true;
    state.total_count = 6U;

    referee::config ref_cfg{};
    ref_cfg.thread_priority = params::referee::thread_priority;
    ref_cfg.receive.game_robot_status = true;
    ref_cfg.receive.power_heat = true;
    ref_cfg.receive.buff = false;
    ref_cfg.receive.robot_hurt = false;
    if (!referee::service::instance().init(ref_cfg))
    {
        state.failure_mask = referee_init_failed;
        state.failed_count = 1U;
        return;
    }

    if (!ui::canvas::instance().init())
    {
        state.failure_mask = ui_init_failed;
        state.failed_count = 1U;
        return;
    }
    create_objects();

    referee_sub = msg::subscribe<referee::data>();
    if (!referee_sub.valid())
    {
        state.failure_mask = subscribe_failed;
        state.failed_count = 1U;
        return;
    }

    if (!monitor_started)
    {
        const UINT status = tx_thread_create(&monitor_thread, const_cast<CHAR*>("referee_ui"),
                                             monitor_entry, 0, monitor_stack,
                                             sizeof(monitor_stack),
                                             ref_cfg.thread_priority + 1U,
                                             ref_cfg.thread_priority + 1U, TX_NO_TIME_SLICE,
                                             TX_AUTO_START);
        if (status != TX_SUCCESS)
        {
            state.failure_mask = thread_create_failed;
            state.failed_count = 1U;
            return;
        }
        monitor_started = true;
    }

    sync_debug({}, referee_initialized | ui_initialized | subscriber_created | monitor_thread_started);
}

} // namespace demo::referee_ui
