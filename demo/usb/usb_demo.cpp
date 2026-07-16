#include "usb_demo.hpp"

#include "bsp_usb.hpp"
#include "demo_debug.hpp"
#include "demo_protocol.hpp"

#include "tx_api.h"

namespace demo::usb
{
namespace
{

bool started = false;
TX_MUTEX response_mutex{};
bool response_mutex_ready = false;
bool response_pending = false;
std::uint32_t response_pending_seq = 0;
protocol::device_packet pending_response{};

void lock_response() noexcept
{
    if (response_mutex_ready)
    {
        tx_mutex_get(&response_mutex, TX_WAIT_FOREVER);
    }
}

void unlock_response() noexcept
{
    if (response_mutex_ready)
    {
        tx_mutex_put(&response_mutex);
    }
}

void sync_bsp_state(debug::link_state& debug) noexcept
{
    const auto& usb = bsp::usb::state();
    debug.connected = usb.connected;
    debug.bsp_read_busy = usb.read_busy;
    debug.bsp_write_busy = usb.write_busy;
    debug.bsp_last_read_status = usb.last_read_status;
    debug.bsp_last_write_status = usb.last_write_status;
    debug.bsp_last_read_len = usb.last_read_len;
    debug.bsp_last_write_requested = usb.last_write_requested;
    debug.bsp_last_write_actual = usb.last_write_actual;
    debug.bsp_pending_write_len = usb.pending_write_len;
    debug.bsp_read_count = usb.read_count;
    debug.bsp_write_count = usb.write_count;
    debug.bsp_error_count = usb.error_count;
    debug.bsp_tx_wake_count = usb.tx_wake_count;
    debug.bsp_fill_count = usb.fill_count;
    debug.tx_pending = response_pending;
    debug.tx_pending_seq = response_pending_seq;
}

void record_request(debug::link_state& debug, const protocol::host_packet& packet) noexcept
{
    debug.last_rx = packet;
    debug.last_seq = packet.seq;
    debug.last_counter = packet.counter;
    debug.last_value = packet.value;
    debug.last_flag = packet.flag != 0U;
}

void on_rx(const protocol::host_packet& packet, void* user_data)
{
    (void)user_data;

    auto& state = debug::debug_instance.usb;
    sync_bsp_state(state);

    record_request(state, packet);
    if (!protocol::validate(packet, protocol::usb_host_magic))
    {
        ++state.error_count;
        state.last_status = protocol::status_code(types::status::invalid_arg);
        lock_response();
        response_pending = false;
        unlock_response();
        return;
    }

    ++state.rx_count;
    protocol::device_packet response = protocol::make_response(
        protocol::usb_device_magic, packet, types::status::ok, state.connected,
        state.rx_count, state.tx_count + 1U, state.error_count);

    lock_response();
    pending_response = response;
    ++response_pending_seq;
    response_pending = true;
    state.tx_pending = true;
    state.tx_pending_seq = response_pending_seq;
    unlock_response();

    state.last_tx = response;
    state.ready = true;
    state.last_status = protocol::status_code(types::status::ok);
}

bool fill_tx(protocol::device_packet& packet, void* user_data)
{
    (void)user_data;

    auto& state = debug::debug_instance.usb;
    sync_bsp_state(state);

    lock_response();
    if (!response_pending)
    {
        ++state.tx_fill_miss_count;
        unlock_response();
        return false;
    }

    packet = pending_response;
    ++state.tx_fill_hit_count;
    state.tx_pending = true;
    state.tx_pending_seq = response_pending_seq;
    unlock_response();

    state.last_tx = packet;
    state.last_status = protocol::status_code(types::status::ok);
    return true;
}

void on_tx_done(uint16_t requested_len, uint16_t actual_len, uint32_t status, void* user_data)
{
    (void)user_data;

    auto& state = debug::debug_instance.usb;
    sync_bsp_state(state);

    if (status == 0U && requested_len == sizeof(protocol::device_packet) && actual_len == requested_len)
    {
        ++state.tx_count;
        state.last_status = protocol::status_code(types::status::ok);
    }
    else
    {
        ++state.error_count;
        state.last_status = protocol::status_code(types::status::error);
    }

    lock_response();
    response_pending = false;
    unlock_response();
    state.tx_pending = false;
}

} // namespace

types::status start() noexcept
{
    auto& state = debug::debug_instance.usb;
    if (started)
    {
        state.started = true;
        sync_bsp_state(state);
        return types::status::ok;
    }

    debug::reset(state);
    state.started = true;

    if (!response_mutex_ready)
    {
        if (tx_mutex_create(&response_mutex, const_cast<CHAR*>("usb_demo_tx"), TX_NO_INHERIT) != TX_SUCCESS)
        {
            state.last_status = protocol::status_code(types::status::error);
            ++state.error_count;
            return types::status::error;
        }
        response_mutex_ready = true;
    }

    bsp::usb::config config = bsp::usb::make_config<protocol::host_packet, protocol::device_packet>(
        on_rx, fill_tx, on_tx_done, nullptr);
    types::status status = bsp::usb::init(config);
    state.last_status = protocol::status_code(status);
    if (status != types::status::ok)
    {
        ++state.error_count;
        return status;
    }

    state.ready = true;
    sync_bsp_state(state);
    started = true;
    return types::status::ok;
}

void poll() noexcept
{
    auto& state = debug::debug_instance.usb;
    sync_bsp_state(state);
}

} // namespace demo::usb
