#include "usart_demo.hpp"

#include "bsp_usart.hpp"
#include "config.hpp"
#include "demo_debug.hpp"
#include "demo_protocol.hpp"
#include "memory.h"

#include <cstdint>

namespace demo::usart
{
namespace
{

alignas(32) std::uint8_t rx_buffer[sizeof(protocol::host_packet)] RAM_D1_BSS{};
bool started = false;

void record_request(debug::link_state& debug, const protocol::host_packet& packet) noexcept
{
    debug.last_rx = packet;
    debug.last_seq = packet.seq;
    debug.last_counter = packet.counter;
    debug.last_value = packet.value;
    debug.last_flag = packet.flag != 0U;
}

void record_error(debug::link_state& debug, types::status status) noexcept
{
    ++debug.error_count;
    debug.last_status = protocol::status_code(status);
}

void on_rx(bsp::usart::port port, const bsp::usart::rx_frame& frame, void* user_data)
{
    (void)user_data;

    auto& state = debug::debug_instance.usart;
    state.ready = true;
    state.connected = true;

    if (port != app::uart::usart1 || frame.data == nullptr || frame.len != sizeof(protocol::host_packet))
    {
        record_error(state, types::status::invalid_arg);
        return;
    }

    protocol::host_packet request{};
    protocol::copy_host_packet(frame.data, request);
    record_request(state, request);

    if (!protocol::validate(request, protocol::usart_host_magic))
    {
        record_error(state, types::status::invalid_arg);
        return;
    }

    ++state.rx_count;

    protocol::device_packet response = protocol::make_response(
        protocol::usart_device_magic, request, types::status::ok, true, state.rx_count,
        state.tx_count + 1U, state.error_count);
    state.last_tx = response;

    const types::status tx_status = bsp::usart::transmit(
        app::uart::usart1, reinterpret_cast<const std::uint8_t*>(&response), sizeof(response), 50U);
    state.last_status = protocol::status_code(tx_status);
    if (tx_status == types::status::ok)
    {
        ++state.tx_count;
    }
    else
    {
        ++state.error_count;
    }
}

} // namespace

types::status start() noexcept
{
    auto& state = debug::debug_instance.usart;
    if (started)
    {
        state.started = true;
        return types::status::ok;
    }

    debug::reset(state);
    state.started = true;

    types::status status = bsp::usart::init(app::uart::usart1, bsp::usart::mode::dma);
    state.last_status = protocol::status_code(status);
    if (status != types::status::ok)
    {
        ++state.error_count;
        return status;
    }

    status = bsp::usart::start_rx_to_idle(
        app::uart::usart1, rx_buffer, sizeof(rx_buffer), on_rx, nullptr);
    state.last_status = protocol::status_code(status);
    if (status != types::status::ok)
    {
        ++state.error_count;
        return status;
    }

    state.ready = true;
    started = true;
    return types::status::ok;
}

} // namespace demo::usart
