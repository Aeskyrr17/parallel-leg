#pragma once
// Generated from board/board.ioc + config/params.json. Do not edit.

#include <array>
#include <cstddef>
#include <cstdint>

#define HW_HAS_USB 1
#define ENABLE_USBX 1
#define HAS_AHRS 1
#define HAS_REMOTER 1
#define HAS_VT03 1
#define ENABLE_DR16 1
#define ENABLE_VT03 0
#define HAS_REFEREE 1
#define HAS_UI 1
#define HAS_LED 1
#define HAS_PWM_TIM3_CH4 1
#define HAS_PWM_TIM12_CH2 1
#define HAS_MOTORS 1
#define MOTOR_DJI 1
#define MOTOR_DM 1
#define MOTOR_LK 0

namespace config::feature {

inline constexpr bool hw_has_usb = 1;
inline constexpr bool enable_usbx = 1;
inline constexpr bool has_ahrs = 1;
inline constexpr bool has_remoter = 1;
inline constexpr bool has_vt03 = 1;
inline constexpr bool enable_dr16 = 1;
inline constexpr bool enable_vt03 = 0;
inline constexpr bool has_referee = 1;
inline constexpr bool has_ui = 1;
inline constexpr bool has_led = 1;
inline constexpr bool has_pwm_tim3_ch4 = 1;
inline constexpr bool has_pwm_tim12_ch2 = 1;
inline constexpr bool has_motors = 1;
inline constexpr bool motor_dji = 1;
inline constexpr bool motor_dm = 1;
inline constexpr bool motor_lk = 0;

} // namespace config::feature

namespace bsp {
namespace can {

enum class bus_type : std::uint8_t { classic = 0, fd = 1 };
enum class id_type : std::uint8_t { standard = 0, extended = 1 };
enum class handle_id : std::uint8_t { none = 0, fdcan1, fdcan2, fdcan3 };
enum class bus : std::uint8_t { fdcan1 = 0, fdcan2 = 1, fdcan3 = 2 };

struct bus_config
{
    bool enabled = false;
    handle_id handle = handle_id::none;
    bus_type type = bus_type::classic;
    id_type filter_id_type = id_type::standard;
};

inline constexpr std::size_t bus_count = 3;
inline constexpr std::size_t max_rx_callbacks = 8;
inline constexpr std::uint32_t tx_delay_comp_tdc = 13;
inline constexpr std::uint32_t tx_delay_comp_filter = 13;

inline constexpr std::array<bus_config, bus_count> configs = {{ { true, handle_id::fdcan1, bus_type::fd, id_type::standard }, { true, handle_id::fdcan2, bus_type::classic, id_type::standard }, { true, handle_id::fdcan3, bus_type::classic, id_type::standard } }};
inline constexpr std::array<bool, bus_count> enabled = { true, true, true };
inline constexpr std::array<bus_type, bus_count> configured_bus_types = { bus_type::fd, bus_type::classic, bus_type::classic };
inline constexpr std::array<id_type, bus_count> filter_id_types = { id_type::standard, id_type::standard, id_type::standard };

} // namespace can

namespace spi {

enum class handle_id : std::uint8_t { none = 0, spi2, spi6 };

struct bus_config
{
    bool enabled = false;
    handle_id handle = handle_id::none;
};

inline constexpr std::size_t bus_count = 2;
inline constexpr std::array<bus_config, bus_count> configs = {{ { true, handle_id::spi2 }, { true, handle_id::spi6 } }};

} // namespace spi

namespace pwm {

enum class timer_id : std::uint8_t { none = 0, tim3, tim12 };
enum class channel_id : std::uint8_t { none = 0, ch1, ch2, ch3, ch4 };

struct channel_config
{
    bool enabled = false;
    timer_id timer = timer_id::none;
    channel_id channel = channel_id::none;
    std::uint32_t timer_clock_hz = 0;
};

inline constexpr std::size_t channel_count = 2;
inline constexpr std::array<channel_config, channel_count> configs = {{ { true, timer_id::tim3, channel_id::ch4, 240000000U }, { true, timer_id::tim12, channel_id::ch2, 240000000U } }};

} // namespace pwm

namespace usart {

using port = std::size_t;

enum class handle_id : std::uint8_t { none = 0, uart5, uart7, usart1 };

struct port_config
{
    bool enabled = false;
    handle_id handle = handle_id::none;
    bool has_rx_dma = false;
    bool has_tx_dma = false;
};

inline constexpr std::size_t port_count = 3;
inline constexpr std::array<port_config, port_count> configs = {{ { true, handle_id::uart5, true, false }, { true, handle_id::uart7, true, true }, { true, handle_id::usart1, true, true } }};
inline constexpr std::array<bool, port_count> enabled = { true, true, true };

} // namespace usart
} // namespace bsp

namespace app {
namespace uart {

inline constexpr bsp::usart::port uart5 = 0;
inline constexpr bsp::usart::port uart7 = 1;
inline constexpr bsp::usart::port usart1 = 2;

inline constexpr bsp::usart::port dr16 = uart5;
inline constexpr bsp::usart::port vt03 = uart7;
inline constexpr bsp::usart::port referee = usart1;
inline constexpr bsp::usart::port test_report = uart7;

} // namespace uart
} // namespace app

namespace params::ahrs {
  inline constexpr float imu_offset_x = 0.0f;
  inline constexpr std::uint32_t imu_thread_priority = 3;
  inline constexpr std::uint32_t temp_thread_priority = 4;
  inline constexpr float target_temp = 45.0f;
} // namespace params::ahrs

namespace params::remoter {
  inline constexpr std::uint32_t thread_priority = 2;
  inline constexpr std::uint32_t rx_timeout_ticks = 100;
} // namespace params::remoter

namespace params::referee {
  inline constexpr std::uint32_t thread_priority = 8;
} // namespace params::referee

namespace params::test {
  inline constexpr std::uint32_t thread_priority = 10;
  inline constexpr bool auto_run_on_boot = true;
} // namespace params::test

namespace params::usb {
  inline constexpr std::uint32_t read_thread_priority = 5;
  inline constexpr std::uint32_t write_thread_priority = 5;
  inline constexpr std::uint32_t period_ticks = 2;
} // namespace params::usb
