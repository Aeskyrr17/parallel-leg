#pragma once
// Generated from robot device tree. Do not edit.

#include "motor.hpp"

#include <cstddef>
#include <cstdint>

namespace robot::motors {

inline constexpr std::size_t motor_count = 2;
inline constexpr bool has_dji = 1;
inline constexpr bool has_dm = 1;
inline constexpr bool has_lk = 0;
inline constexpr bool has_xv2 = 0;
inline constexpr bool has_other = 0;

enum class model : std::uint8_t {
    unknown = 0,
    dji_m2006,
    dji_m3508,
    dji_gm6020,
    dji_xroll,
    dm_dm4310,
    dm_dm8009p,
};

namespace dm {
inline constexpr std::uint32_t id_base = 0x01U;
inline constexpr std::uint32_t master_id_base = 0x05U;
inline constexpr std::size_t max_motors = 4;
} // namespace dm

// dji_gm6020
inline constexpr model motor1_model = model::dji_gm6020;
inline constexpr ::motors::config motor1{
    bsp::can::bus::fdcan2,
    bsp::can::bus_type::classic,
    0x205U,
    ::motors::mode::relax,
};

// dm_dm4310
inline constexpr model motor2_model = model::dm_dm4310;
inline constexpr ::motors::config motor2{
    bsp::can::bus::fdcan1,
    bsp::can::bus_type::fd,
    0x01U,
    ::motors::mode::mit,
};

} // namespace robot::motors
