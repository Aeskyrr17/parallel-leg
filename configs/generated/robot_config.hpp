#pragma once
// Generated from robot device tree. Do not edit.

#include "motor.hpp"

#include <cstddef>
#include <cstdint>

namespace robot::motors {

inline constexpr std::size_t motor_count = 6;
inline constexpr bool has_dji = 0;
inline constexpr bool has_dm = 0;
inline constexpr bool has_lk = 1;
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
    lk_lk8016,
    lk_lk9025,
};

namespace dm {
inline constexpr std::uint32_t id_base = 0x01U;
inline constexpr std::uint32_t master_id_base = 0x05U;
inline constexpr std::size_t max_motors = 4;
} // namespace dm

// lk_lk8016
inline constexpr model ljoint4_model = model::lk_lk8016;
inline constexpr ::motors::config ljoint4{
    bsp::can::bus::fdcan1,
    bsp::can::bus_type::classic,
    0x141U,
    ::motors::mode::torque,
};

// lk_lk8016
inline constexpr model ljoint1_model = model::lk_lk8016;
inline constexpr ::motors::config ljoint1{
    bsp::can::bus::fdcan1,
    bsp::can::bus_type::classic,
    0x142U,
    ::motors::mode::torque,
};

// lk_lk8016
inline constexpr model rjoint4_model = model::lk_lk8016;
inline constexpr ::motors::config rjoint4{
    bsp::can::bus::fdcan1,
    bsp::can::bus_type::classic,
    0x143U,
    ::motors::mode::torque,
};

// lk_lk8016
inline constexpr model rjoint1_model = model::lk_lk8016;
inline constexpr ::motors::config rjoint1{
    bsp::can::bus::fdcan1,
    bsp::can::bus_type::classic,
    0x144U,
    ::motors::mode::torque,
};

// lk_lk9025
inline constexpr model lwheel_model = model::lk_lk9025;
inline constexpr ::motors::config lwheel{
    bsp::can::bus::fdcan3,
    bsp::can::bus_type::classic,
    0x141U,
    ::motors::mode::torque,
};

// lk_lk9025
inline constexpr model rwheel_model = model::lk_lk9025;
inline constexpr ::motors::config rwheel{
    bsp::can::bus::fdcan3,
    bsp::can::bus_type::classic,
    0x142U,
    ::motors::mode::torque,
};

} // namespace robot::motors
