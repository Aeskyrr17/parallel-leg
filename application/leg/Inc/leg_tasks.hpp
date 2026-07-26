#pragma once

#include "leg_config.hpp"
#include "leg_messages.hpp"
#include "msg.hpp"
#include "remoter.hpp"
#include "slope.hpp"
#include "tx_api.h"

#include <cstdint>

namespace app
{

class command_interpreter
{
public:
    command_interpreter() noexcept;

    [[nodiscard]] leg_messages::command update(
        const ::remoter::state& remote,
        const leg_messages::solver_feedback& solver,
        const leg_messages::odometry& odometry,
        std::uint32_t tick) noexcept;

    void reset() noexcept;

private:
    [[nodiscard]] leg_messages::command stop_command(
        const leg_messages::odometry& odometry,
        std::uint32_t tick) noexcept;
    void apply_manual_motion(const ::remoter::state& remote,
                             leg_messages::command& command) noexcept;
    void update_position(leg_messages::command& command,
                         const leg_messages::odometry& odometry,
                         bool force_hold) noexcept;
    void begin_jump_stage(leg_messages::jump_state stage) noexcept;

    slope speed_ramp_;
    slope yaw_rate_ramp_;
    slope leg_length_ramp_;
    float manual_leg_length_m_ = leg_config::normal_leg_length_m;
    float position_target_m_ = 0.0f;
    leg_messages::jump_state jump_state_ = leg_messages::jump_state::idle;
    bool holding_position_ = false;
};

class command_task
{
public:
    static command_task& instance() noexcept;

    bool start() noexcept;
    [[nodiscard]] bool started() const noexcept { return started_; }

private:
    command_task() = default;

    static void thread_entry(ULONG input);
    void run() noexcept;

    TX_THREAD thread_{};
    alignas(8) std::uint8_t stack_[leg_config::command_thread::stack_size]{};
    msg::topic* command_topic_ = nullptr;
    msg::subscriber remoter_sub_{};
    msg::subscriber solver_sub_{};
    msg::subscriber odometry_sub_{};
    command_interpreter interpreter_{};
    bool started_ = false;
};

} // namespace app
