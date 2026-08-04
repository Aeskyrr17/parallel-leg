#pragma once

#include "leg_config.hpp"
#include "leg_messages.hpp"
#include "remoter.hpp"
#include "slope.hpp"

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
    void apply_drive_input(const ::remoter::state& remote,
                           leg_messages::command& command) noexcept;
    void apply_spin_mode(leg_messages::command& command) noexcept;
    void apply_manual_mode(const ::remoter::state& remote,
                           leg_messages::command& command) noexcept;
    void apply_jump_ready_mode(const ::remoter::state& remote,
                               leg_messages::command& command) noexcept;
    void apply_jump_mode(const leg_messages::solver_feedback& solver,
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
    float yaw_target_rad_ = 0.0f;
    leg_messages::jump_state jump_state_ = leg_messages::jump_state::idle;
    bool stopping_position_ = false;
    bool holding_position_ = false;
    bool holding_yaw_ = false;
    bool off_ground_hold_ = false;
};

namespace control_task
{
[[noreturn]] void run() noexcept;
} // namespace control_task

namespace pendulum_task
{
[[noreturn]] void run() noexcept;
} // namespace pendulum_task

namespace solver_task
{
[[noreturn]] void run() noexcept;
} // namespace solver_task

namespace leg_tasks
{
bool start() noexcept;
} // namespace leg_tasks

} // namespace app
