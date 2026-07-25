#pragma once

#include "leg_config.hpp"
#include "leg_math.hpp"
#include "msgs.hpp"
#include "types.hpp"

namespace wbr::control
{

class Function
{
public:
    explicit Function(const command_config& cfg);

    const chassis_command& update(const remoter::state& remote, float odometry_x, float dt);
    void reset();

private:
    static bool is_transition(remoter::sw_state previous, remoter::sw_state current);
    bool valid_remote(const remoter::state& remote) const;
    void update_position(bool spin, float odometry_x, float dt);

    command_config cfg_{};
    chassis_command command_{};

    slope yaw_updater_;
    slope velocity_updater_;

    float maintained_x_ = 0.0f;
    remoter::sw_state previous_control_ = remoter::sw_state::low;

    bool maintaining_x_ = false;
    bool previous_switches_valid_ = false;
    bool transition_cooldown_ = false;
};

} // namespace wbr::control
