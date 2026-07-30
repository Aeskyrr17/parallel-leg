#pragma once

#include "control_config.hpp"
#include "leg_math.hpp"
#include "msgs.hpp"
#include "types.hpp"

namespace wbr
{

class Function
{
public:
    explicit Function(const command_config& cfg);

    const chassis_command& update(const remoter::state& remote, float odom_x, float odom_v,
                                  float total_yaw, float dt);
    void reset();
    void reset_position(float x);
    void reset_yaw(float total_yaw);

private:
    static bool is_transition(remoter::sw_state prev, remoter::sw_state current);
    void update_position(bool spin, float odom_x, float odom_v, float dt);
    void update_yaw(float total_yaw, float dt);

    command_config cfg_{};
    chassis_command cmd_{};

    slope yaw_updater_;
    slope vel_updater_;

    float maintained_x_ = 0.0f;
    float maintained_yaw_ = 0.0f;
    remoter::sw_state prev_ctrl_ = remoter::sw_state::low;

    bool maintaining_x_ = false;
    bool maintaining_yaw_ = false;
    bool prev_sw_valid_ = false;
    bool transition_cooldown_ = false;
};

} // namespace wbr
