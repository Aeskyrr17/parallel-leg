#include "leg.hpp"

#include "leg_config.hpp"
#include "lkmotorhandler.hpp"
#include "robot_config.hpp"

namespace app
{

static_assert(robot::motors::left_joint_4_model == robot::motors::model::lk_lk8016);
static_assert(robot::motors::left_joint_1_model == robot::motors::model::lk_lk8016);
static_assert(robot::motors::right_joint_4_model == robot::motors::model::lk_lk8016);
static_assert(robot::motors::right_joint_1_model == robot::motors::model::lk_lk8016);
static_assert(robot::motors::left_wheel_model == robot::motors::model::lk_lk9025);
static_assert(robot::motors::right_wheel_model == robot::motors::model::lk_lk9025);

leg::leg()
    : left_joint_4_(robot::motors::left_joint_4),
      left_joint_1_(robot::motors::left_joint_1),
      right_joint_4_(robot::motors::right_joint_4),
      right_joint_1_(robot::motors::right_joint_1),
      left_wheel_(robot::motors::left_wheel),
      right_wheel_(robot::motors::right_wheel)
{
    left_joint_4_.offset = static_cast<float>(leg_config::left_joint_4_offset);
    left_joint_1_.offset = static_cast<float>(leg_config::left_joint_1_offset);
    right_joint_4_.offset = static_cast<float>(leg_config::right_joint_4_offset);
    right_joint_1_.offset = static_cast<float>(leg_config::right_joint_1_offset);
    left_wheel_.offset = static_cast<float>(leg_config::left_wheel_offset);
    right_wheel_.offset = static_cast<float>(leg_config::right_wheel_offset);
}

leg& leg::instance() noexcept
{
    static leg instance;
    return instance;
}

bool leg::start() noexcept
{
    if (started_)
    {
        return true;
    }

    auto& handler = motors::lkmotorhandler::instance();

    bool registered = true;
    registered &= handler.register_motor(left_joint_4_);
    registered &= handler.register_motor(left_joint_1_);
    registered &= handler.register_motor(right_joint_4_);
    registered &= handler.register_motor(right_joint_1_);
    registered &= handler.register_motor(left_wheel_);
    registered &= handler.register_motor(right_wheel_);

    started_ = registered;
    return started_;
}

} // namespace app
