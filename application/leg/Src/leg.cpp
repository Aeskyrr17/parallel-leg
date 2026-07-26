#include "leg.hpp"

#include "ahrs.hpp"
#include "config.hpp"
#include "leg_config.hpp"
#include "leg_messages.hpp"
#include "leg_tasks.hpp"
#include "lkmotorhandler.hpp"
#include "msg.hpp"
#include "remoter.hpp"
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
}

void leg::set_motor_offsets() noexcept
{
    left_joint_4_.offset = static_cast<float>(leg_config::left_joint_4_offset);
    left_joint_1_.offset = static_cast<float>(leg_config::left_joint_1_offset);
    right_joint_4_.offset = static_cast<float>(leg_config::right_joint_4_offset);
    right_joint_1_.offset = static_cast<float>(leg_config::right_joint_1_offset);
    left_wheel_.offset = static_cast<float>(leg_config::left_wheel_offset);
    right_wheel_.offset = static_cast<float>(leg_config::right_wheel_offset);
}

bool leg::register_motors() noexcept
{
    auto& handler = motors::lkmotorhandler::instance();

    bool registered = true;
    registered &= handler.register_motor(left_joint_4_);
    registered &= handler.register_motor(left_joint_1_);
    registered &= handler.register_motor(right_joint_4_);
    registered &= handler.register_motor(right_joint_1_);
    registered &= handler.register_motor(left_wheel_);
    registered &= handler.register_motor(right_wheel_);
    return registered;
}

void leg::send_relax_command() noexcept
{
    left_joint_4_.relax();
    left_joint_1_.relax();
    right_joint_4_.relax();
    right_joint_1_.relax();
    left_wheel_.relax();
    right_wheel_.relax();
    motors::lkmotorhandler::instance().send_control();
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

    if (msg::init() != types::status::ok)
    {
        return false;
    }

    ::ahrs::config ahrs_config{};
    ahrs_config.imu_offset_x = params::ahrs::imu_offset_x;
    ahrs_config.imu_thread_priority = params::ahrs::imu_thread_priority;
    ahrs_config.temp_thread_priority = params::ahrs::temp_thread_priority;
    ahrs_config.target_temp = params::ahrs::target_temp;
    if (!::ahrs::service::instance().init(ahrs_config))
    {
        return false;
    }

    ::remoter::config remoter_config{};
    remoter_config.dr16.thread_priority = params::remoter::thread_priority;
    remoter_config.dr16.rx_timeout_ticks = params::remoter::rx_timeout_ticks;
    remoter_config.thread_priority = params::remoter::thread_priority + 1U;
    if (!::remoter::service::instance().init(remoter_config))
    {
        return false;
    }

    set_motor_offsets();
    const bool motors_registered = register_motors();
    send_relax_command();
    if (!motors_registered)
    {
        return false;
    }

    bool messages_ready = true;
    messages_ready &= msg::create<leg_messages::command>() != nullptr;
    messages_ready &= msg::create<leg_messages::solver_feedback>() != nullptr;
    messages_ready &= msg::create<leg_messages::control_target>() != nullptr;
    messages_ready &= msg::create<leg_messages::odometry>() != nullptr;
    if (!messages_ready)
    {
        return false;
    }

    auto& control = control_task::instance();
    auto& pendulum = pendulum_task::instance();
    auto& solver = solver_task::instance();

    bool tasks_ready = control.prepare();
    tasks_ready &= pendulum.prepare();
    tasks_ready &= solver.prepare();
    if (!tasks_ready)
    {
        return false;
    }

    if (!control.start() || !pendulum.start() || !solver.start())
    {
        return false;
    }

    started_ = true;
    return true;
}

} // namespace app
