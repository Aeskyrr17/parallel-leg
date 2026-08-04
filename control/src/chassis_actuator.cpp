#include "chassis_actuator.hpp"

#include "constants.hpp"
#include "constrain.hpp"

#include <cmath>

namespace wbr
{

chassis_actuator::chassis_actuator(const actuator_config& cfg,
                                   const leg_solver& left_leg,
                                   const leg_solver& right_leg,
                                   chassis_motor_handles motors)
    : cfg_(cfg), left_leg_(left_leg), right_leg_(right_leg), motors_(motors)
{
}

joint_state chassis_actuator::make_joint_state(const motors::feedback& joint1,
                                               const motors::feedback& joint4,
                                               const leg_dir& direction)
{
    joint_state joint{};
    joint.q[0] = math::pi + direction.joint1_dir * joint1.position;
    joint.q[1] = direction.joint4_dir * joint4.position;
    joint.dq[0] = direction.joint1_dir * joint1.velocity;
    joint.dq[1] = direction.joint4_dir * joint4.velocity;
    joint.tau[0] = direction.joint1_dir * joint1.torque;
    joint.tau[1] = direction.joint4_dir * joint4.torque;
    return joint;
}

joint_state chassis_actuator::left_joint_state(const motors::feedback& joint1,
                                                const motors::feedback& joint4) const
{
    return make_joint_state(joint1, joint4, cfg_.motor_dir.left_leg);
}

joint_state chassis_actuator::right_joint_state(const motors::feedback& joint1,
                                                 const motors::feedback& joint4) const
{
    return make_joint_state(joint1, joint4, cfg_.motor_dir.right_leg);
}

float chassis_actuator::wheel_velocity(const motors::feedback& left,
                                       const motors::feedback& right,
                                       float wheel_radius) const
{
    return 0.5f * (cfg_.motor_dir.left_wheel_dir * left.velocity +
                   cfg_.motor_dir.right_wheel_dir * right.velocity) *
            wheel_radius;
}

bool chassis_actuator::resolve(const chassis_output& output,
                               actuator_command& command) const
{
    command = {};
    joint_torque left_tau{};
    joint_torque right_tau{};
    if (!left_leg_.resolve_torque(output.left_target, left_tau) ||
        !right_leg_.resolve_torque(output.right_target, right_tau))
    {
        return false;
    }

    actuator_command resolved{};
    resolved.left_joint1 = cfg_.motor_dir.left_leg.joint1_dir *
                           math::limit_abs(left_tau.t1, cfg_.max_hip_tau);
    resolved.left_joint4 = cfg_.motor_dir.left_leg.joint4_dir *
                           math::limit_abs(left_tau.t4, cfg_.max_hip_tau);
    resolved.right_joint1 = cfg_.motor_dir.right_leg.joint1_dir *
                            math::limit_abs(right_tau.t1, cfg_.max_hip_tau);
    resolved.right_joint4 = cfg_.motor_dir.right_leg.joint4_dir *
                            math::limit_abs(right_tau.t4, cfg_.max_hip_tau);
    resolved.left_wheel = cfg_.motor_dir.left_wheel_dir *
                          math::limit_abs(output.tau_w_l, cfg_.max_wheel_tau);
    resolved.right_wheel = cfg_.motor_dir.right_wheel_dir *
                           math::limit_abs(output.tau_w_r, cfg_.max_wheel_tau);

    if (!std::isfinite(resolved.left_joint1) || !std::isfinite(resolved.left_joint4) ||
        !std::isfinite(resolved.right_joint1) || !std::isfinite(resolved.right_joint4) ||
        !std::isfinite(resolved.left_wheel) || !std::isfinite(resolved.right_wheel))
    {
        return false;
    }

    command = resolved;
    return true;
}

void chassis_actuator::write(const actuator_command& command)
{
    motors::command lj1_cmd{};
    motors::command lj4_cmd{};
    motors::command rj1_cmd{};
    motors::command rj4_cmd{};
    lj1_cmd.torque = command.left_joint1;
    lj4_cmd.torque = command.left_joint4;
    rj1_cmd.torque = command.right_joint1;
    rj4_cmd.torque = command.right_joint4;

    motors_.left_joint1.set_command(lj1_cmd, motors::mode::torque);
    motors_.left_joint4.set_command(lj4_cmd, motors::mode::torque);
    motors_.right_joint1.set_command(rj1_cmd, motors::mode::torque);
    motors_.right_joint4.set_command(rj4_cmd, motors::mode::torque);
    motors_.left_wheel.set_torque(command.left_wheel);
    motors_.right_wheel.set_torque(command.right_wheel);
}

} // namespace wbr
