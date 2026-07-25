#include "pendulum.hpp"

#include "leg_math.hpp"

#include <cmath>

namespace wbr::control
{

Pendulum::Pendulum(motors::api& joint1, motors::api& joint4, const leg_calibration& calibration,
                   const chassis_config& cfg)
    : joint1_(joint1), joint4_(joint4), calibration_(calibration), cfg_(cfg), vmc_solver_(cfg),
      len_pd_(cfg.leg_len_pid)
{
}

void Pendulum::solve(const motors::feedback& joint1_feedback,
                     const motors::feedback& joint4_feedback, bool feedback_valid, float pitch,
                     float dpitch, float az, float dt)
{
    joint_state joint{};
    joint.valid = feedback_valid && valid_motor_feedback(joint1_feedback) &&
                  valid_motor_feedback(joint4_feedback) && valid_calibration();

    if (joint.valid)
    {
        joint.q[0] =
            k_pi + calibration_.joint1_direction * joint1_feedback.position;
        joint.q[1] = calibration_.joint4_direction * joint4_feedback.position;
        joint.dq[0] = calibration_.joint1_direction * joint1_feedback.velocity;
        joint.dq[1] = calibration_.joint4_direction * joint4_feedback.velocity;
        joint.tau[0] = calibration_.joint1_direction * joint1_feedback.torque;
        joint.tau[1] = calibration_.joint4_direction * joint4_feedback.torque;
    }

    vmc_solver_.solve(joint, pitch, dpitch, az, dt);
}

float Pendulum::len_control(float reference)
{
    const link_state& feedback = link();
    if (!feedback.valid || !std::isfinite(reference))
    {
        return 0.0f;
    }

    len_pd_.ref = reference;
    len_pd_.fdb = feedback.len;
    len_pd_.update(feedback.dlen);
    return std::isfinite(len_pd_.result) ? len_pd_.result : 0.0f;
}

bool Pendulum::resolve_torque(const leg_wrench& target, float& joint1_tau,
                              float& joint4_tau) const
{
    joint1_tau = 0.0f;
    joint4_tau = 0.0f;

    const joint_torque torque = vmc_solver_.vmc_cal(target);
    if (!torque.valid)
    {
        return false;
    }

    joint1_tau =
        clamp(torque.t1, -cfg_.max_hip_torque, cfg_.max_hip_torque) * calibration_.joint1_direction;
    joint4_tau =
        clamp(torque.t4, -cfg_.max_hip_torque, cfg_.max_hip_torque) * calibration_.joint4_direction;
    if (!std::isfinite(joint1_tau) || !std::isfinite(joint4_tau))
    {
        joint1_tau = 0.0f;
        joint4_tau = 0.0f;
        return false;
    }
    return true;
}

void Pendulum::write_torque(float joint1_tau, float joint4_tau)
{
    if (!link().valid || !std::isfinite(joint1_tau) || !std::isfinite(joint4_tau))
    {
        joint1_tau = 0.0f;
        joint4_tau = 0.0f;
    }

    motors::command joint1_command{};
    motors::command joint4_command{};
    joint1_command.torque = joint1_tau;
    joint4_command.torque = joint4_tau;
    joint1_.set_command(joint1_command, motors::mode::torque);
    joint4_.set_command(joint4_command, motors::mode::torque);
}

void Pendulum::reset_control()
{
    len_pd_.clear();
}

void Pendulum::reset()
{
    reset_control();
    vmc_solver_.reset();
}

bool Pendulum::valid_calibration() const
{
    const bool joint1_direction_valid =
        calibration_.joint1_direction == 1.0f || calibration_.joint1_direction == -1.0f;
    const bool joint4_direction_valid =
        calibration_.joint4_direction == 1.0f || calibration_.joint4_direction == -1.0f;
    return joint1_direction_valid && joint4_direction_valid;
}

bool Pendulum::valid_motor_feedback(const motors::feedback& feedback)
{
    return feedback.error_code == 0U && std::isfinite(feedback.position) &&
           std::isfinite(feedback.velocity) && std::isfinite(feedback.torque);
}

} // namespace wbr::control
