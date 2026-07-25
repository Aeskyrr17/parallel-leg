#include "pendulum.hpp"

#include <cmath>

namespace wbr::control
{

Pendulum::Pendulum(motors::api& joint1, motors::api& joint4, const leg_calibration& calibration,
                   const chassis_config& cfg)
    : joint1_(joint1), joint4_(joint4), calibration_(calibration), cfg_(cfg), vmc_solver_(cfg),
      len_pd_(cfg.leg_pid.len.kp, cfg.leg_pid.len.ki, cfg.leg_pid.len.kd, cfg.leg_pid.len.max_out,
              cfg.leg_pid.len.max_i_out, cfg.leg_pid.len.mode),
      phi_pd_(cfg.leg_pid.phi.kp, cfg.leg_pid.phi.ki, cfg.leg_pid.phi.kd, cfg.leg_pid.phi.max_out,
              cfg.leg_pid.phi.max_i_out, cfg.leg_pid.phi.mode),
      phi_updater_(0.0f, cfg.leg_pid.phi_slope_step)
{
}

void Pendulum::solve(const motor_feedback_sample& joint1_feedback,
                     const motor_feedback_sample& joint4_feedback, float pitch, float dpitch,
                     float az, float dt)
{
    joint_state joint{};
    joint.valid = valid_motor_feedback(joint1_feedback) && valid_motor_feedback(joint4_feedback) &&
                  valid_calibration();

    if (joint.valid)
    {
        const float j1_q = joint1_feedback.position - calibration_.joint1_zero_rad;
        const float j4_q = joint4_feedback.position - calibration_.joint4_zero_rad;

        // The solver convention is phi1 = pi at joint1's mechanical zero
        // and phi4 = 0 at joint4's mechanical zero.
        joint.q[0] = k_pi + calibration_.joint1_direction * j1_q;
        joint.q[1] = calibration_.joint4_direction * j4_q;
        joint.dq[0] = calibration_.joint1_direction * joint1_feedback.velocity;
        joint.dq[1] = calibration_.joint4_direction * joint4_feedback.velocity;
        joint.tau[0] = calibration_.joint1_direction * joint1_feedback.torque;
        joint.tau[1] = calibration_.joint4_direction * joint4_feedback.torque;
    }

    vmc_solver_.solve(joint, pitch, dpitch, az, dt);
}

float Pendulum::len_control(float reference)
{
    return len_control(reference, link());
}

float Pendulum::len_control(float reference, const link_state& feedback)
{
    if (!feedback.valid || !std::isfinite(reference))
    {
        return 0.0f;
    }

    len_pd_.ref = reference;
    len_pd_.fdb = feedback.len;
    len_pd_.update(feedback.dlen);
    return std::isfinite(len_pd_.result) ? len_pd_.result : 0.0f;
}

float Pendulum::phi_control(float target_phi, float kp, float kd, float slope_path, bool positive)
{
    if (!link().valid || !std::isfinite(target_phi) || !std::isfinite(kp) || !std::isfinite(kd) ||
        !std::isfinite(slope_path) || slope_path <= 0.0f)
    {
        return 0.0f;
    }

    if (!delta_init_)
    {
        phi_updater_.reset(link().total_phi);
        phi_updater_.set_path(slope_path);
        target_phi_ = target_phi;

        while (target_phi_ - link().total_phi > k_pi)
        {
            target_phi_ -= k_two_pi;
        }
        while (target_phi_ - link().total_phi < -k_pi)
        {
            target_phi_ += k_two_pi;
        }
        if (positive && target_phi_ < link().total_phi)
        {
            target_phi_ += k_two_pi;
        }
        else if (!positive && target_phi_ > link().total_phi)
        {
            target_phi_ -= k_two_pi;
        }
        delta_init_ = true;
    }

    phi_pd_.tune(kp, 0.0f, kd);
    phi_pd_.ref = phi_updater_.update(target_phi_);
    phi_pd_.fdb = link().total_phi;
    phi_pd_.update(link().dphi);
    return std::isfinite(phi_pd_.result) ? phi_pd_.result : 0.0f;
}

bool Pendulum::resolve_torque(const link_force& force, float& joint1_tau, float& joint4_tau) const
{
    joint1_tau = 0.0f;
    joint4_tau = 0.0f;

    const joint_torque torque = vmc_solver_.vmc_cal(force);
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
    delta_init_ = false;
    target_phi_ = 0.0f;
    len_pd_.clear();
    phi_pd_.clear();
    phi_updater_.reset();
}

void Pendulum::reset()
{
    reset_control();
    vmc_solver_.reset();
}

void Pendulum::relax()
{
    reset();
    joint1_.relax();
    joint4_.relax();
}

void Pendulum::tune_len_pd(float kp, float ki, float kd)
{
    len_pd_.tune(kp, ki, kd);
}

bool Pendulum::valid_calibration() const
{
    const bool joint1_direction_valid =
        calibration_.joint1_direction == 1.0f || calibration_.joint1_direction == -1.0f;
    const bool joint4_direction_valid =
        calibration_.joint4_direction == 1.0f || calibration_.joint4_direction == -1.0f;
    return joint1_direction_valid && joint4_direction_valid &&
           std::isfinite(calibration_.joint1_zero_rad) &&
           std::isfinite(calibration_.joint4_zero_rad);
}

bool Pendulum::valid_motor_feedback(const motor_feedback_sample& feedback)
{
    return feedback.valid && feedback.online && feedback.error_code == 0U &&
           std::isfinite(feedback.position) && std::isfinite(feedback.velocity) &&
           std::isfinite(feedback.torque);
}

} // namespace wbr::control
