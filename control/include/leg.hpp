#pragma once

#include "leg_config.hpp"
#include "leg_math.hpp"
#include "leg_types.hpp"

#include <cmath>

namespace wbr::control
{

class link_solver
{
public:
    explicit link_solver(const chassis_config& cfg);

    bool solve(const joint_state& joint, float pitch, float dpitch, float az, float dt);

    joint_torque vmc_cal(const link_force& force) const;
    link_force vmc_rev_cal(const joint_torque& torque) const;
    void vmc_vel_cal(const float qdot[2], float xdot[2]) const;

    const link_state& state() const { return state_; }

    void reset();

private:
    bool resolve(float phi1, float phi4);
    bool calc_jacobian();
    void calc_spring_force();
    void calc_support_force(float az, float dt);
    void invalidate(bool reachable = false, bool near_singularity = false);

    // Geometry is expressed in metres. This epsilon protects both metre-scale
    // lengths and dimensionless sine denominators without masking normal poses.
    static constexpr float k_singularity_epsilon = 1.0e-5f;

    // Keep the original spring-model guard: its derivatives become unusably
    // large before the strict five-bar singularity is reached.
    static constexpr float k_spring_singularity_epsilon = 5.0e-2f;

    // The migrated controller is expected near 1 kHz. Values outside this
    // window are treated as a missed/invalid sample, not differentiated.
    static constexpr float k_min_dt_seconds = 1.0e-5f;
    static constexpr float k_max_dt_seconds = 5.0e-2f;

    chassis_config cfg_{};
    link_state state_{};

    float u2_ = 0.0f;
    float u3_ = 0.0f;
    float j_mat_[4] = {};
    float jt_mat_[4] = {};
    float jt_inv_mat_[4] = {};

    float prev_dlen_ = 0.0f;
    float prev_dalpha_ = 0.0f;
    float last_phi_ = 0.0f;

    bool jacobian_valid_ = false;
    bool derivative_history_valid_ = false;
    bool phi_history_valid_ = false;
};

template <typename hip_type>
class leg_controller
{
public:
    leg_controller(hip_type& joint1, hip_type& joint4, const leg_calibration& calibration,
                   const chassis_config& cfg)
        : joint1_(joint1), joint4_(joint4), calibration_(calibration), cfg_(cfg), link_solver_(cfg),
          len_pd_(cfg.leg_pid.len.kp, cfg.leg_pid.len.ki, cfg.leg_pid.len.kd,
                  cfg.leg_pid.len.max_out, cfg.leg_pid.len.max_i_out, cfg.leg_pid.len.mode),
          phi_pd_(cfg.leg_pid.phi.kp, cfg.leg_pid.phi.ki, cfg.leg_pid.phi.kd,
                  cfg.leg_pid.phi.max_out, cfg.leg_pid.phi.max_i_out, cfg.leg_pid.phi.mode)
    {
    }

    void solve(float pitch, float dpitch, float az, float dt)
    {
        const auto& j1_feedback = joint1_.get_feedback();
        const auto& j4_feedback = joint4_.get_feedback();

        joint_state joint{};
        joint.valid = valid_motor_feedback(joint1_, j1_feedback) &&
                      valid_motor_feedback(joint4_, j4_feedback) && valid_calibration();

        if (joint.valid)
        {
            const float j1_q = j1_feedback.position - calibration_.joint1_zero_rad;
            const float j4_q = j4_feedback.position - calibration_.joint4_zero_rad;

            // The solver convention is phi1 = pi at joint1's mechanical zero
            // and phi4 = 0 at joint4's mechanical zero.
            joint.q[0] = k_pi + calibration_.joint1_direction * j1_q;
            joint.q[1] = calibration_.joint4_direction * j4_q;
            joint.dq[0] = calibration_.joint1_direction * j1_feedback.velocity;
            joint.dq[1] = calibration_.joint4_direction * j4_feedback.velocity;
            joint.tau[0] = calibration_.joint1_direction * j1_feedback.torque;
            joint.tau[1] = calibration_.joint4_direction * j4_feedback.torque;
        }

        link_solver_.solve(joint, pitch, dpitch, az, dt);
    }

    const link_state& link() const { return link_solver_.state(); }

    float len_control(float reference)
    {
        if (!link().valid || !std::isfinite(reference))
        {
            return 0.0f;
        }

        len_pd_.ref = reference;
        len_pd_.fdb = link().len;
        len_pd_.update(link().dlen);
        return std::isfinite(len_pd_.result) ? len_pd_.result : 0.0f;
    }

    float phi_control(float target_phi, float kp, float kd, float slope_path, bool positive)
    {
        if (!link().valid || !std::isfinite(target_phi) || !std::isfinite(kp) ||
            !std::isfinite(kd) || !std::isfinite(slope_path) || slope_path <= 0.0f)
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

    bool resolve_torque(const link_force& force, float& joint1_tau, float& joint4_tau) const
    {
        joint1_tau = 0.0f;
        joint4_tau = 0.0f;

        const joint_torque torque = link_solver_.vmc_cal(force);
        if (!torque.valid)
        {
            return false;
        }

        joint1_tau =
            clamp(torque.t1, -cfg_.thip_max, cfg_.thip_max) * calibration_.joint1_direction;
        joint4_tau =
            clamp(torque.t4, -cfg_.thip_max, cfg_.thip_max) * calibration_.joint4_direction;
        if (!std::isfinite(joint1_tau) || !std::isfinite(joint4_tau))
        {
            joint1_tau = 0.0f;
            joint4_tau = 0.0f;
            return false;
        }
        return true;
    }

    void write_torque(float joint1_tau, float joint4_tau)
    {
        if (!link().valid || !std::isfinite(joint1_tau) || !std::isfinite(joint4_tau))
        {
            joint1_tau = 0.0f;
            joint4_tau = 0.0f;
        }

        joint1_.set_torque(joint1_tau);
        joint4_.set_torque(joint4_tau);
    }

    void relax()
    {
        delta_init_ = false;
        target_phi_ = 0.0f;
        len_pd_.clear();
        phi_pd_.clear();
        phi_updater_.reset();
        link_solver_.reset();
        joint1_.relax();
        joint4_.relax();
    }

    void tune_len_pd(float kp, float ki, float kd) { len_pd_.tune(kp, ki, kd); }

private:
    bool valid_calibration() const
    {
        const bool joint1_direction_valid =
            calibration_.joint1_direction == 1.0f || calibration_.joint1_direction == -1.0f;
        const bool joint4_direction_valid =
            calibration_.joint4_direction == 1.0f || calibration_.joint4_direction == -1.0f;
        return joint1_direction_valid && joint4_direction_valid &&
               std::isfinite(calibration_.joint1_zero_rad) &&
               std::isfinite(calibration_.joint4_zero_rad);
    }

    template <typename feedback_type>
    static bool valid_motor_feedback(const hip_type& motor, const feedback_type& feedback)
    {
        using state_type = decltype(motor.status());
        return motor.status() == state_type::online && feedback.error_code == 0U &&
               std::isfinite(feedback.position) && std::isfinite(feedback.velocity) &&
               std::isfinite(feedback.torque);
    }

    hip_type& joint1_;
    hip_type& joint4_;
    leg_calibration calibration_{};
    chassis_config cfg_{};

    link_solver link_solver_;
    ::control::pid len_pd_;
    ::control::pid phi_pd_;
    slope phi_updater_{0.0f, 0.005f};

    float target_phi_ = 0.0f;
    bool delta_init_ = false;
};

} // namespace wbr::control
