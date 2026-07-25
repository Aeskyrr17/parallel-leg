#pragma once

#include "leg_config.hpp"
#include "leg_math.hpp"
#include "leg_types.hpp"
#include "motor.hpp"
#include "vmc.hpp"

namespace wbr::control
{

class Pendulum
{
public:
    Pendulum(motors::api& joint1, motors::api& joint4, const leg_calibration& calibration,
             const chassis_config& cfg);

    void solve(const motor_feedback_sample& joint1_feedback,
               const motor_feedback_sample& joint4_feedback, float pitch, float dpitch, float az,
               float dt);

    const link_state& link() const { return vmc_solver_.state(); }

    float len_control(float reference);
    float len_control(float reference, const link_state& feedback);
    float phi_control(float target_phi, float kp, float kd, float slope_path, bool positive);

    bool resolve_torque(const link_force& force, float& joint1_tau, float& joint4_tau) const;
    void write_torque(float joint1_tau, float joint4_tau);

    void reset_control();
    void reset();
    void relax();
    void tune_len_pd(float kp, float ki, float kd);

private:
    bool valid_calibration() const;
    static bool valid_motor_feedback(const motor_feedback_sample& feedback);

    motors::api& joint1_;
    motors::api& joint4_;
    leg_calibration calibration_{};
    chassis_config cfg_{};

    VMCsolver vmc_solver_;
    ::control::pid len_pd_;
    ::control::pid phi_pd_;
    slope phi_updater_;

    float target_phi_ = 0.0f;
    bool delta_init_ = false;
};

} // namespace wbr::control
