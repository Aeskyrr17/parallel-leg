#pragma once

#include "leg_config.hpp"
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

    void solve(const motors::feedback& joint1_feedback, const motors::feedback& joint4_feedback,
               bool feedback_valid, float pitch, float dpitch, float az, float dt);

    const link_state& link() const { return vmc_solver_.state(); }

    float len_control(float reference);

    bool resolve_torque(const leg_wrench& target, float& joint1_tau, float& joint4_tau) const;
    void write_torque(float joint1_tau, float joint4_tau);

    void reset_control();
    void reset();

private:
    bool valid_calibration() const;
    static bool valid_motor_feedback(const motors::feedback& feedback);

    motors::api& joint1_;
    motors::api& joint4_;
    leg_calibration calibration_{};
    chassis_config cfg_{};

    VMCsolver vmc_solver_;
    ::control::pid len_pd_;
};

} // namespace wbr::control
