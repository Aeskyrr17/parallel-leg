#pragma once

#include "control_config.hpp"
#include "leg_types.hpp"
#include "motor.hpp"
#include "vmc.hpp"

namespace wbr::control
{

class leg_controller
{
public:
    leg_controller(motors::api& joint1, motors::api& joint4, const leg_dir& dir,
                   const leg_config& cfg);

    void solve(const motors::feedback& joint1_fdb, const motors::feedback& joint4_fdb,
               float pitch, float dpitch, float az, float dt, float wheel_side_mass, float gravity);

    const link_state& link() const { return solver_.state(); }

    float len_control(float len_ref);

    bool resolve_torque(const virtual_force& force, float& joint1_tau, float& joint4_tau) const;
    void write_torque(float joint1_tau, float joint4_tau);

    void reset_control();
    void reset();
    void relax();

private:
    motors::api& joint1_;
    motors::api& joint4_;
    const leg_dir dir_;
    const leg_config& cfg_;

    link_solver solver_;
    ::control::pid len_pd_;
};

} // namespace wbr::control
