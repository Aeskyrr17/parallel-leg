#pragma once

#include "control_config.hpp"
#include "leg_types.hpp"
#include "vmc.hpp"

namespace wbr
{

class leg_solver
{
public:
    explicit leg_solver(const leg_config& cfg) : solver_(cfg) {}

    bool solve(const joint_state& joint, float pitch, float dpitch, float az, float dt,
               float wheel_side_mass, float gravity)
    {
        return solver_.solve(joint, pitch, dpitch, az, dt, wheel_side_mass, gravity);
    }

    const link_state& state() const { return solver_.state(); }

    bool resolve_torque(const virtual_force& force, joint_torque& torque) const
    {
        return solver_.vmc_cal(force, torque);
    }
    bool estimate_link_force(const joint_torque& torque, virtual_force& force) const
    {
        return solver_.vmc_rev_cal(torque, force);
    }

    void reset() { solver_.reset(); }

private:
    link_solver solver_;
};

} // namespace wbr
