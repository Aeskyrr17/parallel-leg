#include "leg.hpp"

namespace wbr
{

leg_solver::leg_solver(const leg_config& cfg) : solver_(cfg)
{
}

bool leg_solver::solve(const joint_state& joint, float pitch, float dpitch, float az,
                       float dt, float wheel_side_mass, float gravity)
{
    return solver_.solve(joint, pitch, dpitch, az, dt, wheel_side_mass, gravity);
}

bool leg_solver::resolve_torque(const virtual_force& force, joint_torque& torque) const
{
    return solver_.vmc_cal(force, torque);
}

bool leg_solver::estimate_link_force(const joint_torque& torque, virtual_force& force) const
{
    return solver_.vmc_rev_cal(torque, force);
}

void leg_solver::reset()
{
    solver_.reset();
}

} // namespace wbr
