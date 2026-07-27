#include "leg.hpp"

#include "constants.hpp"
#include "constrain.hpp"

namespace wbr::control
{

leg_controller::leg_controller(motors::api& joint1, motors::api& joint4,
                               const leg_dir& dir, const leg_config& cfg)
    : joint1_(joint1), joint4_(joint4), dir_(dir), cfg_(cfg), solver_(cfg),
      len_pd_(cfg.len_pid)
{
}

void leg_controller::solve(const motors::feedback& joint1_fdb,
                           const motors::feedback& joint4_fdb, float pitch, float dpitch,
                           float az, float dt, float wheel_side_mass, float gravity)
{
    joint_state joint{};
    joint.q[0] = math::pi + dir_.joint1_dir * joint1_fdb.position;
    joint.q[1] = dir_.joint4_dir * joint4_fdb.position;
    joint.dq[0] = dir_.joint1_dir * joint1_fdb.velocity;
    joint.dq[1] = dir_.joint4_dir * joint4_fdb.velocity;
    joint.tau[0] = dir_.joint1_dir * joint1_fdb.torque;
    joint.tau[1] = dir_.joint4_dir * joint4_fdb.torque;

    solver_.solve(joint, pitch, dpitch, az, dt, wheel_side_mass, gravity);
}

float leg_controller::len_control(float len_ref)
{
    const link_state& state = link();
    if (!state.valid)
    {
        return 0.0f;
    }

    len_pd_.ref = len_ref;
    len_pd_.fdb = state.len;
    len_pd_.update(state.dlen);
    return len_pd_.result;
}

bool leg_controller::resolve_torque(const virtual_force& force, float& joint1_tau,
                                    float& joint4_tau) const
{
    joint1_tau = 0.0f;
    joint4_tau = 0.0f;

    joint_torque tau{};
    if (!solver_.vmc_cal(force, tau))
    {
        return false;
    }

    joint1_tau = math::limit_abs(tau.t1, cfg_.max_hip_tau) * dir_.joint1_dir;
    joint4_tau = math::limit_abs(tau.t4, cfg_.max_hip_tau) * dir_.joint4_dir;
    return true;
}

void leg_controller::write_torque(float joint1_tau, float joint4_tau)
{
    motors::command j1_cmd{};
    motors::command j4_cmd{};
    j1_cmd.torque = joint1_tau;
    j4_cmd.torque = joint4_tau;
    joint1_.set_command(j1_cmd, motors::mode::torque);
    joint4_.set_command(j4_cmd, motors::mode::torque);
}

void leg_controller::reset_control()
{
    len_pd_.clear();
}

void leg_controller::reset()
{
    reset_control();
    solver_.reset();
}

void leg_controller::relax()
{
    reset();
    joint1_.relax();
    joint4_.relax();
}

} // namespace wbr::control
