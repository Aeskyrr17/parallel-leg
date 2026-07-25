#pragma once

#include "leg_config.hpp"
#include "leg_types.hpp"

namespace wbr::control
{

class VMCsolver
{
public:
    explicit VMCsolver(const chassis_config& cfg);

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

} // namespace wbr::control
