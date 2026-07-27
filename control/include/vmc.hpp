#pragma once

#include "constants.hpp"
#include "constrain.hpp"
#include "control_config.hpp"
#include "leg_types.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace wbr::control
{

class link_solver
{
public:
    explicit link_solver(const leg_config& cfg);

    bool solve(const joint_state& joint, float pitch, float dpitch, float az, float dt,
               float wheel_side_mass, float gravity);

    bool vmc_cal(const virtual_force& force, joint_torque& tau) const;
    bool vmc_rev_cal(const joint_torque& tau, virtual_force& force) const;

    const link_state& state() const { return state_; }

    void reset();

private:
    bool resolve(float phi1, float phi4);
    bool calc_jacobian();
    void calc_spring_force();
    void calc_support_force(float az, float dt, float wheel_side_mass, float gravity);
    void invalidate();

    const leg_config& cfg_;
    link_state state_{};

    float u2_ = 0.0f;
    float u3_ = 0.0f;
    float j_mat_[4] = {};
    float jt_mat_[4] = {};
    float jt_inv_mat_[4] = {};

    float prev_dlen_ = 0.0f;
    float prev_dalpha_ = 0.0f;

    bool jacobian_valid_ = false;
    bool derivative_history_valid_ = false;
};

inline link_solver::link_solver(const leg_config& cfg) : cfg_(cfg) {}

inline bool link_solver::solve(const joint_state& joint, float pitch, float dpitch, float az, float dt,
                               float wheel_side_mass, float gravity)
{
    state_.valid = false;
    jacobian_valid_ = false;

    if (!resolve(joint.q[0], joint.q[1]))
    {
        invalidate();
        return false;
    }

    state_.dlen = j_mat_[0] * joint.dq[0] + j_mat_[1] * joint.dq[1];
    state_.dphi = j_mat_[2] * joint.dq[0] + j_mat_[3] * joint.dq[1];

    state_.alpha =
        math::clamp_loop(state_.phi - 0.5f * math::pi + pitch, -math::pi, math::pi);
    state_.dalpha = state_.dphi + dpitch;

    const joint_torque fdb_tau{joint.tau[0], joint.tau[1]};
    if (!vmc_rev_cal(fdb_tau, state_.fdb))
    {
        invalidate();
        return false;
    }

    calc_support_force(az, dt, wheel_side_mass, gravity);

    state_.valid = true;
    return true;
}

inline bool link_solver::vmc_cal(const virtual_force& force, joint_torque& tau) const
{
    tau = {};
    if (!state_.valid || !jacobian_valid_)
    {
        return false;
    }

    tau.t1 = jt_mat_[0] * force.F + jt_mat_[1] * force.Tp;
    tau.t4 = jt_mat_[2] * force.F + jt_mat_[3] * force.Tp;
    return true;
}

inline bool link_solver::vmc_rev_cal(const joint_torque& tau, virtual_force& force) const
{
    force = {};
    if (!jacobian_valid_)
    {
        return false;
    }

    force.F = jt_inv_mat_[0] * tau.t1 + jt_inv_mat_[1] * tau.t4;
    force.Tp = jt_inv_mat_[2] * tau.t1 + jt_inv_mat_[3] * tau.t4;
    return true;
}

inline void link_solver::reset()
{
    state_ = {};
    std::fill(std::begin(j_mat_), std::end(j_mat_), 0.0f);
    std::fill(std::begin(jt_mat_), std::end(jt_mat_), 0.0f);
    std::fill(std::begin(jt_inv_mat_), std::end(jt_inv_mat_), 0.0f);
    u2_ = 0.0f;
    u3_ = 0.0f;
    prev_dlen_ = 0.0f;
    prev_dalpha_ = 0.0f;
    jacobian_valid_ = false;
    derivative_history_valid_ = false;
}

inline bool link_solver::resolve(float phi1, float phi4)
{
    state_.phi1 = phi1;
    state_.phi4 = phi4;
    const float sin1 = std::sin(phi1);
    const float cos1 = std::cos(phi1);
    const float sin4 = std::sin(phi4);
    const float cos4 = std::cos(phi4);

    const float half_motor_distance = 0.5f * cfg_.motor_distance;
    const float xdb = cfg_.motor_distance + cfg_.l1 * (cos4 - cos1);
    const float ydb = cfg_.l1 * (sin4 - sin1);
    const float a0 = 2.0f * cfg_.l2 * xdb;
    const float b0 = 2.0f * cfg_.l2 * ydb;
    const float c0 = xdb * xdb + ydb * ydb;
    const float discriminant = a0 * a0 + b0 * b0 - c0 * c0;

    if (discriminant < 0.0f)
    {
        return false;
    }

    const float root = std::sqrt(discriminant);
    u2_ = 2.0f * std::atan2(b0 + root, a0 + c0);

    const float bx = cfg_.l1 * cos1 - half_motor_distance;
    const float by = cfg_.l1 * sin1;
    const float cx = bx + cfg_.l2 * std::cos(u2_);
    const float cy = by + cfg_.l2 * std::sin(u2_);
    const float dx = cfg_.l1 * cos4 + half_motor_distance;
    const float dy = cfg_.l1 * sin4;

    u3_ = math::pi + std::atan2(dy - cy, dx - cx);
    state_.phi = std::atan2(cy, cx);
    state_.len = std::sqrt(cx * cx + cy * cy);

    if (!calc_jacobian())
    {
        return false;
    }

    calc_spring_force();
    return true;
}

inline bool link_solver::calc_jacobian()
{
    const float sin32 = std::sin(u3_ - u2_);
    const float sin12 = std::sin(state_.phi1 - u2_);
    const float sin34 = std::sin(u3_ - state_.phi4);

    const float epsilon = cfg_.numerics.singularity_epsilon;
    if (std::fabs(sin32) <= epsilon || std::fabs(sin12) <= epsilon ||
        std::fabs(sin34) <= epsilon || std::fabs(state_.len) <= epsilon)
    {
        return false;
    }

    const float cos03 = std::cos(state_.phi - u3_);
    const float cos02 = std::cos(state_.phi - u2_);
    const float sin03 = std::sin(state_.phi - u3_);
    const float sin02 = std::sin(state_.phi - u2_);

    j_mat_[0] = cfg_.l1 * sin03 * sin12 / sin32;
    j_mat_[1] = cfg_.l1 * sin02 * sin34 / sin32;
    j_mat_[2] = cfg_.l1 * cos03 * sin12 / (sin32 * state_.len);
    j_mat_[3] = cfg_.l1 * cos02 * sin34 / (sin32 * state_.len);

    jt_mat_[0] = j_mat_[0];
    jt_mat_[1] = j_mat_[2];
    jt_mat_[2] = j_mat_[1];
    jt_mat_[3] = j_mat_[3];

    jt_inv_mat_[0] = -cos02 / (sin12 * cfg_.l1);
    jt_inv_mat_[1] = cos03 / (sin34 * cfg_.l1);
    jt_inv_mat_[2] = sin02 * state_.len / (sin12 * cfg_.l1);
    jt_inv_mat_[3] = -sin03 * state_.len / (sin34 * cfg_.l1);

    jacobian_valid_ = true;
    return true;
}

inline void link_solver::calc_spring_force()
{
    state_.Fs = 0.0f;

    if (!cfg_.has_spring)
    {
        return;
    }

    // phi2 implicit derivative uses sin(phi2 - phi3).
    const float sin23 = std::sin(u2_ - u3_);
    const float epsilon = cfg_.numerics.singularity_epsilon;
    if (std::fabs(sin23) < cfg_.numerics.spring_singularity_epsilon)
    {
        return;
    }

    const float ang_p = state_.phi1 + cfg_.spring_angle - 0.5f * math::pi;
    const float px = cfg_.spring_offset_1 * std::cos(ang_p);
    const float py = cfg_.spring_offset_1 * std::sin(ang_p);

    const float cos1 = std::cos(state_.phi1);
    const float sin1 = std::sin(state_.phi1);
    const float cos2 = std::cos(u2_);
    const float sin2 = std::sin(u2_);
    const float qx = cfg_.l1 * cos1 + cfg_.spring_offset_2 * cos2;
    const float qy = cfg_.l1 * sin1 + cfg_.spring_offset_2 * sin2;

    const float dpqx = qx - px;
    const float dpqy = qy - py;
    const float spring_len_sq = dpqx * dpqx + dpqy * dpqy;
    if (spring_len_sq <= epsilon * epsilon)
    {
        return;
    }

    const float spring_len = std::sqrt(spring_len_sq);
    const float fsx = cfg_.spring_force * dpqx / spring_len;
    const float fsy = cfg_.spring_force * dpqy / spring_len;
    const float inv_l2_sin23 = 1.0f / (cfg_.l2 * sin23);
    const float dphi2_dphi1 = cfg_.l1 * std::sin(u3_ - state_.phi1) * inv_l2_sin23;
    const float dphi2_dphi4 = cfg_.l1 * std::sin(state_.phi4 - u3_) * inv_l2_sin23;

    const float dpx_d1 = -cfg_.spring_offset_1 * std::sin(ang_p);
    const float dpy_d1 = cfg_.spring_offset_1 * std::cos(ang_p);
    const float dqx_d1 = -cfg_.l1 * sin1 - cfg_.spring_offset_2 * sin2 * dphi2_dphi1;
    const float dqy_d1 = cfg_.l1 * cos1 + cfg_.spring_offset_2 * cos2 * dphi2_dphi1;
    const float dqx_d4 = -cfg_.spring_offset_2 * sin2 * dphi2_dphi4;
    const float dqy_d4 = cfg_.spring_offset_2 * cos2 * dphi2_dphi4;

    const float tau_s1 = fsx * (dqx_d1 - dpx_d1) + fsy * (dqy_d1 - dpy_d1);
    const float tau_s4 = fsx * dqx_d4 + fsy * dqy_d4;
    state_.Fs = jt_inv_mat_[0] * tau_s1 + jt_inv_mat_[1] * tau_s4;
}

inline void link_solver::calc_support_force(float az, float dt, float wheel_side_mass, float gravity)
{
    float ddlen = 0.0f;
    float ddalpha = 0.0f;

    if (derivative_history_valid_)
    {
        ddlen = (state_.dlen - prev_dlen_) / dt;
        ddalpha = (state_.dalpha - prev_dalpha_) / dt;
    }
    else
    {
        derivative_history_valid_ = true;
    }

    prev_dlen_ = state_.dlen;
    prev_dalpha_ = state_.dalpha;

    const float cos_alpha = std::cos(state_.alpha);
    const float sin_alpha = std::sin(state_.alpha);
    const float projected_force = (state_.fdb.F + state_.Fs) * cos_alpha;
    const float wheel_vertical_acceleration =
        (az - gravity) - ddlen * cos_alpha + 2.0f * state_.dlen * state_.dalpha * sin_alpha +
        state_.len * ddalpha * sin_alpha + state_.len * state_.dalpha * state_.dalpha * cos_alpha;

    state_.N = projected_force + wheel_side_mass * gravity +
               wheel_side_mass * wheel_vertical_acceleration;
}

inline void link_solver::invalidate()
{
    state_ = {};
    jacobian_valid_ = false;
    derivative_history_valid_ = false;
    prev_dlen_ = 0.0f;
    prev_dalpha_ = 0.0f;
}

} // namespace wbr::control
