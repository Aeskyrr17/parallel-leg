#include "vmc.hpp"

#include "leg_math.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace wbr::control
{
namespace
{

bool finite_four(const float values[4])
{
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]) &&
           std::isfinite(values[3]);
}

bool finite_joint(const joint_state& joint)
{
    return std::isfinite(joint.q[0]) && std::isfinite(joint.q[1]) && std::isfinite(joint.dq[0]) &&
           std::isfinite(joint.dq[1]) && std::isfinite(joint.tau[0]) && std::isfinite(joint.tau[1]);
}

bool finite_state(const link_state& state)
{
    return std::isfinite(state.phi1) && std::isfinite(state.phi4) && std::isfinite(state.phi) &&
           std::isfinite(state.dphi) && std::isfinite(state.alpha) &&
           std::isfinite(state.dalpha) && std::isfinite(state.len) &&
           std::isfinite(state.dlen) && std::isfinite(state.freal) &&
           std::isfinite(state.treal) && std::isfinite(state.fs) && std::isfinite(state.n);
}

} // namespace

VMCsolver::VMCsolver(const chassis_config& cfg) : cfg_(cfg) {}

bool VMCsolver::solve(const joint_state& joint, float pitch, float dpitch, float az, float dt)
{
    state_.valid = false;
    jacobian_valid_ = false;

    if (!joint.valid || !finite_joint(joint) || !std::isfinite(pitch) || !std::isfinite(dpitch) ||
        !std::isfinite(az) || !std::isfinite(dt) || dt < cfg_.runtime.min_dt_s ||
        dt > cfg_.runtime.max_dt_s)
    {
        invalidate();
        return false;
    }

    if (!resolve(joint.q[0], joint.q[1]))
    {
        const bool reachable = state_.reachable;
        const bool near_singularity = state_.near_singularity;
        invalidate(reachable, near_singularity);
        return false;
    }

    state_.dlen = j_mat_[0] * joint.dq[0] + j_mat_[1] * joint.dq[1];
    state_.dphi = j_mat_[2] * joint.dq[0] + j_mat_[3] * joint.dq[1];

    state_.alpha = loop_clamp(state_.phi - 0.5f * k_pi + pitch, -k_pi, k_pi);
    state_.dalpha = state_.dphi + dpitch;

    state_.freal = jt_inv_mat_[0] * joint.tau[0] + jt_inv_mat_[1] * joint.tau[1];
    state_.treal = jt_inv_mat_[2] * joint.tau[0] + jt_inv_mat_[3] * joint.tau[1];

    calc_support_force(az, dt);

    if (!finite_state(state_))
    {
        invalidate(state_.reachable, state_.near_singularity);
        return false;
    }

    state_.valid = true;
    return true;
}

joint_torque VMCsolver::vmc_cal(const virtual_force& force) const
{
    joint_torque torque{};
    if (!state_.valid || !jacobian_valid_ || !std::isfinite(force.f) || !std::isfinite(force.tp))
    {
        return torque;
    }

    torque.t1 = jt_mat_[0] * force.f + jt_mat_[1] * force.tp;
    torque.t4 = jt_mat_[2] * force.f + jt_mat_[3] * force.tp;
    torque.valid = std::isfinite(torque.t1) && std::isfinite(torque.t4);
    if (!torque.valid)
    {
        torque.t1 = 0.0f;
        torque.t4 = 0.0f;
    }
    return torque;
}

void VMCsolver::reset()
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

bool VMCsolver::resolve(float phi1, float phi4)
{
    state_.phi1 = phi1;
    state_.phi4 = phi4;
    state_.reachable = false;
    state_.near_singularity = false;

    const float sin1 = std::sin(phi1);
    const float cos1 = std::cos(phi1);
    const float sin4 = std::sin(phi4);
    const float cos4 = std::cos(phi4);

    const float xdb = cfg_.l1 * (cos4 - cos1);
    const float ydb = cfg_.l1 * (sin4 - sin1);
    const float a0 = 2.0f * cfg_.l2 * xdb;
    const float b0 = 2.0f * cfg_.l2 * ydb;
    const float c0 = xdb * xdb + ydb * ydb;
    const float discriminant = a0 * a0 + b0 * b0 - c0 * c0;

    if (!std::isfinite(discriminant) || discriminant < 0.0f)
    {
        return false;
    }

    const float root = std::sqrt(discriminant);
    u2_ = 2.0f * std::atan2(b0 + root, a0 + c0);

    const float bx = cfg_.l1 * cos1;
    const float by = cfg_.l1 * sin1;
    const float cx = bx + cfg_.l2 * std::cos(u2_);
    const float cy = by + cfg_.l2 * std::sin(u2_);
    const float dx = cfg_.l1 * cos4;
    const float dy = cfg_.l1 * sin4;

    u3_ = k_pi + std::atan2(dy - cy, dx - cx);
    state_.phi = std::atan2(cy, cx);
    state_.len = std::sqrt(cx * cx + cy * cy);
    state_.reachable = std::isfinite(u2_) && std::isfinite(u3_) && std::isfinite(state_.phi) &&
                       std::isfinite(state_.len);
    if (!state_.reachable)
    {
        return false;
    }

    if (!calc_jacobian())
    {
        state_.near_singularity = true;
        return false;
    }

    calc_spring_force();
    return std::isfinite(state_.fs);
}

bool VMCsolver::calc_jacobian()
{
    const float sin32 = std::sin(u3_ - u2_);
    const float sin12 = std::sin(state_.phi1 - u2_);
    const float sin34 = std::sin(u3_ - state_.phi4);

    const float epsilon = cfg_.numerics.singularity_epsilon;
    if (std::fabs(sin32) <= epsilon || std::fabs(sin12) <= epsilon || std::fabs(sin34) <= epsilon ||
        std::fabs(state_.len) <= epsilon || std::fabs(cfg_.l1) <= epsilon)
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

    jacobian_valid_ = finite_four(j_mat_) && finite_four(jt_mat_) && finite_four(jt_inv_mat_);
    return jacobian_valid_;
}

void VMCsolver::calc_spring_force()
{
    state_.fs = 0.0f;

    const float sin32 = std::sin(u3_ - u2_);
    const float epsilon = cfg_.numerics.singularity_epsilon;
    if (std::fabs(sin32) < cfg_.numerics.spring_singularity_epsilon ||
        std::fabs(cfg_.l2) <= epsilon)
    {
        return;
    }

    const float ang_p = state_.phi1 + cfg_.ang_spring - 0.5f * k_pi;
    const float px = cfg_.dspring1 * std::cos(ang_p);
    const float py = cfg_.dspring1 * std::sin(ang_p);

    const float cos1 = std::cos(state_.phi1);
    const float sin1 = std::sin(state_.phi1);
    const float cos2 = std::cos(u2_);
    const float sin2 = std::sin(u2_);
    const float qx = cfg_.l1 * cos1 + cfg_.dspring2 * cos2;
    const float qy = cfg_.l1 * sin1 + cfg_.dspring2 * sin2;

    const float dpqx = qx - px;
    const float dpqy = qy - py;
    const float spring_len_sq = dpqx * dpqx + dpqy * dpqy;
    if (!std::isfinite(spring_len_sq) || spring_len_sq <= epsilon * epsilon)
    {
        return;
    }

    const float spring_len = std::sqrt(spring_len_sq);
    const float fsx = cfg_.fspring * dpqx / spring_len;
    const float fsy = cfg_.fspring * dpqy / spring_len;
    const float inv_l2_sin32 = 1.0f / (cfg_.l2 * sin32);
    const float dphi2_dphi1 = cfg_.l1 * std::sin(u3_ - state_.phi1) * inv_l2_sin32;
    const float dphi2_dphi4 = cfg_.l1 * std::sin(state_.phi4 - u3_) * inv_l2_sin32;

    const float dpx_d1 = -cfg_.dspring1 * std::sin(ang_p);
    const float dpy_d1 = cfg_.dspring1 * std::cos(ang_p);
    const float dqx_d1 = -cfg_.l1 * sin1 - cfg_.dspring2 * sin2 * dphi2_dphi1;
    const float dqy_d1 = cfg_.l1 * cos1 + cfg_.dspring2 * cos2 * dphi2_dphi1;
    const float dqx_d4 = -cfg_.dspring2 * sin2 * dphi2_dphi4;
    const float dqy_d4 = cfg_.dspring2 * cos2 * dphi2_dphi4;

    const float tau_s1 = fsx * (dqx_d1 - dpx_d1) + fsy * (dqy_d1 - dpy_d1);
    const float tau_s4 = fsx * dqx_d4 + fsy * dqy_d4;
    const float spring_force = jt_inv_mat_[0] * tau_s1 + jt_inv_mat_[1] * tau_s4;
    if (std::isfinite(spring_force))
    {
        state_.fs = spring_force;
    }
}

void VMCsolver::calc_support_force(float az, float dt)
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
    const float projected_force = (state_.freal + state_.fs) * cos_alpha;
    const float wheel_vertical_acceleration =
        (az - cfg_.gravity) - ddlen * cos_alpha + 2.0f * state_.dlen * state_.dalpha * sin_alpha +
        state_.len * ddalpha * sin_alpha + state_.len * state_.dalpha * state_.dalpha * cos_alpha;

    state_.n = projected_force + cfg_.wheel_side_mass * cfg_.gravity +
               cfg_.wheel_side_mass * wheel_vertical_acceleration;
}

void VMCsolver::invalidate(bool reachable, bool near_singularity)
{
    state_ = {};
    state_.reachable = reachable;
    state_.near_singularity = near_singularity;
    jacobian_valid_ = false;
    derivative_history_valid_ = false;
    prev_dlen_ = 0.0f;
    prev_dalpha_ = 0.0f;
}

} // namespace wbr::control
