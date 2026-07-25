#pragma once

namespace wbr::control
{

struct joint_state
{
    // Unified mechanical joint coordinates: rad, rad/s, N*m.
    float q[2] = {};
    float dq[2] = {};
    float tau[2] = {};

    bool valid = false;
};

struct leg_wrench //广义力
{
    // Virtual leg-axis force in N; positive extends the leg.
    float F = 0.0f;

    // Virtual hip torque in N*m; positive is counter-clockwise.
    float Tp = 0.0f;
};

struct joint_torque
{
    // Joint torques in N*m, expressed in unified mechanical coordinates.
    float t1 = 0.0f;
    float t4 = 0.0f;

    bool valid = false;
};

struct link_state
{
    // Joint and virtual-leg angles in rad; angular velocities in rad/s.
    float phi1 = 0.0f;
    float phi4 = 0.0f;

    float phi = 0.0f;
    float dphi = 0.0f;

    float alpha = 0.0f;
    float dalpha = 0.0f;

    // Leg length and velocity in m and m/s.
    float len = 0.0f;
    float dlen = 0.0f;

    // Generalized leg load reconstructed from joint torque feedback.
    leg_wrench fdb{};

    // Equivalent spring force and support force in N.
    float Fs = 0.0f;
    float N = 0.0f;

    bool reachable = false;
    bool near_singularity = false;
    bool valid = false;
};

} // namespace wbr::control
