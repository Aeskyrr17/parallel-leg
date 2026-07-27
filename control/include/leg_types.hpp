#pragma once

namespace wbr::control
{

struct joint_state
{
    // Unified mechanical coordinates: rad, rad/s, N*m.
    float q[2] = {};
    float dq[2] = {};
    float tau[2] = {};
};

struct virtual_force
{
    float F = 0.0f;  // N, positive extends the leg
    float Tp = 0.0f; // N*m, positive counter-clockwise
};

struct joint_torque
{
    // N*m, unified mechanical coordinates.
    float t1 = 0.0f;
    float t4 = 0.0f;
};

struct link_state
{
    float phi1 = 0.0f;
    float phi4 = 0.0f;

    float phi = 0.0f;    // rad, relative to body +x
    float dphi = 0.0f;   // rad/s
    float alpha = 0.0f;  // rad, relative to world vertical; pitch included
    float dalpha = 0.0f; // rad/s; pitch rate included

    float len = 0.0f;
    float dlen = 0.0f;

    virtual_force fdb{};
    float Fs = 0.0f; // N, equivalent spring force along the leg
    float N = 0.0f;  // N, support force
    bool valid = false;
};

} // namespace wbr::control
