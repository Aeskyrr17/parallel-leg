#pragma once

#include "constrain.hpp"
#include "control_config.hpp"
#include "lqr_coeffs.hpp"

#include <cmath>

namespace wbr
{

struct lqr_state
{
    float x = 0.0f;  // m
    float dx = 0.0f; // m/s

    float phi = 0.0f;  // rad, chassis yaw
    float dphi = 0.0f; // rad/s

    float theta_l_l = 0.0f;  // rad
    float dtheta_l_l = 0.0f; // rad/s

    float theta_l_r = 0.0f;  // rad
    float dtheta_l_r = 0.0f; // rad/s

    float theta_b = 0.0f;  // rad, body pitch
    float dtheta_b = 0.0f; // rad/s
};

struct lqr_output
{
    float tau_w_l = 0.0f; // N*m
    float tau_w_r = 0.0f; // N*m

    float tau_l_l = 0.0f; // N*m
    float tau_l_r = 0.0f; // N*m
};

struct lqr_state_diagnostic
{
    float error = 0.0f;
    float tau_w_l = 0.0f;
    float tau_w_r = 0.0f;
    float tau_l_l = 0.0f;
    float tau_l_r = 0.0f;
};

struct lqr_diagnostics
{
    lqr_state_diagnostic x{};
    lqr_state_diagnostic dx{};
    lqr_state_diagnostic phi{};
    lqr_state_diagnostic dphi{};
    lqr_state_diagnostic theta_l_l{};
    lqr_state_diagnostic dtheta_l_l{};
    lqr_state_diagnostic theta_l_r{};
    lqr_state_diagnostic dtheta_l_r{};
    lqr_state_diagnostic theta_b{};
    lqr_state_diagnostic dtheta_b{};
    lqr_output total{};
};

class LQR
{
public:
    explicit LQR(const lqr_config& cfg) : cfg_(cfg) {}

    lqr_output solve(float left_leg_len, float right_leg_len, bool offground,
                     const lqr_state& obs, const lqr_state& ref,
                     lqr_diagnostics* diagnostics = nullptr) const
    {
        lqr_output output{};
        if (diagnostics != nullptr)
        {
            *diagnostics = {};
        }
        const float left =
            std::round(math::clamp(left_leg_len, cfg_.min_leg_len, cfg_.max_leg_len) /
                       cfg_.leg_len_resolution) *
            cfg_.leg_len_resolution;
        const float right =
            std::round(math::clamp(right_leg_len, cfg_.min_leg_len, cfg_.max_leg_len) /
                       cfg_.leg_len_resolution) *
            cfg_.leg_len_resolution;
        const float polynomial[6] = {
            1.0f, left, right, left * left, left * right, right * right,
        };

        float gain[40] = {};
        for (int row = 0; row < 40; ++row)
        {
            if (offground && !offground_coefficient(row))
            {
                continue;
            }
            for (int coefficient = 0; coefficient < 6; ++coefficient)
            {
                gain[row] += k_lqr_coefficients[row][coefficient] * polynomial[coefficient];
            }
        }

        const float error[10] = {
            obs.x - ref.x,
            obs.dx - ref.dx,
            obs.phi - ref.phi,
            obs.dphi - ref.dphi,
            obs.theta_l_l - ref.theta_l_l,
            obs.dtheta_l_l - ref.dtheta_l_l,
            obs.theta_l_r - ref.theta_l_r,
            obs.dtheta_l_r - ref.dtheta_l_r,
            obs.theta_b - ref.theta_b,
            obs.dtheta_b - ref.dtheta_b,
        };

        if (diagnostics != nullptr)
        {
            for (int state = 0; state < 10; ++state)
            {
                state_diagnostic(*diagnostics, state).error = error[state];
            }
        }

        float tau[4] = {};
        for (int actuator = 0; actuator < 4; ++actuator)
        {
            for (int state = 0; state < 10; ++state)
            {
                const float contribution = gain[actuator * 10 + state] * error[state];
                tau[actuator] += contribution;
                if (diagnostics != nullptr)
                {
                    lqr_state_diagnostic& state_output =
                        state_diagnostic(*diagnostics, state);
                    if (actuator == 0)
                    {
                        state_output.tau_w_l = contribution;
                    }
                    else if (actuator == 1)
                    {
                        state_output.tau_w_r = contribution;
                    }
                    else if (actuator == 2)
                    {
                        state_output.tau_l_l = contribution;
                    }
                    else
                    {
                        state_output.tau_l_r = contribution;
                    }
                }
            }
        }
        output.tau_w_l = tau[0];
        output.tau_w_r = tau[1];
        output.tau_l_l = tau[2];
        output.tau_l_r = tau[3];
        if (diagnostics != nullptr)
        {
            diagnostics->total = output;
        }
        return output;
    }

private:
    static lqr_state_diagnostic& state_diagnostic(lqr_diagnostics& diagnostics, int state)
    {
        switch (state)
        {
        case 0: return diagnostics.x;
        case 1: return diagnostics.dx;
        case 2: return diagnostics.phi;
        case 3: return diagnostics.dphi;
        case 4: return diagnostics.theta_l_l;
        case 5: return diagnostics.dtheta_l_l;
        case 6: return diagnostics.theta_l_r;
        case 7: return diagnostics.dtheta_l_r;
        case 8: return diagnostics.theta_b;
        default: return diagnostics.dtheta_b;
        }
    }

    static bool offground_coefficient(int index)
    {
        return (index >= 24 && index < 28) || (index >= 34 && index < 38);
    }

    lqr_config cfg_{};
};

} // namespace wbr
