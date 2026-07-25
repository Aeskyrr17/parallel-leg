#pragma once

#include "leg_config.hpp"

#include <array>
#include <cmath>

namespace app
{

struct leg_force
{
    float axial_force_n = 0.0f;
    float torque_nm = 0.0f;
};

struct joint_torque
{
    float joint_1_nm = 0.0f;
    float joint_4_nm = 0.0f;
};

struct joint_velocity
{
    float joint_1_rad_s = 0.0f;
    float joint_4_rad_s = 0.0f;
};

struct leg_velocity
{
    float length_mps = 0.0f;
    float angle_rad_s = 0.0f;
};

class vmc
{
public:
    [[nodiscard]] bool resolve(float joint_1_angle_rad, float joint_4_angle_rad) noexcept
    {
        invalidate();

        if (!finite(joint_1_angle_rad) || !finite(joint_4_angle_rad))
        {
            return false;
        }

        joint_1_angle_rad_ = joint_1_angle_rad;
        joint_4_angle_rad_ = joint_4_angle_rad;

        const float sin_1 = std::sin(joint_1_angle_rad_);
        const float cos_1 = std::cos(joint_1_angle_rad_);
        const float sin_4 = std::sin(joint_4_angle_rad_);
        const float cos_4 = std::cos(joint_4_angle_rad_);

        const float x_db = leg_config::motor_distance_m +
                           leg_config::vmc_link_1_length_m * (cos_4 - cos_1);
        const float y_db = leg_config::vmc_link_1_length_m * (sin_4 - sin_1);

        const float a0 = 2.0f * leg_config::vmc_link_2_length_m * x_db;
        const float b0 = 2.0f * leg_config::vmc_link_2_length_m * y_db;
        const float c0 = x_db * x_db + y_db * y_db;
        float discriminant = a0 * a0 + b0 * b0 - c0 * c0;

        if (!finite(discriminant) || discriminant < -discriminant_tolerance)
        {
            invalidate();
            return false;
        }
        if (discriminant < 0.0f)
        {
            discriminant = 0.0f;
        }

        const float u2_half = std::atan2(b0 + std::sqrt(discriminant), a0 + c0);
        u2_rad_ = 2.0f * u2_half;

        coordinate_b_[0] = leg_config::vmc_link_1_length_m * cos_1 -
                           leg_config::half_motor_distance_m;
        coordinate_b_[1] = leg_config::vmc_link_1_length_m * sin_1;

        coordinate_c_[0] = coordinate_b_[0] +
                           leg_config::vmc_link_2_length_m * std::cos(u2_rad_);
        coordinate_c_[1] = coordinate_b_[1] +
                           leg_config::vmc_link_2_length_m * std::sin(u2_rad_);

        coordinate_d_[0] = leg_config::vmc_link_1_length_m * cos_4 +
                           leg_config::half_motor_distance_m;
        coordinate_d_[1] = leg_config::vmc_link_1_length_m * sin_4;

        u3_rad_ = pi + std::atan2(coordinate_d_[1] - coordinate_c_[1],
                                 coordinate_d_[0] - coordinate_c_[0]);

        leg_angle_rad_ = std::atan2(coordinate_c_[1], coordinate_c_[0]);
        leg_length_m_ = std::sqrt(coordinate_c_[0] * coordinate_c_[0] +
                                  coordinate_c_[1] * coordinate_c_[1]);

        if (!finite_geometry() || leg_length_m_ < min_leg_length_m)
        {
            invalidate();
            return false;
        }

        const float sin_32 = std::sin(u3_rad_ - u2_rad_);
        const float sin_12 = std::sin(joint_1_angle_rad_ - u2_rad_);
        const float sin_34 = std::sin(u3_rad_ - joint_4_angle_rad_);

        if (std::fabs(sin_32) < min_sine ||
            std::fabs(sin_12) < min_sine ||
            std::fabs(sin_34) < min_sine)
        {
            invalidate();
            return false;
        }

        const float cos_03 = std::cos(leg_angle_rad_ - u3_rad_);
        const float cos_02 = std::cos(leg_angle_rad_ - u2_rad_);
        const float sin_03 = std::sin(leg_angle_rad_ - u3_rad_);
        const float sin_02 = std::sin(leg_angle_rad_ - u2_rad_);
        const float link_1 = leg_config::vmc_link_1_length_m;

        jacobian_[0] = link_1 * sin_03 * sin_12 / sin_32;
        jacobian_[1] = link_1 * sin_02 * sin_34 / sin_32;
        jacobian_[2] = link_1 * cos_03 * sin_12 / (sin_32 * leg_length_m_);
        jacobian_[3] = link_1 * cos_02 * sin_34 / (sin_32 * leg_length_m_);

        jacobian_transpose_[0] = link_1 * sin_03 * sin_12 / sin_32;
        jacobian_transpose_[1] = link_1 * cos_03 * sin_12 / (sin_32 * leg_length_m_);
        jacobian_transpose_[2] = link_1 * sin_02 * sin_34 / sin_32;
        jacobian_transpose_[3] = link_1 * cos_02 * sin_34 / (sin_32 * leg_length_m_);

        jacobian_transpose_inverse_[0] = -cos_02 / (sin_12 * link_1);
        jacobian_transpose_inverse_[1] = cos_03 / (sin_34 * link_1);
        jacobian_transpose_inverse_[2] = sin_02 * leg_length_m_ / (sin_12 * link_1);
        jacobian_transpose_inverse_[3] = -sin_03 * leg_length_m_ / (sin_34 * link_1);

        if (!finite_matrix(jacobian_) ||
            !finite_matrix(jacobian_transpose_) ||
            !finite_matrix(jacobian_transpose_inverse_))
        {
            invalidate();
            return false;
        }

        valid_ = true;
        return true;
    }

    [[nodiscard]] joint_torque force_to_joint_torque(const leg_force& force) const noexcept
    {
        if (!valid_ || !finite(force.axial_force_n) || !finite(force.torque_nm))
        {
            return {};
        }

        joint_torque output{
            jacobian_transpose_[0] * force.axial_force_n +
                jacobian_transpose_[1] * force.torque_nm,
            jacobian_transpose_[2] * force.axial_force_n +
                jacobian_transpose_[3] * force.torque_nm,
        };

        return finite(output.joint_1_nm) && finite(output.joint_4_nm)
                   ? output
                   : joint_torque{};
    }

    [[nodiscard]] leg_force joint_torque_to_force(const joint_torque& torque) const noexcept
    {
        if (!valid_ || !finite(torque.joint_1_nm) || !finite(torque.joint_4_nm))
        {
            return {};
        }

        leg_force output{
            jacobian_transpose_inverse_[0] * torque.joint_1_nm +
                jacobian_transpose_inverse_[1] * torque.joint_4_nm,
            jacobian_transpose_inverse_[2] * torque.joint_1_nm +
                jacobian_transpose_inverse_[3] * torque.joint_4_nm,
        };

        return finite(output.axial_force_n) && finite(output.torque_nm)
                   ? output
                   : leg_force{};
    }

    [[nodiscard]] leg_velocity
    joint_velocity_to_leg_velocity(const joint_velocity& velocity) const noexcept
    {
        if (!valid_ ||
            !finite(velocity.joint_1_rad_s) ||
            !finite(velocity.joint_4_rad_s))
        {
            return {};
        }

        leg_velocity output{
            jacobian_[0] * velocity.joint_1_rad_s +
                jacobian_[1] * velocity.joint_4_rad_s,
            jacobian_[2] * velocity.joint_1_rad_s +
                jacobian_[3] * velocity.joint_4_rad_s,
        };

        return finite(output.length_mps) && finite(output.angle_rad_s)
                   ? output
                   : leg_velocity{};
    }

    [[nodiscard]] float leg_length_m() const noexcept
    {
        return leg_length_m_;
    }

    [[nodiscard]] float leg_angle_rad() const noexcept
    {
        return leg_angle_rad_;
    }

    [[nodiscard]] bool valid() const noexcept
    {
        return valid_;
    }

private:
    static constexpr float pi = 3.14159265358979323846f;
    static constexpr float min_leg_length_m = 1.0e-4f;
    static constexpr float min_sine = 1.0e-5f;
    static constexpr float discriminant_tolerance = 1.0e-6f;

    static bool finite(float value) noexcept
    {
        return std::isfinite(value);
    }

    static bool finite_matrix(const std::array<float, 4>& matrix) noexcept
    {
        for (const float value : matrix)
        {
            if (!finite(value))
            {
                return false;
            }
        }
        return true;
    }

    bool finite_geometry() const noexcept
    {
        return finite(u2_rad_) &&
               finite(u3_rad_) &&
               finite(leg_length_m_) &&
               finite(leg_angle_rad_) &&
               finite(coordinate_b_[0]) &&
               finite(coordinate_b_[1]) &&
               finite(coordinate_c_[0]) &&
               finite(coordinate_c_[1]) &&
               finite(coordinate_d_[0]) &&
               finite(coordinate_d_[1]);
    }

    void invalidate() noexcept
    {
        jacobian_.fill(0.0f);
        jacobian_transpose_.fill(0.0f);
        jacobian_transpose_inverse_.fill(0.0f);
        coordinate_b_.fill(0.0f);
        coordinate_c_.fill(0.0f);
        coordinate_d_.fill(0.0f);
        joint_1_angle_rad_ = 0.0f;
        joint_4_angle_rad_ = 0.0f;
        u2_rad_ = 0.0f;
        u3_rad_ = 0.0f;
        leg_length_m_ = 0.0f;
        leg_angle_rad_ = 0.0f;
        valid_ = false;
    }

    std::array<float, 4> jacobian_{};
    std::array<float, 4> jacobian_transpose_{};
    std::array<float, 4> jacobian_transpose_inverse_{};
    std::array<float, 2> coordinate_b_{};
    std::array<float, 2> coordinate_c_{};
    std::array<float, 2> coordinate_d_{};

    float joint_1_angle_rad_ = 0.0f;
    float joint_4_angle_rad_ = 0.0f;
    float u2_rad_ = 0.0f;
    float u3_rad_ = 0.0f;
    float leg_length_m_ = 0.0f;
    float leg_angle_rad_ = 0.0f;
    bool valid_ = false;
};

} // namespace app
