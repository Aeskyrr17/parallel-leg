#pragma once

#include "lkmotors.hpp"

namespace app
{

class leg
{
public:
    static leg& instance() noexcept;

    bool start() noexcept;
    [[nodiscard]] bool started() const noexcept { return started_; }

    motors::lk8016& left_joint_4() noexcept { return left_joint_4_; }
    motors::lk8016& left_joint_1() noexcept { return left_joint_1_; }
    motors::lk8016& right_joint_4() noexcept { return right_joint_4_; }
    motors::lk8016& right_joint_1() noexcept { return right_joint_1_; }
    motors::lk9025& left_wheel() noexcept { return left_wheel_; }
    motors::lk9025& right_wheel() noexcept { return right_wheel_; }

private:
    leg();

    motors::lk8016 left_joint_4_;
    motors::lk8016 left_joint_1_;
    motors::lk8016 right_joint_4_;
    motors::lk8016 right_joint_1_;
    motors::lk9025 left_wheel_;
    motors::lk9025 right_wheel_;

    bool started_ = false;
};

} // namespace app
