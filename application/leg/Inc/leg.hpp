#pragma once

namespace app
{

class leg
{
public:
    static leg& instance() noexcept;

    bool start() noexcept;
    [[nodiscard]] bool started() const noexcept { return started_; }

private:
    leg() = default;

    bool started_ = false;
};

} // namespace app
