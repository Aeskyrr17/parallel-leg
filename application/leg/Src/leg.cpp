#include "leg.hpp"

namespace app
{

leg& leg::instance() noexcept
{
    static leg instance;
    return instance;
}

bool leg::start() noexcept
{
    if (started_)
    {
        return true;
    }

    started_ = true;
    return true;
}

} // namespace app
