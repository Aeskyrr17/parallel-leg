#pragma once

#include "usertypes.hpp"

namespace demo::usb
{

types::status start() noexcept;
void poll() noexcept;

} // namespace demo::usb
