#include "control_task.hpp"

extern "C" void app_start()
{
    (void)wbr::control::start_control_task();
}
