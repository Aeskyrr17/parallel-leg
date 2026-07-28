#include "control_task.hpp"

extern "C" void app_start()
{
    (void)wbr::start_control_task();
}
