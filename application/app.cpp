#include "leg/Inc/leg.hpp"

extern "C"
{
volatile bool application_started = false;
}

extern "C" void app_start()
{
    application_started = app::leg::instance().start();
}
