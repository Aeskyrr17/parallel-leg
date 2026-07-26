#include "leg/Inc/leg.hpp"
#include "leg/Inc/leg_tasks.hpp"

#include "config.hpp"
#include "remoter.hpp"

extern "C"
{
volatile bool application_started = false;
}

extern "C" void app_start()
{
    ::remoter::config remoter_config{};
    remoter_config.dr16.thread_priority = params::remoter::thread_priority;
    remoter_config.dr16.rx_timeout_ticks = params::remoter::rx_timeout_ticks;
    remoter_config.thread_priority = params::remoter::thread_priority + 1U;

    const bool remoter_started = ::remoter::service::instance().init(remoter_config);
    const bool leg_started = app::leg::instance().start();
    const bool command_started = remoter_started && app::command_task::instance().start();
    application_started = remoter_started && leg_started && command_started;
}
