#include "function_task.hpp"

#include "function.hpp"
#include "msg.hpp"
#include "msgs.hpp"
#include "remoter.hpp"
#include "tx_api.h"

#include <cstdint>

namespace wbr
{

namespace
{

Function function{k_default_control.chassis.cmd};

TX_THREAD function_thread{};
alignas(8) std::uint8_t function_stack[2048]{};

msg::subscriber remote_sub{};
msg::subscriber feedback_sub{};
msg::topic* command_topic = nullptr;
bool task_started = false;

void function_entry(ULONG /*arg*/)
{
    const float dt = k_default_control.chassis.runtime.dt;
    remoter::state remote{};
    chassis_feedback feedback{};

    for (;;)
    {
        (void)msg::read(remote_sub, remote);
        (void)msg::read(feedback_sub, feedback);

        const chassis_command command =
            function.update(remote, feedback.x, feedback.v, feedback.total_yaw, dt);
        (void)msg::publish(command_topic, command);

        tx_thread_sleep(1);
    }
}

} // namespace

bool start_function_task() noexcept
{
    if (task_started)
    {
        return true;
    }

    remote_sub = msg::subscribe<remoter::state>();
    feedback_sub = msg::subscribe<chassis_feedback>();
    command_topic = msg::create<chassis_command>();
    if (!remote_sub.valid() || !feedback_sub.valid() || command_topic == nullptr)
    {
        return false;
    }

    const auto priority = k_default_control.chassis.runtime.function_thread_priority;
    const UINT status = tx_thread_create(
        &function_thread, const_cast<CHAR*>("wbr_function"), function_entry, 0U,
        function_stack, sizeof(function_stack), priority, priority, TX_NO_TIME_SLICE,
        TX_AUTO_START);
    task_started = status == TX_SUCCESS;
    return task_started;
}

} // namespace wbr
