#include "leg_tasks.hpp"

#include "tx_api.h"

#include <cstdint>

namespace app
{
namespace
{

TX_THREAD control_thread_object{};
TX_THREAD pendulum_thread_object{};
TX_THREAD solver_thread_object{};

alignas(8) std::uint8_t
    control_stack[leg_config::control_task_thread::stack_size]{};
alignas(8) std::uint8_t
    pendulum_stack[leg_config::control_thread::stack_size]{};
alignas(8) std::uint8_t
    solver_stack[leg_config::solver_thread::stack_size]{};

bool control_created = false;
bool pendulum_created = false;
bool solver_created = false;
bool control_started = false;
bool pendulum_started = false;
bool solver_started = false;
bool prepared = false;
bool started = false;

[[noreturn]] void control_entry(ULONG /*input*/)
{
    control_task::run();
}

[[noreturn]] void pendulum_entry(ULONG /*input*/)
{
    pendulum_task::run();
}

[[noreturn]] void solver_entry(ULONG /*input*/)
{
    solver_task::run();
}

bool create_task_thread(TX_THREAD& thread,
                        bool& created,
                        const char* name,
                        void (*entry)(ULONG),
                        void* stack,
                        ULONG stack_size,
                        UINT priority) noexcept
{
    if (created)
    {
        return true;
    }

    if (tx_thread_create(&thread, const_cast<CHAR*>(name), entry, 0U,
                         stack, stack_size, priority, priority,
                         TX_NO_TIME_SLICE, TX_DONT_START) != TX_SUCCESS)
    {
        return false;
    }

    created = true;
    return true;
}

bool start_task_thread(TX_THREAD& thread, bool& thread_started) noexcept
{
    if (thread_started)
    {
        return true;
    }
    if (tx_thread_resume(&thread) != TX_SUCCESS)
    {
        return false;
    }

    thread_started = true;
    return true;
}

} // namespace

namespace leg_tasks
{

bool prepare() noexcept
{
    if (prepared)
    {
        return true;
    }

    bool initialized = control_task::init();
    initialized &= pendulum_task::init();
    initialized &= solver_task::init();
    if (!initialized)
    {
        return false;
    }

    if (!create_task_thread(
            control_thread_object,
            control_created,
            "leg_control",
            control_entry,
            control_stack,
            sizeof(control_stack),
            leg_config::control_task_thread::priority) ||
        !create_task_thread(
            pendulum_thread_object,
            pendulum_created,
            "leg_pendulum",
            pendulum_entry,
            pendulum_stack,
            sizeof(pendulum_stack),
            leg_config::control_thread::priority) ||
        !create_task_thread(
            solver_thread_object,
            solver_created,
            "leg_solver",
            solver_entry,
            solver_stack,
            sizeof(solver_stack),
            leg_config::solver_thread::priority))
    {
        return false;
    }

    prepared = true;
    return true;
}

bool start() noexcept
{
    if (started)
    {
        return true;
    }
    if (!prepared ||
        !start_task_thread(control_thread_object, control_started) ||
        !start_task_thread(pendulum_thread_object, pendulum_started) ||
        !start_task_thread(solver_thread_object, solver_started))
    {
        return false;
    }

    started = true;
    return true;
}

} // namespace leg_tasks
} // namespace app
