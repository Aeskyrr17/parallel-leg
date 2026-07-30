#include "config.hpp"
#include "imu_demo.hpp"
#include "motor_demo.hpp"
#include "ps2_demo.hpp"
#include "referee_ui_demo.hpp"
#include "remoter_demo.hpp"
#include "usart_demo.hpp"
#include "usb_demo.hpp"

extern "C" void app_start()
{
    demo::imu::run();
    // demo::motor::run();
    if constexpr (config::feature::enable_ps2)
    {
        demo::ps2::run();
    }
    else
    {
        demo::remoter::run();
    }
    // demo::referee_ui::run();
    // demo::usart::start();
    // demo::usb::start();
}
