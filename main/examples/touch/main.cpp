/*
 * @Description: iic_scan
 * @Author: LILYGO_L
 * @Date: 2025-06-13 12:06:14
 * @LastEditTime: 2025-06-24 16:05:40
 * @License: GPL 3.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "pin_config.h"
#include "cpp_bus_driver_library.h"

size_t Cycle_Time = 0;

volatile bool interrupt_flag = false;

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(IIC_SDA, IIC_SCL, I2C_NUM_0);

auto Ft3168 = std::make_unique<Cpp_Bus_Driver::Ft3x68>(Iic_Bus, FT3168_DEVICE_DEFAULT_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

void IIC_Scan(void)
{
    std::vector<uint8_t> address;
    if (Iic_Bus->scan_7bit_address(&address) == true)
    {
        for (size_t i = 0; i < address.size(); i++)
        {
            printf("Discovered IIC devices[%u]: %#X\n", i, address[i]);
        }
    }
}

extern "C" void app_main(void)
{
    printf("Ciallo\n");

    Ft3168->create_gpio_interrupt(TP_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::FALLING,
                                  [](void *arg) -> IRAM_ATTR void
                                  {
                                      interrupt_flag = true;
                                  });

    Ft3168->begin();

    while (1)
    {
        // IIC_Scan();

        if (esp_log_timestamp() > Cycle_Time)
        {
            if (interrupt_flag == true)
            {
                interrupt_flag = false;

                Cpp_Bus_Driver::Ft3x68::Touch_Point tp;

                if (Ft3168->get_multiple_touch_point(tp) == true)
                {
                    printf("touch finger: %d\n", tp.finger_count);

                    for (uint8_t i = 0; i < tp.info.size(); i++)
                    {
                        printf("touch num [%d] x: %d y: %d\n", i + 1, tp.info[i].x, tp.info[i].y);
                    }
                }
            }

            Cycle_Time = esp_log_timestamp() + 100;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
