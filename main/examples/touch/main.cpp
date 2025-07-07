/*
 * @Description: ft3168
 * @Author: LILYGO_L
 * @Date: 2025-06-13 12:06:14
 * @LastEditTime: 2025-07-07 11:41:37
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
#include "esp_lcd_touch_cst9217.h"

size_t Cycle_Time = 0;

volatile bool interrupt_flag = false;

#if defined H0175Y003AM

esp_lcd_touch_handle_t Touch = NULL;

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_2>(IIC_SDA, IIC_SCL, I2C_NUM_0);

#elif defined DO0143FAT01 || defined DO0143FMST10

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(IIC_SDA, IIC_SCL, I2C_NUM_0);

auto Touch = std::make_unique<Cpp_Bus_Driver::Ft3x68>(Iic_Bus, FT3168_DEVICE_DEFAULT_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

// void IIC_Scan(void)
// {
//     std::vector<uint8_t> address;
//     if (Iic_Bus->scan_7bit_address(&address) == true)
//     {
//         for (size_t i = 0; i < address.size(); i++)
//         {
//             printf("Discovered IIC devices[%u]: %#X\n", i, address[i]);
//         }
//     }
// }

// void touch_isr_cb(esp_lcd_touch_handle_t tp)
// {
//     interrupt_flag = true;
// }

extern "C" void app_main(void)
{
    printf("Ciallo\n");

#if defined H0175Y003AM

    // const i2c_config_t i2c_conf =
    //     {
    //         .mode = I2C_MODE_MASTER,
    //         .sda_io_num = IIC_SDA,
    //         .scl_io_num = IIC_SCL,
    //         .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //         .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //         .master = {
    //             .clk_speed = 400 * 1000,
    //         },
    //         .clk_flags = 0,
    //     };
    // ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    // ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));

    Iic_Bus->begin();

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
    // Attach the TOUCH to the I2C bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle));

    const esp_lcd_touch_config_t tp_cfg =
        {
            .x_max = LCD_WIDTH,
            .y_max = LCD_HEIGHT,
            .rst_gpio_num = -1,
            .int_gpio_num = TP_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },

            .interrupt_callback = [](esp_lcd_touch_handle_t tp) -> IRAM_ATTR void
            {
                interrupt_flag = true;
            },
        };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &Touch));

#elif defined DO0143FAT01 || defined DO0143FMST10

    Touch->create_gpio_interrupt(TP_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::FALLING,
                                 [](void *arg) -> IRAM_ATTR void
                                 {
                                     interrupt_flag = true;
                                 });

    Touch->begin();
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

    while (1)
    {
        // IIC_Scan();

        if (esp_log_timestamp() > Cycle_Time)
        {
            if (interrupt_flag == true)
            {
#if defined H0175Y003AM
                uint16_t tp_x;
                uint16_t tp_y;
                uint8_t tp_cnt = 0;

                esp_lcd_touch_read_data(Touch);

                /* Read data from touch controller */
                bool tp_pressed = esp_lcd_touch_get_coordinates(Touch, &tp_x, &tp_y, NULL, &tp_cnt, 1);
                if (tp_pressed && tp_cnt > 0)
                {
                    printf("touch finger: %d\n", tp_cnt);
                    printf("touch num [%d] x: %d y: %d\n", tp_cnt, tp_x, tp_y);
                }

#elif (defined DO0143FAT01) || (defined DO0143FMST10)

                Cpp_Bus_Driver::Ft3x68::Touch_Point tp;

                // if (Touch->get_multiple_touch_point(tp) == true)
                // {
                //     printf("touch finger: %d\n", tp.finger_count);

                //     for (uint8_t i = 0; i < tp.info.size(); i++)
                //     {
                //         printf("touch num [%d] x: %d y: %d\n", i + 1, tp.info[i].x, tp.info[i].y);
                //     }
                // }

                if (Touch->get_single_touch_point(tp, 1) == true)
                {
                    // printf("touch finger: %d\n", tp.finger_count);
                    printf("touch num [1] x: %d y: %d\n", tp.info[0].x, tp.info[0].y);
                }

#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

                interrupt_flag = false;

                Cycle_Time = esp_log_timestamp() + 100;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
