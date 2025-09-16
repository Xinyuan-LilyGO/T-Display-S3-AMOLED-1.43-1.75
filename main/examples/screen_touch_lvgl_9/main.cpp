/*
 * @Description: ft3168
 * @Author: LILYGO_L
 * @Date: 2025-06-13 12:06:14
 * @LastEditTime: 2025-09-15 17:47:07
 * @License: GPL 3.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "pin_config.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "cpp_bus_driver_library.h"
#include "esp_lcd_touch_cst9217.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "image_rgb565_16bit_473x467px.h"

#define LVGL_TICK_PERIOD_MS 1

size_t Cycle_Time = 0;

volatile bool interrupt_flag = false;

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
_lock_t lvgl_api_lock;

#if defined H0175Y003AM

esp_lcd_touch_handle_t Touch = NULL;

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_2>(IIC_SDA, IIC_SCL, I2C_NUM_0);

auto Screen = std::make_unique<Cpp_Bus_Driver::Co5300>(Qspi_Bus, LCD_WIDTH, LCD_HEIGHT, LCD_CS, LCD_RST, 0, 0, Cpp_Bus_Driver::Co5300::Color_Format::RGB565);

#elif defined DO0143FMST10

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(IIC_SDA, IIC_SCL, I2C_NUM_0);

auto Qspi_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Qspi>(LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3, LCD_SCLK, SPI2_HOST);

auto Touch = std::make_unique<Cpp_Bus_Driver::Ft3x68>(Iic_Bus, FT3168_DEVICE_DEFAULT_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

auto Screen = std::make_unique<Cpp_Bus_Driver::Co5300>(Qspi_Bus, LCD_WIDTH, LCD_HEIGHT, LCD_CS, LCD_RST, 0, 0, Cpp_Bus_Driver::Co5300::Color_Format::RGB565);

#elif defined DO0143FAT01

auto Iic_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(IIC_SDA, IIC_SCL, I2C_NUM_0);

auto Qspi_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Qspi>(LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3, LCD_SCLK, SPI2_HOST);

auto Touch = std::make_unique<Cpp_Bus_Driver::Ft3x68>(Iic_Bus, FT3168_DEVICE_DEFAULT_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

auto Screen = std::make_unique<Cpp_Bus_Driver::Sh8601>(Qspi_Bus, LCD_WIDTH, LCD_HEIGHT, LCD_CS, LCD_RST, 0, 0, Cpp_Bus_Driver::Sh8601::Color_Format::RGB565);

#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

auto ESP32S3 = std::make_unique<Cpp_Bus_Driver::Tool>();

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

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
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

            data->state = LV_INDEV_STATE_PR;

            /*Set the coordinates*/
            data->point.x = tp_x;
            data->point.y = tp_y;
        }
        else
        {
            data->state = LV_INDEV_STATE_REL;
        }

#elif (defined DO0143FAT01) || (defined DO0143FMST10)

        Cpp_Bus_Driver::Ft3x68::Touch_Point tp;

        if (Touch->get_single_touch_point(tp) == true)
        {
            printf("touch finger: %d\n x: %d y: %d\n", tp.finger_count, tp.info[0].x, tp.info[0].y);

            data->state = LV_INDEV_STATE_PR;

            /*Set the coordinates*/
            data->point.x = tp.info[0].x;
            data->point.y = tp.info[0].y;
        }
        else
        {
            data->state = LV_INDEV_STATE_REL;
        }
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

        interrupt_flag = false;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

void Lvgl_Init(void)
{
    printf("initialize lvgl\n");

    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, Screen.get());
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // create draw buffer
    printf("allocate separate lvgl draw buffers\n");
    size_t draw_buffer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t);
    void *buf1 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    assert(buf1);
    // void *buf2 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    // assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // set the callback which can copy the rendered image to an area of the display

    lv_display_set_flush_cb(display, [](lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
                            {
                                auto screen = static_cast<Cpp_Bus_Driver::Co5300 *>(lv_display_get_user_data(disp));
                                int x = area->x1;
                                int y = area->y1;
                                int w = area->x2 - area->x1 + 1;
                                int h = area->y2 - area->y1 + 1;

                                // printf("flush area: x=%d, y=%d, w=%d, h=%d\n", x, y, w, h);

                                lv_draw_sw_rgb565_swap(px_map, w * h);

                                // lvgl的px_map是行优先，rgb565格式
                                screen->send_color_stream(x, y, w, h, px_map);
                                // 通知lvgl刷新完成
                                lv_display_flush_ready(disp); });

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

    printf("use esp_timer as lvgl tick timer\n");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = [](void *arg)
        {
            lv_tick_inc(LVGL_TICK_PERIOD_MS);
        },
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
}

void Screen_Init()
{
    printf("Screen_Init\n");

    Screen->begin(SPI_MASTER_FREQ_40M);
}

void Touch_Init()
{
    printf("Touch_Init\n");

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

    Touch->begin();
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

    ESP32S3->create_gpio_interrupt(TP_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::FALLING,
                                   [](void *arg) -> IRAM_ATTR void
                                   {
                                       interrupt_flag = true;
                                   });
}

void lvgl_ui_task(void *arg)
{
    printf("lvgl_ui_task start\n");
    uint32_t time_till_next_ms = 0;

    while (1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);

        // in case of task watch dog timeout, set the minimal delay to 10ms
        if (time_till_next_ms < 10)
        {
            time_till_next_ms = 10;
        }
        usleep(1000 * time_till_next_ms);

        // lv_timer_handler();
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    printf("Ciallo\n");

    ESP32S3->pin_mode(LCD_EN, Cpp_Bus_Driver::Tool::Pin_Mode::OUTPUT);
    ESP32S3->pin_write(LCD_EN, 1);

    Touch_Init();

    Screen_Init();

    Screen->send_color_stream(0, 0, LCD_WIDTH, LCD_HEIGHT, gImage_1);
    for (uint8_t i = 0; i < 255; i += 5)
    {
        Screen->set_brightness(i);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

#if defined H0175Y003AM || defined DO0143FMST10

    Screen->set_color_enhance(Cpp_Bus_Driver::Co5300::Color_Enhance::LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Co5300::Color_Enhance::MEDIUM);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Co5300::Color_Enhance::HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Co5300::Color_Enhance::OFF);
    vTaskDelay(pdMS_TO_TICKS(1000));

#elif defined DO0143FAT01

    Screen->set_color_enhance(Cpp_Bus_Driver::Sh8601::Color_Enhance::LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Sh8601::Color_Enhance::MEDIUM);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Sh8601::Color_Enhance::HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    Screen->set_color_enhance(Cpp_Bus_Driver::Sh8601::Color_Enhance::OFF);
    vTaskDelay(pdMS_TO_TICKS(1000));

#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

    Lvgl_Init();
    xTaskCreate(lvgl_ui_task, "lvgl_ui_task", 10 * 1024, NULL, 1, NULL);

    // 创建一个简单的按钮，点击时在串口打印 "Ciallo"
    lv_obj_t *btn = lv_btn_create(lv_screen_active());
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Ciallo");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, [](lv_event_t *e)
                        { printf("Ciallo\n"); }, LV_EVENT_CLICKED, NULL);

    // 设置整个屏幕为白色
    // size_t screen_size = LCD_WIDTH * LCD_HEIGHT * 2; // RGB565: 2 bytes per pixel
    // size_t data_cache_line_size = 16;                                    // 通常16或32，具体可查芯片手册
    // void *white_buf = heap_caps_aligned_calloc(data_cache_line_size, 1, screen_size, MALLOC_CAP_SPIRAM);
    // if (white_buf)
    // {
    //     uint16_t *p = (uint16_t *)white_buf;
    //     for (size_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i)
    //     {
    //         p[i] = 0xFFFF; // RGB565白色
    //     }
    //     esp_err_t err = esp_lcd_panel_draw_bitmap(Screen_Panel, 0, 0, LCD_WIDTH, LCD_HEIGHT, white_buf);
    //     if (err != ESP_OK)
    //     {
    //         printf("esp_lcd_panel_draw_bitmap (white) fail (error code: %#X)\n", err);
    //     }
    //     heap_caps_free(white_buf);
    // }

    //     while (1)
    //     {
    //         // IIC_Scan();

    //         if (esp_log_timestamp() > Cycle_Time)
    //         {
    //             if (interrupt_flag == true)
    //             {
    // #if defined H0175Y003AM
    //                 uint16_t tp_x;
    //                 uint16_t tp_y;
    //                 uint8_t tp_cnt = 0;

    //                 esp_lcd_touch_read_data(Touch);

    //                 /* Read data from touch controller */
    //                 bool tp_pressed = esp_lcd_touch_get_coordinates(Touch, &tp_x, &tp_y, NULL, &tp_cnt, 1);
    //                 if (tp_pressed && tp_cnt > 0)
    //                 {
    //                     printf("touch finger: %d\n", tp_cnt);
    //                     printf("touch num [%d] x: %d y: %d\n", tp_cnt, tp_x, tp_y);
    //                 }

    // #elif (defined DO0143FAT01) || (defined DO0143FMST10)

    //                 Cpp_Bus_Driver::Ft3x68::Touch_Point tp;

    //                 if (Touch->get_multiple_touch_point(tp) == true)
    //                 {
    //                     printf("touch finger: %d\n", tp.finger_count);

    //                     for (uint8_t i = 0; i < tp.info.size(); i++)
    //                     {
    //                         printf("touch num [%d] x: %d y: %d\n", i + 1, tp.info[i].x, tp.info[i].y);
    //                     }
    //                 }

    // #else
    // #error "Unknown macro definition. Please select the correct macro definition."
    // #endif

    //                 interrupt_flag = false;
    //             }
    //             Cycle_Time = esp_log_timestamp() + 100;
    //         }

    //         vTaskDelay(pdMS_TO_TICKS(10));
    //     }
}
