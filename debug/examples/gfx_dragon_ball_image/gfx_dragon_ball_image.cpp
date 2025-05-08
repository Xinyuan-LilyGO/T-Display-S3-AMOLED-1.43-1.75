/*
 * @Description: GFX屏幕显示+触摸切换图片
 * @version: V1.0.0
 * @Author: LILYGO_L
 * @Date: 2023-09-06 10:58:19
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-05-08 11:22:09
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "dragon_ball_material_16bit_466x466px.h"

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
    LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

// DO0143FAT01
// Arduino_GFX *gfx = new Arduino_SH8601(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT);

// H0175Y003AM
Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RST /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT,
                                      6 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    pinMode(LCD_EN, OUTPUT);
    digitalWrite(LCD_EN, HIGH);

    gfx->begin(80000000);
    gfx->fillScreen(WHITE);

    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_0, LCD_WIDTH, LCD_HEIGHT); // RGB

    for (int i = 0; i <= 255; i++)
    {
        gfx->Display_Brightness(i);
        delay(3);
    }

    delay(3000);
}

void loop()
{
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_1, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_2, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_3, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_4, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_5, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_6, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)dragon_ball_gImage_7, LCD_WIDTH, LCD_HEIGHT); // RGB
    delay(1000);
}
