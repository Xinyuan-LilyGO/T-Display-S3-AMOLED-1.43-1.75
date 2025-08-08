/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-06-05 13:01:59
 * @LastEditTime: 2025-08-08 10:17:50
 */
#pragma once

// 这里选择你使用的屏幕
#if defined CONFIG_SCREEN_TYPE_DO0143FMST10
#define DO0143FMST10 // 1.43 inches (CO5300 FT3168)
#elif defined CONFIG_SCREEN_TYPE_DO0143FAT01
#define DO0143FAT01 // DO0143FMST02//1.43 inches (SH8601 FT3168)
#elif defined CONFIG_SCREEN_TYPE_H0175Y003AM
#define H0175Y003AM // 1.75 inches (CO5300 CST9217)
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

#define LCD_SDIO0 11
#define LCD_SDIO1 13
#define LCD_SDIO2 14
#define LCD_SDIO3 15
#define LCD_SCLK 12
#define LCD_CS 10
#define LCD_RST 17

#if defined H0175Y003AM
#define LCD_WIDTH 473
#define LCD_HEIGHT 467
#elif defined DO0143FAT01 || defined DO0143FMST10
#define LCD_WIDTH 473
#define LCD_HEIGHT 467
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

#define LCD_EN 16

// IIC
#define IIC_SDA 7
#define IIC_SCL 6

// TOUCH
#define TP_INT 9

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA 4

// SD
#define SD_CS 38
#define SD_MOSI 39
#define SD_MISO 40
#define SD_SCLK 41

// PCF8563
#define PCF8563_INT 9
