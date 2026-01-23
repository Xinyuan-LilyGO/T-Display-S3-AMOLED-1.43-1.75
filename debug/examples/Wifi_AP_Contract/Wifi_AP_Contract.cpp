/*
 * @Description: None
 * @version: V1.0.0
 * @Author: None
 * @Date: 2024-06-19 17:21:47
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2026-01-23 12:10:49
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"

const char *ssid = "T-Display-S3-AMOLED-1.43-1.75";
const char *password = "88888888";

static size_t CycleTime = 0;
static size_t Contract_Total_Size = 0;
static size_t Contract_Count_Time = 0;

static uint8_t Wifi_Buffer[1024 * 4] = {0};

WiFiServer server(80);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

std::unique_ptr<Arduino_IIC> SY6970(new Arduino_SY6970(IIC_Bus, SY6970_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    while (SY6970->begin() == false)
    {
        Serial.println("SY6970 initialization fail");
        delay(2000);
    }
    Serial.println("SY6970 initialization successfully");

    // 开启ADC测量功能
    while (SY6970->IIC_Write_Device_State(SY6970->Arduino_IIC_Power::Device::POWER_DEVICE_ADC_MEASURE,
                                          SY6970->Arduino_IIC_Power::Device_State::POWER_DEVICE_ON) == false)
    {
        Serial.println("SY6970 ADC Measure ON fail");
        delay(2000);
    }
    Serial.println("SY6970 ADC Measure ON successfully");

    // 禁用看门狗定时器喂狗功能
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_WATCHDOG_TIMER, 0);
    // 热调节阈值设置为60度
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_THERMAL_REGULATION_THRESHOLD, 60);
    // 充电目标电压电压设置为4224mV
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_CHARGING_TARGET_VOLTAGE_LIMIT, 4224);
    // 最小系统电压限制为3600mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_MINIMUM_SYSTEM_VOLTAGE_LIMIT, 3600);
    // 设置OTG电压为5062mV
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_OTG_VOLTAGE_LIMIT, 5062);
    // 输入电流限制设置为600mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_INPUT_CURRENT_LIMIT, 600);
    // 快速充电电流限制设置为2112mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_FAST_CHARGING_CURRENT_LIMIT, 2112);
    // 预充电电流限制设置为192mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_PRECHARGE_CHARGING_CURRENT_LIMIT, 192);
    // 终端充电电流限制设置为320mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_TERMINATION_CHARGING_CURRENT_LIMIT, 320);
    // OTG电流限制设置为500mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_OTG_CHARGING_LIMIT, 500);

    IIC_Bus->WriteC8D8(0x09, 0B01100100);
    delay(3000);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    server.begin();

    Serial.printf("softAPIP address: ");
    Serial.println(WiFi.softAPIP());

    // 填充数据包
    memset(Wifi_Buffer, 'A', sizeof(Wifi_Buffer));
}

void loop()
{
    WiFiClient client = server.available();

    if (client)
    {
        size_t temp_buff_1 = 0;
        size_t temp_start_time = micros();
        // 发送数据包
        temp_buff_1 = client.write(Wifi_Buffer, sizeof(Wifi_Buffer));
        size_t temp_end_time = micros();

        if (temp_buff_1 > 0)
        {
            Contract_Total_Size += temp_buff_1;
        }
        Contract_Count_Time = Contract_Count_Time + (temp_end_time - temp_start_time);

        if (CycleTime < millis())
        {
            if (Contract_Total_Size > 0 && Contract_Count_Time > 0)
            {
                Serial.printf("Upload Speed: %f KB/s\n", (Contract_Total_Size / 1024.0) / (Contract_Count_Time / 1000.0 / 1000.0));

                Contract_Total_Size = 0;
                Contract_Count_Time = 0;
            }
            else
            {
                Serial.printf("Upload Speed: 0 KB/s\n");
            }

            CycleTime = millis() + 1000;
        }
    }
    else
    {
        client.stop();
    }
}