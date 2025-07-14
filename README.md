<!--
 * @Description: None
 * @version: V1.0.0
 * @Author: LILYGO_L
 * @Date: 2023-09-11 16:13:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-07-14 09:56:01
 * @License: GPL 3.0
-->
<h1 align = "center">T-Display-S3-AMOLED-1.43-1.75</h1>

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## **English | [中文](./README_CN.md)**

## Version iteration:
| Version                              | Update date                       |Update description|
| :-------------------------------: | :-------------------------------: | :-------------------------------: |
| T-Display-S3-AMOLED-1.43_V1.0     | 2024-05-20      |Original version      |
| T-Display-S3-AMOLED-1.43-1.75_V1.0     | 2024-11-25       |   Additional wiring harness, additional H0175Y003AM screen compatibility   |

## PurchaseLink

| Product                     | SOC           |  FLASH  |  PSRAM   | Link                   |
| :------------------------: | :-----------: |:-------: | :---------: | :------------------: |
| T-Display-S3-AMOLED-1.43-1.75_V1.0   | ESP32S3R8 |   16M   | 8M (Octal SPI) | [LILYGO Mall](https://lilygo.cc/products/t-display-s3-amoled-1-64?_pos=8&_sid=eacfa858d&_ss=r)  |

## Directory
- [Describe](#describe)
- [Preview](#preview)
- [Module](#module)
- [QuickStart](#quickstart)
- [PinOverview](#pinoverview)
- [RelatedTests](#RelatedTests)
- [FAQ](#faq)
- [Project](#project)
- [Information](#information)
- [DependentLibraries](#dependentlibraries)

## Describe

T-Display-S3-AMOLED-1.43-1.75 is a development board featuring a high-definition AMOLED display with a circular shape, based on the ESP32S3, designed for standalone battery connectivity.

## Preview

### Actual Product Image

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## Module

### 1.MCU

* Chip: ESP32-S3-R8
* PSRAM: 8M (Octal SPI) 
* FLASH: 16M
* For more details, please visit[Espressif ESP32-S3 Datashee](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

### 2. Screen

* Size: 1.43-inch AMOLED circular screen
* Resolution: 466x466px
* Screen type: AMOLED
* Driver chip: SH8601
* Compatibility library: Arduino_GFX
* Bus communication protocol: QSPI

### 3. Touch

* Chip: FT3168
* Bus communication protocol: IIC

### 4. Charging chip

* Chip: SY6970
* Bus communication protocol: IIC
* Other: The output waveform of the chip will be highly unstable when powered by a 5V supply without a battery connected. To stabilize the situation, it is necessary to either connect a battery or use software to disable the battery channel. By doing so, the instability will be alleviated.

### 5. RTC

* Chip: PCF8563
* Bus communication protocol: IIC

## QuickStart

### Examples Support

| example | `[platformio ide][espressif32-v6.5.0]`<br />`[arduino ide][esp32_v2.0.14]`|`[vscode][esp-idf-v5.4.0]`| description | picture |
| ------  | ------  | ------ | ------ | ------ |
| [FT3168](./examples/FT3168) | <p align="center">![alt text][supported] |<p align="center">![alt text][supported] | |  |
| [Original Test](./examples/Original_Test) | <p align="center">![alt text][supported]  || DO0143FAT01(DO0143FMST02)Product factory original testing |  |
| [GFX](./examples/GFX) | <p align="center">![alt text][supported] |  | |  |
| [GFX FT3168 Image](./examples/GFX_FT3168_Image) | <p align="center">![alt text][supported] |  | |  |
| [SY6970](./examples/SY6970) | <p align="center">![alt text][supported] | | |  |
| [SY6970 OTG](./examples/SY6970_OTG) | <p align="center">![alt text][supported] |  | |  |
| [Light Sleep Wake Up](./examples/Light_Sleep_Wake_Up) | <p align="center">![alt text][supported] |  | |  |
| [Deep Sleep Wake Up](./examples/Deep_Sleep_Wake_Up) | <p align="center">![alt text][supported] |  | |  |
| [PCF8563](./examples/PCF8563) | <p align="center">![alt text][supported] |  | |  |
| [PCF8563 Scheduled INT](./examples/PCF8563_Scheduled_INT) | <p align="center">![alt text][supported] | |  |  |
| [PCF8563 Timer INT](./examples/PCF8563_Timer_INT) | <p align="center">![alt text][supported] |  | |  |
| [Deep Sleep PCF8563 Scheduled Wake Up](./examples/Deep_Sleep_PCF8563_Scheduled_Wake_Up) | <p align="center">![alt text][supported] |  | |  |
| [Deep Sleep PCF8563 Timer Wake Up](./examples/Deep_Sleep_PCF8563_Timer_Wake_Up) | <p align="center">![alt text][supported] |  | |  |
| [SD](./examples/SD) | <p align="center">![alt text][supported] |  | |  |
| [Original_Test_H0175Y003AM](./examples/Original_Test_H0175Y003AM) | <p align="center">![alt text][supported]  || H0175Y003AM new screen adaptation |  |
| [Original_Test_DO0143FMST10](./examples/Original_Test_DO0143FMST10) | <p align="center">![alt text][supported]  || DO0143FMST10 new screen adaptation |  |
| [Deep_Sleep_Wake_Up_H0175Y003AM](./examples/Deep_Sleep_Wake_Up_H0175Y003AM) | <p align="center">![alt text][supported] | | H0175Y003AM new screen adaptation |  |
| [Light_Sleep_Wake_Up_H0175Y003AM](./examples/Light_Sleep_Wake_Up_H0175Y003AM) | <p align="center">![alt text][supported]  || H0175Y003AM new screen adaptation |  |
| [lvgl_benchmark](./examples/lvgl_benchmark) | <p align="center">![alt text][supported] |  |  |
| [iic_scan](./main/examples/iic_scan) |  |<p align="center">![alt text][supported]  |  |
| [touch](./main/examples/touch) |  |<p align="center">![alt text][supported]  |  |
| [screen_touch_lvgl_9](./main/examples/screen_touch_lvgl_9) |  |<p align="center">![alt text][supported]  |  |

[supported]: https://img.shields.io/badge/-supported-green "example"

| firmware | description | picture |
| ------  | ------  | ------ |
| [Original_Test](./firmware/(修改屏幕频率为6.5MHz)[T-Display-S3-AMOLED-1.43_V1.0][Original_Test_DO0143FAT01(DO0143FMST02)]_firmware_202501231354.bin) | Original |  |
| [Light_Sleep](./firmware/[T-Display-S3-AMOLED-1.43_V1.0][Light_Sleep_Wake_Up]_firmware_V1.0.0.bin) |  |  |
| [Deep_Sleep](./firmware/[T-Display-S3-AMOLED-1.43_V1.0][Deep_Sleep_Wake_Up]_firmware_V1.0.0.bin) |  |  |
| [Original_Test_H0175Y003AM](./firmware/(更新开机启动认证图片)[T-Display-S3-AMOLED-1.75_V1.0][Original_Test_H0175Y003AM]_firmware_202501141532.bin) | Original |  |
| [Original_Test_DO0143FMST10](./firmware/(更新开机启动认证图片)[T-Display-S3-AMOLED-1.43_V1.0][Original_Test_DO0143FMST10]_firmware_202501141532.bin) | Original |  |

### PlatformIO
1. Install[VisualStudioCode](https://code.visualstudio.com/Download),Choose installation based on your system type.

2. Open the "Extension" section of the Visual Studio Code software sidebar(Alternatively, use "<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>X</kbd>" to open the extension),Search for the "PlatformIO IDE" extension and download it.

3. During the installation of the extension, you can go to GitHub to download the program. You can download the main branch by clicking on the "<> Code" with green text, or you can download the program versions from the "Releases" section in the sidebar.

4. After the installation of the extension is completed, open the Explorer in the sidebar(Alternatively, use "<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>E</kbd>" go open it),Click on "Open Folder," locate the project code you just downloaded (the entire folder), and click "Add." At this point, the project files will be added to your workspace.

5. Open the "platformio.ini" file in the project folder (PlatformIO will automatically open the "platformio.ini" file corresponding to the added folder). Under the "[platformio]" section, uncomment and select the example program you want to burn (it should start with "default_envs = xxx") Then click "<kbd>[√](image/4.png)</kbd>" in the bottom left corner to compile,If the compilation is correct, connect the microcontroller to the computer and click "<kbd>[→](image/5.png)</kbd>" in the bottom left corner to download the program.

### Arduino
1. Install[Arduino](https://www.arduino.cc/en/software),Choose installation based on your system type.

2. Open the "example" directory within the project folder, select the example project folder, and open the file ending with ".ino" to open the Arduino IDE project workspace.

3. Open the "Tools" menu at the top right -> Select "Board" -> "Board Manager." Find or search for "esp32" and download the board files from the author named "Espressif Systems." Then, go back to the "Board" menu and select the development board type under "ESP32 Arduino." The selected development board type should match the one specified in the "platformio.ini" file under the [env] section with the header "board = xxx." If there is no corresponding development board, you may need to manually add the development board from the "board" directory within your project folder.

4. Open menu bar "[File](image/6.png)" -> "[Preferences](image/6.png)" ,Find "[Sketchbook location](image/7.png)"  here,copy and paste all library files and folders from the "libraries" folder in the project directory into the "libraries" folder in this directory.

5. Select the correct settings in the Tools menu, as shown in the table below.

#### ESP32-S3
| Setting                               | Value                                 |
| :-------------------------------: | :-------------------------------: |
| Board                                 | ESP32S3 Dev Module           |
| Upload Speed                     | 921600                               |
| USB Mode                           | Hardware CDC and JTAG     |
| USB CDC On Boot                | Enabled                              |
| USB Firmware MSC On Boot | Disabled                             |
| USB DFU On Boot                | Disabled                             |
| CPU Frequency                   | 240MHz (WiFi)                    |
| Flash Mode                         | QIO 80MHz                         |
| Flash Size                           | 16MB (128Mb)                    |
| Core Debug Level                | None                                 |
| Partition Scheme                | 16M Flash (3MB APP/9.9MB FATFS) |
| PSRAM                                | OPI PSRAM                         |
| Arduino Runs On                  | Core 1                               |
| Events Run On                     | Core 1                               |           

6. Select the correct port.

7. Click "<kbd>[√](image/8.png)</kbd>" in the upper right corner to compile,If the compilation is correct, connect the microcontroller to the computer,Click "<kbd>[→](image/9.png)</kbd>" in the upper right corner to download.

### firmware download
1. Open the project file "tools" and locate the ESP32 burning tool. Open it.

2. Select the correct burning chip and burning method, then click "OK." As shown in the picture, follow steps 1->2->3->4->5 to burn the program. If the burning is not successful, press and hold the "BOOT-0" button and then download and burn again.

3. Burn the file in the root directory of the project file "[firmware](./firmware/)" file,There is a description of the firmware file version inside, just choose the appropriate version to download.

<p align="center" width="100%">
    <img src="image/10.png" alt="example">
    <img src="image/11.png" alt="example">
</p>


## PinOverview

| AMOLED Screen Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| SDIO0         | IO11       |
| SDIO1         | IO13       |
| SDIO2         | IO14       |
| SDIO3         | IO15       |
| SCLK         | IO12       |
| CS         | IO10       |
| RST         | IO17       |
| EN         | IO16       |

| Touch Chip Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| INT         | IO9       |
| SDA         | IO7       |
| SCL         | IO6       |

| Power Chip Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| SDA         | IO7       |
| SCL         | IO6       |

| Battery Measurement Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| BATTERY_VOLTAGE_ADC_DATA         | IO4       |

| SD Card Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| CS         | IO4       |
| MOSI        | IO39       |
| MISO         | IO40       |
| SCLK         | IO41       |

## RelatedTests

### Power Dissipation
| Firmware | Program| Description | Picture |
| ------  | ------  | ------ | ------ | 
| `[T-Display-S3-AMOLED-1.43_V1.0][Light_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Light Sleep Wake Up` | Power dissipation: 1282.8uA | <p align="center" width="10%"> <img src="image/13.jpg" alt="example" width="50%"> </p> |
| `[T-Display-S3-AMOLED-1.43_V1.0][Deep_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Deep Sleep Wake Up` | Power dissipation: 174.2uA |<p align="center" width="10%"> <img src="image/12.jpg" alt="example" width="50%"> </p> |

## FAQ

* Q. After reading the above tutorials, I still don't know how to build a programming environment. What should I do?
* A. If you still don't understand how to build an environment after reading the above tutorials, you can refer to the [LilyGo-Document](https://github.com/Xinyuan-LilyGO/LilyGo-Document) document instructions to build it.

<br />

* Q. Why does Arduino IDE prompt me to update library files when I open it? Should I update them or not?
* A. Choose not to update library files. Different versions of library files may not be mutually compatible, so it is not recommended to update library files.

<br />

* Q. Why is there no serial data output on the "Uart" interface on my board? Is it defective and unusable?
* A. The default project configuration uses the USB interface as Uart0 serial output for debugging purposes. The "Uart" interface is connected to Uart0, so it won't output any data without configuration.<br />For PlatformIO users, please open the project file "platformio.ini" and modify the option under "build_flags = xxx" from "-D ARDUINO_USB_CDC_ON_BOOT=true" to "-D ARDUINO_USB_CDC_ON_BOOT=false" to enable external "Uart" interface.<br />For Arduino users, open the "Tools" menu and select "USB CDC On Boot: Disabled" to enable the external "Uart" interface.

<br />

* Q. Why is my board continuously failing to download the program?
* A. Please hold down the "BOOT-0" button and try downloading the program again.

## Project
* [T-Display-S3-AMOLED-1.43-1.75_V1.0](./project/T-Display-S3-AMOLED-1.43-1.75_V1.0.pdf)

## Information
* [FT3168](./information/FT3168.pdf)
* [PCF8563](./information/PCF8563.pdf)
* [SH8601](./information/SH8601Z.pdf)
* [DO0143FAT01](./information/SPEC-DO0143FAT01-20230830.pdf)
* [AN_SY6970](./information/AN_SY6970.pdf)
* [EVB_SY6970](./information/EVB_SY6970.pdf)

## DependentLibraries
* [Arduino_GFX-1.3.7](https://github.com/moononournation/Arduino_GFX)
* [Arduino_DriveBus-1.1.12](https://github.com/Xk-w/Arduino_DriveBus)
* [JPEGDEC-1.2.8](https://github.com/bitbank2/JPEGDEC)
* [lvgl-8.3.5](https://lvgl.io)
* [MiniTV](https://github.com/moononournation/MiniTV)
* [SensorLib](https://github.com/lewisxhe/SensorsLib)

