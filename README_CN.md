<!--
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-09-11 16:13:14
 * @LastEditTime: 2025-08-04 16:08:43
 * @License: GPL 3.0
-->
<h1 align = "center">T-Display-S3-AMOLED-1.43-1.75</h1>

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## **[English](./README.md) | 中文**

## 版本迭代:
| Version         | Update date     |Update description|
| :-------------------------------: | :-------------------------------: | :-------------------------------: |
| T-Display-S3-AMOLED-1.43_V1.0     | 2024-05-20       |初始版本      |
| T-Display-S3-AMOLED-1.43-1.75_V1.0     | 2024-11-25       |新增排线、新增H0175Y003AM屏幕适配      |
| T-Display-S3-AMOLED-1.43-1.75_V1.0     | 2025-01-13       |新增DO0143FMST10屏幕适配      |

## 购买链接

| Product                     | SOC           |  FLASH  |  PSRAM   | Link                   |
| :------------------------: | :-----------: |:-------: | :---------: | :------------------: |
| T-Display-S3-AMOLED-1.43-1.75   | ESP32S3R8 |   16M   | 8M (Octal SPI) | [LILYGO Mall](https://lilygo.cc/products/t-display-s3-amoled-1-64?_pos=8&_sid=eacfa858d&_ss=r) |

## 目录
- [描述](#描述)
- [预览](#预览)
- [模块](#模块)
- [快速开始](#快速开始)
- [引脚总览](#引脚总览)
- [相关测试](#相关测试)
- [常见问题](#常见问题)
- [项目](#项目)
- [资料](#资料)
- [依赖库](#依赖库)

## 描述

T-Display-S3-AMOLED-1.43-1.75是一款基于ESP32S3开发的板载高清AMOLED屏幕的圆屏开发板，可连接外接电池独立供电。

## 预览

### 实物图

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## 模块

### 1. MCU

* 芯片：ESP32-S3-R8
* PSRAM：8M (Octal SPI) 
* FLASH：16M
* 其他说明：更多资料请访问[乐鑫官方ESP32-S3数据手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

### 2. 屏幕

* 尺寸：1.43英寸AMOLED圆屏幕
* 分辨率：466x466px
* 屏幕类型：AMOLED
* 驱动芯片：SH8601
* 兼容库：Arduino_GFX
* 总线通信协议：QSPI

### 3. 触摸

* 芯片：FT3268
* 总线通信协议：IIC

### 4. 充电芯片

* 芯片：SY6970
* 总线通信协议：IIC
* 其他：该芯片在未接电池5V供电时输出波形将非常不稳定，需要连接电池使用或者软件关闭电池通道，这样的话情况将会得到缓解

### 5. RTC

* 芯片：PCF8563
* 总线通信协议：IIC

## 快速开始

### 示例支持

| example | `[platformio ide][espressif32-v6.5.0]`<br />`[arduino ide][esp32_v2.0.14]`|`[vscode][esp-idf-v5.4.0]`| description | picture |
| ------  | ------  | ------ | ------ | ------ |
| [FT3168](./examples/FT3168) | <p align="center">![alt text][supported] | | |  |
| [Original Test](./examples/Original_Test) | <p align="center">![alt text][supported]  || DO0143FAT01(DO0143FMST02)出厂测试 |  |
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
| [Original_Test_H0175Y003AM](./examples/Original_Test_H0175Y003AM) | <p align="center">![alt text][supported]  || 出厂测试 H0175Y003AM 新屏幕适配 |  |
| [Original_Test_DO0143FMST10](./examples/Original_Test_DO0143FMST10) | <p align="center">![alt text][supported]  || 出厂测试 DO0143FMST10 新屏幕适配 |  |
| [Deep_Sleep_Wake_Up_H0175Y003AM](./examples/Deep_Sleep_Wake_Up_H0175Y003AM) | <p align="center">![alt text][supported] | | H0175Y003AM 新屏幕适配 |  |
| [Light_Sleep_Wake_Up_H0175Y003AM](./examples/Light_Sleep_Wake_Up_H0175Y003AM) | <p align="center">![alt text][supported]  || H0175Y003AM 新屏幕适配 |  |
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
| [Original_Test_DO0143FMST10](./firmware/(更新开机启动认证图片)[T-Display-S3-AMOLED-1.43_V1.0][Original_Test_DO0143FMST10]_firmware_202501141532.bin.bin) | Original |  |

### PlatformIO
1. 安装[VisualStudioCode](https://code.visualstudio.com/Download)，根据你的系统类型选择安装。

2. 打开VisualStudioCode软件侧边栏的“扩展”（或者使用<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>X</kbd>打开扩展），搜索“PlatformIO IDE”扩展并下载。

3. 在安装扩展的期间，你可以前往GitHub下载程序，你可以通过点击带绿色字样的“<> Code”下载主分支程序，也通过侧边栏下载“Releases”版本程序。

4. 扩展安装完成后，打开侧边栏的资源管理器（或者使用<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>E</kbd>打开），点击“打开文件夹”，找到刚刚你下载的项目代码（整个文件夹），点击“添加”，此时项目文件就添加到你的工作区了。

5. 打开项目文件中的“platformio.ini”（添加文件夹成功后PlatformIO会自动打开对应文件夹的“platformio.ini”）,在“[platformio]”目录下取消注释选择你需要烧录的示例程序（以“default_envs = xxx”为标头），然后点击左下角的“<kbd>[√](image/4.png)</kbd>”进行编译，如果编译无误，将单片机连接电脑，点击左下角“<kbd>[→](image/5.png)</kbd>”即可进行烧录。

### Arduino
1. 安装[Arduino](https://www.arduino.cc/en/software)，根据你的系统类型选择安装。

2. 打开项目文件夹的“example”目录，选择示例项目文件夹，打开以“.ino”结尾的文件即可打开Arduino IDE项目工作区。

3. 打开右上角“工具”菜单栏->选择“开发板”->“开发板管理器”，找到或者搜索“esp32”，下载作者名为“Espressif Systems”的开发板文件。接着返回“开发板”菜单栏，选择“ESP32 Arduino”开发板下的开发板类型，选择的开发板类型由“platformio.ini”文件中以[env]目录下的“board = xxx”标头为准，如果没有对应的开发板，则需要自己手动添加项目文件夹下“board”目录下的开发板。

4. 打开菜单栏“[文件](image/6.png)”->“[首选项](image/6.png)”，找到“[项目文件夹位置](image/7.png)”这一栏，将项目目录下的“libraries”文件夹里的所有库文件连带文件夹复制粘贴到这个目录下的“libraries”里边。

5. 在 "工具 "菜单中选择正确的设置，如下表所示。

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

6. 选择正确的端口。

7. 点击右上角“<kbd>[√](image/8.png)</kbd>”进行编译，如果编译无误，将单片机连接电脑，点击右上角“<kbd>[→](image/9.png)</kbd>”即可进行烧录。

### ESP-IDF Visual Studio Code
1. 安装 [Visual Studio Code](https://code.visualstudio.com/Download) ，根据你的系统类型选择安装。

2. 打开 VisualStudioCode 软件侧边栏的“扩展”（或者使用<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>X</kbd>打开扩展），搜索“ESP-IDF”扩展并下载。

3. 在安装扩展的期间，使用git命令克隆仓库

        git clone --recursive https://github.com/Xinyuan-LilyGO/T-Display-S3-AMOLED-1.43-1.75.git

    克隆时候需要同时加上“--recursive”，如果克隆时候未加上那么之后使用的时候需要初始化一下子模块

        git submodule update --init --recursive

4. 下载安装 [ESP-IDF v5.4.1](https://dl.espressif.cn/dl/esp-idf/?idf=4.4)，记录一下安装路径，打开之前安装好的“ESP-IDF”扩展打开“配置 ESP-IDF 扩展”，选择“USE EXISTING SETUP”菜单，选择“Search ESP-IDF in system”栏，正确配置之前记录的安装路径：
   - **ESP-IDF directory (IDF_PATH):** `你的安装路径xxx\Espressif\frameworks\esp-idf-v5.4`  
   - **ESP-IDF Tools directory (IDF_TOOLS_PATH):** `你的安装路径xxx\Espressif`  
    点击右下角的“install”进行框架安装。

5. 点击 Visual Studio Code 底部菜单栏的 ESP-IDF 扩展菜单“SDK 配置编辑器”，在搜索栏里搜索“Select the example to build”字段，选择你所需要编译的项目，再在搜索栏里搜索“Select the camera type”字段，选择你的板子板载的摄像头类型，点击保存。

6. 点击 Visual Studio Code 底部菜单栏的“设置乐鑫设备目标”，选择**ESP32P4**，点击底部菜单栏的“构建项目”，等待构建完成后点击底部菜单栏的“选择要使用的端口”，之后点击底部菜单栏的“烧录项目”进行烧录程序。

<p align="center" width="100%">
    <img src="image/1.jpg" alt="example">
</p>

### firmware烧录
1. 打开项目文件“tools”找到ESP32烧录工具，打开。

2. 选择正确的烧录芯片以及烧录方式点击“OK”，如图所示根据步骤1->2->3->4->5即可烧录程序，如果烧录不成功，请按住“BOOT-0”键再下载烧录。

3. 烧录文件在项目文件根目录“[firmware](./firmware/)”文件下，里面有对firmware文件版本的说明，选择合适的版本下载即可。

<p align="center" width="100%">
    <img src="image/10.png" alt="example">
    <img src="image/11.png" alt="example">
</p>


## 引脚总览

| AMOLED屏幕引脚  | ESP32S3引脚|
| :------------------: | :------------------:|
| SDIO0         | IO11       |
| SDIO1         | IO13       |
| SDIO2         | IO14       |
| SDIO3         | IO15       |
| SCLK         | IO12       |
| CS         | IO10       |
| RST         | IO17       |
| EN         | IO16       |

| 触摸芯片引脚  | ESP32S3引脚|
| :------------------: | :------------------:|
| INT         | IO9       |
| SDA         | IO7       |
| SCL         | IO6       |

| 电源芯片引脚  | ESP32S3引脚|
| :------------------: | :------------------:|
| SDA         | IO7       |
| SCL         | IO6       |

| 电池测量引脚  | ESP32S3引脚|
| :------------------: | :------------------:|
| BATTERY_VOLTAGE_ADC_DATA         | IO4       |

| SD卡引脚  | ESP32S3引脚|
| :------------------: | :------------------:|
| CS         | IO4       |
| MOSI        | IO39       |
| MISO         | IO40       |
| SCLK         | IO41       |

## 相关测试

### 功耗
| Firmware | Program| Description | Picture |
| ------  | ------  | ------ | ------ | 
| `[T-Display-S3-AMOLED-1.43_V1.0][Light_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Light Sleep Wake Up` | 功耗: 1282.8uA | <p align="center" width="10%"> <img src="image/13.jpg" alt="example" width="50%"> </p> |
| `[T-Display-S3-AMOLED-1.43_V1.0][Deep_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Deep Sleep Wake Up` | 功耗: 174.2uA |<p align="center" width="10%"> <img src="image/12.jpg" alt="example" width="50%"> </p> |

## 常见问题

* Q. 看了以上教程我还是不会搭建编程环境怎么办？
* A. 如果看了以上教程还不懂如何搭建环境的可以参考[LilyGo-Document](https://github.com/Xinyuan-LilyGO/LilyGo-Document)文档说明来搭建。

<br />

* Q. 为什么打开Arduino IDE时他会提醒我是否要升级库文件？我应该升级还是不升级？
* A. 选择不升级库文件，不同版本的库文件可能不会相互兼容所以不建议升级库文件。

<br />

* Q. 为什么我的板子上“Uart”接口没有输出串口数据，是不是坏了用不了啊？
* A. 因为项目文件默认配置将USB接口作为Uart0串口输出作为调试，“Uart”接口连接的是Uart0，不经配置自然是不会输出任何数据的。<br />PlatformIO用户请打开项目文件“platformio.ini”，将“build_flags = xxx”下的选项“-DARDUINO_USB_CDC_ON_BOOT=true”修改成“-DARDUINO_USB_CDC_ON_BOOT=false”即可正常使用外部“Uart”接口。<br />Arduino用户打开菜单“工具”栏，选择USB CDC On Boot: “Disabled”即可正常使用外部“Uart”接口。

<br />

* Q. 为什么我的板子一直烧录失败呢？
* A. 请按住“BOOT-0”按键重新下载程序。

## 项目
* [T-Display-S3-AMOLED-1.43-1.75_V1.0](./project/T-Display-S3-AMOLED-1.43-1.75_V1.0.pdf)

## 资料
* [PCF8563](./information/PCF8563.pdf)
* [SH8601](./information/SH8601Z.pdf)
* [DO0143FAT10](./information/DO0143FMST10-DWO.pdf)
* [AN_SY6970](./information/AN_SY6970.pdf)
* [EVB_SY6970](./information/EVB_SY6970.pdf)

## 依赖库
* [Arduino_GFX-1.3.7](https://github.com/moononournation/Arduino_GFX)
* [Arduino_DriveBus-1.1.12](https://github.com/Xk-w/Arduino_DriveBus)
* [JPEGDEC-1.2.8](https://github.com/bitbank2/JPEGDEC)
* [lvgl-8.3.5](https://lvgl.io)
* [MiniTV](https://github.com/moononournation/MiniTV)
* [SensorLib](https://github.com/lewisxhe/SensorsLib)