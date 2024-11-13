# Agora Video Doorbell for ESP32

_English | [简体中文](README.cn.md)_

## Introduction

This demo shows how to simulate a typical audio/video calling scenario using an ESP32-S3 Korvo V3 development board and an Android phone. It demonstrates calling the mobile APP by pressing the call button, with the APP answering; or the mobile APP viewing the device's camera feed in real-time.

### File Structure

```
├── CMakeLists.txt
├── components                                  Agora iot sdk component
│   ├── agora_iot_sdk
│   │   ├── CMakeLists.txt
│   │   ├── include                             Agora iot sdk header files
│   │   │   ├── agora_iot_api.h
│   │   │   └── agora_iot_call.h
│   │   └── lib
│   │       ├── esp32
│   │       │   └── PLACEHOLDER
│   │       ├── esp32s2
│   │       │   └── PLACEHOLDER
│   │       └── esp32s3                         Agora iot sdk libraries
│   │           ├── libagora-cjson.a
│   │           ├── libagora-iot-solution.a
│   │           ├── libagora-mbedtls.a
│   │           ├── libagora-webclient.a
│   │           ├── libahpl.a
│   │           ├── libhttp-parser.a
│   │           ├── libiot-license.a
│   │           ├── libiot-utility.a
│   │           ├── libmedia-engine.a
│   │           ├── librtsa.a
│   │           └── PLACEHOLDER
│   └── camera                                  Camera component
├── firmware
│   ├── ag_videodoorbell_esp32.bin
│   ├── bootloader.bin
│   └── partition-table.bin
├── main                                        Video doorbell Demo code
│   ├── app_config.h
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild
│   └── video_doorbell.c
├── partitions.csv                              partition table
├── README.en.md
├── README.md
├── sdkconfig.defaults
└── sdkconfig.defaults.esp32s3
```

## Environment Setup

### Hardware Requirements

This demo currently only supports the `ESP32-S3-Korvo-2` development board.

The recommended camera is OV3660, with an external speaker.

## Compilation and Download

### Default IDF Branch

Espressif IoT Development Framework (ESP-IDF). For installation instructions, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

This demo supports IDF release/v[4.4] and later branches, with release/v[4.4] being the default branch.

To select the IDF branch:

```bash
cd $IDF_PATH
git checkout release/v4.4
git pull
git submodule update --init --recursive
```

Espressif Audio Development Framework (ESP-ADF). For installation instructions, see the [ESP-ADF Programming Guide](https://docs.espressif.com/projects/esp-adf/en/latest/get-started/index.html).

This demo supports the latest master branch of ADF.

### Apply IDF Patch

This demo requires applying one patch to IDF. Apply it using:
cd $IDF_PATH
git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch

### Compile Firmware

Copy this demo (agora-demo-for-esp32) to the ~/esp directory. Run the following commands:

```bash
$ export ADF_PATH=~/esp/esp-adf
$ . $HOME/esp/esp-idf/export.sh
$ cd ~/esp/agora-demo-for-esp32
$ idf.py set-target esp32s3
$ idf.py menuconfig
$ idf.py build
```

### Download Firmware

#### Linux Operating System

Run the following command:

```bash
$ idf.py -p /dev/ttyUSB0 flash monitor
```

Note: If you encounter permission issues with /dev/ttyUSB0, execute: sudo usermod -aG dialout $USER

After successful download, the demo will run automatically. Once initialization is complete, you'll see the serial output: "Agora: Press [REC] key to ring the doorbell ...". The device is now in `low-power standby state`.

#### Windows Operating System

(TBD)

### Download Pre-compiled Firmware

You can skip the above steps and directly download the pre-compiled firmware included in this demo.

#### Linux Operating System

Run the following command:

```bash
esptool.py --chip esp32s3 \
--port /dev/cu.usbserial-1110 --baud 921600 \
--before default_reset \
--after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect \
0x0      ./firmware/bootloader.bin \
0x8000   ./firmware/partition-table.bin \
0x10000  ./firmware/ag_videodoorbell_esp32.bin
```

#### Windows Operating System

(TBD)

## How to Use the Demo

### Enable Agora Link IoT Service and Create a Product

See [Enable Agora Link Service](https://docs-preprod.agora.io/en/iot-apaas/enable_agora_link) in the official documentation.

### Obtain License

The device-side SDK uses License for device authentication. A License is bound to a device, and one License can only be bound to one device at a time.

Agora provides each developer with 10 free test Licenses valid for 6 months. Contact iot@agora.io to apply for free Licenses or purchase commercial Licenses directly.

### Quickly Build Custom ESP32 Pure Call Demo

You can simply modify the ESP32 S3 pure call demo parameters to use your own product parameters.

In the device-side demo source code, you only need to modify the following parameters in main/app_config.h:

#define CONFIG_AGORA_APP_ID "4b31f\***\*\*\*\*\*\*\***\*\*\*\*\***\*\*\*\*\*\*\***3037"

#define CONFIG_CUSTOMER_KEY "8620f**\*\***\*\***\*\***\*\***\*\***\*\***\*\***7363"

#define CONFIG_CUSTOMER_SECRET "492c1\***\*\*\*\*\*\*\***\*\*\*\*\***\*\*\*\*\*\*\***e802"

#define CONFIG_MASTER_SERVER_URL "https://app.agoralink-iot-cn.sd-rtn.com"

#define CONFIG_SLAVE_SERVER_URL "https://api.agora.io/agoralink/cn/api"

#define CONFIG_PRODUCT_KEY "EJIJ\***\*\*\*\*\***5lI4"

You can obtain these parameters from the Agora Link Management Platform under Application Configuration >> Developer Options.

After modifying the parameters, refer to the compilation and flashing steps to build your own ESP32 pure call demo. For corresponding APP demo development, please refer to the Android and iOS documentation.

#define CONFIG_USER_ID "6875\***\*\*\*\***3440"

This parameter is used for the device to call the APP. After logging into the APP demo, it will display its user ID information. Modify this macro definition in the ESP32 firmware source code to match the corresponding user ID, compile and flash to call that APP Demo.

### Demo 1: Device Calls APP for Video Call

Open the installed custom application, enter the login interface, input account -> click `Login`.

Return to the development board (device side), confirm that the above firmware compilation and download steps are completed, and see the serial port printing every few seconds: "Agora: Press [REC] key to ring the doorbell ..."

Press the `[REC]` button to call the APP user. (Note: APP user information is specified by CONFIG_USER_ID macro definition)

The APP will show an incoming call interface displaying "Someone at the door" and wait for answer. You can already see the device's real-time video and hear its audio, but cannot talk to the device yet.

Click `Answer` in the APP to start real-time two-way communication with the device.

Click `Hang up` in the APP to end the call with the device.

Note: Before pressing [REC] to call the remote APP, the device is in low-power standby state with typical power consumption below 800uA. After calling, the doorbell automatically switches to full-power mode.

### Demo 2: APP Remotely Wakes Device for Video Call

Open the installed custom application, enter the login interface, input account -> click `Login`.

Return to the development board (device side), confirm that the above firmware compilation and download steps are completed, and see the serial port printing every few seconds: "Agora: Press [REC] key to ring the doorbell ..."

In the APP, input the device ID to call (printed in device-side information as device_id), click to call the device.

You can immediately see the device's real-time video feed, typically within about 1 second.

Click the phone button in the APP to start real-time two-way communication with the device.

## About Agora

Agora's audio and video IoT platform solution, based on Agora's self-built underlying real-time transmission network Agora SD-RTN™ (Software Defined Real-time Network), provides real-time audio and video stream transmission capabilities over the Internet for all network-enabled Linux/RTOS devices. This solution fully utilizes Agora's global network nodes and intelligent dynamic routing algorithms, while supporting various combined anti-weak network strategies such as forward error correction, intelligent retransmission, bandwidth prediction, and stream smoothing. It can deliver the best audio and video network experience with high connectivity, real-time performance, and stability even in various uncertain network environments where devices are located. Additionally, this solution has an extremely small package size and memory footprint, suitable for running on any resource-constrained IoT device, including all Espressif ESP32 series products.

## Technical Support

Please get technical support through the following links:

- If you find bugs in the sample code, please submit an [issue](https://github.com/AgoraIO-Community/ag-iot-callkit-esp32-demo/issues)
- For other questions, you can also directly contact the doorbell solution manager (WeChat: JJ2dog)

We will respond as soon as possible.
