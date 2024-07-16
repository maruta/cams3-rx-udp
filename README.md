# ESP32S3 Camera Firmware for FPV RC Car Experiments

This project contains firmware for ESP32S3-based camera modules, designed to create an FPV (First Person View) RC car. It's suitable for AI car experiments and control theory research.

## Project Objectives

- Develop a prototype for FPV RC cars
- Provide an experimental platform for AI cars
- Facilitate practical research and verification in control theory

## Project Structure

```
.
├── platformio.ini
├── src
│   └── main.cpp
└── host
    └── python
        ├── cams3tx.py
        └── test_cams3tx.py
```

## Supported Hardware

This project supports two hardware configurations:

1. **Unit CamS3 Wi-Fi Camera (OV2640)**
2. **Seeed Studio XIAO ESP32S3 Sense**

## Setup and Usage

### 1. Cloning the Repository

This repository includes submodules. Use the following command to clone the repository and initialize the submodules:

```
git clone --recursive [repository URL]
```

If you've already cloned the repository, initialize the submodules with:

```
git submodule update --init --recursive
```

### 2. Setting Up the Development Environment

1. Install Visual Studio Code: https://code.visualstudio.com/
2. Install PlatformIO IDE for VSCode:
   - In VSCode, go to the Extensions tab and search for "PlatformIO IDE"
3. Open the cloned project folder in VSCode

### 3. Selecting Hardware and Switching Environments

Use the `platformio.ini` file to select the environment for your hardware:

1. Open the `platformio.ini` file in VSCode.

2. The file contains environment definitions like this:

   ```ini
   [env:m5_cams3unit]
   board = esp32s3box
   board_build.partitions = custom-16M.csv
   build_flags = 
       ${env.build_flags}
       -D ESP32S3BOX

   [env:seeed_xiao_esp32s3]
   board = seeed_xiao_esp32s3
   board_build.partitions = custom-8M.csv
   build_flags = 
       ${env.build_flags}
       -D SEEED_XIAO_ESP32S3
   ```

3. Add the `default_envs` option at the top of the file to specify the environment you want to use:
   - For Unit CamS3 Wi-Fi Camera (OV2640):
     ```ini
     [platformio]
     default_envs = m5_cams3unit
     ```
   - For Seeed Studio XIAO ESP32S3 Sense:
     ```ini
     [platformio]
     default_envs = seeed_xiao_esp32s3
     ```

4. Save the file.

This applies the appropriate hardware settings for the selected environment. To switch environments, simply change the value of `default_envs`.

### 4. Configuring WiFi Settings

1. In the `platformio.ini` file, locate the following lines:

   ```ini
   build_flags = 
       -DCAMS3RX_WIFI_SSID=${sysenv.CAMS3RX_WIFI_SSID}
       -DCAMS3RX_WIFI_PASSWORD=${sysenv.CAMS3RX_WIFI_PASSWORD}
       -DCAMS3RX_WIFI_MODE_${sysenv.CAMS3RX_WIFI_MODE}
   ```

2. Set environment variables or specify values directly:
   - For AP mode:
     ```ini
     build_flags = 
         -DCAMS3RX_WIFI_SSID='"ESP32CAM_AP"'
         -DCAMS3RX_WIFI_PASSWORD='"password123"'
         -DCAMS3RX_WIFI_MODE_AP
     ```
   - For STA mode:
     ```ini
     build_flags = 
         -DCAMS3RX_WIFI_SSID='"YourHomeWiFi"'
         -DCAMS3RX_WIFI_PASSWORD='"YourWiFiPassword"'
         -DCAMS3RX_WIFI_MODE_STA
     ```

### 5. Building and Uploading the Firmware

1. Open the PlatformIO tab at the bottom of VSCode
2. Click on "PROJECT TASKS" → "Build" to build the firmware
3. Connect the ESP32S3 board to your PC via USB
4. Click "Upload" to upload the firmware

### 6. Setting Up the Host PC

1. Install Python: https://www.python.org/downloads/
2. Install required packages:
   ```
   pip install opencv-python pygame
   ```

### 7. Starting the ESP32S3 and Connecting

1. Power on the ESP32S3 board
2. WiFi connection:
   - AP mode: Connect to the access point created by ESP32S3 in your PC's WiFi settings
   - STA mode: Ensure both ESP32S3 and your PC are connected to the same WiFi network

### 8. Running the Test Script

1. Navigate to the `host/python` directory in the command line
2. Run the following command:
   ```
   python test_cams3tx.py
   ```
3. The camera image will be displayed in a window, and RTT and frame rate information will be output to the console

## Known Issues and Notes

1. **ESP32S3 Wireless Transmission Power**: Some revisions of ESP32S3 may require lowering the wireless transmission power for stable communication. In the `main.cpp` file, you'll find a commented-out line:

   ```cpp
   // esp_wifi_set_max_tx_power(55);
   ```

   If you experience unstable communication, try uncommenting this line and adjusting the value. 

2. **Unit CamS3 Camera Clock and WiFi Interference**: Under certain conditions, Unit CamS3 may experience interference between the camera clock and WiFi, leading to unstable communication. There are currently no known solutions to this issue. If you encounter this problem, you may need to experiment with different WiFi channels or other approaches.
