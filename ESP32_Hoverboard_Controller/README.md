# ESP32 Hoverboard Controller

This project implements an ESP32-based controller for a hoverboard motor system, enabling remote control via WiFi.

## Robot Description

The robot is built using a hoverboard motor controller with two motors, a caster wheel for stability, and an aluminum extrusion frame.

## Code Functionality

The code provides:
- Serial communication with the hoverboard's STM32 main board.
- WiFi access point for remote control.
- Web-based interface for adjusting speed and steering.
- Real-time feedback display of motor speeds, battery voltage, and board temperature.
- OTA (Over-The-Air) updates for firmware.

## Hardware Requirements

- ESP32 microcontroller
- Hoverboard motor controller (STM32-based)
- Two hoverboard motors
- Caster wheel
- Aluminum extrusion frame

## Software Setup

1. Configure WiFi credentials in `include/wifi_config.h`.
2. Upload the code using PlatformIO.
3. Connect to the ESP32's WiFi network.
4. Access the web interface at the ESP32's IP address.

## Usage

- Use the web interface sliders to control speed and steering.
- Monitor feedback data in real-time.
- Update firmware wirelessly via OTA.