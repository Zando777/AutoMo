# AutoMo

> **WORK IN PROGRESS** - This project is under active development. I hope to have it finished in the coming weeks.

An autonomous lawn mower built from repurposed parts and hobby components. This project aims to create a fully autonomous ROS-based automatic mower for my parents' backyard.

![CAD Isometric View](Images/cad_isometric.png)

## Project Overview

AutoMo is an attempt to build an automatic lawn mower using mostly parts I have lying around. The project started on 3rd January 2026 and is being developed rapidly in spare time.

### Key Components

- **Drive Motors**: Repurposed hoverboard motors with custom flashed firmware enabling UART-based FOC control and wheel odometry
- **Cutting Motors**: 2x large drone motors as the cutting heads
- **Control Board**: Hoverboard control board with [custom FOC firmware](Custom_Hoverboard_Firmware_ROS/) (based on [hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC))
- **LiDAR**: LD06 360-degree LIDAR module - see my [LD06 interface project](https://github.com/Zando777/LD06)
- **Co-processor**: ESP32 for motor control and sensor interfacing
- **Main Computer**: Raspberry Pi Zero 2 for ROS and autonomous navigation
- **Power**: 2x 18V drill batteries
- **Frame**: 2020 aluminium extrusion with 3D printed mounting brackets

## CAD Design

![Side View](Images/cad_side_view.png)

![Front View](Images/cad_front_view.png)

![Top Isometric](Images/cad_top_isometric.png)

![Rear View](Images/cad_rear_view.png)

## Current Build

### Chassis

![Chassis Overview](Images/chassis_overview.jpg)

The chassis uses 2020 aluminium extrusion as the main frame with hoverboard hub motors for the drive wheels. A caster wheel at the rear provides stability.

![Chassis Top View](Images/chassis_top_view.jpg)

Top view showing the dual drone motors mounted as cutter heads, the hoverboard control board, and wiring to the ESP32.

![Chassis Angle View](Images/chassis_angle_view.jpg)

### Cutter Head Assembly

![Cutter Head](Images/cutter_head.jpg)

3D printed blade guard with drone motor mounted on 2020 extrusion mast.

## Hardware Requirements

- Hoverboard with custom firmware (included in this repo, based on [hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC))
- ESP32 development board
- Raspberry Pi Zero 2
- LD06 LIDAR module
- 2x drone motors (for cutting)
- 2x 18V drill batteries
- 2020 aluminium extrusion
- 3D printer for custom brackets

## Current Status

**Working:**
- Drivetrain and frame
- Cutter heads
- ESP32 motor control with web interface
- Wheel odometry feedback

**In Progress:**
- ROS integration
- Proper electronics mounting
- LiDAR integration
- Autonomous navigation

## Related Projects

- [LD06](https://github.com/Zando777/LD06) - ESP32 interface for LD06 360-degree LIDAR with real-time Python visualisation

## Acknowledgements

- [Emanuel FERU](https://github.com/EmanuelFeru) for the original hoverboard FOC firmware hack which the custom firmware is based on
