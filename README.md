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
- ROS2 Jazzy integration
- Teleop control via keyboard

**In Progress:**
- Proper electronics mounting
- LiDAR integration
- Autonomous navigation

## Software Setup

### 1. ROS2 Jazzy Installation (Ubuntu 24.04)

```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# Install additional dependencies
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions
```

Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. ROS2 Workspace Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone hoverboard driver (use humble branch for ROS2)
git clone -b humble https://github.com/hoverboard-robotics/hoverboard-driver.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. ESP32 ROS Bridge Setup

The ESP32 acts as a WiFi-to-Serial bridge between ROS and the hoverboard.

1. Install PlatformIO: `pip install platformio`

2. Configure WiFi credentials in `ESP32_ROS_Bridge/include/wifi_config.h`:
```c
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Static IP configuration
IPAddress staticIP(192, 168, 68, 100);      // ESP32's fixed IP
IPAddress gateway(192, 168, 68, 1);         // Your router's IP
IPAddress subnet(255, 255, 252, 0);
IPAddress dns(192, 168, 68, 1);
```

3. Flash the ESP32:
```bash
cd ESP32_ROS_Bridge
pio run -t upload -e esp32dev
```

4. For subsequent OTA updates:
```bash
pio run -t upload -e esp32dev-ota
```

### 4. Running the System

**Terminal 1 - Start socat bridge:**
```bash
socat pty,link=/tmp/hoverboard,raw tcp:192.168.68.100:8880
```

**Terminal 2 - Launch ROS driver:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hoverboard_driver diffbot.launch.py
```

**Terminal 3 - Teleop control:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/hoverboard_base_controller/cmd_vel -p stamped:=true -p frame_id:=base_link
```

Teleop controls:
- `i` - forward
- `,` - backward
- `j` - turn left
- `l` - turn right
- `k` - stop
- `z` - decrease speed
- `q` - increase speed

**Important:** Reduce speed to ~0.2 before driving (press `z` multiple times).

### 5. Monitoring

- ESP32 web dashboard: http://192.168.68.100
- ROS topics:
  - `/hoverboard_base_controller/odom` - odometry
  - `/hoverboard/battery_voltage` - battery level
  - `/hoverboard/left_wheel/velocity` - wheel speeds

## Related Projects

- [LD06](https://github.com/Zando777/LD06) - ESP32 interface for LD06 360-degree LIDAR with real-time Python visualisation

## Acknowledgements

- [Emanuel FERU](https://github.com/EmanuelFeru) for the original hoverboard FOC firmware hack which the custom firmware is based on
