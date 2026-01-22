<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  One Brain, any Bot.
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Website-000?logo=vercel&logoColor=white" alt="Website"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">BotBrain ROS2 Workspace</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="License: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

<p align="center">
  <a href="README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="../docs/i18n/software/README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="../docs/i18n/software/README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="../docs/i18n/software/README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="../docs/i18n/software/README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

## Overview

The **BotBrain Workspace** is a modular, open-source ROS2 framework for autonomous robot control, navigation, and localization. Designed with a robot-agnostic architecture, it enables rapid development and deployment of advanced robotics applications across multiple robot platforms.

**Key Features:**
- 🤖 **Multi-Robot Support**: Single codebase for Go2, Tita, G1 and custom robots
- 🗺️ **Visual SLAM**: RTABMap-based localization with dual camera support
- 🎮 **Multiple Control Modes**: Joystick, web interface, and autonomous navigation
- 👁️ **AI Vision**: YOLOv8/v11 object detection
- 🐳 **Docker Ready**: Containerized deployment with GPU acceleration
- 🔄 **Lifecycle Management**: Robust node orchestration and failure recovery


## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
- [Repository Structure](#repository-structure)
- [Creating a Custom Robot Package](#creating-a-custom-robot-package)
- [Package Overview](#package-overview)
- [Docker Services](#docker-services)
- [Configuration](#configuration)

## Hardware Requirements

### Supported Robot Platforms
- **Unitree Go2** 
- **Unitree G1** 
- **Tita**
- **Custom Robots** - Follow the [Custom Robot Package Guide](#creating-a-custom-robot-package)

### Required Hardware
- **Robot Platform**: One of the supported robots above
- **Onboard Computer**:
  - Nvidia Jetson Orin Series or newer
- **Sensors**:
  - Intel RealSense cameras (for visual SLAM)
  - LiDAR (for LiDAR-based SLAM)
- **Network**:
  - Ethernet connection to robot
  - Wi-Fi adapter (for remote control)

### Optional Hardware
- **Game Controller**: For teleoperation

## Quick Start

### Launch Using Docker Compose

For containerized deployment:

```bash
# Start all services
docker compose up -d

# Start specific services
docker compose up -d state_machine bringup localization navigation

# View logs
docker compose logs -f bringup

# Stop services
docker compose down
```

### Verify System is Running

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list
```

### Developer Container

I you want to use the same docker image for development, without creating a new service, it is possible to run an iterative dev container:

```bash
# Init the dev container
cd botbrain_ws
docker compose up dev -d

# Init an iterative terminal
docker compose exec dev bash
```

Once the iterative terminal opens, you can use it to create, build and run new features that are not yet integrated with the docker services.

## Repository Structure

```
botbrain_ws/
├── README.md                          # This file
├── LICENSE                            # MIT License
│
├── robot_config.yaml                  # Main configuration file
├── install.sh                         # Automated installation script
├── robot_select.sh                    # Robot selection helper
│
├── docker-compose.yaml                # Docker services definition
├── botbrain.service                   # Systemd autostart service
├── cyclonedds_config.xml              # DDS middleware configuration
│
└── src/                               # ROS 2 packages
    │
    ├── Core System Packages
    │   ├── bot_bringup/               # Main launch & twist mux coordination
    │   ├── bot_custom_interfaces/     # Custom messages, services, actions
    │   └── bot_state_machine/         # Lifecycle & state management
    │
    ├── Robot Model & Visualization
    │   └── bot_description/           # URDF/XACRO models & robot_state_publisher
    │
    ├── Navigation & Localization
    │   ├── bot_localization/          # RTABMap SLAM (visual & LiDAR)
    │   └── bot_navigation/            # Nav2 navigation stack
    │
    ├── Perception & Control
    │   ├── bot_yolo/                  # YOLOv8/v11 object detection
    │   └── joystick-bot/              # Game controller interface
    │
    ├── AI & Monitoring
    │   ├── bot_jetson_stats/          # Jetson hardware monitoring
    │   └── bot_rosa/                  # ROSA AI natural language control
    │
    └── Robot-Specific Packages
        ├── g1_pkg/                    # Unitree G1 hardware interface
        ├── go2_pkg/                   # Unitree Go2 hardware interface
        ├── tita_pkg/                  # Tita hardware interface
        └── your_robot_pkg/            # Your custom robot (see guide below)
```

## Creating a Custom Robot Package

To add support for a new robot platform, follow this guide using [go2_pkg](src/go2_pkg) as a reference template.

**Note**: The go2_pkg package communicates with the Unitree Go2 robot via ROS 2 topics (subscribing to Unitree's native ROS 2 topics and republishing in BotBrain format). Your custom robot package may use similar topic-based communication, direct hardware APIs, or SDK interfaces depending on your robot's architecture. The idea is to create a standard package interface between botbrain_ws packages and the the robot.

### Required Package Structure

Your custom robot package must follow this naming convention to work seamlessly with all the packages: `{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # ROS 2 package manifest
├── CMakeLists.txt                     # Build configuration
├── README.md                          # Package documentation
├── launch/
│   └── robot_interface.launch.py     # REQUIRED: Main hardware interface launcher
├── config/
│   └── nav2_params.yaml               # REQUIRED: Navigation parameters
├── scripts/
│   ├── {robot_model}_read.py                # REQUIRED: Reads sensor data from robot
│   └── {robot_model}_write.py               # REQUIRED: Sends commands to robot
├── {robot_model}_pkg/                 # Python package directory
│   └── tools/                         # OPTIONAL: ROSA AI assistant tools
│       ├── __init__.py                # Empty package initialization
│       └── {robot_model}.py           # LangChain tools for ROSA integration
├── xacro/
│   └── robot.xacro                    # REQUIRED: Robot URDF model
└── meshes/
    └── *.dae, *.stl                   # Visual & collision meshes
```

### Step-by-Step Creation Guide

#### 1. Create New ROS 2 Package

Create the package structure using ROS 2 tools (if you do not have ros2 installed in your host system, this can be done from a dev container):

```bash
cd src/
ros2 pkg create {robot_model}_pkg --build-type ament_cmake --dependencies rclcpp rclpy 
cd {robot_model}_pkg
```

Create required directories:
```bash
mkdir -p launch config scripts xacro meshes maps
```

#### 2. Configure package.xml

Edit `package.xml` and add required dependencies:
- Add `bot_custom_interfaces` as a dependency
- Update package name, version, description, and maintainer information
- Ensure all sensor message dependencies are included

#### 3. Configure CMakeLists.txt

Update the build configuration to install all package resources:
- Install launch files directory
- Install config files directory
- Install scripts as executables
- Install xacro, urdf, and meshes directories
- Use `ament_python_install_package()` for Python modules

#### 4. Create Hardware Interface Launch File

**CRITICAL**: Create `launch/robot_interface.launch.py` (exact name required)

This launch file must:
- Read `robot_config.yaml` from workspace root
- Extract `robot_name` for namespace configuration
- Launch lifecycle nodes for hardware read and write
- Use `LifecycleNode` from `launch_ros.actions`
- Apply correct namespace to all nodes

Reference: See [go2_pkg/launch/robot_interface.launch.py](src/go2_pkg/launch/robot_interface.launch.py) for complete example.

#### 5. Implement Hardware Interface Nodes

**Create `scripts/{robot_model}_read.py`** - Reads sensor data and publishes to ROS 2:

This lifecycle node must:
- Initialize as a `LifecycleNode` with name `robot_read_node`
- Implement lifecycle callbacks: `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- In `on_configure`: Create publishers for odometry, IMU, joint states, and battery
- In `on_activate`: Start data reading loop (typically 50Hz) from robot hardware/topics
- Process robot sensor data and publish to ROS 2 topics
- In `on_deactivate`: Stop data publishing but maintain connections
- In `on_cleanup`: Close hardware connections and release resources

Reference: See [go2_pkg/scripts/go2_read.py](src/go2_pkg/scripts/go2_read.py) for complete implementation.

**Create `scripts/{robot_model}_write.py`** - Receives commands and sends to robot:

This lifecycle node must:
- Initialize as a `LifecycleNode` with name `robot_write_node`
- In `on_configure`: Create subscriber for `cmd_vel_out` topic and establish robot communication
- Implement callback to receive velocity commands and forward to robot hardware
- In `on_deactivate`: Send stop command (zero velocity) to robot
- In `on_cleanup`: Close hardware connections and release resources
- Optionally: Implement robot-specific services (mode switching, gait control, etc.)

Reference: See [go2_pkg/scripts/go2_write.py](src/go2_pkg/scripts/go2_write.py) for complete implementation.

#### 6. Create Navigation Parameters

Create `config/nav2_params.yaml` with your robot's specifications. See [Nav2 Documentation](https://docs.nav2.org/) as reference.

You will need to add a wildcard to the node configuration sections. See [go2_pkg/config/nav2_params.yaml](src/go2_pkg/config/nav2_params.yaml) for complete configuration example.

#### 7. Create Robot Description (XACRO)

Create `xacro/robot.xacro` with your robot's URDF model:

Your XACRO file should define:
- `base_link` as the main robot body link
- `interface_link` as the interface part between robot and BotBrain
- All robot joints and links (legs, arms, etc.)
- Sensor links (cameras, LiDAR, IMU)
- Visual meshes for RViz visualization
- Collision meshes for navigation
- Joint limits and dynamics
- Inertial properties

Reference: See [go2_pkg/xacro/robot.xacro](src/go2_pkg/xacro/robot.xacro) for complete robot description.

#### 8. Configure Workspace

Update the workspace `robot_config.yaml` (it can be done from install.sh):

```yaml
robot_configuration:
  robot_name: "my_robot"               # Namespace for all topics
  robot_model: "your_robot"            # Must match your package name without "_pkg"
  description_file_type: "xacro"       # "xacro" or "urdf"
  network_interface: "eth0"            # Network interface for robot communication
```

**IMPORTANT**: The `robot_model` field must match your package name **without** the `_pkg` suffix:
- Package name: `your_robot_pkg`
- robot_model: `your_robot`

#### 9. Build and Test

```bash
# Build your package
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select your_robot_pkg

# Source the workspace
source install/setup.bash

# Test your hardware interface
ros2 launch your_robot_pkg robot_interface.launch.py

# Launch with full system
ros2 launch bot_bringup bringup.launch.py
```

You can build and test the new package using a dev container.

#### 10. Create ROSA Tools (Optional)

**ROSA** (Robot Operating System Assistant) is an AI assistant that enables natural language control of your robot. By creating tools for ROSA, users can interact with your robot using conversational commands.

**Create the tools directory structure:**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### Package Integration Points

The BotBrain system will automatically find and use your package based on these conventions:

1. **Package Naming**: `{robot_model}_pkg` format
2. **Launch File**: `launch/robot_interface.launch.py` (exact name required)
3. **Navigation Config**: `config/nav2_params.yaml` (used by bot_navigation)
4. **Description Files**: `xacro/robot.xacro` or `urdf/robot.urdf` (used by bot_description)

### Required Topics Your Package Must Provide

For full system integration, your hardware interface should publish:

| Topic | Message Type | Description | Frequency |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | Robot odometry | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | IMU data | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | Joint positions/velocities | 50Hz |

And subscribe to:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | Velocity commands from twist_mux |


## Package Overview

### Core System Packages

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_bringup](src/bot_bringup) | Main launch coordination, twist multiplexer, and system orchestration | [README](src/bot_bringup/README.md) |
| [bot_state_machine](src/bot_state_machine) | Lifecycle management, node coordination, and system state control | [README](src/bot_state_machine/README.md) |
| [bot_custom_interfaces](src/bot_custom_interfaces) | Custom ROS 2 messages, services, and actions | [README](src/bot_custom_interfaces/README.md) |
| [bot_description](src/bot_description) | Robot URDF/XACRO models and robot_state_publisher | [README](src/bot_description/README.md) |

### Navigation & Localization

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_localization](src/bot_localization) | RTABMap SLAM with support for visual and LiDAR-based mapping | [README](src/bot_localization/README.md) |
| [bot_navigation](src/bot_navigation) | Nav2 navigation stack with robot-agnostic configuration | [README](src/bot_navigation/README.md) |

### Perception & Control

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_yolo](src/bot_yolo) | YOLOv8/v11 object detection with TensorRT acceleration | [README](src/bot_yolo/README.md) |
| [joystick-bot](src/joystick-bot) | Game controller interface with dead-man switch safety | [README](src/joystick-bot/README.md) |

### Robot-Specific Packages

| Package | Description | Documentation |
|---------|-------------|---------------|
| [go2_pkg](src/go2_pkg) | Unitree Go2 quadruped hardware interface and description | [README](src/go2_pkg/README.md) |
| [tita_pkg](src/tita_pkg) | Tita quadruped hardware interface and description | [README](src/tita_pkg/README.md) |

## Docker Services

The workspace includes multiple Docker services for containerized deployment:

| Service | Description | Auto-start | Dependencies |
|---------|-------------|------------|--------------|
| `dev` | Development container (interactive) | No | - |
| `builder_base` | Builds all workspace packages | No | - |
| `state_machine` | Lifecycle management service | Yes | - |
| `bringup` | Main robot bringup | Yes | state_machine |
| `localization` | RTABMap localization | Yes | bringup |
| `navigation` | Nav2 navigation servers | Yes | localization |
| `rosa` | AI tool calling services | Yes | bringup |
| `yolo` | Object detection service | Yes | bringup |

### Docker Usage

```bash
# Start all services
docker compose up -d

# Start specific service with dependencies
docker compose up -d navigation  # Automatically starts bringup, localization

# View logs
docker compose logs -f bringup

# Stop all services
docker compose down

# Rebuild after code changes
docker compose build
docker compose up -d
```

## Configuration

### Main Configuration File

The [robot_config.yaml](robot_config.yaml) file is the central configuration point:

```yaml
robot_configuration:

  # Robot identifier - used as namespace for all topics
  robot_name: ""                    # Example: "go2_robot1", "tita_lab"

  # Robot type - determines which hardware package to launch
  robot_model: "go2"                # Options: "go2", "tita", "your_robot"

  # Description file format
  description_file_type: "xacro"    # Options: "xacro", "urdf"

  # Network interface for ROS2 communication
  network_interface: "eno1"         # Example: "eth0", "wlan0", "eno1"

  # Tita-specific: namespace for Tita robot communication
  tita_namespace: "tita3036731"     # Only used when robot_model: "tita"

  # OpenAI API Key for AI features (optional)
  openai_api_key: ""                # Required for ROSA AI assistant

  # Wi-Fi configuration (optional)
  wifi_interface: ""                # Wi-Fi interface name (e.g., "wlan0")
  wifi_ssid: ""                     # Wi-Fi network SSID
  wifi_password: ""                 # Wi-Fi network password
```

### Network Configuration

The workspace uses CycloneDDS for ROS 2 communication. Configuration in [cyclonedds_config.xml](cyclonedds_config.xml):

Set the network interface in [robot_config.yaml](robot_config.yaml) to match your hardware connection.

### Systemd Autostart Service

The [botbrain.service](botbrain.service) file enables automatic startup on boot:

```bash
# Install service (done by install.sh)
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# Manual control
sudo systemctl start botbrain.service   # Start now
sudo systemctl stop botbrain.service    # Stop
sudo systemctl status botbrain.service  # Check status

# View logs
journalctl -u botbrain.service -f
```

### Adding Support for New Robots

See the [Creating a Custom Robot Package](#creating-a-custom-robot-package) guide above. We especially welcome contributions that add support for new robot platforms!

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
