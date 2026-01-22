# Bot Jetson Stats

A comprehensive ROS2 package for monitoring and controlling NVIDIA Jetson hardware statistics, providing real-time diagnostic information and hardware control capabilities for robotics applications.

## Overview

Bot Jetson Stats is a ROS2-based monitoring and control system designed specifically for NVIDIA Jetson devices. It provides comprehensive hardware monitoring, diagnostic reporting, and control services for various Jetson components including CPU, GPU, memory, temperature, power consumption, and fan control.

**Original Author**: [Raffaello Bonghi](https://github.com/rbonghi/jetson_stats) - Creator of the [jetson_stats](https://github.com/rbonghi/jetson_stats) monitoring utilities and jtop library that form the foundation of this ROS2 package.

## Features

### 🔍 **Hardware Monitoring**
- **CPU Monitoring**: Real-time CPU usage, frequency, and temperature for each core
- **GPU Monitoring**: GPU utilization, memory usage, and temperature
- **Memory Monitoring**: RAM, SWAP, and EMC memory usage statistics
- **Power Monitoring**: Power consumption and voltage monitoring
- **Temperature Monitoring**: Comprehensive temperature readings with configurable warning/error thresholds
- **Fan Control**: Fan speed and profile management
- **Disk Monitoring**: Storage device health and usage statistics

### 🎛️ **Hardware Control Services**
- **Fan Control Service**: Adjust fan speed and profile (quiet/cool modes)
- **Power Mode Service**: Switch between different NVIDIA power modes (NVPModel)
- **Jetson Clocks Service**: Enable/disable Jetson performance clocks

### 📊 **Diagnostic System**
- ROS2 Diagnostic Messages for integration with diagnostic tools
- Human-readable terminal output for easy monitoring
- Configurable monitoring intervals and threshold levels
- Network mode status publishing

### 🚀 **ROS2 Integration**
- Native ROS2 nodes and services
- Diagnostic message publishing
- Launch file configurations
- Namespace support for multi-robot systems

## Package Structure

```
bot_jetson_stats/
├── bot_jetson_stats/                 # Main package
│   ├── launch/                       # Launch files
│   │   ├── jetson_stats.launch.py   # Main monitoring launch
│   │   ├── stats.launch.py          # Stats monitoring
│   │   └── network_mode_service.launch.py  # Network status
│   ├── scripts/                      # Python executables
│   │   ├── ros2_jtop_node.py        # Main monitoring node
│   │   ├── network_mode_publisher.py # Network status publisher
│   │   └── diagnostic_stats_terminal_viewer.py # Terminal viewer
│   ├── bot_jetson_stats/            # Python utilities
│   │   └── utils.py                 # Diagnostic message utilities
│   └── CMakeLists.txt               # Build configuration
├── bot_jetson_stats_interfaces/      # Service definitions
│   ├── srv/                         # Service files
│   │   ├── Fan.srv                  # Fan control service
│   │   ├── JetsonClocks.srv         # Jetson clocks service
│   │   └── NVPModel.srv             # Power mode service
│   └── CMakeLists.txt               # Interface build config
└── README.md                         # This file
```

## Dependencies

### System Dependencies
- **jtop**: NVIDIA Jetson monitoring library (from [jetson_stats](https://github.com/rbonghi/jetson_stats))
- **Python 3**: Python 3.x runtime
- **Linux**: Linux operating system (tested on Tegra-based systems)

### ROS2 Dependencies
- **rclpy**: Python ROS2 client library
- **rclcpp**: C++ ROS2 client library
- **diagnostic_msgs**: ROS2 diagnostic message types
- **std_msgs**: ROS2 standard message types
- **ament_cmake**: Build system
- **ament_cmake_python**: Python build support

## Installation

### Prerequisites
1. Install ROS2 (Humble or later recommended)
2. Install jtop library:
   ```bash
   sudo pip3 install jtop
   ```

### Building from Source
1. Clone this repository to your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> bot_jetson_stats
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select bot_jetson_stats bot_jetson_stats_interfaces
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Quick Start

1. **Launch the main monitoring system**:
   ```bash
   ros2 launch bot_jetson_stats jetson_stats.launch.py
   ```

2. **View diagnostic statistics in terminal**:
   ```bash
   ros2 run bot_jetson_stats diagnostic_stats_terminal_viewer
   ```

3. **Monitor diagnostic messages**:
   ```bash
   ros2 topic echo /diagnostics
   ```

### Launch Files

#### Main Monitoring Launch (`jetson_stats.launch.py`)
Launches the complete Jetson monitoring system including:
- Jetson stats monitoring node
- Diagnostic terminal viewer
- Configurable robot namespace support

#### Stats Launch (`stats.launch.py`)
Launches only the statistics monitoring components.

#### Network Mode Service (`network_mode_service.launch.py`)
Launches the network mode status publisher.

### Services

#### Fan Control Service (`/jtop/fan`)
- **Request**: `mode` (string: "quiet" or "cool"), `speed` (int64)
- **Response**: `set_fan_mode`, `set_fan_speed`
- **Usage**: Control fan speed and profile

#### Power Mode Service (`/jtop/nvpmodel`)
- **Request**: `nvpmodel` (int64: power mode ID)
- **Response**: `power_mode` (string: current power mode)
- **Usage**: Switch between different NVIDIA power modes

#### Jetson Clocks Service (`/jtop/jetson_clocks`)
- **Request**: `status` (bool: enable/disable)
- **Response**: `done` (bool: operation completion status)
- **Usage**: Enable/disable Jetson performance clocks

### Topics

#### Published Topics
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): ROS2 diagnostic messages
- `/diagnostic_stats` (std_msgs/String): Human-readable diagnostic summary
- `/network_mode_status` (std_msgs/String): Network mode status updates

## Configuration

### Node Parameters

The main monitoring node (`ros2_jtop_node.py`) supports the following parameters:

- **`interval`** (default: 2): Monitoring update interval in seconds
- **`level_error`** (default: 60): Temperature error threshold in Celsius
- **`level_warning`** (default: 40): Temperature warning threshold in Celsius
- **`level_ok`** (default: 20): Temperature OK threshold in Celsius

### Robot Configuration

The system supports robot-specific configuration through a `robot_config.yaml` file located in the workspace root:

```yaml
robot_configuration:
  robot_name: "my_robot"
```

## Monitoring Features

### Real-time Statistics
- **CPU**: Per-core utilization, frequency, and temperature
- **GPU**: Utilization, memory usage, temperature
- **Memory**: RAM, SWAP, and EMC usage with thresholds
- **Power**: Consumption monitoring and voltage readings
- **Temperature**: Multi-zone temperature monitoring with configurable alerts
- **Fan**: Speed control and profile management
- **System**: Uptime, power mode, and Jetson clocks status

### Diagnostic Integration
- Compatible with ROS2 diagnostic tools (rqt_diagnostic, rqt_topic)
- Standard diagnostic message format
- Configurable warning and error thresholds
- Hardware identification and status reporting

## Troubleshooting

### Common Issues

1. **Permission Errors**: Ensure jtop has proper permissions to access Jetson hardware
2. **Service Failures**: Check if the Jetson device supports the requested operation
3. **Temperature Readings**: Verify sensor access and calibration


### Monitoring Tools

- **rqt_topic**: View and monitor ROS2 topics
- **rqt_diagnostic**: Visualize diagnostic information
- **rqt_graph**: View node and topic connections

## Development

### Adding New Monitoring Features

1. Extend the `utils.py` file with new diagnostic functions
2. Add new service definitions in the interfaces package
3. Update the main node to include new monitoring capabilities
4. Add appropriate launch file configurations


## Acknowledgments

- **[Raffaello Bonghi](https://github.com/rbonghi/jetson_stats)**: Original author and creator of the [jetson_stats](https://github.com/rbonghi/jetson_stats) monitoring utilities and jtop library
- **jtop library**: NVIDIA Jetson monitoring capabilities (part of jetson_stats project)
- **ROS2 community**: Framework and diagnostic message support
- **Original contributors**: Base monitoring utilities and concepts from the jetson_stats project

## License Notice

The `jetson_stats` project and the `jtop` library are licensed under the **MIT License**.  
This repository uses them as external dependencies and does not include their source code.

---

**Note**: This package is specifically designed for NVIDIA Jetson devices and requires appropriate hardware to function properly.
