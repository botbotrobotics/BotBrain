<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# joystick Bot - ROS2 Joystick Interface

[🇧🇷 Versão em Português](README_br.md)

A flexible and configurable ROS2 package for interfacing game controllers/joysticks with robots. This package uses pygame to read joystick inputs and publishes them as ROS2 topics, making it easy to control your robot with any standard game controller.

## Features

- **Lifecycle Node Architecture**: Proper state management with ROS2 lifecycle nodes
- **Hot-plug Support**: Automatically detects joystick connection and disconnection
- **Fully Configurable**: Button and axis mappings can be customized via YAML configuration
- **Customizable Topic Names**: Publish to any topic names you prefer
- **Dead Man Switch**: Safety feature for robot control
- **Deadband Support**: Configurable deadzones for analog sticks to prevent drift
- **Default DualSense (PS5) Support**: Pre-configured for PlayStation 5 DualSense controller

### Demo Video

[▶️ Watch the demo video](https://youtu.be/TQeibdXH21g)

*Demo video showing the node launch, lifecycle configuration, and topic publishing in action.*

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Published Topics](#published-topics)
- [Parameters](#parameters)
- [Supported Controllers](#supported-controllers)
- [Troubleshooting](#troubleshooting)
- [Lifecycle Management](#lifecycle-management)
- [Directory Structure](#directory-structure)
- [Custom Message Types](#custom-message-types)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)

## Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3
- pygame

### Install pygame

```bash
pip3 install pygame
```

### Build the Package

```bash
cd ~/your_ros2_workspace
colcon build --packages-select joystick_bot
source install/setup.bash
```

## Quick Start

### Launch the Node

```bash
ros2 launch joystick_bot js.launch.py
```

### Configure and Activate the Lifecycle Node

The node starts in the `unconfigured` state. To use it, you need to configure and activate it:

```bash
# Configure the node
ros2 lifecycle set /joystick_interface configure

# Activate the node
ros2 lifecycle set /joystick_interface activate
```

### Check Published Topics

```bash
# View velocity commands
ros2 topic echo /cmd_vel_joy

# View button states
ros2 topic echo /button_state

# View dead man switch state
ros2 topic echo /dead_man_switch
```

## Configuration

The main configuration file is located at `config/js_config.yaml`. You can customize all aspects of the joystick interface.

### Default Configuration

The package comes pre-configured with default values suitable for the **PlayStation 5 DualSense controller**. If you're using a DualSense controller, you can use the package without any configuration changes.

### Example Configuration

```yaml
/**/joystick_interface:
  ros__parameters:
    device_input: "/dev/input/js0"
    linear_deadband: 0.02
    angular_deadband: 0.02

    # Topic names
    cmd_vel_topic: "cmd_vel_joy"
    button_state_topic: "button_state"
    dead_man_switch_topic: "dead_man_switch"

    # Button mappings (pygame button indices)
    # Default values configured for DualSense (PS5) controller
    button_mapping:
      x_button: 0
      a_button: 1
      b_button: 2
      y_button: 3
      l1_button: 4
      r1_button: 5
      l2_button: 6
      r2_button: 7
      select_button: 8
      start_button: 9
      dead_man_button: 4  # L1 button by default

    # Axis mappings (pygame axis indices)
    # Default values configured for DualSense (PS5) controller
    axis_mapping:
      linear_x_axis: 1    # Left stick vertical
      linear_y_axis: 0    # Left stick horizontal
      angular_y_axis: 5   # Right stick vertical
      angular_z_axis: 2   # Right stick horizontal
```

### Custom Configuration File

You can specify a custom configuration file when launching:

```bash
ros2 launch joystick_bot js.launch.py config_file:=/path/to/your/config.yaml
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_joy` (default) | `geometry_msgs/Twist` | Velocity commands from joystick axes |
| `/button_state` (default) | `joystick_bot/ControllerButtonsState` | Current state of all controller buttons |
| `/dead_man_switch` (default) | `std_msgs/Bool` | Dead man switch state (L1 button by default) |

## Parameters

### General Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_input` | string | `/dev/input/js0` | Device path for the joystick |
| `linear_deadband` | double | 0.02 | Deadband threshold for linear axes |
| `angular_deadband` | double | 0.02 | Deadband threshold for angular axes |

### Topic Name Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cmd_vel_topic` | string | `cmd_vel_joy` | Topic name for velocity commands |
| `button_state_topic` | string | `button_state` | Topic name for button states |
| `dead_man_switch_topic` | string | `dead_man_switch` | Topic name for dead man switch |

### Button Mapping Parameters

All button mappings are pygame button indices (integers). **Default values are for DualSense (PS5) controller:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `button_mapping.x_button` | 0 | X/Square button |
| `button_mapping.a_button` | 1 | A/Cross button |
| `button_mapping.b_button` | 2 | B/Circle button |
| `button_mapping.y_button` | 3 | Y/Triangle button |
| `button_mapping.l1_button` | 4 | L1/Left bumper |
| `button_mapping.r1_button` | 5 | R1/Right bumper |
| `button_mapping.l2_button` | 6 | L2/Left trigger |
| `button_mapping.r2_button` | 7 | R2/Right trigger |
| `button_mapping.select_button` | 8 | Select/Share button |
| `button_mapping.start_button` | 9 | Start/Options button |
| `button_mapping.dead_man_button` | 4 | Dead man switch button (L1) |

### Axis Mapping Parameters

All axis mappings are pygame axis indices (integers). **Default values are for DualSense (PS5) controller:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `axis_mapping.linear_x_axis` | 1 | Left stick vertical (forward/backward) |
| `axis_mapping.linear_y_axis` | 0 | Left stick horizontal (left/right strafe) |
| `axis_mapping.angular_y_axis` | 5 | Right stick vertical (pitch) |
| `axis_mapping.angular_z_axis` | 2 | Right stick horizontal (yaw/rotation) |

## Supported Controllers

#### Controller Button Reference

The following diagram illustrates the standard button naming and position conventions used in this package. Use this as a reference when configuring your own controller:

![Controller Button Layout](docs/images/controller_layout.png)

If your controller has a different layout, you can customize the button and axis mappings through the js_config.yaml file.

### PlayStation 5 DualSense Controller (Default)

The default configuration is optimized for the **DualSense (PS5) controller**. Simply connect your controller via USB or Bluetooth and launch the node.

#### DualSense Control Mapping (Default):

| Control | Function | Parameter | Index |
|---------|----------|-----------|-------|
| **Left Stick** | Robot movement (forward/backward, left/right) | `linear_x_axis`, `linear_y_axis` | Axes 1, 0 |
| **Right Stick** | Robot rotation (yaw/pitch) | `angular_z_axis`, `angular_y_axis` | Axes 2, 5 |
| **Cross (X)** | A Button | `a_button` | Button 1 |
| **Circle (O)** | B Button | `b_button` | Button 2 |
| **Square** | X Button | `x_button` | Button 0 |
| **Triangle** | Y Button | `y_button` | Button 3 |
| **L1** | Dead man switch / L1 Button | `dead_man_button`, `l1_button` | Button 4 |
| **R1** | R1 Button | `r1_button` | Button 5 |
| **L2** | L2 Trigger | `l2_button` | Button 6 |
| **R2** | R2 Trigger | `r2_button` | Button 7 |
| **Share** | Select Button | `select_button` | Button 8 |
| **Options** | Start Button | `start_button` | Button 9 |
| **D-pad** | Directional buttons | HAT 0 | HAT values |


### Other Controllers

To use other controllers (Xbox, Logitech, etc.), you'll need to determine the button and axis mappings for your specific controller.

#### Finding Button and Axis Mappings

The easiest way to find button and axis mappings is using the `jstest` command-line tool:

```bash
# Install jstest if not already available
sudo apt-get install joystick

# Monitor joystick events in real-time
jstest /dev/input/js0
```

Press buttons and move sticks to see which button/axis indices are triggered. The output will show:
- Button numbers when pressed/released
- Axis numbers and their values when sticks are moved

Once you identify the indices for your controller, update the `config/js_config.yaml` file with your custom mappings.

## Troubleshooting

### Joystick Not Detected

1. Check if the joystick is recognized by the system:
   ```bash
   ls /dev/input/js*
   ```

2. Verify pygame can detect it:
   ```bash
   python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print(f'Joysticks found: {pygame.joystick.get_count()}')"
   ```

3. Check permissions:
   ```bash
   sudo chmod a+rw /dev/input/js0
   ```

### Connection Issues

- If using Bluetooth, ensure the controller is properly paired
- Try reconnecting the controller (USB or Bluetooth)
- Check `dmesg` for any USB/input device errors
- The node supports hot-plugging, so you can disconnect and reconnect while running

### Drift or Unwanted Movement

- Increase the deadband values in the configuration:
  ```yaml
  linear_deadband: 0.05
  angular_deadband: 0.05
  ```

### Wrong Button Mappings

- Use the button/axis discovery scripts above to find correct indices
- Update `config/js_config.yaml` with your controller's mappings

## Lifecycle Management

This node uses ROS2 lifecycle management with the following states:

- **Unconfigured**: Initial state, no resources allocated
- **Inactive**: Configured but not publishing (safe state)
- **Active**: Fully operational, reading joystick and publishing data
- **Finalized**: Cleaned up and shut down

### Lifecycle Commands

```bash
# Configure the node
ros2 lifecycle set /joystick_interface configure

# Activate the node
ros2 lifecycle set /joystick_interface activate

# Deactivate (keeps configuration)
ros2 lifecycle set /joystick_interface deactivate

# Cleanup (returns to unconfigured)
ros2 lifecycle set /joystick_interface cleanup

# Shutdown
ros2 lifecycle set /joystick_interface shutdown
```

## Directory Structure

```
joystick_bot/
├── joystick_bot/                  # Python package directory
│   └── __init__.py               # Package initialization
├── config/                        # Configuration files
│   └── js_config.yaml            # Joystick configuration parameters
├── docs/                          # Documentation files
│   └── images/                   # Images and media
│       ├── controller_layout.png # Controller button reference diagram
│       └── running_example.mp4   # Demo video
├── launch/                        # Launch files
│   └── js.launch.py              # Joystick interface launcher
├── msg/                          # Custom message definitions
│   └── ControllerButtonsState.msg # Button state message
├── scripts/                       # Executable scripts
│   └── js_node.py                # Main joystick interface node
├── CMakeLists.txt                # CMake build configuration
├── package.xml                   # ROS2 package manifest
├── README.md                     # This file
└── README_br.md                  # Portuguese version of README
```

## Custom Message Types

### ControllerButtonsState

```
bool a_button
bool b_button
bool x_button
bool y_button
bool start_button
bool select_button
bool up_button      # D-pad
bool down_button    # D-pad
bool left_button    # D-pad
bool right_button   # D-pad
bool l1_button
bool l2_button
bool r1_button
bool r2_button
```

## Dependencies

### ROS2 Packages
- `rclpy` - Python ROS2 client library
- `rclcpp` - C++ ROS2 client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry message types
- `rosidl_default_generators` - Interface generation

### External Dependencies
- `pygame` - Joystick input handling library
- Linux input subsystem (`/dev/input/js*`)

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

### Development Workflow

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
