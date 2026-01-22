<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# tita_pkg

**Tita Robot Hardware Interface Package**

The `tita_pkg` package provides the ROS 2 hardware abstraction layer for Tita robots. It handles bidirectional communication with the robot, sensor data publishing, command execution, and robot-specific services through topic bridging.

<!-- INSERT ROBOT IMAGE HERE -->
<!-- ![Tita Robot](docs/tita_robot.jpg) -->

## Package Purpose

This package bridges the Tita robot's internal ROS 2 topics with the BotBrain framework, enabling:

- **Topic Bridging**: Communication between the Tita internal namespace and BotBrain system
- **Sensor Integration**: Publishing odometry, IMU, joint states, battery, and perception data
- **Motion Control**: Receiving and executing velocity commands from twist_mux
- **Robot Services**: Mode switching via service calls
- **Controller Integration**: Translating joystick inputs to robot-specific commands

## Nodes

All nodes in this package are **lifecycle nodes**, providing managed state transitions for robust startup, shutdown, and error recovery.

### Lifecycle Management

#### Common Lifecycle States

| State | Description |
|-------|-------------|
| **Unconfigured** | Initial state after node creation, no resources allocated |
| **Configured** | Resources created (publishers, subscribers, services), ready to activate |
| **Active** | Node fully operational, processing data and executing functions |
| **Deactivated** | Node paused, resources maintained but processing stopped |
| **Finalized** | All resources cleaned up, node ready for termination |

#### Standard Lifecycle Transitions

| Transition | Description |
|------------|-------------|
| `configure` | Allocate resources (create publishers, subscribers, services) |
| `activate` | Start processing (begin publishing, accepting commands) |
| `deactivate` | Pause processing (stop publishing but maintain resources) |
| `cleanup` | Destroy resources (close connections, free memory) |
| `shutdown` | Emergency cleanup and immediate termination |

#### Managing Lifecycle States

```bash
# Check current state
ros2 lifecycle get /{namespace}/robot_read_node

# Transition through states
ros2 lifecycle set /{namespace}/robot_read_node configure
ros2 lifecycle set /{namespace}/robot_read_node activate

# Deactivate (pause)
ros2 lifecycle set /{namespace}/robot_read_node deactivate

# Cleanup (release resources)
ros2 lifecycle set /{namespace}/robot_read_node cleanup
```

**Note**: The `bot_state_machine` package automatically manages lifecycle transitions for all nodes during system startup and shutdown.

---

### robot_read_node

Lifecycle node that subscribes to Tita internal topics and republishes to BotBrain namespace.

**Executable**: `tita_read.py`

**Description**: Acts as a topic bridge, subscribing to Tita's internal topics (under `tita_namespace`) and republishing data under the robot's BotBrain namespace for system-wide integration.

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | 50 Hz | Robot odometry (position, velocity) from Tita chassis controller |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU data (orientation, angular velocity, linear acceleration) |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 50 Hz | Joint positions and velocities for all leg joints |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 1 Hz | Aggregated battery state from left/right packs |
| `/{namespace}/pointcloud` | `sensor_msgs/PointCloud2` | 10 Hz | 3D point cloud from Tita perception system |
| `/{namespace}/camera/image` | `sensor_msgs/Image` | 30 Hz | Camera image feed from Tita cameras |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{tita_namespace}/chassis/odometry` | `nav_msgs/Odometry` | Tita internal odometry topic |
| `/{tita_namespace}/imu_sensor_broadcaster` | `sensor_msgs/Imu` | Tita internal IMU topic |
| `/{tita_namespace}/joint_states` | `sensor_msgs/JointState` | Tita internal joint states |
| `/{tita_namespace}/system/battery/left` | `sensor_msgs/BatteryState` | Tita left battery state |
| `/{tita_namespace}/system/battery/right` | `sensor_msgs/BatteryState` | Tita right battery state |
| `/{tita_namespace}/perception/camera/point_cloud` | `sensor_msgs/PointCloud2` | Tita perception point cloud |
| `/{tita_namespace}/perception/camera/image/raw` | `sensor_msgs/Image` | Tita camera images |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefix for frame ids (e.g., `odom`, `base_link`) |
| `tita_namespace` | string | `""` | Tita robot's internal namespace for topic bridging |

---

### robot_write_node

Lifecycle node that receives BotBrain commands and forwards them to Tita internal topics.

**Executable**: `tita_write.py`

**Description**: Subscribes to BotBrain velocity commands and republishes to Tita's internal command topics. Provides a service for robot mode control and configures `use_sdk` on the Tita command node.

#### Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{tita_namespace}/command/user/command` | `tita_locomotion_interfaces/msg/LocomotionCmd` | Locomotion commands forwarded to Tita controller |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Velocity commands from twist_mux |

#### Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Switch robot operational mode (stand up/down) |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefix for subscribed topics |
| `tita_namespace` | string | `""` | Tita robot's internal namespace |

**Note**:
- On `activate`, this node attempts to set `use_sdk=true` via `/{tita_namespace}/active_command_node/set_parameters`.

---

### controller_commands_node

Lifecycle node that translates game controller inputs to Tita-specific commands.

**Executable**: `tita_controller_commands.py`

**Description**: Receives joystick button events and calls the `mode` service to stand up or stand down based on button combos.

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/button_state` | `joystick_bot/msg/ControllerButtonsState` | Processed button states |
| `/{tita_namespace}/locomotion/body/fsm_mode` | `std_msgs/String` | Current robot FSM mode |

---

## Launch Files

### robot_interface.launch.py

Main hardware interface launcher that starts all Tita robot nodes.

**Path**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Description**: Launches all three lifecycle nodes for complete Tita robot integration. Lifecycle handlers exist but are currently commented out.

#### What Gets Launched

1. **robot_read_node**: Topic bridge for sensor data
2. **robot_write_node**: Command forwarder and service provider
3. **controller_commands_node**: Joystick command translator

#### Launch Arguments

None - Configuration read from workspace [robot_config.yaml](../../../../robot_config.yaml)

#### Configuration Source

```yaml
robot_configuration:
  robot_name: "my_tita"            # BotBrain namespace
  robot_model: "tita"              # Must be "tita"
  tita_namespace: "tita3036731"    # Tita internal namespace (unique per robot)
```

#### Usage

```bash
# Launch Tita hardware interface
ros2 launch tita_pkg robot_interface.launch.py

# Verify nodes are running
ros2 node list | grep tita

# Check lifecycle states
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Note**: This launch file is automatically included by `bot_bringup` when `robot_model: "tita"` in `robot_config.yaml`.

## Configuration Files

### nav2_params.yaml

Navigation2 parameters tuned specifically for Tita robot kinematics and dynamics.

**Path**: [config/nav2_params.yaml](config/nav2_params.yaml)

**Description**: Robot-specific tuning for Nav2 stack including velocity limits, acceleration limits, footprint, and controller parameters optimized for Tita's locomotion.

This file is automatically loaded by `bot_navigation` when Tita is selected.

### camera_config.yaml

Camera and perception configuration for Tita sensors.

**Path**: [config/camera_config.yaml](config/camera_config.yaml)

**Description**: Configuration for Intel RealSense cameras mounted on the Tita robot. Defines camera types, serial numbers, mounting positions, and TF transforms for single or multi-camera SLAM and perception.

#### Configuration Structure

```yaml
camera_configuration:
  front:
    type: "d435i"                      # Intel RealSense camera model
    serial_number: ""                  # Unique camera identifier
    parent_frame: "botbrain_base"      # Parent TF frame (robot base)
    child_frame: "front_camera_link"   # Camera TF frame name
    tf:
      x: 0.08                          # Forward offset (meters)
      y: 0.0175                        # Lateral offset (meters)
      z: 0.043                         # Vertical offset (meters)
      roll: 0                          # Rotation around X-axis (radians)
      pitch: 0                         # Rotation around Y-axis (radians)
      yaw: 0                           # Rotation around Z-axis (radians)

  back:
    type: "d435i"                      # Intel RealSense D435i camera
    serial_number: ""                  # Unique camera identifier
    parent_frame: "botbrain_base"      # Parent TF frame (robot base)
    child_frame: "back_camera_link"    # Camera TF frame name
    tf:
      x: -0.08                         # Backward offset (meters)
      y: -0.0175                       # Lateral offset (meters)
      z: 0.043                         # Vertical offset (meters)
      roll: 0                          # No roll rotation
      pitch: 0                         # No pitch rotation
      yaw: 3.14159                     # 180 deg rotation (facing backward)
```

#### Parameter Descriptions

**Camera Identification**:
- `type`: Camera model identifier (e.g., "d435i", "d455", "t265")
  - Used to select appropriate ROS 2 launch parameters
- `serial_number`: Unique device serial number
  - Required when multiple cameras of same type are connected
  - Ensures correct camera is launched with correct configuration
  - Find with: `rs-enumerate-devices`

**Transform Frames**:
- `parent_frame`: Reference frame for camera mounting
  - Typically `"botbrain_base"` or `"base_link"`
  - Must match frame defined in robot URDF/XACRO
- `child_frame`: Camera's optical frame name
  - Convention: `"{position}_camera_link"` (e.g., "front_camera_link")
  - Published by camera drivers or static transform publishers

**Transform (TF) Parameters**:
- `x`, `y`, `z`: 3D position offset from parent frame (meters)
  - **x**: Positive forward, negative backward
  - **y**: Positive left, negative right (robot's perspective)
  - **z**: Positive up, negative down
  - Example: Front camera at `x: 0.08` is 8cm in front of base

- `roll`, `pitch`, `yaw`: Rotation angles in radians
  - **roll**: Rotation around X-axis (forward axis)
  - **pitch**: Rotation around Y-axis (lateral axis)
  - **yaw**: Rotation around Z-axis (vertical axis)
  - Example: Back camera with `yaw: 3.14159` (pi radians = 180 deg) faces backward

## Robot Description Files

### XACRO Files

Located in [xacro/](xacro/) directory:

- **robot.xacro**: Main Tita robot description including body and leg assemblies
- **[component].xacro**: Additional XACRO components for legs, sensors, etc.

### Meshes

Located in [meshes/](meshes/) directory:

Visual and collision meshes for Tita robot visualization in RViz and collision detection in Nav2.

## Transforms (TF)

### TF Broadcasters

The robot_read_node broadcasts transforms based on odometry:

| Parent Frame | Child Frame | Source | Rate |
|--------------|-------------|--------|------|
| `odom` | `base_link` | Tita odometry | 50 Hz |

Complete robot kinematic tree is published by `robot_state_publisher` (from `bot_description`) using joint states from this package.

## Integration with BotBrain System

### Automatic Loading

This package is automatically loaded by `bot_bringup` when configured:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "tita"  # Triggers tita_pkg loading
```

The bringup system:
1. Reads `robot_model: "tita"`
2. Constructs package name: `tita_pkg`
3. Includes `tita_pkg/launch/robot_interface.launch.py`
4. Loads description from `tita_pkg/xacro/robot.xacro`

### Topic Bridging Architecture

The package implements a bidirectional topic bridge:

**Sensor Data Flow** (Tita → BotBrain):
```
Tita Internal Topic → robot_read_node → BotBrain Topic
/{tita_namespace}/chassis/odometry → [bridge] → /{robot_name}/odom
/{tita_namespace}/imu_sensor_broadcaster → [bridge] → /{robot_name}/imu/data
/{tita_namespace}/perception/camera/point_cloud → [bridge] → /{robot_name}/pointcloud
/{tita_namespace}/perception/camera/image/raw → [bridge] → /{robot_name}/camera/image
```

**Command Flow** (BotBrain → Tita):
```
BotBrain Topic → robot_write_node → Tita Internal Topic
/{robot_name}/cmd_vel_out → [bridge] → /{tita_namespace}/command/user/command
```

### Required Dependencies

**System Dependencies**:
- Network connectivity to Tita robot (Ethernet or WiFi)
- Proper ROS 2 domain configuration

**ROS 2 Package Dependencies**:
- `rclcpp` / `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `std_msgs`
- `bot_custom_interfaces`
- `tita_locomotion_interfaces`
- `joystick_bot`
- `rcl_interfaces`
- `tf2_ros`

### Topic Integration

**Publishes to BotBrain System**:
- Sensor topics → Used by `bot_localization`, `bot_navigation`
- Joint states → Used by `bot_description` (robot_state_publisher)

**Subscribes from BotBrain System**:
- `cmd_vel_out` from `bot_bringup` (twist_mux output)
- `button_state` from `joystick_bot`

## Usage

### Standalone Testing

Test Tita interface without full system:

```bash
# Source workspace
source install/setup.bash

# Launch Tita interface only
ros2 launch tita_pkg robot_interface.launch.py

# In another terminal, check nodes
ros2 node list

# Expected output:
# /robot_name/robot_read_node
# /robot_name/robot_write_node
# /robot_name/controller_commands_node
```

## Directory Structure

```
tita_pkg/
├── launch/
│   └── robot_interface.launch.py     # Main hardware interface launcher
│
├── scripts/
│   ├── tita_read.py                  # Topic bridge for sensor data
│   ├── tita_write.py                 # Command forwarder node
│   └── tita_controller_commands.py   # Controller translator node
│
├── config/
│   ├── nav2_params.yaml              # Navigation parameters for Tita
│   └── camera_config.yaml            # Camera/perception configuration
│
├── xacro/
│   └── [XACRO files]                 # Robot description files
│
├── meshes/
│   └── [mesh files]                  # 3D visualization meshes
│
├── maps/
│   └── [map files]                   # Pre-built maps for Tita
│
├── tita_pkg/
│   └── tools/                        # Utility modules
│
├── tita_setup.bash                   # Environment setup script
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package manifest
└── README.md                         # This file
```

---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
