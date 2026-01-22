<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# go2_pkg

**Unitree Go2 Quadruped Robot Hardware Interface Package**

The `go2_pkg` package provides the ROS 2 hardware abstraction layer for Unitree Go2 quadruped robots. It handles bidirectional communication with the robot, sensor data publishing, command execution, video streaming, and robot-specific services.


## Package Purpose

This package interfaces with the Unitree Go2 robot through ROS 2 topics, enabling:

- **Hardware Communication**: Real-time bidirectional data exchange with Go2 robot
- **Sensor Integration**: Publishing odometry, IMU, joint states, and battery data
- **Motion Control**: Receiving and executing velocity commands from twist_mux
- **Video Streaming**: Publishing camera feeds 
- **Robot Services**: Mode switching, gait control, pose adjustment, and safety features
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

Lifecycle node that reads sensor data from the Go2 robot and publishes to ROS 2 topics.

**Executable**: `go2_read.py`

**Description**: Subscribes to Unitree ROS 2 topics, processes robot state data, and publishes standard ROS 2 sensor messages. Provides odometry, IMU, joint states, battery status, and LiDAR data.

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | 50 Hz | Robot odometry (position, velocity) in base_link frame |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU data (orientation, angular velocity, linear acceleration) |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 50 Hz | Joint positions and velocities for all 12 leg joints |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 1 Hz | Battery voltage, current, percentage, and health |
| `/{namespace}/imu_temp` | `std_msgs/Float32` | 1 Hz | IMU temperature in Celsius |
| `/{namespace}/pointcloud` | `sensor_msgs/PointCloud2` | 10 Hz | LiDAR point cloud data (if equipped) |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Unitree ROS 2 sport mode state data |
| `/lf/lowstate` | `unitree_go/LowState` | Unitree ROS 2 low-level state data |
| `/utlidar/robot_pose` | `geometry_msgs/PoseStamped` | Robot pose from Unitree LiDAR |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Point cloud from Unitree LiDAR |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Topic prefix (namespace) for published data |

---

### lifecycle_robot_write_node

Lifecycle node that receives ROS 2 commands and sends them to the Go2 robot.

**Executable**: `go2_write.py`

**Description**: Subscribes to velocity commands from twist_mux and sends them to the Go2 robot via Unitree ROS 2 API topics. Provides comprehensive services for mode switching, gait control, pose adjustment, and safety features. Monitors robot state and publishes status information.

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/robot_status` | `bot_custom_interfaces/msg/RobotStatus` | 2 Hz | Robot operational status (mode, gait, emergency state) |
| `/api/sport/request` | `unitree_api/msg/Request` | Event | Unitree sport mode API requests |
| `/api/robot_state/request` | `unitree_api/msg/Request` | Event | Unitree robot state API requests |
| `/api/vui/request` | `unitree_api/msg/Request` | Event | Unitree voice UI API requests |
| `/api/obstacles_avoid/request` | `unitree_api/msg/Request` | Event | Unitree obstacle avoidance API requests |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Velocity commands from twist_mux (linear.x, linear.y, angular.z) |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | Unitree sport mode state feedback |
| `/api/sport/response` | `unitree_api/msg/Response` | Unitree sport API responses |
| `/api/robot_state/response` | `unitree_api/msg/Response` | Unitree robot state responses |
| `/api/vui/response` | `unitree_api/msg/Response` | Unitree voice UI responses |

#### Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Switch robot operational mode (stand, walk, lie down) |
| `/{namespace}/switch_gait` | `bot_custom_interfaces/srv/SwitchGait` | Change gait pattern (trot, walk, run) |
| `/{namespace}/body_height` | `bot_custom_interfaces/srv/BodyHeight` | Adjust body height (-0.1 to +0.1 meters) |
| `/{namespace}/foot_raise_height` | `bot_custom_interfaces/srv/FootRaiseHeight` | Set foot lift height during walking (0.06-0.1 m) |
| `/{namespace}/speed_level` | `bot_custom_interfaces/srv/SpeedLevel` | Set speed level (1-5, where 5 is fastest) |
| `/{namespace}/pose` | `bot_custom_interfaces/srv/Pose` | Set body pose (roll, pitch, yaw angles) |
| `/{namespace}/euler` | `bot_custom_interfaces/srv/Euler` | Set orientation using Euler angles |
| `/{namespace}/continuous_gait` | `bot_custom_interfaces/srv/ContinuousGait` | Enable continuous gait transitions |
| `/{namespace}/switch_joystick` | `bot_custom_interfaces/srv/SwitchJoystick` | Switch between joystick control modes |
| `/{namespace}/current_mode` | `bot_custom_interfaces/srv/CurrentMode` | Query current robot operational mode |
| `/{namespace}/emergency_stop` | `std_srvs/srv/SetBool` | Trigger emergency stop (halts all motion) |
| `/{namespace}/light_control` | `bot_custom_interfaces/srv/LightControl` | Control robot LED lights |
| `/{namespace}/obstacle_avoidance` | `bot_custom_interfaces/srv/ObstacleAvoidance` | Enable/disable onboard obstacle avoidance |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Topic prefix (namespace) for subscribed topics |

**Note**:
- On `activate`, this node automatically performs initialization sequence:
  1. Sends `stand_down` command to lay the robot on the ground
  2. Disables **MCF (Motion Control Framework)** mode and switches to **Sport Mode**
  3. Sends `stand_up` command to put robot in standing position
- On `deactivate`, this node automatically sends a stop command (zero velocity) to the robot for safety.

---

### controller_commands_node

Lifecycle node that translates game controller inputs to robot-specific commands.

**Executable**: `go2_controller_commands.py`

**Description**: Receives joystick button and axis events, translates them to Go2-specific commands such as mode switches, gait changes, and pose adjustments. Provides button mapping for special robot functions.

#### Publishers

Various command topics based on button mappings (calls services on robot_write_node)

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/joy` | `sensor_msgs/Joy` | Raw joystick input from joystick node |
| `/{namespace}/controller_state` | `bot_custom_interfaces/msg/ControllerButtonsState` | Processed button states |

### robot_video_stream

Lifecycle node that captures and publishes video from Go2 onboard cameras.

**Executable**: `go2_video_stream.py`

**Description**: Connects to Go2 camera streams, decodes H.264/H.265 video, and publishes as ROS 2 Image messages. Supports multiple cameras and provides camera calibration info.

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Raw camera image (decoded from H.264/H.265) |
| `/{namespace}/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Camera calibration parameters |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Topic prefix for published images |
| `network_interface` | string | `"eth0"` | Network interface for video streaming |

---

## Launch Files

### robot_interface.launch.py

Main hardware interface launcher that starts all Go2 robot nodes.

**Path**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Description**: Launches all four lifecycle nodes for complete Go2 robot integration. Automatically configures and activates nodes (lifecycle handlers are commented out but available).

#### What Gets Launched

1. **robot_read_node**: Sensor data publisher
2. **lifecycle_robot_write_node**: Command executor and robot status monitor
3. **controller_commands_node**: Joystick command translator
4. **robot_video_stream**: Camera stream publisher

#### Launch Arguments

None - Configuration read from workspace [robot_config.yaml](../../../../robot_config.yaml)

#### Configuration Source

```yaml
robot_configuration:
  robot_name: "go2_robot"          # Namespace for all nodes
  network_interface: "eth0"         # Network interface for robot communication
```

#### Usage

```bash
# Launch Go2 hardware interface
ros2 launch go2_pkg robot_interface.launch.py

# Verify nodes are running
ros2 node list | grep go2

# Check lifecycle states
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Note**: This launch file is automatically included by `bot_bringup` when `robot_model: "go2"` in `robot_config.yaml`.

## Configuration Files

### nav2_params.yaml

Navigation2 parameters tuned specifically for Unitree Go2 quadruped kinematics and dynamics.

**Path**: [config/nav2_params.yaml](config/nav2_params.yaml)

**Description**: Robot-specific tuning for Nav2 stack including velocity limits, acceleration limits, footprint, and controller parameters optimized for quadruped locomotion.

This file is automatically loaded by `bot_navigation` when Go2 is selected.

### camera_config.yaml

Camera calibration parameters for Go2 onboard cameras.

**Path**: [config/camera_config.yaml](config/camera_config.yaml)

**Description**: Configuration for Intel RealSense cameras mounted on the Go2 robot. Defines camera types, serial numbers, mounting positions, and TF transforms for single or multi-camera SLAM and perception.

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
      yaw: 3.14159                     # 180° rotation (facing backward)
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
  - Example: Back camera with `yaw: 3.14159` (π radians = 180°) faces backward

## Robot Description Files

### XACRO Files

Located in [xacro/](xacro/) directory:

- **robot.xacro**: Main Go2 robot description including body and leg assemblies
- **leg.xacro**: Leg kinematic chain (hip, thigh, calf joints)
- **const.xacro**: Constants and measurements (link lengths, masses, inertias)
- **materials.xacro**: Visual material definitions for RViz

### Meshes

Located in [meshes/](meshes/) directory:

Visual and collision meshes in DAE/STL format:
- `trunk.dae` - Robot body
- `hip.dae` - Hip joint
- `thigh.dae` / `thigh_mirror.dae` - Thigh links
- `calf.dae` / `calf_mirror.dae` - Calf links
- `foot.dae` - Foot contact links

These files are used by `bot_description` for visualization in RViz and by Nav2 for collision detection.

## Transforms (TF)

### TF Broadcasters

The robot_read_node broadcasts transforms based on odometry:

| Parent Frame | Child Frame | Source | Rate |
|--------------|-------------|--------|------|
| `odom` | `base_link` | Odometry integration | 50 Hz |

Complete robot kinematic tree is published by `robot_state_publisher` (from `bot_description`) using joint states from this package.

## Integration with BotBrain System

### Automatic Loading

This package is automatically loaded by `bot_bringup` when configured:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "go2"  # Triggers go2_pkg loading
```

The bringup system:
1. Reads `robot_model: "go2"`
2. Constructs package name: `go2_pkg`
3. Includes `go2_pkg/launch/robot_interface.launch.py`
4. Loads description from `go2_pkg/xacro/robot.xacro`


## Usage

### Standalone Testing

Test Go2 interface without full system:

```bash
# Source workspace
source install/setup.bash

# Launch Go2 interface only
ros2 launch go2_pkg robot_interface.launch.py

# In another terminal, check nodes
ros2 node list

# Expected output:
# /robot_name/robot_read_node
# /robot_name/lifecycle_robot_write_node
# /robot_name/controller_commands_node
# /robot_name/robot_video_stream
```

## Directory Structure

```
go2_pkg/
├── launch/
│   └── robot_interface.launch.py     # Main hardware interface launcher
│
├── scripts/
│   ├── go2_read.py                   # Sensor data publisher node
│   ├── go2_write.py                  # Command executor node
│   ├── go2_controller_commands.py    # Controller translator node
│   └── go2_video_stream.py           # Video publisher node
│
├── config/
│   ├── nav2_params.yaml              # Navigation parameters for Go2
│   └── camera_config.yaml            # Camera calibration
│
├── xacro/
│   ├── robot.xacro                   # Main robot description
│   ├── leg.xacro                     # Leg kinematic chain
│   ├── const.xacro                   # Constants and measurements
│   └── materials.xacro               # Visual materials
│
├── meshes/
│   ├── trunk.dae                     # Body mesh
│   ├── hip.dae                       # Hip joint meshes
│   ├── thigh.dae, thigh_mirror.dae   # Thigh meshes
│   ├── calf.dae, calf_mirror.dae     # Calf meshes
│   └── foot.dae                      # Foot mesh
│
├── maps/
│   └── [environment maps]            # Pre-built maps for Go2
│
├── go2_pkg/
│   └── tools/
│       └── go2.py                    # Utility functions
│
├── go2_setup.bash                    # Environment setup script
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package manifest
└── README.md                         # This file
```


---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
