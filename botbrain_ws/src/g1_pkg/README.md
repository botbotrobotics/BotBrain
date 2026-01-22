<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# g1_pkg

**Unitree G1 Humanoid Robot Hardware Interface Package**

The `g1_pkg` package provides the ROS 2 hardware abstraction layer for the Unitree G1 humanoid robot. It delivers real-time telemetry, locomotion command execution, upper-body pose control, joystick integration, Livox MID360 streaming, and helper tools for system bringup.

## Package Purpose

This package connects the G1 to the ROS 2 ecosystem, enabling:

- **Hardware Communication**: native Unitree DDS interface for command/state exchange
- **Sensor Integration**: publishes battery, IMU, odometry, joint states, and TF
- **Motion Control**: consumes `cmd_vel_out`, changes Finite State Machine (FSM) modes (the robot’s internal Unitree locomotion state machine), and handles emergency stops
- **Upper Body Control**: saves and replays arm poses stored in `arm_poses.txt`
- **Controller Integration**: maps joystick button combos to FSM transitions
- **LiDAR Pipeline**: Livox MID360 driver plus PointCloud→LaserScan conversion

## Nodes

All nodes are **lifecycle nodes** for predictable configuration, activation, and teardown.

### Lifecycle Management

#### Common Lifecycle States

| State | Description |
|-------|-------------|
| **Unconfigured** | Freshly created node with no resources |
| **Configured** | Parameters loaded, publishers/subscribers/services created |
| **Active** | Full processing; topics/services are live |
| **Deactivated** | Resources kept but processing paused |
| **Finalized** | All resources cleaned before termination |

#### Standard Lifecycle Transitions

| Transition | Description |
|------------|-------------|
| `configure` | Declare parameters and initialize Unitree drivers |
| `activate` | Connect publishers/subscribers/services and start processing |
| `deactivate` | Stop publishing/commanding while keeping resources |
| `cleanup` | Destroy resources and close connections |
| `shutdown` | Immediate finalization with cleanup logic |

#### Managing Lifecycle States

```bash
# Check current state
ros2 lifecycle get /{namespace}/robot_read_node

# Standard sequence
ros2 lifecycle set /{namespace}/robot_read_node configure
ros2 lifecycle set /{namespace}/robot_read_node activate

# Pause and cleanup
ros2 lifecycle set /{namespace}/robot_read_node deactivate
ros2 lifecycle set /{namespace}/robot_read_node cleanup
```

**Note**: The `bot_state_machine` package automatically manages lifecycle transitions for all nodes during system startup and shutdown.

---

### robot_read_node

Lifecycle node that subscribes to Unitree topics (`/lf/bmsstate`, `/lf/lowstate`, `/lf/odommodestate`) and republishes ROS-standard messages.

**Executable**: `g1_read.py`

**Description**: Converts raw telemetry into `sensor_msgs`/`nav_msgs`, generates `JointState` for all 23 actuators, and broadcasts the `odom → base_link` transform (20 Hz stream).

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 20 Hz | Battery voltage/current/SOC |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 20 Hz | 23 joints (legs + upper body) |
| `/{namespace}/odom` | `nav_msgs/Odometry` | 20 Hz | Pose and twist in `base_link` |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 20 Hz | Quaternion, gyro, and acceleration |
| `/{namespace}/imu_temp` | `std_msgs/Float32` | 20 Hz | IMU temperature |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lf/bmsstate` | `unitree_hg/BmsState` | Battery pack information |
| `/lf/lowstate` | `unitree_hg/LowState` | Per-actuator telemetry |
| `/lf/odommodestate` | `unitree_go/SportModeState` | IMU, odometry, velocities |

#### Services

None

#### Actions

None

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Namespace prefix appended to frames and topics |

---

### robot_write_node

Lifecycle node (C++) that sends locomotion commands to the G1, controls the FSM, applies arm poses, and manages emergency stops.

**Executable**: `g1_write_node` (wraps `g1_write.cpp` + `g1_driver`)

**Description**: Creates a `G1Driver` instance (official DDS), receives velocity commands, exposes services for mode control/emergency handling/pose management, and publishes the list of stored poses.

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/pose/names` | `bot_custom_interfaces/msg/Names` | 0.5 Hz | Current pose names sourced from `arm_poses.txt` |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Velocity commands (internally clamped to 0.6 m/s and 1.0 rad/s) |

#### Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Set FSM (`zero_torque`, `damp`, `preparation`, `run`, `squat`, `start`) with confirmation |
| `/{namespace}/current_mode` | `bot_custom_interfaces/srv/CurrentMode` | Get current FSM state |
| `/{namespace}/emergency_stop` | `std_srvs/srv/SetBool` | Full emergency sequence (stop → squat → damping) |
| `/{namespace}/arm_cmd` | `bot_custom_interfaces/srv/ArmCmd` | Save/load/apply/release/delete arm poses |

#### Actions

None

#### Parameters

Values defined in [`config/g1_params.yaml`](config/g1_params.yaml):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot.keep_move` | bool | `false` | Keep continuous gait after commands |
| `robot.speed_mode` | int | `0` | Speed profile (`0`=1.0 m/s … `3`=3.0 m/s) |

> During `on_deactivate()` the node automatically issues `stop_move()` for safety.

---

### controller_commands_node

Lifecycle node (Python) that reads joystick events and translates them into FSM mode commands.

**Executable**: `g1_controller_commands.py`

**Description**: Subscribes to `button_state`, tracks button history, and fires mode services (`mode`, `current_mode`) for combos such as `L2+B`, `R2+A`, and long-press `L2+B` for `run → damp`.

#### Publishers

None (acts through service clients)

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/button_state` | `joystick_bot/ControllerButtonsState` | Raw joystick button states |

#### Services

None (uses service clients for `mode` and `current_mode`)

#### Actions

None

#### Parameters

None

---

### Livox MID360 Pipeline

Two additional components are launched alongside the core nodes:

1. **livox_lidar_publisher**  
   - **Executable**: `livox_ros_driver2_node`  
   - **Config**: [`config/MID360_config.json`](config/MID360_config.json)  
   - **Output**: `/{namespace}/pointcloud` (remapped in `pc2ls`)  
   - **Parameters**: `xfer_format`, `publish_freq`, `frame_id = {prefix}mid360_link`, etc.

2. **pointcloud_to_laserscan_node**  
   - **Executable**: `pointcloud_to_laserscan_node`  
   - **Config**: [`config/pointcloud_to_laserscan_params.yaml`](config/pointcloud_to_laserscan_params.yaml)  
    - **Remappings**: `cloud_in → pointcloud`, `scan → scan`  
    - **Output**: `/{namespace}/scan` (2D laser scan generated from the MID360 cloud)

## Launch Files

### robot_interface.launch.py

**Path**: [`launch/robot_interface.launch.py`](launch/robot_interface.launch.py)

**Description**: Reads the workspace-level `robot_config.yaml` to obtain `robot_name`, computes the namespace prefix, and launches:

1. `robot_write_node` (C++)  
2. `robot_read_node`  
3. `controller_commands_node`  
4. `livox_lidar_publisher`  
5. `pointcloud_to_laserscan_node`

#### Launch Arguments

None (prefix derived from the global `robot_config.yaml`).

#### Configuration Source

```yaml
robot_configuration:
  robot_name: "g1_robot"         # Namespace applied to every node
  network_interface: "eno1"      # Network interface for robot communication
```

## Configuration Files

| File | Description |
|------|-------------|
| [`config/g1_params.yaml`](config/g1_params.yaml) | Parameters for `robot_write_node` (keep_move, speed_mode) |
| [`config/nav2_params.yaml`](config/nav2_params.yaml) | Nav2 tuning for humanoid kinematics (velocity/accel limits, footprint) |
| [`config/camera_config.yaml`](config/camera_config.yaml) | Intel RealSense calibration (frames, offsets, serials) |
| [`config/MID360_config.json`](config/MID360_config.json) | Livox LiDAR networking and extrinsics |
| [`config/pointcloud_to_laserscan_params.yaml`](config/pointcloud_to_laserscan_params.yaml) | Height filters, min/max range, angular step for LaserScan |
| [`config/arm_poses.txt`](config/arm_poses.txt) | Arm pose library consumed by the `arm_cmd` service |

### nav2_params.yaml

Navigation2 parameters tuned specifically for the Unitree G1 humanoid kinematics and dynamics.

**Path**: [`config/nav2_params.yaml`](config/nav2_params.yaml)

**Description**: Robot-specific tuning for the Nav2 stack including velocity limits, acceleration constraints, footprint, and controller settings tailored for humanoid locomotion. This file is typically loaded by `bot_navigation` when the `robot_model` is set to `g1`.

### camera_config.yaml

Camera calibration parameters for the onboard Intel RealSense sensors.

**Path**: [`config/camera_config.yaml`](config/camera_config.yaml)

**Description**: Defines camera types, serial numbers, mounting frames, and TF transforms so SLAM/perception nodes can consume the feeds with the correct extrinsics.

#### Configuration Structure

```yaml
camera_configuration:
  front:
    type: "d435i"
    serial_number: "344422070967"
    parent_frame: "torso_link"
    child_frame: "front_camera_link"
    tf:
      x: 0.0576235
      y: 0.01753
      z: 0.41987
      roll: 0
      pitch: 0.8307767239493009
      yaw: 0

  back:
    type: ""
    serial_number: "339222071455"
    parent_frame: "base_link"
    child_frame: "back_camera_link"
    tf:
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
```

#### Parameter Descriptions

**Camera Identification**
- `type`: Camera model identifier (e.g., `d435i`, `d455`, `t265`) used to select the correct launch parameters.
- `serial_number`: Unique device serial; required when multiple identical cameras are plugged in (discover via `rs-enumerate-devices`).

**Transform Frames**
- `parent_frame`: Frame where the camera is mounted (`torso_link`, `base_link`, etc.) and must exist in the URDF/XACRO.
- `child_frame`: Camera optical frame (convention: `{position}_camera_link`) published by the driver or a static TF.

**Transform (TF) Parameters**
- `x`, `y`, `z`: Position offsets (meters) from the parent frame. Positive `x` forward, `y` left, `z` up.
- `roll`, `pitch`, `yaw`: Orientation offsets (radians). Example: the front camera has ~0.83 rad pitch to look slightly downward.

## Robot Description Files

- **URDF**: [`urdf/`](urdf/) holds ready-to-use descriptions for RViz/Gazebo.
- **XACRO**: [`xacro/`](xacro/) contains reusable building blocks (`robot.xacro`, `leg.xacro`, `const.xacro`, `materials.xacro`).
- **Meshes**: [`meshes/`](meshes/) stores STL/DAE models for trunk, limbs, and accessories (visual + collision).

## Transforms (TF)

### TF Broadcasters

| Parent Frame | Child Frame | Source | Rate |
|--------------|-------------|--------|------|
| `/{namespace}/odom` | `/{namespace}/base_link` | `robot_read_node` (odometry integration) | 20 Hz |

The remaining kinematic tree is published by `robot_state_publisher`, which consumes the joint states emitted by `robot_read_node`.

### Auxiliary Frames

- **Cameras**: `front_camera_link`, `back_camera_link` according to `config/camera_config.yaml`.  
- **LiDAR**: `mid360_link` provided by `livox_lidar_publisher` (prefix applied via launch arguments).  
- **Arm Poses**: Any additional static transforms defined in the URDF/XACRO are maintained through the description package. 

These frames enable perception and SLAM pipelines to align camera/LiDAR data with the humanoid base.

## Arm Pose Workflow

```bash
# Save current pose
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 0, name: 'pose_wave'}"

# Monitor pose name topic (20 Hz list updates)
ros2 topic hz /{namespace}/pose/names

# Reapply pose
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 1, name: 'pose_wave'}"

# Release arm joints (return control to onboard controller)
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 2, name: ''}"

# Delete a stored pose
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 3, name: 'pose_wave'}"
```

## Integration with BotBrain System

### Automatic Loading

This package is automatically launched by `bot_bringup` when `robot_model: "g1"` is selected inside the workspace-level `robot_config.yaml`:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "g1"   # Triggers g1_pkg loading
```

The bringup pipeline then:

1. Reads `robot_model: "g1"` from the config file.
2. Resolves the package name `g1_pkg`.
3. Includes `g1_pkg/launch/robot_interface.launch.py` into the system launch.
4. Loads the humanoid description from `g1_pkg/xacro/robot.xacro`.

Once `bot_bringup` is running with `robot_model: "g1"`, all lifecycle nodes described above start automatically and `bot_state_machine` handles their transitions (configure → activate) during startup and shutdown.

## Usage

### Standalone Testing

```bash
# Source the workspace
source install/setup.bash

# Launch the full interface (outside bot_bringup)
ros2 launch g1_pkg robot_interface.launch.py

# Check running nodes
ros2 node list | grep g1_robot

# Expected output:
# /robot_name/robot_read_node
# /robot_name/robot_write_node
# /robot_name/controller_commands_node
# /robot_name/robot_video_stream

# Manually change lifecycle (example)
ros2 lifecycle set /g1_robot/robot_write_node activate
```

## Directory Structure

```
g1_pkg/
├── launch/
│   ├── robot_interface.launch.py     # Primary bringup (hardware + sensors)
│   ├── livox_MID360.launch.py        # Livox driver with dedicated config
│   └── pc2ls.launch.py               # PointCloud → LaserScan conversion
├── scripts/
│   ├── g1_read.py                    # Telemetry lifecycle node
│   └── g1_controller_commands.py     # Joystick lifecycle node
├── src/
│   ├── g1_write.cpp                  # Lifecycle writer implementation
│   ├── g1_write_node.cpp             # Main C++
│   └── g1_driver/                    # Unitree DDS wrapper for the G1
├── include/
│   └── g1_write.hpp                  # G1Write class declaration
├── config/
│   ├── g1_params.yaml                # Write-node parameters (speed_mode, keep_move)
│   ├── nav2_params.yaml              # Navigation parameters tuned for G1
│   ├── camera_config.yaml            # RealSense calibration (frames, offsets, serials)
│   ├── MID360_config.json            # Livox MID360 networking/extrinsics
│   ├── pointcloud_to_laserscan_params.yaml  # PointCloud→LaserScan filters
│   └── arm_poses.txt                 # Saved upper-body poses
├── urdf/                             # Ready-to-use URDF models
├── xacro/                            # Reusable Xacro blocks
├── meshes/                           # DAE/STL visual & collision meshes
├── maps/                             # Pre-built navigation maps (if any)
├── g1_pkg/__init__.py                # Python helper package
├── g1_setup.bash                     # Environment setup script
├── CMakeLists.txt / package.xml      # ROS 2 metadata
└── README.md                         # This file
```

---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
