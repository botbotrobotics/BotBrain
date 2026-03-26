<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# k1_pkg

**Booster Robotics K1 Humanoid Robot Hardware Interface Package**

The `k1_pkg` package provides the ROS 2 hardware abstraction layer for the Booster Robotics K1 humanoid robot. It handles bidirectional communication with the robot, sensor data publishing, command execution, stereo camera streaming, and robot-specific services.


## Package Purpose

This package interfaces with the Booster Robotics K1 robot through ROS 2 topics, enabling:

- **Hardware Communication**: Real-time bidirectional data exchange with the K1 robot via Booster RPC service
- **Sensor Integration**: Publishing odometry, IMU, battery data, and stereo camera feeds
- **Motion Control**: Receiving and executing velocity commands from twist_mux
- **Mode Switching**: Switching between robot operational modes (damping, prepare, walking, soccer)

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

Lifecycle node that reads sensor data from the K1 robot and publishes to ROS 2 topics.

**Executable**: `k1_read.py`

**Description**: Subscribes to Booster Robotics ROS 2 topics, processes robot state data, and publishes standard ROS 2 sensor messages. Provides odometry, IMU, battery status, and stereo camera data.

#### Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position, yaw) in base_link frame |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (orientation, angular velocity, linear acceleration) |
| `/battery` | `sensor_msgs/BatteryState` | Battery voltage, current, and state of charge |
| `/rgb/image` | `sensor_msgs/Image` | Rectified stereo RGB image |
| `/depth/image` | `sensor_msgs/Image` | Stereo depth image |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | Stereo camera calibration parameters |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/odometer_state` | `booster_interface/Odometer` | Booster odometry state (x, y, theta) |
| `/low_state` | `booster_interface/LowState` | Low-level state including IMU (RPY, gyro, acc) |
| `/battery_state` | `booster_interface/BatteryState` | Battery voltage, current, state of charge |
| `/StereoNetNode/rectified_image` | `sensor_msgs/Image` | Stereo rectified RGB image |
| `/StereoNetNode/stereonet_depth` | `sensor_msgs/Image` | Stereo depth image |
| `/StereoNetNode/stereonet_depth/camera_info` | `sensor_msgs/CameraInfo` | Depth camera info |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Topic prefix (namespace) for published data |

---

### lifecycle_robot_write_node

Lifecycle node that receives ROS 2 commands and sends them to the K1 robot.

**Executable**: `k1_write.py`

**Description**: Subscribes to velocity commands from twist_mux and sends movement requests to the K1 robot via the Booster RPC service. Provides a service for mode switching between the robot's operational states.

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `cmd_vel_out` | `geometry_msgs/Twist` | Velocity commands (linear.x, linear.y, angular.z) |

#### Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `change_mode` | `bot_custom_interfaces/srv/Mode` | Switch robot operational mode (damping, prepare, walking, soccer) |

#### RPC Clients

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `booster_rpc_service` | `booster_interface/srv/RpcService` | Booster Robotics RPC interface for movement and mode commands |

#### Mode Map

| Mode Name | API ID | Description |
|-----------|--------|-------------|
| `damping` | 0 | Damping mode — joints go limp (safe stop) |
| `prepare` | 1 | Prepare mode — robot stands ready |
| `walking` | 2 | Walking mode — human-like bipedal gait |
| `soccer` | 4 | Soccer mode — agile soccer gait |

#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Topic prefix (namespace) for subscribed topics |

---

## Launch Files

### robot_interface.launch.py

Main hardware interface launcher that starts all K1 robot nodes.

**Path**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Description**: Launches the lifecycle nodes for complete K1 robot integration.

#### What Gets Launched

1. **robot_read_node**: Sensor data publisher
2. **lifecycle_robot_write_node**: Command executor

#### Launch Arguments

None - Configuration read from workspace [robot_config.yaml](../../../../robot_config.yaml)

#### Configuration Source

```yaml
robot_configuration:
  robot_name: "k1_robot"           # Namespace for all nodes
  network_interface: "eth0"         # Network interface for robot communication
```

#### Usage

```bash
# Launch K1 hardware interface
ros2 launch k1_pkg robot_interface.launch.py

# Verify nodes are running
ros2 node list | grep k1

# Check lifecycle states
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Note**: This launch file is automatically included by `bot_bringup` when `robot_model: "k1"` in `robot_config.yaml`.

## Configuration Files

### nav2_params.yaml

Navigation2 parameters tuned for the K1 humanoid robot dynamics.

**Path**: [config/nav2_params.yaml](config/nav2_params.yaml)

### camera_config.yaml

Camera configuration for the K1 stereo camera system.

**Path**: [config/camera_config.yaml](config/camera_config.yaml)

## Robot Description Files

### URDF Files

Located in [urdf/](urdf/) directory:

- **robot.urdf**: Main K1 robot description including torso, arms, and leg assemblies

### Meshes

Located in [meshes/](meshes/) directory:

Visual and collision meshes in STL format for the full humanoid body:
- `Trunk.STL` / `Trunk_Collision.STL` - Robot torso
- `Head_1.STL`, `Head_2.STL` - Head components
- `Left_Hip_Yaw.STL`, `Left_Hip_Roll.STL`, `Left_Hip_Pitch.STL` - Left hip joints
- `Left_Shank.STL`, `Left_Foot.STL` - Left lower leg and foot
- `Left_Arm_1-4.STL` - Left arm links
- Equivalent right-side meshes for all above

These files are used by `bot_description` for visualization in RViz and by Nav2 for collision detection.

## Transforms (TF)

### TF Broadcasters

The robot_read_node broadcasts transforms based on odometry:

| Parent Frame | Child Frame | Source | Description |
|--------------|-------------|--------|-------------|
| `odom` | `base_link` | Odometry integration | 2D pose from `/odometer_state` |

## Integration with BotBrain System

### Automatic Loading

This package is automatically loaded by `bot_bringup` when configured:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "k1"  # Triggers k1_pkg loading
```

The bringup system:
1. Reads `robot_model: "k1"`
2. Constructs package name: `k1_pkg`
3. Includes `k1_pkg/launch/robot_interface.launch.py`
4. Loads description from `k1_pkg/urdf/robot.urdf`


## Usage

### Standalone Testing

Test K1 interface without full system:

```bash
# Source workspace
source install/setup.bash

# Launch K1 interface only
ros2 launch k1_pkg robot_interface.launch.py

# In another terminal, check nodes
ros2 node list

# Expected output:
# /robot_name/robot_read_node
# /robot_name/lifecycle_robot_write_node
```

### Change Robot Mode

```bash
# Switch to walking mode
ros2 service call /{namespace}/change_mode bot_custom_interfaces/srv/Mode "{mode: 'walking'}"

# Switch to damping (safe stop)
ros2 service call /{namespace}/change_mode bot_custom_interfaces/srv/Mode "{mode: 'damping'}"
```

## Directory Structure

```
k1_pkg/
├── launch/
│   └── robot_interface.launch.py     # Main hardware interface launcher
│
├── scripts/
│   ├── k1_read.py                    # Sensor data publisher node
│   └── k1_write.py                   # Command executor node
│
├── config/
│   ├── nav2_params.yaml              # Navigation parameters for K1
│   └── camera_config.yaml            # Camera configuration
│
├── urdf/
│   └── robot.urdf                    # Main robot description
│
├── meshes/
│   ├── Trunk.STL                     # Robot torso mesh
│   ├── Head_1.STL, Head_2.STL        # Head meshes
│   ├── Left_Hip_*.STL                # Left hip joint meshes
│   ├── Left_Shank.STL                # Left lower leg mesh
│   ├── Left_Foot.STL                 # Left foot mesh
│   ├── Left_Arm_*.STL                # Left arm meshes
│   └── [Right-side equivalents]      # Mirror meshes for right side
│
├── maps/
│   └── [environment maps]            # Pre-built maps for K1
│
├── k1_pkg/
│   └── tools/
│
├── k1_setup.bash                     # Environment setup script
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package manifest
└── README.md                         # This file
```


---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
