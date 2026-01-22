<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_description - Robot Model & Transform Publishing

Robot description package providing URDF/XACRO models and transform publishing for the BotBrain robot system. This package manages the robot's kinematic model, visualization meshes, and publishes the robot state to the TF tree for navigation and control.

## Purpose

The `bot_description` package provides the robot's mechanical and visual description to the ROS2 ecosystem. It manages:
- Robot URDF/XACRO model definitions
- 3D visualization meshes for RViz
- Robot state publishing (joint states to TF transforms)
- Static transforms between robot components
- Integration of robot hardware model with BotBrain interface module

## Package Files

### Launch Files

#### `launch/description.launch.py`
Main description launcher that orchestrates the complete robot model:

**What it launches:**
1. **robot_description.launch.py**: Robot hardware model (e.g., Unitree Go2 quadruped)
2. **botbrain_description.launch.py**: BotBrain interface module model (cameras, sensors)
3. **static_transform_publisher**: Links robot base to BotBrain module (`interface_link` to `botbrain_base`)

**Configuration:**
- Reads `robot_config.yaml` from workspace root
- Uses `robot_name` field for namespace and TF frame prefixes

**Usage:**
```bash
ros2 launch bot_description description.launch.py
```

This is typically launched automatically by `bot_bringup` during system startup.

#### `launch/robot_description.launch.py`
Publishes the robot hardware URDF model and robot state:

**Nodes:**
- **robot_state_publisher**: Converts URDF to TF transforms
- Publishes robot joint states to `/robot_description` topic
- Provides forward kinematics for the robot base

**Model Source:**
- Loads XACRO file for the specific robot platform
- Supports different robot models (Go2, Tita, etc.)

#### `launch/botbrain_description.launch.py`
Publishes the BotBrain module URDF model:

**Nodes:**
- **robot_state_publisher**: Publishes BotBrain module transforms
- Includes camera mounts, sensor positions, and interface links
- Provides mounting geometry for perception sensors

**Integration:**
- BotBrain module is linked to robot hardware via static transform
- Transform published from robot `interface_link` to `botbrain_base`

### XACRO Files

Located in `xacro/` directory:

#### `xacro/botbrain.xacro`
XACRO model defining the BotBrain interface module:

**Includes:**
- Interface mounting link
- Camera mount positions (D435i, D455)
- Sensor mounting points
- Module geometry and collision meshes

**Parameters:**
- Configurable mounting offsets
- Camera positioning and orientation
- Sensor frame definitions

### Meshes

Located in `meshes/` directory:

**Contents:**
- 3D visualization meshes for RViz
- STL or DAE format mesh files
- Robot and module visual geometry
- Collision geometry for motion planning

**Usage:**
- Referenced by URDF/XACRO files
- Loaded by RViz for visualization
- Used by collision detection systems

## Topics

### Published
- `/{namespace}/robot_description` (std_msgs/String) - Complete robot URDF
- `/tf_static` (tf2_msgs/TFMessage) - Static transforms
- `/tf` (tf2_msgs/TFMessage) - Dynamic transforms (from joint states)

### Subscribed
- `/{namespace}/joint_states` (sensor_msgs/JointState) - Robot joint positions (if available)

## Directory Structure

```
bot_description/
├── launch/
│   ├── description.launch.py           # Main launcher
│   ├── robot_description.launch.py     # Robot hardware model
│   └── botbrain_description.launch.py  # BotBrain module model
├── xacro/
│   └── botbrain.xacro                  # BotBrain XACRO model
├── meshes/
│   └── [mesh files]                    # 3D visualization meshes
├── CMakeLists.txt                      # Build configuration
└── package.xml                         # Package manifest
```

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
