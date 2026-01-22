<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_localization - Visual SLAM & Localization System

Visual SLAM and localization package for autonomous robot navigation. This package provides RTAB-Map based visual SLAM using single or dual Intel RealSense cameras, or Lidar for robust robot localization, mapping, and autonomous navigation support.

## Purpose

The `bot_localization` package serves as the core localization system for the BotBrain robot platform. It manages:
- Visual SLAM (Simultaneous Localization and Mapping) using RTAB-Map
- Single/Dual RealSense camera or Lidar integration
- Map building and localization modes
- Transform chain management (map → odom → base_link)
- 2D occupancy grid generation for navigation

## Package Files

### Launch Files

---

#### `launch/localization.launch.py`

Main localization system launcher that orchestrates all SLAM and perception components.

**Description**: Master launch file that brings up the complete vision-based localization system. This file is the entry point for all localization functionality and includes multiple sub-launch files.

**What Gets Launched**:

This launch file includes the following sub-launch files in sequence:

1. **realsense.launch.py**: Camera drivers and depth processing
2. **rtabmap.launch.py**: RTAB-Map SLAM system
3. **rtab_manager.launch.py**: RTAB-Map services management
4. **map_odom.launch.py**: Map-to-odometry transform publisher

**Launch Arguments**:

None - Configuration read from workspace `robot_config.yaml` and `{robot_model}_pkg/config/camera_config.yaml`

**Usage**:
```bash
# Launch complete localization system
ros2 launch bot_localization localization.launch.py

```

---

#### `launch/realsense.launch.py`

Intel RealSense camera drivers and depth-to-laser conversion launcher.

**Description**: Dynamically configures and launches Intel RealSense camera nodes based on camera configuration. Creates camera drivers, static TF publishers, and depth-to-laserscan converters for obstacle detection.

**What Gets Launched**:

The launch file reads `{robot_model}_pkg/config/camera_config.yaml` and launches:

**For each configured camera (front/back)**:
- **RealSense camera node** (LifecycleNode): `realsense2_camera_node`
- **Static TF publisher**: Publishes camera-to-base transform
  - Based on `tf` parameters in camera_config.yaml (x, y, z, roll, pitch, yaw)
- **depth_to_laserscan node**: Converts depth image to 2D laser scan

**Additional nodes**:
- **realsense_compressed_node** (LifecycleNode): Compresses RealSense topics for bandwidth optimization

**Configuration Source**:

- Robot namespace from `robot_config.yaml`
- Camera configuration from `{robot_model}_pkg/config/camera_config.yaml`
  - Camera types (d435i, d455, etc.)
  - Serial numbers for device identification
  - TF transforms (position and orientation)

**Example Configuration**:

```yaml
camera_configuration:
  front:
    type: "d435i"
    serial_number: "123456789"
    parent_frame: "botbrain_base"
    child_frame: "front_camera_link"
    tf: {x: 0.08, y: 0.0175, z: 0.043, roll: 0, pitch: 0, yaw: 0}
  back:
    type: "d435i"
    serial_number: "987654321"
    parent_frame: "botbrain_base"
    child_frame: "back_camera_link"
    tf: {x: -0.08, y: -0.0175, z: 0.043, roll: 0, pitch: 0, yaw: 3.14159}
```

---

#### `launch/map_odom.launch.py`

Map-to-base_link pose publisher for localization monitoring.

**Description**: Launches a lifecycle node that reads the TF transform from map to base_link and publishes it as a PoseStamped message on a topic.

**What Gets Launched**:

- **map_odom_node** (LifecycleNode): `map_odom.py`
  - Reads TF transform from `map` to `base_link` at 10Hz
  - Publishes the transform as a PoseStamped message on `titan/map_odom` topic
  - Useful for monitoring robot position in the map frame

**Parameters**:

| Parameter | Type | Description |
|-----------|------|-------------|
| `prefix` | string | Namespace prefix for TF frames (e.g., "tita/") |

**Topics Published**:

- `map_odom` (PoseStamped): Robot pose in map frame (position and orientation)

**Note**: This node does NOT publish TF transforms. It only publishes pose messages by reading existing TF data from RTAB-Map.

---

#### `launch/rtabmap.launch.py`

Dynamic RTAB-Map launcher that selects appropriate SLAM configuration.

**Description**: Intelligent launcher that reads camera/sensor configuration and includes the correct RTAB-Map launch file based on robot model and available sensors.

**Selection Logic**:

The launch file determines which RTAB-Map configuration to use:

1. **If robot_model == "g1"**:
   - Includes `rtabmap_lidar.launch.py`
   - LiDAR-based SLAM for G1 humanoid robot

2. **If robot has cameras**:
   - **0 cameras**: No RTAB-Map launched
   - **1 camera** (front OR back): Includes `rtabmap_single_camera.launch.py`
   - **2 cameras** (front AND back): Includes `rtabmap_double_camera.launch.py`

**Configuration Source**:

- Reads `robot_config.yaml` for `robot_model` and `robot_name`
- Reads `{robot_model}_pkg/config/camera_config.yaml` to count cameras
- Counts cameras based on `type` field presence in `front` and `back` sections

**Included Launch Files**:

**rtabmap_single_camera.launch.py**:
- RTAB-Map configured for single RGB-D camera
- Direct subscription to camera topics (no sync required)
- Topics: `front_camera/color/image_raw`, `front_camera/aligned_depth_to_color/image_raw`

**rtabmap_double_camera.launch.py**:
- RTAB-Map configured for dual RGB-D cameras
- Includes two `rgbd_sync` nodes for front and back cameras
- RTAB-Map subscribes to synchronized `rgbd_image0` and `rgbd_image1`
- Parameters: `rgbd_cameras: 2`, `subscribe_rgbd: True`

**rtabmap_lidar.launch.py** (G1 robot only):
- RTAB-Map configured for 3D LiDAR SLAM
- Includes `lidar_deskewing` node for motion compensation
- ICP-based registration with point-to-plane matching
- Subscribes to `/{namespace}/pointcloud` topic

**RTAB-Map Common Configuration**:

All RTAB-Map variants share these settings:

**Localization Mode** (not mapping):
- `Mem/IncrementalMemory: False`: Localization only, no new map creation
- `Mem/InitWMWithAllNodes: True`: Load full map into memory
- `database_path`: `{robot_model}_pkg/maps/rtabmap.db`
- `delete_db_on_start: False`: Preserve existing map

**2D SLAM Parameters** (camera-based):
- `Reg/Force3DoF: true`: Constrain to ground plane (x, y, yaw)
- `Grid/3D: false`: Generate 2D occupancy grid
- `Grid/RangeMax: 3.0m`: Maximum obstacle detection range
- `Grid/CellSize: 0.10m`: 10cm grid resolution
- `Grid/MaxGroundHeight: 0.05m`: Ground plane tolerance
- `Grid/MaxObstacleHeight: 0.8m`: Maximum obstacle height

**Frame Configuration**:
- `frame_id`: `{prefix}base_link`
- `map_frame_id`: `{prefix}map`
- `odom_frame_id`: `{prefix}odom`

---

#### `launch/rtab_manager.launch.py`

RTAB-Map lifecycle state manager for mapping/localization mode control.

**Description**: Launches a lifecycle node that provides services to manage RTAB-Map operation modes and database state.

**What Gets Launched**:

- **rtab_manager** (LifecycleNode): `rtab_manager.py`
  - Manages RTAB-Map lifecycle transitions
  - Provides services to reset map, switch modes, save/load databases
  - Monitors RTAB-Map node health

**Parameters**:

| Parameter | Type | Description |
|-----------|------|-------------|
| `robot_model` | string | Robot model name for database path resolution |

**Services Provided**:

Services for controlling RTAB-Map operation (implementation-dependent):
- Reset database
- Switch between mapping and localization modes
- Save/load map databases
- Trigger map optimization

### Scripts

#### `scripts/map_odom.py`
Lifecycle node that manages map-to-odom transform publishing:

**Functionality:**
- Listens to RTAB-Map pose estimates
- Computes map → odom transform from map → base_link
- Publishes transform at 20Hz for smooth navigation
- Handles lifecycle transitions (configure/activate/deactivate)

**Topics:**
- Subscribes: `/{namespace}/rtabmap/localization_pose` (PoseStamped)
- Publishes: `/tf` (TFMessage) - map → odom transform

## Configuration

The package reads from workspace-level `robot_config.yaml`:

```yaml
robot_configuration:
  robot_name: "my_robot"                    # Namespace for all topics
  robot_model: "go2"                        # Robot model
  d435i_serial_number: ""       # Rear camera serial
  d455_serial_number: ""        # Front camera serial
```


## Topics

### Published
- `/{namespace}/rtabmap/grid_map` (OccupancyGrid) - 2D map for navigation
- `/{namespace}/rtabmap/cloud_map` (PointCloud2) - 3D point cloud map
- `/{namespace}/rtabmap/localization_pose` (PoseStamped) - Robot pose in map
- `/{namespace}/rtabmap/mapData` (MapData) - SLAM map data
- `/tf` (TFMessage) - map → odom transform

### Subscribed
- `/{namespace}/d455/color/image_raw` (Image) - Front camera RGB
- `/{namespace}/d455/depth/image_rect_raw` (Image) - Front depth
- `/{namespace}/d435i/color/image_raw` (Image) - Rear camera RGB
- `/{namespace}/d435i/depth/image_rect_raw` (Image) - Rear depth
- `/{namespace}/d435i/imu` (Imu) - IMU data from D435i

## Services

### RTAB-Map Services
- `/{namespace}/rtabmap/reset_session` - Start new mapping session
- `/{namespace}/rtabmap/save_map` - Save current map to database
- `/{namespace}/rtabmap/load_map` - Load map from database
- `/{namespace}/rtabmap/pause` - Pause SLAM processing
- `/{namespace}/rtabmap/resume` - Resume SLAM processing

### Custom Services (via bot_custom_interfaces)
- `/{namespace}/start_mapping` - Start mapping mode (creates new map)
- `/{namespace}/stop_mapping` - Stop mapping and save database

## Mapping Workflow

### Creating a New Map

1. **Start Mapping Session**:
```bash
ros2 launch bot_localization localization.launch.py
ros2 service call /{namespace}/start_mapping bot_custom_interfaces/srv/StartMapping
```

2. **Drive Robot**: Teleoperate robot through environment
   - Cover all areas to be mapped
   - Ensure good lighting and visual features
   - Move slowly for better feature tracking

3. **Save Map**:
```bash
ros2 service call /{namespace}/stop_mapping bot_custom_interfaces/srv/StopMapping
```

4. **Database Saved**: Map database saved to configured path

### Using Existing Map for Localization

1. **Configure Launch File**: Set database path to existing map
2. **Launch Localization**:
```bash
ros2 launch bot_localization localization.launch.py
```
3. **Initial Pose**: Robot may need initial pose hint if far from map origin
4. **Automatic Localization**: RTAB-Map automatically localizes using visual features


## Directory Structure

```
bot_localization/
├── bot_localization/                          # Main package directory
│   ├── bot_localization/                      # Python package
│   │   └── __init__.py                       # Package initialization
│   ├── launch/
│   │   ├── localization.launch.py            # Main launcher (orchestrates all components)
│   │   ├── realsense.launch.py               # Camera drivers and depth processing
│   │   ├── map_odom.launch.py                # Pose publisher node
│   │   ├── rtabmap.launch.py                 # Dynamic RTAB-Map launcher (selector)
│   │   ├── rtabmap_single_camera.launch.py   # Single camera RTAB-Map configuration
│   │   ├── rtabmap_double_camera.launch.py   # Dual camera RTAB-Map configuration
│   │   ├── rtabmap_lidar.launch.py           # LiDAR RTAB-Map configuration (G1)
│   │   └── rtab_manager.launch.py            # RTAB-Map lifecycle manager
│   ├── scripts/
│   │   ├── map_odom.py                       # Map-to-base_link pose publisher
│   │   ├── rtab_manager.py                   # RTAB-Map manager node
│   │   └── compressed_realsense.py           # RealSense compression node
│   ├── CMakeLists.txt                        # Build configuration
│   ├── package.xml                           # Package manifest
│   └── README.md                             # This file
├── bot_localization_interfaces/               # Custom service/message interfaces
│   ├── srv/                                  # Service definitions
│   ├── CMakeLists.txt                        # Interface build config
│   └── package.xml                           # Interface package manifest
└── README.md                                 # Top-level README
```

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
