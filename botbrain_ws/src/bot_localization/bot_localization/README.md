# bot_localization

The localization and SLAM package for the R1 robot platform. This package provides visual SLAM capabilities using RTAB-Map with dual RealSense cameras for robust robot localization and mapping.

## Description

The `bot_localization` package serves as the core localization system for the R1 robot platform, providing visual SLAM (Simultaneous Localization and Mapping) capabilities using RTAB-Map. It integrates dual Intel RealSense cameras (D435i and D455) to provide robust localization and mapping for autonomous navigation.

## Directory Structure

```
bot_localization/
├── bot_localization/                # Python package directory
│   └── __init__.py                 # Package initialization
├── launch/                         # Launch files
│   ├── localization.launch.py     # Main localization launcher
│   ├── rtabmap.launch.py          # RTAB-Map SLAM configuration
│   ├── realsense.launch.py        # RealSense camera setup
│   └── map_odom.launch.py         # Map-odometry transform publisher
├── scripts/                        # Executable scripts
│   └── map_odom.py                # Map-odometry transform node
├── CMakeLists.txt                 # CMake build configuration
├── package.xml                    # ROS2 package manifest
└── README.md                      # This file
```

## Files Explanation

### Launch Files

#### `launch/localization.launch.py`
The main localization launch file that orchestrates all localization components:

- **RealSense Cameras**: Launches dual camera setup (D435i and D455)
- **RTAB-Map SLAM**: Launches RTAB-Map in localization mode
- **Map-Odometry**: Launches map-odometry transform publisher
- **Complete System**: Provides full localization pipeline

#### `launch/rtabmap.launch.py`
RTAB-Map SLAM configuration and processing:

- **Dual Camera Sync**: Synchronizes RGB-D data from both cameras
- **SLAM Processing**: RTAB-Map localization and mapping
- **Point Cloud Generation**: Converts depth data to point clouds
- **Database Management**: Handles map database loading and saving

**Key Features:**
- Dual camera RGB-D synchronization
- RTAB-Map localization mode
- Point cloud generation for both cameras
- Configurable SLAM parameters

#### `launch/realsense.launch.py`
RealSense camera setup and configuration:

- **Dual Camera Setup**: Configures D435i and D455 cameras
- **Lifecycle Management**: Proper camera initialization sequence
- **Static Transforms**: Publishes camera-to-robot transforms
- **Filter Configuration**: Spatial and temporal filtering

**Camera Configuration:**
- D435i: Rear-facing camera with IMU
- D455: Front-facing camera with extended range
- Synchronized RGB-D streams
- Optimized depth filtering

#### `launch/map_odom.launch.py`
Map-odometry transform publisher:

- **Transform Publishing**: Publishes map-to-odom transform
- **Lifecycle Management**: Robust startup and shutdown
- **TF Integration**: Integrates with ROS2 transform system

### Scripts

#### `scripts/map_odom.py`
Map-odometry transform publisher that:

- **Transform Listening**: Listens to map-to-base_link transform
- **Pose Publishing**: Publishes robot pose in map frame
- **Lifecycle Management**: Proper resource management
- **TF Integration**: Integrates with ROS2 transform system

**Key Features:**
- Real-time pose estimation
- Transform chain management
- Lifecycle-managed operation
- Robust error handling

## Configuration

The package reads configuration from the workspace-level `robot_config.yaml`:

```yaml
robot_configuration:
  robot_name: "my_robot"           # Robot namespace
  robot_model: "go2"               # Robot model (go2/tita)
  d435i_serial_number: "216322070465" # D435i camera serial
  d455_serial_number: "341522302139"  # D455 camera serial
```

### Camera Configuration

- **D435i**: Rear-facing camera with IMU for odometry
- **D455**: Front-facing camera with extended range
- **Synchronization**: RGB-D data synchronized between cameras
- **Filtering**: Spatial and temporal depth filtering enabled

### RTAB-Map Parameters

- **Localization Mode**: Uses existing map for localization
- **Dual Camera**: Processes two RGB-D streams
- **Grid Mapping**: 2D occupancy grid generation
- **Database**: Portable database path configuration

## System Architecture

### Localization Pipeline

1. **Camera Data**: Dual RealSense cameras provide RGB-D data
2. **Synchronization**: RGB-D data synchronized between cameras
3. **SLAM Processing**: RTAB-Map processes visual features
4. **Map Loading**: Existing map loaded for localization
5. **Pose Estimation**: Robot pose estimated in map frame
6. **Transform Publishing**: Map-odom transform published

### Coordinate Frames

- **map**: Global map frame
- **odom**: Odometry frame
- **base_link**: Robot base frame
- **d455_camera_link**: Front camera frame
- **d435i_camera_link**: Rear camera frame

### Data Flow

```
RealSense Cameras → RGB-D Sync → RTAB-Map → Pose Estimation → Transform Publishing
```

## Contributing

### Development Workflow

1. **Create Feature Branch**: Create a feature branch from `dev`
   ```bash
   git checkout dev
   git pull origin dev
   git checkout -b feature/my_new_localization_feature
   ```

2. **Make Changes**: Implement your changes following ROS2 best practices
   - Use proper lifecycle management
   - Follow ROS2 naming conventions
   - Add appropriate parameter declarations
   - Ensure camera compatibility

3. **Test Changes**: Test thoroughly in the Docker environment
   ```bash
   # Build the package
   docker compose up builder_base

   # Test the changes
   docker compose up -d localization
   ros2 launch bot_localization localization.launch.py
   ```

4. **Update Documentation**: Update this README if adding new functionality

5. **Submit Pull Request**: Create a pull request to the `dev` branch

---

**Note**: This package requires Intel RealSense cameras and sufficient computational resources for real-time SLAM processing. Ensure cameras are properly connected and calibrated before deployment.
