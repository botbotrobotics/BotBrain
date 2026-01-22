<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_bringup

**Core system orchestration package for BotBrain robotics framework**

The `bot_bringup` package serves as the central launch coordinator for the core robot system. It manages robot description loading, hardware interface initialization, velocity command multiplexing with priority-based arbitration, and communication interfaces.


## Package Purpose

This package is responsible for:

- **System Orchestration**: Launching all core subsystems in correct dependency order
- **Robot Interface Management**: Dynamically loading robot-specific hardware packages
- **Velocity Command Arbitration**: Priority-based multiplexing of commands from multiple sources
- **Safety Baseline**: Ensuring robot stops when no active commands
- **Communication Bridges**: Web interface and remote control connectivity

## Nodes

### zero_vel_publisher

Lifecycle node that continuously publishes zero velocity commands as a safety baseline.

**Executable**: `zero_vel_publisher.py`

**Description**: Provides lowest-priority velocity command to ensure the robot stops when all other command sources timeout or become inactive. This acts as a safety mechanism.

#### Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `cmd_vel_zero` | `geometry_msgs/Twist` | Zero velocity command (all fields = 0.0) |

#### Lifecycle States

| State | Description |
|-------|-------------|
| **Unconfigured** | Initial state, no resources allocated |
| **Configured** | Publisher created but inactive |
| **Active** | Timer running, publishing zero velocity at 10Hz |
| **Deactivated** | Timer stopped, publisher exists but inactive |
| **Finalized** | All resources cleaned up |

**Lifecycle Transitions**:
- `configure`: Creates publisher for `cmd_vel_zero`
- `activate`: Starts 10Hz timer for publishing zero velocity
- `deactivate`: Stops timer, publisher remains
- `cleanup`: Destroys publisher and timer
- `shutdown`: Emergency cleanup of all resources

### twist_mux

Third-party node from `twist_mux` package that arbitrates velocity commands based on priority.

**Package**: `twist_mux`

**Description**: Multiplexes velocity commands from multiple sources (joystick, web, navigation, AI) and forwards the highest-priority active command to the robot. Includes dead-man switch safety lock.

#### Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `cmd_vel_out` | `geometry_msgs/Twist` | Final arbitrated velocity command sent to robot |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostic information about active inputs |

#### Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `cmd_vel_zero` | `geometry_msgs/Twist` | Zero velocity baseline (priority: 1) |
| `cmd_vel_rosa` | `geometry_msgs/Twist` | AI assistant commands (priority: 5) |
| `cmd_vel_nav` | `geometry_msgs/Twist` | Navigation stack commands (priority: 10) |
| `cmd_vel_nipple` | `geometry_msgs/Twist` | Web interface joystick (priority: 99) |
| `cmd_vel_joy` | `geometry_msgs/Twist` | Physical joystick commands (priority: 100) |
| `dead_man_switch` | `std_msgs/Bool` | Safety lock, must be true for motion (priority: 200) |


#### Parameters

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `twist_watchdog_timeout` | double | `0.5` | Global timeout (sec) for all twist inputs |

**Input Topics Configuration** (from [config/twist_mux.yaml](config/twist_mux.yaml)):

| Input Name | Topic | Priority | Timeout (s) | Description |
|------------|-------|----------|-------------|-------------|
| `zero_velocity` | `cmd_vel_zero` | 1 | 0.1 | Safety baseline - always active |
| `rosa` | `cmd_vel_rosa` | 5 | 0.2 | AI assistant generated commands |
| `navigation` | `cmd_vel_nav` | 10 | 0.2 | Autonomous navigation from Nav2 |
| `nipple` | `cmd_vel_nipple` | 99 | 0.2 | Browser-based virtual joystick |
| `joystick` | `cmd_vel_joy` | 100 | 0.2 | Physical game controller (highest) |

**Lock Topics Configuration**:

| Lock Name | Topic | Priority | Timeout (s) | Description |
|-----------|-------|----------|-------------|-------------|
| `dead_man` | `dead_man_switch` | 200 | 0.0 | Safety switch - blocks all motion when false |

**Priority Behavior**:
- Higher priority values override lower priority commands
- Commands timeout if not received within specified duration
- Dead-man switch (priority 200) blocks ALL motion when inactive/false
- Zero velocity (priority 1) ensures robot stops when all commands timeout

## Launch Files

### bringup.launch.py

Main system launcher that orchestrates all core robot components.

**Path**: [launch/bringup.launch.py](launch/bringup.launch.py)

**Description**: Launches the complete robot system including description, hardware interface, motion control, teleoperation, and web communication.

#### What Gets Launched

1. **Robot Description** (`bot_description`):
   - Loads robot URDF/XACRO model from robot-specific package
   - Starts `robot_state_publisher` for TF publishing
   - Configured based on `robot_model` in `robot_config.yaml`

2. **Robot Hardware Interface** (`{robot_model}_pkg`):
   - Dynamically includes robot-specific launch file
   - Example: `go2_pkg/launch/robot_interface.launch.py` for Go2
   - Launches hardware read/write nodes for robot communication

3. **Twist Multiplexer** (via `twist_mux.launch.py`):
   - Priority-based velocity command arbitration
   - Zero velocity safety baseline
   - Dead-man switch integration

4. **Joystick Interface** (`joystick_bot`):
   - Game controller teleoperation
   - Publishes to `cmd_vel_joy`
   - Manages dead-man switch

5. **ROSBridge WebSocket Server** (`rosbridge_server`):
   - WebSocket bridge for browser-based control
   - Enables web UI and remote monitoring
   - Port: 9090 (default)

#### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| None - Configured via `robot_config.yaml` | - | - | All settings read from workspace configuration file |

#### Configuration Source

The launch file reads [robot_config.yaml](../../../../robot_config.yaml) from workspace root:

### twist_mux.launch.py

Launches the motion control subsystem with velocity arbitration.

**Path**: [launch/twist_mux.launch.py](launch/twist_mux.launch.py)

**Description**: Starts the twist_mux node for command arbitration and zero_vel_publisher for safety baseline.

#### What Gets Launched

1. **twist_mux node**:
   - Multiplexes velocity commands from multiple sources
   - Configured via `config/twist_mux.yaml`
   - Namespace-aware based on `robot_config.yaml`

2. **zero_vel_publisher (Lifecycle Node)**:
   - Publishes zero velocity at 10Hz
   - Auto-configured and activated via lifecycle transitions
   - Provides safety baseline

#### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| None - Configured via files | - | - | Uses `robot_config.yaml` and `twist_mux.yaml` |

## Configuration Files

### twist_mux.yaml

Velocity command priority and timeout configuration.

**Path**: [config/twist_mux.yaml](config/twist_mux.yaml)

**Description**: Defines priority levels, timeouts, and safety locks for velocity command arbitration.


#### Priority Levels Explained

| Priority | Source | Use Case | Override Behavior |
|----------|--------|----------|-------------------|
| 1 | Zero Velocity | Safety baseline | Overridden by any active command |
| 5 | AI Assistant (ROSA) | Autonomous tasks | Overridden by navigation and manual control |
| 10 | Navigation (Nav2) | Autonomous navigation | Overridden by manual control |
| 99 | Web Joystick | Remote manual control | Overridden only by physical joystick |
| 100 | Physical Joystick | Direct manual control | Highest command priority |
| 200 | Dead-Man Switch | Safety lock | Blocks ALL commands when false |

#### Timeout Behavior

- If a command source stops publishing for longer than its timeout, it becomes inactive
- The next highest-priority active source takes over
- Zero velocity (priority 1) has shortest timeout (0.1s) to ensure quick activation
- Dead-man switch has 0.0 timeout - immediately blocks motion when false

## Topics

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Variable | Final arbitrated velocity command sent to robot hardware |
| `/{namespace}/cmd_vel_zero` | `geometry_msgs/Twist` | 10 Hz | Zero velocity safety baseline |
| `/{namespace}/twist_mux/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Diagnostic status of twist_mux inputs |

### Subscribed Topics

All velocity command inputs (subscribed by twist_mux):

| Topic | Message Type | Priority | Timeout | Description |
|-------|--------------|----------|---------|-------------|
| `/{namespace}/cmd_vel_joy` | `geometry_msgs/Twist` | 100 | 0.2s | Physical joystick commands |
| `/{namespace}/cmd_vel_nipple` | `geometry_msgs/Twist` | 99 | 0.2s | Web interface commands |
| `/{namespace}/cmd_vel_nav` | `geometry_msgs/Twist` | 10 | 0.2s | Navigation stack commands |
| `/{namespace}/cmd_vel_rosa` | `geometry_msgs/Twist` | 5 | 0.2s | AI assistant commands |
| `/{namespace}/cmd_vel_zero` | `geometry_msgs/Twist` | 1 | 0.1s | Zero velocity baseline |
| `/{namespace}/dead_man_switch` | `std_msgs/Bool` | 200 (lock) | 0.0s | Safety switch for motion enable |

## Services

None - This package provides launch orchestration and does not expose services directly. Individual launched nodes (from other packages) may provide their own services.

## Transforms (TF)

This package does not directly provide TF transforms. The `robot_state_publisher` launched from `bot_description` publishes the robot's kinematic tree.

### TF Listeners

None directly, but launched nodes may listen to transforms from `bot_description`

### TF Broadcasters

None directly - TF broadcasting handled by `robot_state_publisher` from `bot_description` package

## Integration with Other Packages

### Dependencies

| Package | Purpose |
|---------|---------|
| `bot_description` | Robot URDF/XACRO models and robot_state_publisher |
| `{robot_model}_pkg` | Robot-specific hardware interface (e.g., go2_pkg, tita_pkg) |
| `joystick_bot` | Controller interface and dead-man switch |
| `rosbridge_server` | WebSocket bridge for web interface |
| `twist_mux` | Velocity command multiplexing (third-party) |

### Dynamic Robot Package Loading

The bringup system dynamically loads robot packages based on configuration:

**Configuration**:
```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "go2"  # Options: "go2", "tita", "g1", "custom_robot"
```

**Loading Logic**:
1. Read `robot_model` from config
2. Construct package name: `{robot_model}_pkg`
3. Include launch file: `{robot_model}_pkg/launch/robot_interface.launch.py`

**Example**:
- `robot_model: "go2"` → Launches `go2_pkg/launch/robot_interface.launch.py`
- `robot_model: "tita"` → Launches `tita_pkg/launch/robot_interface.launch.py`

### Expected Topics from Robot Packages

Robot-specific packages must publish these topics for full integration:

| Topic | Message Type | Required | Description |
|-------|--------------|----------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | Yes | Robot odometry for navigation |
| `/{namespace}/imu` | `sensor_msgs/Imu` | Yes | IMU data for localization |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | Yes | Joint states for robot_state_publisher |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | Yes | Battery status and charge level |

And subscribe to:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel` | `geometry_msgs/Twist` | Final velocity commands from twist_mux |


## Directory Structure

```
bot_bringup/
├── bot_bringup/
│   └── __init__.py                # Python package marker
│
├── launch/
│   ├── bringup.launch.py          # Main system launcher
│   └── twist_mux.launch.py        # Motion control subsystem launcher
│
├── config/
│   └── twist_mux.yaml             # Velocity command priorities and timeouts
│
├── scripts/
│   └── zero_vel_publisher.py      # Safety zero velocity publisher node
│
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # Package manifest
└── README.md                      # This file
```

---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
