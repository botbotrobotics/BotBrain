<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# **bot_navigation – Autonomous Navigation Stack**

Autonomous navigation package built on ROS 2 Navigation2 (Nav2). This package provides path planning, obstacle avoidance, and goal-directed autonomous navigation for the BotBrain robot platform.

## **Purpose**

The `bot_navigation` package enables autonomous robot navigation by integrating the Nav2 stack with the BotBrain system. It manages:
- Global path planning
- Local trajectory planning and obstacle avoidance
- Behavior coordination (recovery behaviors, waypoint following)
- Velocity command generation for autonomous motion
- Integration with localization and mapping systems
- Runtime utilities for interacting with Nav2 actions

## **Package Files**

### **Launch Files**

### **launch/nav2.launch.py**

Main Nav2 launcher that starts all componets:

**Nodes Launched:**

1. **controller_server** – Executes planned paths and generates velocity commands
2. **smoother_server** – Smooths planned paths
3. **planner_server** – Computes global paths
4. **behavior_server** – Handles recovery behaviors (backup, spin, wait)
5. **bt_navigator** – Behavior Tree–based navigation coordinator
6. **waypoint_follower** – Executes waypoint navigation

---

### **launch/nav_utils.launch.py**

Launches auxiliary navigation utility nodes.

**Nodes Launched:**

- **nav2_utils** – Lifecycle node providing runtime services for Nav2 interaction

---

### **launch/navigation.launch.py**

High-level launcher that composes the navigation stack.

**Includes:**

- nav_utils.launch.py
- nav2.launch.py

This is the recommended entry point for starting navigation.

**Usage:**

```
ros2 launch bot_navigation navigation.launch.py
```

### **Navigation Utilities**

### **scripts/nav2_utils.py**

Lifecycle node that exposes a service interface for interacting with Nav2 actions.

**Features:**

- Provides the /cancel_nav2_goal service (std_srvs/Trigger)
- Cancels active Nav2 goals for:
    - NavigateToPose
    - FollowWaypoints
- Designed to run alongside Nav2 using a multi-threaded executor
- Namespaced automatically using robot_name from robot_config.yaml

**Use case examples:**

- Emergency stop or external cancellation of navigation goals
- Integration with higher-level behavior or safety systems

## **Robot-Specific Configuration**

Each robot package provides tuned navigation parameters:

- **go2_pkg/config/nav2_params.yaml** – Unitree Go2
- **tita_pkg/config/nav2_params.yaml** – Tita quadruped
- **g1_pkg/config/nav2_params.yaml** – G1 humanoid

**Parameters include:**

- Robot footprint and collision geometry
- Kinematic limits (velocity, acceleration)
- Controller tuning
- Costmap layer configuration

## **Integration with Robot System**

### **Required Components**

Navigation requires the following BotBrain packages:

1. **Localization** (bot_localization) – Provides map frame and robot pose
2. **Description** (bot_description) – Provides robot model and TF tree
3. **Twist Mux** (bot_bringup) – Multiplexes navigation commands with other motion sources

## **Directory Structure**

```
bot_navigation/
├── bot_navigation/
│   └── __init__.py
├── launch/
│   ├── nav2.launch.py              # Main Nav2 launcher
│   ├── nav_utils.launch.py         # Nav2 utilities launcher
│   └── navigation.launch.py        # High-level navigation launcher
├── scripts/
│   └── nav2_utils.py               # Nav2 utility lifecycle node
├── CMakeLists.txt
├── package.xml
└── README.md
```
---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
