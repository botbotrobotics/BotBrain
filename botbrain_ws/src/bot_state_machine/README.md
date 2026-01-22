<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_state_machine

**Lifecycle coordination layer for ROS 2 node in the BotBot stack.**

`bot_state_machine` keeps the platform’s lifecycle nodes synchronized: it parses the node graph, applies configure/activate/deactivate transitions in the proper order, exposes a high-level command service, monitors transition events, and automatically falls back between autonomous and teleop modes when navigation nodes misbehave.

## Package Purpose

This package glues the rest of the system together by:

- **Coordinated Bringup/Shutdown** – Drives every node through the ROS 2 lifecycle in dependency order (CORE → NAVIGATION → accessories, etc.).
- **Dependency Management** – Recursively validates dependencies/dependents and ensures PAYLOAD class nodes are never toggled automatically.
- **Event Monitoring** – Subscribes to lifecycle transition events and reacts to unexpected UNCONFIGURED/ACTIVE changes.
- **Command Interface** – Offers a `StateMachine` service to trigger activation, deactivation, or restart for any managed node.
- **Status Aggregation** – Publishes `bot_custom_interfaces/msg/StatusArray` with the lifecycle state, priority, and health of every node.
- **Mode Arbitration** – Maintains BRINGUP / AUTONOMOUS / TELEOP / BRINGDOWN / RESTARTING states, switching modes based on navigation health.

---

## Core Components

### `LifecycleManager`

- Loads JSON node descriptions (per class) and converts them into `NodeProfile` entries.
- Creates lifecycle clients (`/get_state`, `/change_state`) for every managed node.
- Publishes `StatusArray` and exposes `bot_custom_interfaces/srv/StateMachine`.
- Routes lifecycle transition events to the controller and clears stale events.
- Periodically invokes `update_callback`, `publish_callback`, and `event_callback`.

### `StateController`

Finite-state machine: 

- States: `BRINGUP`, `AUTONOMOUS`, `TELEOP`, `BRINGDOWN`, `IDLE`, `ERROR`, `RESTARTING`.
- Performs dependency checks, activation/deactivation, and restart logic.
- Maintains event queues, protected node sets, and command processing.
- Handles navigation fallbacks: if any NAVIGATION node fails activation, all NAV nodes are deactivated and the controller transitions to TELEOP.

### `GraphNode`

- Lightweight helper that parses the class-specific JSON manifests.
- Converts entries into `NodeProfile` objects and builds dependency/dependent lists.
- Sorts nodes by class and priority to guarantee deterministic bringup/bringdown ordering.

| File | Description |
|------|-------------|
| `core.json` | Core services (Docker, web socket, etc.) |
| `navigation.json` | Nav2 stack, localization, planners |
| `camera.json` | RealSense / perception nodes |
| `audio.json` | Voice pipeline |
| `accessories.json` | Miscellaneous utilities |
| `ia_stack.json` | AI/ML components |
| `payload.json` | Third-party payloads that the controller only monitors |

Each JSON entry contains the node name, display name, class, startup order, description, and dependency list.

---

## Lifecycle Model

### Controller States

| State | Description |
|-------|-------------|
| `BRINGUP` | Sequentially configure and activate nodes based on class/order. |
| `AUTONOMOUS` | Navigation nodes are active; events may trigger restarts. |
| `TELEOP` | Navigation failed; system remains operational minus autonomous stack. |
| `BRINGDOWN` | Reverse-order deactivation & cleanup. |
| `IDLE` | All nodes inactive, waiting for next bringup. |
| `ERROR` | Bringdown encountered an unrecoverable failure. |
| `RESTARTING` | Controller is forcing a bringdown followed by bringup. |

---

## Launching

### `launch/state_machine.launch.py`

| Input | Description |
|-------|-------------|
| `robot_config.yaml` (workspace root) | Provides `robot_configuration.robot_name` used as the namespace prefix. |
| `config/` directory | Passed through the `nodes_json_path` parameter. |

The launch file:

1. Reads `robot_config.yaml` to determine the namespace (e.g., `g1_robot/`).
2. Starts the `state_machine_node` lifecycle node.
3. After the process appears, emits a `configure` transition.
4. When the node reaches `inactive`, automatically emits `activate`.

```bash
# From the workspace root
source install/setup.bash
ros2 launch bot_state_machine state_machine.launch.py
```

---

## Monitoring & Control

### Lifecycle Queries

```bash
# List lifecycle states for a managed node
ros2 lifecycle get /<ns>/<node_name>

# Manually request a transition (advanced usage)
ros2 lifecycle set /<ns>/<node_name> deactivate
```

### State Machine Service

```bash
# Restart a navigation node
ros2 service call /<ns>/state_machine bot_custom_interfaces/srv/StateMachine \
"{node: 'nav2_bt_navigator', command: 3}"
```

### Status Topic

`/<ns>/state_machine/status` publishes `bot_custom_interfaces/msg/StatusArray` containing the lifecycle state, priority, class, and diagnostic message for each node. Use `ros2 topic echo` or plug it into dashboards.

---

## Configuration Workflow

1. **Declare Nodes** – Update the JSON file that matches the node’s class (e.g., `navigation.json`).
2. **Set Dependencies** – Add `dependencies` (nodes that must be active before this node) and `dependents_of` (nodes that rely on this node for teardown ordering).
3. **Assign Priority & Order** – `priority` impacts status reporting and UI ordering; `order` controls startup order inside each class.
4. **Deploy** – Rebuild the workspace or redeploy containers; no code changes required.

The controller automatically reloads definitions on configure.

---

## Directory Structure

```
bot_state_machine/
├── include/
│   ├── lifecycle_manager.hpp    # Lifecycle node orchestrator
│   ├── state_controller.hpp     # FSM controller declaration
│   ├── graph_node.hpp           # Helpers for parsing JSON node graphs
│   └── types.hpp                # Shared enums, structs, and timeouts
├── src/
│   ├── state_controller.cpp     # Bringup/bringdown logic + event manager
│   ├── lifecycle_manager.cpp    # LifecycleNode implementation
│   └── state_machine_node.cpp   # Main executable
├── config/                      # JSON node graph grouped by class
├── launch/
│   └── state_machine.launch.py  # Lifecycle launch + auto configure/activate
├── CMakeLists.txt / package.xml # ROS 2 build metadata
└── README.md                    # This document
```

---

## Usage Examples

```bash
# Start the state machine under a specific namespace (from robot_config.yaml)
ros2 launch bot_state_machine state_machine.launch.py

# Monitor status output
ros2 topic echo /g1_robot/state_machine/status

# Force a full restart (through the service API)
ros2 service call /g1_robot/state_machine bot_custom_interfaces/srv/StateMachine \
"{node: 'state_machine', command: 3}"
```

---

<p align="center">Made with ❤️ in Brazil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
