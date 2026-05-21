# Package Architecture

This workspace contains the ROS2 packages for multi-robot Sphero control, UWB positioning, and ArUco-based localization.

## sphero_instance_controller

**Primary multi-robot package** - designed for independent multi-robot control with namespaced topics.

- **Architecture:** Core `Sphero` class (ROS-independent) + ROS2 node wrappers
- **Namespacing:** All topics under `sphero/<sphero_name>/`
- **Nodes:**
  - `sphero_instance_device_controller_node.py` - Low-level commands
  - `sphero_instance_task_controller_node.py` - High-level tasks
  - `sphero_instance_statemachine_controller_node.py` - State machine
- **Required parameter:** `sphero_name` (e.g., "SB-3660")
- **Key insight:** Includes collision detection patching via `spherov2_collision_patch.py`

## multirobot_webserver

Central web dashboard for managing multiple Spheros simultaneously.

- **Main app:** `multirobot_webapp.py` (standalone Flask, not a ROS2 node)
- **WebSocket servers:** `sphero_instance_websocket_server.py` (one per Sphero, ports 5001+)
- **Port 5000:** Main dashboard
- **Architecture:** Each Sphero gets dedicated WebSocket server + all three controllers

## aruco_slam

ArUco marker-based localization for Sphero position tracking.

- **Main node:** `aruco_slam_node.py`
- **Marker IDs:**
  - 0-3: Field corners (calibration)
  - 10-13: Sphero robots (SB-3660, SB-74FB, SB-3716, SB-58EF)
- **Topics:** `/aruco_slam/<sphero_name>/position` (geometry_msgs/PoseStamped)
- **Coordinate system:** Origin at top-left, +X right, +Y down, units in cm

## sphero_uwb_positioning

UWB-based positioning for Sphero robots using Arduino-based tag/anchor hardware. See `src/sphero_uwb_positioning/` for node-level details.

## Topic Namespacing Strategy

**Per-instance namespace:** `sphero/<sphero_name>/*`
- `sphero/SB-3660/led`, `sphero/SB-3660/sensors`, `sphero/SB-3660/state`
- `sphero/SB-1234/led`, `sphero/SB-1234/sensors`, `sphero/SB-1234/state`

**State machine topics:** `sphero/<sphero_name>/state_machine/config`, `sphero/<sphero_name>/state_machine/status`

**ArUco topics:** `/aruco_slam/<sphero_name>/position`

## Key Files

### Core Implementation
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero.py` - Core Sphero class (ROS-independent)
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/task.py` - Task definitions
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py` - State machine logic

### Multi-Robot WebSocket
- `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py` - Central dashboard
- `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_websocket_server.py` - Per-robot WebSocket

### Localization
- `src/aruco_slam/aruco_slam/aruco_slam_node.py` - ArUco detection and field mapping
- `src/aruco_slam/aruco_slam/field_mapper.py` - Perspective transformation logic

## Message Types

### Custom Messages (sphero_instance_controller)
- `SpheroSensor.msg` - Accelerometer, gyroscope, velocity, location data
- `SpheroCommand.msg`, `SpheroRoll.msg`, `SpheroLed.msg`, etc.

### Standard ROS Messages Used
- `std_msgs/String` - JSON-encoded commands and states
- `sensor_msgs/BatteryState` - Battery status
- `geometry_msgs/PoseStamped` - ArUco position tracking

## State Machine Configuration

State machines accept JSON configurations with:
- **States:** name, description, entry_condition (always/timer/topic_value), task
- **Transitions:** source, destination, trigger, condition
- **Condition types:**
  - `always` - Immediate entry
  - `timer` - Duration-based
  - `topic_value` - Topic field comparison (supports nested paths like "linear.x")
  - `topic_message` - Any message received on topic

## Implementation Details

1. **Namespacing consistency:** When working with multi-robot code, always use `sphero/<sphero_name>/` prefix. Hyphens in Sphero names become underscores in node names.

2. **ArUco marker dictionary:** 4x4_50 dictionary. Markers must be flat and well-lit for detection.

3. **Collision detection patching:** `spherov2_collision_patch.py` fixes upstream issues with spherov2 library collision callbacks.

4. **External localization:** ArUco SLAM provides external position data via `/aruco_slam/<sphero_name>/position`. Spheros also have internal odometry from `toy.get_location()`.

5. **Stall detection logic:** Task executors include timeout/stall detection for move_to tasks (see patrol method implementation).
