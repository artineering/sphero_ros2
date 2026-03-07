# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 workspace for controlling Sphero robots with multi-robot coordination, state machines, ArUco-based localization, and web interfaces. The workspace contains both single-robot and multi-robot packages with sophisticated task execution and game implementations.

## Build System

**Build entire workspace:**
```bash
colcon build
source install/setup.bash
```

**Build specific package:**
```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

**Clean rebuild:**
```bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Package Architecture

### Single-Robot Packages (Original Implementation)

#### sphero_package
Core low-level Sphero control package. Provides direct hardware interface via spherov2 library.
- **Main node:** `sphero_controller_node.py`
- **Topics:** Uses flat namespace (`/sphero/led`, `/sphero/sensors`, etc.)
- **Use case:** Single-robot scenarios, direct hardware control

#### sphero_web_interface
Web-based control for single Sphero instance.
- **Main node:** `web_server_node.py`
- **Port:** 5000
- **Features:** LED control, matrix patterns, motion control, sensor visualization

#### sphero_statemachine
Dynamic JSON-configurable state machine for single Sphero.
- **Main nodes:** `state_machine_controller.py`, `task_executor.py`
- **Key feature:** Topic-based transitions (timer, topic_value, topic_message conditions)
- **Web UI:** http://localhost:5000/state_machine

#### sphero_task_controller
High-level task execution (move_to, patrol, circle, LED sequences).
- **Main node:** `task_controller_node.py`
- **Topics:** `/sphero/task`, `/sphero/task/status`

### Multi-Robot Packages (Advanced Implementation)

#### sphero_instance_controller
**Primary multi-robot package** - designed for independent multi-robot control with namespaced topics.
- **Architecture:** Core `Sphero` class (ROS-independent) + ROS2 node wrappers
- **Namespacing:** All topics under `sphero/<sphero_name>/`
- **Nodes:**
  - `sphero_instance_device_controller_node.py` - Low-level commands
  - `sphero_instance_task_controller_node.py` - High-level tasks
  - `sphero_instance_statemachine_controller_node.py` - State machine
- **Required parameter:** `sphero_name` (e.g., "SB-3660")
- **Key insight:** Includes collision detection patching via `spherov2_collision_patch.py`

#### multirobot_webserver
Central web dashboard for managing multiple Spheros simultaneously.
- **Main app:** `multirobot_webapp.py` (standalone Flask, not a ROS2 node)
- **WebSocket servers:** `sphero_instance_websocket_server.py` (one per Sphero, ports 5001+)
- **Port 5000:** Main dashboard
- **Architecture:** Each Sphero gets dedicated WebSocket server + all three controllers

#### aruco_slam
ArUco marker-based localization for Sphero position tracking.
- **Main node:** `aruco_slam_node.py`
- **Marker IDs:**
  - 0-3: Field corners (calibration)
  - 10-13: Sphero robots (SB-3660, SB-74FB, SB-3716, SB-58EF)
- **Topics:** `/aruco_slam/<sphero_name>/position` (geometry_msgs/PoseStamped)
- **Coordinate system:** Origin at top-left, +X right, +Y down, units in cm

#### soccer_game_controller
Orchestrates multi-robot soccer game with ArUco calibration.
- **Launch file:** `soccer_game.launch.py`
- **Dependencies:** aruco_slam, multirobot_webserver, sphero_instance_controller
- **Workflow:** Field calibration → robot detection → controller activation → positioning → demo

### Game Implementations

#### battleship_game
Battleship game implementation with Sphero robots.
- **Nodes:** `game_controller_node.py`, `human_controller_node.py`, `sphero_agent_node.py`
- **Custom messages:** Attack, AttackResult, NewBoard
- **Architecture:** Distributed node-based (see DESIGN.md)

## Common Development Tasks

### Running Single Sphero (Simple)
```bash
# Web interface (easiest)
ros2 run sphero_web_interface web_server

# OR direct controller
ros2 run sphero_package sphero_controller_node.py --ros-args -p toy_name:=SB-3660
```

### Running Multi-Robot Setup
```bash
# Individual instances
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-3660
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-1234

# OR use multi-robot webserver
ros2 run multirobot_webserver multirobot_webapp
# Then open http://localhost:5000 and add Spheros via UI
```

### State Machine Usage
```bash
# Single robot
ros2 run sphero_statemachine state_machine_controller &
ros2 run sphero_statemachine task_executor &
# Load config via web UI at http://localhost:5000/state_machine

# Multi-robot (per instance)
ros2 run sphero_instance_controller sphero_instance_statemachine_controller_node.py --ros-args -p sphero_name:=SB-3660
```

### ArUco Localization
```bash
# Generate markers first
ros2 run aruco_slam marker_generator

# Run SLAM
ros2 run aruco_slam aruco_slam_node --ros-args \
  -p camera_id:=0 \
  -p field_width_cm:=600.0 \
  -p field_height_cm:=400.0
```

### Soccer Game Demo
```bash
ros2 launch soccer_game_controller soccer_game.launch.py
```

## Topic Namespacing Strategy

**Single-robot packages:** Flat namespace `/sphero/*`
- `/sphero/led`, `/sphero/sensors`, `/sphero/state`, `/sphero/task`

**Multi-robot packages:** Per-instance namespace `sphero/<sphero_name>/*`
- `sphero/SB-3660/led`, `sphero/SB-3660/sensors`, `sphero/SB-3660/state`
- `sphero/SB-1234/led`, `sphero/SB-1234/sensors`, `sphero/SB-1234/state`

**State machine topics:**
- Single: `/state_machine/config`, `/state_machine/status`
- Multi: `sphero/<sphero_name>/state_machine/config`, `sphero/<sphero_name>/state_machine/status`

**ArUco topics:** `/aruco_slam/<sphero_name>/position`

## Key Files to Understand

### Core Implementation
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero.py` - Core Sphero class (ROS-independent)
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/task.py` - Task definitions
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py` - State machine logic

### Topic-Based State Machines
- `src/sphero_statemachine/sphero_statemachine/state_machine_controller.py` - Implements timer/topic transition conditions
- See `TOPIC_TRANSITIONS_UPDATE.md` for transition condition documentation

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

### Custom Messages (battleship_game)
- `Attack.msg`, `AttackResult.msg`, `NewBoard.msg`

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

Example templates in `src/sphero_statemachine/examples/`

## Debugging Tips

**Check running nodes:**
```bash
ros2 node list
ros2 node info /<node_name>
```

**Monitor topics:**
```bash
ros2 topic list
ros2 topic echo /sphero/state
ros2 topic echo sphero/SB-3660/state
ros2 topic hz /sphero/sensors
```

**Publish test commands:**
```bash
# Single robot
ros2 topic pub /sphero/led std_msgs/String '{"data": "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'

# Multi-robot
ros2 topic pub sphero/SB-3660/led std_msgs/String '{"data": "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'
```

**Sphero connection issues:**
```bash
hciconfig  # Check Bluetooth is UP RUNNING
# Reset Sphero: place on charger for 2 seconds, remove
```

**Web interface not loading:**
```bash
colcon build --packages-select sphero_web_interface  # Rebuild templates
# Clear browser cache
```

## Important Implementation Details

1. **Both task executor AND state machine controller required:** For state machines to execute Sphero tasks, both nodes must run simultaneously.

2. **Namespacing consistency:** When working with multi-robot code, always use `sphero/<sphero_name>/` prefix. Hyphens in Sphero names become underscores in node names.

3. **ArUco marker dictionary:** 4x4_50 dictionary. Markers must be flat and well-lit for detection.

4. **Collision detection patching:** `spherov2_collision_patch.py` fixes upstream issues with spherov2 library collision callbacks.

5. **External localization:** ArUco SLAM provides external position data via `/aruco_slam/<sphero_name>/position`. Spheros also have internal odometry from `toy.get_location()`.

6. **Stall detection logic:** Task executors include timeout/stall detection for move_to tasks (see patrol method implementation).

## Test Scripts

Root directory contains test scripts for validating functionality:
- `test_sphero_roll.py` - Roll command testing
- `test_sphero_commands.py` - Command interface testing
- `test_multi_sphero.py` - Multi-robot coordination
- `test_aruco_detector.py` - ArUco detection validation

## Documentation Files

- `README.md` - Package overview and quick start
- `DESIGN.md` - Battleship game architecture
- `TOPIC_TRANSITIONS_UPDATE.md` - State machine transition conditions
- Individual package READMEs in `src/*/README.md`
- `ARCHITECTURE.md` (sphero_instance_controller) - Detailed architecture diagrams

## ROS2 Distribution

This workspace uses ROS2 Rolling. Maintained by Siddharth Vaghela (siddharth.vaghela@tufts.edu).

**Note:** All code written with assistance from Claude Code. Errors may exist in AI-generated code - users assume risk.
