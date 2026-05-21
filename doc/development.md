# Development Guide

Operational reference for running, debugging, and testing the workspace. See [`package.md`](package.md) for architecture and topic layout.

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

## Common Development Tasks

### Running Multi-Robot Setup
```bash
# Individual instances
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-3660
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-1234

# OR use multi-robot webserver
ros2 run multirobot_webserver multirobot_webapp
# Then open http://localhost:5000 and add Spheros via UI
```

### State Machine (per instance)
```bash
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

## Debugging Tips

**Check running nodes:**
```bash
ros2 node list
ros2 node info /<node_name>
```

**Monitor topics:**
```bash
ros2 topic list
ros2 topic echo sphero/SB-3660/state
ros2 topic hz sphero/SB-3660/sensors
```

**Publish test commands:**
```bash
ros2 topic pub sphero/SB-3660/led std_msgs/String '{"data": "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'
```

**Sphero connection issues:**
```bash
hciconfig  # Check Bluetooth is UP RUNNING
# Reset Sphero: place on charger for 2 seconds, remove
```

## Test Scripts

Located in `scripts/`:
- `test_aruco_detector.py` - ArUco detection validation
- `test_ble_uwb_scanner.py` - BLE UWB scanner validation

## Documentation Files

- `README.md` - Repo overview and quick start
- `doc/package.md` - Package architecture, topics, messages, state machine config
- `doc/development.md` - This file
- `doc/AGENTS.md` - SME agent overview
- `doc/ROS2_AGENT_GUIDE.md`, `doc/ARDUINO_AGENT_GUIDE.md`, `doc/WEB_AGENT_GUIDE.md` - Per-agent guides
- `doc/UWB_LIBRARIES_INSTALLED.md`, `doc/UWB_SYSTEM_COMPLETE.md` - UWB system docs
- `doc/ARDUINO_CLI_INSTALLATION_COMPLETE.md` - Arduino CLI setup
- `doc/CLUSTER.md` - Cluster notes
- Individual package READMEs in `src/*/README.md`
