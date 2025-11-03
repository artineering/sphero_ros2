# ROS2 Workspace - Source Packages

This workspace contains ROS2 packages for controlling and interacting with Sphero robots. The packages provide comprehensive control, web-based interfaces, state management, and game implementations.

## 📦 Packages Overview

### 1. **sphero_package**
Core package for Sphero robot control and communication.

**Purpose**: Provides low-level control, sensor reading, and command execution for Sphero robots using the spherov2 API.

**Key Features**:
- Direct Sphero robot control via Bluetooth
- Real-time sensor data acquisition (accelerometer, gyroscope, velocity, location)
- LED and LED matrix control
- Motion control (heading, speed, roll)
- Battery monitoring
- State management and publishing

**Main Nodes**:
- `sphero_controller_node.py`: Main controller for Sphero robots
  - Subscribes to command topics
  - Publishes state, sensor, and battery data
  - Manages Sphero connection lifecycle

**Topics Published**:
- `/sphero/state` (std_msgs/String): Complete robot state as JSON
- `/sphero/sensors` (sphero_package/SpheroSensor): Sensor readings
- `/sphero/battery` (sensor_msgs/BatteryState): Battery information
- `/sphero/status` (std_msgs/String): Health/heartbeat messages

**Topics Subscribed**:
- `/sphero/led`: LED color commands
- `/sphero/roll`: Roll movement commands
- `/sphero/heading`: Heading control
- `/sphero/speed`: Speed control
- `/sphero/stop`: Stop commands
- `/sphero/matrix`: LED matrix patterns (BOLT only)

**Dependencies**:
- spherov2 (Python library for Sphero control)
- ROS2 (rclpy, std_msgs, sensor_msgs)

**Documentation**: See [sphero_package/README.md](sphero_package/README.md)

---

### 2. **sphero_web_interface**
Modern web-based control interface for Sphero robots.

**Purpose**: Provides an intuitive browser-based UI for controlling and monitoring Sphero robots through a Flask web server integrated with ROS2.

**Key Features**:
- **Web UI** with tabbed interface:
  - Connection management
  - Real-time state monitoring
  - Sensor data visualization
  - LED matrix pattern control
  - Motion control (heading, speed, directional pad)
- **Real-time updates** via WebSocket (Socket.IO)
- **Error handling** with automatic disconnection
- **Process management** for sphero_controller_node

**Main Components**:
- `web_server_node.py`: ROS2 node with integrated Flask server
  - Manages sphero_controller lifecycle
  - Bridges web interface with ROS2 topics
  - Provides REST API and WebSocket endpoints
- `templates/index.html`: Main web interface
- `static/css/style.css`: Modern, responsive styling
- `static/js/app.js`: Client-side JavaScript with real-time updates

**Web Interface Tabs**:
1. **Connection**: Enter Sphero name and connect/disconnect
2. **State**: View connection status, battery, motion, LED color
3. **Sensors**: Real-time accelerometer, gyroscope, velocity, location
4. **Matrix**: LED control and matrix patterns (15 built-in patterns)
5. **Motion**: Heading compass, speed control, directional pad

**API Endpoints**:
- `GET /`: Main web interface
- `POST /api/connect`: Connect to Sphero
- `POST /api/disconnect`: Disconnect from Sphero
- `GET /api/status`: Get connection status
- `POST /api/led`: Set LED color
- `POST /api/matrix`: Display matrix pattern
- `POST /api/matrix/clear`: Clear matrix
- `POST /api/motion/*`: Motion control endpoints

**How to Use**:
```bash
# Start the web server
ros2 run sphero_web_interface web_server

# Open browser to http://localhost:5000
# Enter Sphero name (e.g., "SB-3660") and click Connect
```

**Dependencies**:
- Flask, Flask-SocketIO (web framework)
- sphero_package (controller node)
- ROS2 (rclpy, std_msgs, sensor_msgs)

**Documentation**: See [sphero_web_interface/README.md](sphero_web_interface/README.md)

---

### 3. **sphero_task_controller**
High-level task-based controller for Sphero robots.

**Purpose**: Accepts complex, high-level tasks via a ROS2 topic and executes them by publishing appropriate commands to sphero_controller_node. Provides a task queue system for sequential task execution.

**Key Features**:
- **Task Queue System**: Queue multiple tasks for automatic sequential execution
- **9 Built-in Task Types**:
  - `move_to`: Navigate to specific coordinates
  - `patrol`: Follow waypoints with optional looping
  - `circle`: Circular motion patterns
  - `square`: Automated square path
  - `led_sequence`: Timed LED color sequences
  - `matrix_sequence`: Matrix pattern sequences
  - `spin`: Rotation in place
  - `stop`: Immediate stop
  - `custom`: User-defined command sequences
- **State Feedback**: Closed-loop control using Sphero state
- **Task Status Publishing**: Real-time task execution status
- **JSON-based API**: Easy integration with other systems

**Main Node**:
- `task_controller`: Task queue manager and executor
  - Subscribes to `/sphero/task` for task commands
  - Publishes commands to sphero_controller
  - Publishes status to `/sphero/task/status`

**Example Tasks**:
```bash
# Move to position
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"move_to\",
    \"parameters\": {\"x\": 100, \"y\": 50, \"speed\": 100}
}'}"

# LED rainbow sequence
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"led_sequence\",
    \"parameters\": {
        \"sequence\": [{\"red\": 255, \"green\": 0, \"blue\": 0}, ...],
        \"interval\": 0.5
    }
}'}"
```

**Use Cases**:
- Autonomous navigation missions
- Choreographed performances
- Automated testing sequences
- Interactive demonstrations
- Educational programming

**Dependencies**:
- sphero_package (controller node)
- ROS2 (rclpy, std_msgs)

**Documentation**: See [sphero_task_controller/README.md](src/sphero_task_controller/README.md)

---

### 4. **sphero_statemachine**
State machine implementation for Sphero robot behaviors.

**Purpose**: Provides structured state-based control for complex Sphero behaviors and task sequences.

**Key Features**:
- Finite state machine (FSM) for robot behavior control
- Predefined states and transitions
- Event-driven architecture
- Reusable behavior patterns

**Use Cases**:
- Autonomous navigation sequences
- Game behavior implementation
- Task automation
- Complex choreographed movements

**Documentation**: See [sphero_statemachine/README.md](sphero_statemachine/README.md) (if available)

---

### 4. **battleship_game**
Implementation of a battleship game using Sphero robots.

**Purpose**: Demonstrates interactive game mechanics using Sphero robots as game pieces on a physical board.

**Key Features**:
- Game logic for battleship
- Physical board mapping
- Collision detection
- Player interaction through Sphero movement
- Score tracking and game state management

**Use Cases**:
- Educational demonstrations
- Robot interaction games
- Multi-robot coordination
- Human-robot interaction research

**Documentation**: See [battleship_game/README.md](battleship_game/README.md) (if available)

---

## 🚀 Quick Start

### Prerequisites
```bash
# Install ROS2 (Humble/Rolling/Iron)
# Install Python dependencies
pip3 install spherov2 flask flask-socketio python-socketio
```

### Building the Workspace
```bash
cd ~/ros2_ws_2
colcon build
source install/setup.bash
```

### Running the Web Interface (Recommended)
```bash
# Start the web interface (automatically manages controller)
ros2 run sphero_web_interface web_server

# Open browser: http://localhost:5000
# Enter Sphero name and connect
```

### Running the Controller Directly
```bash
# Start controller for a specific Sphero
ros2 run sphero_package sphero_controller_node.py --ros-args -p toy_name:=SB-3660

# In another terminal, control via topics
ros2 topic pub /sphero/led std_msgs/String '{"data": "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'
ros2 topic pub /sphero/roll std_msgs/String '{"data": "{\"heading\": 0, \"speed\": 100}"}'
```

---

## 📡 ROS2 Topic Architecture

### Core Data Flow
```
Sphero Robot (Bluetooth)
    ↕
sphero_controller_node
    ↕ (ROS2 Topics)
sphero_web_interface / Other Nodes
    ↕
User Interface / Applications
```

### Topic Hierarchy
```
/sphero/
├── Commands (Subscribed by controller)
│   ├── /led          - LED color control
│   ├── /roll         - Roll movement
│   ├── /heading      - Set heading
│   ├── /speed        - Set speed
│   ├── /stop         - Stop robot
│   └── /matrix       - LED matrix patterns
│
└── Data (Published by controller)
    ├── /state        - Complete robot state (JSON)
    ├── /sensors      - Sensor readings
    ├── /battery      - Battery status
    └── /status       - Heartbeat/health
```

---

## 🔧 Development

### Package Structure
Each package follows standard ROS2 Python package structure:
```
package_name/
├── package_name/          # Python modules
│   ├── __init__.py
│   ├── node_name.py       # Main node scripts
│   └── ...
├── package.xml            # Package manifest
├── setup.py               # Python setup script
├── setup.cfg              # Setup configuration
└── README.md              # Package documentation
```

### Adding a New Package
```bash
cd ~/ros2_ws_2/src
ros2 pkg create --build-type ament_python package_name --dependencies rclpy std_msgs
```

### Building Individual Packages
```bash
colcon build --packages-select package_name
source install/setup.bash
```

---

## 🐛 Troubleshooting

### Sphero Connection Issues
1. **Check Bluetooth**: `hciconfig` (should show UP RUNNING)
2. **Verify Sphero name**: Check robot label or Sphero Edu app
3. **Reset Sphero**: Place on charger for 2 seconds, remove
4. **Check battery**: Sphero should be charged (>20%)

### Web Interface Issues
1. **Port in use**: Kill process using port 5000 or change port in web_server_node.py
2. **Template not found**: Rebuild package: `colcon build --packages-select sphero_web_interface`
3. **No updates**: Refresh browser, check WebSocket connection (F12 console)

### Build Issues
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

### ROS2 Topics Not Publishing
```bash
# List active topics
ros2 topic list

# Echo a topic to verify data
ros2 topic echo /sphero/state

# Check node status
ros2 node list
ros2 node info /sphero_controller_node
```

---

## 📚 Additional Resources

### Documentation
- **Sphero SDK**: https://sdk.sphero.com/
- **spherov2 Library**: https://github.com/artificial-intelligence-class/spherov2.py
- **ROS2 Documentation**: https://docs.ros.org/

### Useful Commands
```bash
# List all ROS2 packages
ros2 pkg list | grep sphero

# Get package info
ros2 pkg prefix sphero_package

# List executables in a package
ros2 pkg executables sphero_package

# Monitor topic frequency
ros2 topic hz /sphero/state

# Record topics to bag file
ros2 bag record -a

# Replay bag file
ros2 bag play <bag_file>
```

---

## 🤝 Contributing

When adding or modifying packages:
1. Update package README.md
2. Update this top-level README.md
3. Document all ROS2 topics and parameters
4. Include usage examples
5. Add troubleshooting tips
6. Test with `colcon test` (if tests available)

---

## 📄 License

Check individual package directories for specific license information.

---

## ✨ Features Summary

| Package | Purpose | UI | Real-time | Multi-Robot |
|---------|---------|----|-----------| ------------|
| sphero_package | Core control | CLI | ✓ | ✓ |
| sphero_web_interface | Web control | Web | ✓ | ✗ |
| sphero_statemachine | Behavior FSM | - | ✓ | ✓ |
| battleship_game | Game impl. | Custom | ✓ | ✓ |

---

## 🎯 Use Case Guide

### For Basic Control
→ Use **sphero_web_interface**
- Easy browser-based control
- No coding required
- Visual feedback

### For Automation/Scripting
→ Use **sphero_package** directly
- Publish to ROS2 topics
- Integrate with other ROS2 nodes
- Custom control logic

### For Complex Behaviors
→ Use **sphero_statemachine**
- Structured state-based control
- Event-driven architecture
- Reusable patterns

### For Games/Interactions
→ Use **battleship_game** as template
- Multi-robot coordination
- Interactive gameplay
- Physical board integration

---

**Last Updated**: November 2025
**ROS2 Distribution**: Rolling
**Maintained by**: Siddharth Vaghela (siddharth.vaghela@tufts.edu)

**Created with assistance from Claude Code (Anthropic)**

**Disclaimer**: All the code in this repository has been written with the help of Claude Code. While the maintainer is trying their best to iron out mistakes in the implementation, some errors might still exist in the code. The maintainer takes no responsibility for the errors in the code and the users assume the risk of using AI-generated code.
