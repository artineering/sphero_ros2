# Sphero Package

ROS2 package for controlling Sphero robots (BOLT, Mini, etc.) using the spherov2 API.

**Created with assistance from Claude Code (Anthropic)**

## Overview

This package provides a ROS2 interface for Sphero robots, enabling topic-based control and sensor data publishing. It includes comprehensive state management, LED/matrix control, and configurable sensor publishing rates.

## Features

- **Motion Control**: Roll, spin, heading, and speed commands
- **LED Control**: Main LED color control
- **Matrix Display**: 8x8 LED matrix patterns (for BOLT)
- **Sensor Publishing**: Periodic publishing of orientation, accelerometer, gyroscope, position, velocity, and battery data
- **State Management**: Comprehensive Sphero state representation with get/set API
- **Configurable Rates**: ROS parameters for sensor and heartbeat publishing rates

## Package Contents

### Nodes

#### sphero_controller_node
Main controller node that interfaces with Sphero robots and provides ROS2 topic-based control.

### Core Modules

- **core/sphero/state.py**: Complete Sphero state data classes
  - `SpheroState`: Main state class with sensor data, motion state, and battery information
  - Automatic device querying via `update_from_device()`
  - ROS message conversion methods
  - Get/set API for property access

- **core/sphero/matrix_patterns.py**: Predefined 8x8 LED matrix patterns
  - 15 built-in patterns (smile, frown, arrows, shapes, etc.)
  - Helper functions for pattern retrieval and validation

### Messages

All message definitions are in the `msg/` directory:

- **SpheroSensor.msg**: Complete sensor data (orientation, acceleration, gyroscope, position, velocity, battery)
- **SpheroCommand.msg**: Generic command structure
- **SpheroLED.msg**: LED color control
- **SpheroMatrix.msg**: 8x8 matrix display control
- **SpheroRoll.msg**: Roll motion command
- **SpheroSpin.msg**: Spin motion command
- **SpheroHeading.msg**: Heading control
- **SpheroSpeed.msg**: Speed control

## Dependencies

- ROS2 (Rolling/Humble/etc.)
- Python 3
- spherov2 library
- rclpy
- std_msgs
- sensor_msgs
- builtin_interfaces
- Bluetooth support

## Installation

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select sphero_package --symlink-install
source install/setup.bash
```

## Usage

### Running the Controller Node

```bash
ros2 run sphero_package sphero_controller_node.py
```

### ROS Parameters

- **sensor_rate** (default: 10.0 Hz): Frequency of sensor data publishing
- **heartbeat_rate** (default: 5.0 seconds): Interval for battery health publishing

Example with custom parameters:
```bash
ros2 run sphero_package sphero_controller_node --ros-args -p sensor_rate:=20.0 -p heartbeat_rate:=10.0
```

## Topics

### Published Topics

- **/sphero/state** (std_msgs/String): Complete Sphero state as JSON
- **/sphero/sensors** (sphero_package/SpheroSensor): Sensor data in structured message format
- **/sphero/status** (std_msgs/String): Battery and health status as JSON
- **/sphero/battery** (sensor_msgs/BatteryState): Battery state in ROS standard format

### Subscribed Topics

- **/sphero/led** (sphero_package/SpheroLED): Set main LED color
- **/sphero/roll** (sphero_package/SpheroRoll): Roll command (heading, speed, duration)
- **/sphero/spin** (sphero_package/SpheroSpin): Spin command (angle, duration)
- **/sphero/heading** (sphero_package/SpheroHeading): Set heading
- **/sphero/speed** (sphero_package/SpheroSpeed): Set speed
- **/sphero/matrix** (sphero_package/SpheroMatrix): Display pattern on LED matrix
- **/sphero/stop** (std_msgs/String): Emergency stop command

## Examples

### Set LED Color

```bash
ros2 topic pub --once /sphero/led sphero_package/msg/SpheroLED "{r: 255, g: 0, b: 0}"
```

### Roll Command

```bash
ros2 topic pub --once /sphero/roll sphero_package/msg/SpheroRoll "{heading: 0, speed: 50, duration: 2.0}"
```

### Display Matrix Pattern

Display a smile pattern:
```bash
ros2 topic pub --once /sphero/matrix sphero_package/msg/SpheroMatrix "{pattern_name: 'smile', color_r: 0, color_g: 255, color_b: 0}"
```

### Custom Matrix Pattern

Display a custom 8x8 pattern (64 values):
```bash
ros2 topic pub --once /sphero/matrix sphero_package/msg/SpheroMatrix "{pattern: [0,0,0,0,0,0,0,0, ...], color_r: 255, color_g: 255, color_b: 255}"
```

### Stop Robot

```bash
ros2 topic pub --once /sphero/stop std_msgs/msg/String "{data: 'stop'}"
```

## Available Matrix Patterns

The following predefined patterns are available:

- `smile` - Smiley face
- `frown` - Sad face
- `cross` - X mark
- `checkmark` - Check mark
- `arrow_up`, `arrow_down`, `arrow_left`, `arrow_right` - Directional arrows
- `heart` - Heart shape
- `star` - Star shape
- `circle` - Circle outline
- `square` - Square outline
- `plus` - Plus sign
- `exclamation` - Exclamation mark
- `question` - Question mark

## State Management

The `SpheroState` class provides comprehensive state management:

```python
from sphero_package.core.sphero.state import SpheroState, SpheroConnectionState

# Create state instance
state = SpheroState(toy_name="SB-3660")
state.set_api(sphero_api)

# Update from device
state.update_from_device()

# Get properties
pitch = state.get("orientation", "pitch")
battery = state.get("battery", "percentage")

# Set properties
state.set("led", Color(r=255, g=0, b=0))
state.set("orientation", 45.0, "pitch")

# Convert to ROS message
sensor_msg = state.to_sphero_sensor_msg()
battery_msg = state.battery.to_battery_state_msg()
```

## Package Structure

```
sphero_package/
├── sphero_package/
│   ├── core/
│   │   └── sphero/
│   │       ├── state.py           # State management
│   │       └── matrix_patterns.py  # LED patterns
│   ├── sphero_controller_node.py   # Main controller
│   └── sphero_node.py              # Basic node
├── msg/
│   ├── SpheroSensor.msg
│   ├── SpheroLED.msg
│   ├── SpheroMatrix.msg
│   └── ... (other messages)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Troubleshooting

### Cannot find Sphero robot
- Ensure Bluetooth is enabled on your system
- Make sure the Sphero is powered on and in pairing mode
- Verify the robot name matches the one specified in the code

### Connection issues
- Try resetting the Sphero robot
- Check that no other applications are connected to the robot
- Ensure you have the necessary Bluetooth permissions

### Message import errors
- Make sure the package is built: `colcon build --packages-select sphero_package`
- Source the workspace: `source install/setup.bash`

## License

Apache 2.0

## Maintainer

siddharth.vaghela@tufts.edu

## Contributing

This package is part of a ROS2 workspace for Sphero robot control. Contributions should maintain compatibility with the spherov2 API and follow ROS2 best practices.
