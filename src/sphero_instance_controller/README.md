# Sphero Instance Controller

Multi-robot Sphero controller package designed for multi-robot setups where multiple instances can be spawned to control different Sphero robots independently without cross-talk.

## Features

- **Per-instance namespacing**: All topics are prefixed with `sphero/<sphero_name>/`
- **Required sphero_name parameter**: Each instance must specify which Sphero it connects to
- **Independent operation**: Multiple controllers can run simultaneously without interference
- **Full Sphero control**: LED, movement, matrix display, sensors, collision detection, and more
- **Clean architecture**: Core Sphero functionality encapsulated in reusable `Sphero` class
- **Self-contained**: Includes its own message definitions, no external package dependencies (except spherov2)

## Architecture

The package follows a clean separation of concerns:

### Core Classes (ROS-independent)

- **`core.sphero.Sphero`**: Core class that encapsulates all Sphero hardware interaction, state management, and command execution. This class can be used independently of ROS.
- **`core.sphero.TaskExecutor`**: High-level task execution system for complex behaviors (move_to, patrol, circle, LED sequences, etc.). Uses the `Sphero` class for low-level control.
- **`core.sphero.StateMachine`**: Dynamic state machine with configurable states, transitions, and conditions. Can orchestrate complex behaviors using the TaskExecutor.
- **`core.sphero.SpheroState`**: State management and sensor data tracking.
- **`core.sphero.matrix_patterns`**: Predefined LED matrix patterns for Sphero BOLT.

### ROS2 Nodes

- **`sphero_instance_device_controller_node.py`**: Basic controller providing low-level command interface via topics
- **`sphero_instance_task_controller_node.py`**: High-level task controller for complex autonomous behaviors
- **`sphero_instance_statemachine_controller_node.py`**: Dynamic state machine controller for behavior orchestration

## Installation

1. Build the package:
```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_instance_controller
source install/setup.bash
```

## Usage

### Device Controller (Low-level Commands)

Launch the basic controller for low-level command control:

```bash
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-3660
```

This provides direct topic-based control for LED, roll, heading, speed, etc.

### Task Controller (High-level Behaviors)

Launch the task controller for autonomous task execution:

```bash
ros2 run sphero_instance_controller sphero_instance_task_controller_node.py --ros-args -p sphero_name:=SB-3660
```

This allows you to send high-level tasks like "move_to", "patrol", "circle", etc.

### State Machine Controller (Behavior Orchestration)

Launch the state machine controller for dynamic state-based behaviors:

```bash
ros2 run sphero_instance_controller sphero_instance_statemachine_controller_node.py --ros-args -p sphero_name:=SB-3660
```

This provides a configurable state machine that can orchestrate complex behaviors with states, transitions, and conditions.

Replace `SB-3660` with your actual Sphero's name in all cases.

### Multiple Sphero Instances

Launch multiple controllers for different Spheros:

**Terminal 1:**
```bash
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-3660
```

**Terminal 2:**
```bash
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=SB-1234
```

Each controller will use its own namespaced topics:
- Sphero SB-3660: `sphero/SB-3660/...`
- Sphero SB-1234: `sphero/SB-1234/...`

## Topic Namespacing

All topics are namespaced under `sphero/<sphero_name>/`:

### Subscribed Topics (Commands)

- `sphero/<sphero_name>/led` - LED color control
- `sphero/<sphero_name>/roll` - Roll movement commands
- `sphero/<sphero_name>/spin` - Spin commands
- `sphero/<sphero_name>/heading` - Heading control
- `sphero/<sphero_name>/speed` - Speed control
- `sphero/<sphero_name>/raw_motor` - Raw motor control (independent left/right motors)
- `sphero/<sphero_name>/stop` - Stop movement
- `sphero/<sphero_name>/reset_aim` - Reset orientation and position
- `sphero/<sphero_name>/matrix` - LED matrix display (BOLT only)
- `sphero/<sphero_name>/collision` - Collision detection control
- `sphero/<sphero_name>/stabilization` - Stabilization control

### Published Topics (Status/Sensors)

- `sphero/<sphero_name>/sensors` - Sensor data (SpheroSensor message)
- `sphero/<sphero_name>/state` - Complete state as JSON
- `sphero/<sphero_name>/battery` - Battery status (BatteryState message)
- `sphero/<sphero_name>/status` - Health heartbeat JSON
- `sphero/<sphero_name>/tap` - Tap detection events
- `sphero/<sphero_name>/obstacle` - Obstacle collision events

### Task Controller Topics

When using the task controller:

**Subscribe:**
- `sphero/<sphero_name>/task` - Task commands in JSON format

**Publish:**
- `sphero/<sphero_name>/task/status` - Task execution status updates

## Example: Controlling Multiple Spheros

### Python example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MultiSpheroController(Node):
    def __init__(self):
        super().__init__('multi_sphero_controller')

        # Create publishers for two different Spheros
        self.sphero1_led_pub = self.create_publisher(
            String, 'sphero/SB-3660/led', 10)
        self.sphero2_led_pub = self.create_publisher(
            String, 'sphero/SB-1234/led', 10)

    def set_sphero1_red(self):
        msg = String()
        msg.data = json.dumps({"red": 255, "green": 0, "blue": 0})
        self.sphero1_led_pub.publish(msg)

    def set_sphero2_blue(self):
        msg = String()
        msg.data = json.dumps({"red": 0, "green": 0, "blue": 255})
        self.sphero2_led_pub.publish(msg)
```

### Command line example:

```bash
# Set LED color for Sphero SB-3660
ros2 topic pub /sphero/SB-3660/led std_msgs/msg/String \
  '{data: "{\"red\": 255, \"green\": 0, \"blue\": 0}"}'

# Set LED color for Sphero SB-1234
ros2 topic pub /sphero/SB-1234/led std_msgs/msg/String \
  '{data: "{\"red\": 0, \"green\": 0, \"blue\": 255}"}'

# Make Sphero SB-3660 roll
ros2 topic pub /sphero/SB-3660/roll std_msgs/msg/String \
  '{data: "{\"heading\": 90, \"speed\": 100, \"duration\": 2.0}"}'

# Control raw motors independently (left motor faster than right for curved motion)
ros2 topic pub /sphero/SB-3660/raw_motor std_msgs/msg/String \
  '{data: "{\"left_mode\": \"forward\", \"left_speed\": 200, \"right_mode\": \"forward\", \"right_speed\": 150}"}'
```

## Parameters

- `sphero_name` (required, string): Name of the Sphero to connect to
- `sensor_rate` (optional, float, default: 10.0): Sensor publishing rate in Hz
- `heartbeat_rate` (optional, float, default: 5.0): Heartbeat publishing interval in seconds

## Differences from sphero_package

1. **Required sphero_name parameter**: Must be specified for each instance
2. **Namespaced topics**: All topics include `sphero/<sphero_name>/` prefix
3. **Multi-robot ready**: Designed from the ground up for concurrent operation
4. **Self-contained**: Includes its own message definitions - fully independent package
5. **Task execution**: Includes high-level task controller with autonomous behaviors
6. **Clean architecture**: Separation between core logic and ROS interface

## Launch File Example

Create a launch file to start multiple Spheros:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sphero_instance_controller',
            executable='sphero_instance_device_controller_node.py',
            name='sphero_device_controller_1',
            parameters=[
                {'sphero_name': 'SB-3660'},
                {'sensor_rate': 10.0},
                {'heartbeat_rate': 5.0}
            ]
        ),
        Node(
            package='sphero_instance_controller',
            executable='sphero_instance_device_controller_node.py',
            name='sphero_device_controller_2',
            parameters=[
                {'sphero_name': 'SB-1234'},
                {'sensor_rate': 10.0},
                {'heartbeat_rate': 5.0}
            ]
        ),
    ])
```

## Troubleshooting

### "sphero_name parameter is required" error
Make sure to provide the `sphero_name` parameter when launching:
```bash
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py --ros-args -p sphero_name:=YOUR_SPHERO_NAME
```

### Cannot find Sphero
- Ensure Bluetooth is enabled
- Verify the Sphero name matches exactly (case-sensitive)
- Check that the Sphero is charged and turned on
- Make sure the Sphero is not already connected to another device

### Topics not appearing
- Verify the controller started successfully
- Check the topic namespace matches your sphero_name parameter
- Use `ros2 topic list | grep sphero` to see all Sphero topics

## Task Controller Usage Examples

### Sending Tasks via Command Line

**Move to a specific position:**
```bash
ros2 topic pub /sphero/SB-3660/task std_msgs/msg/String \
  '{data: "{\"task_type\": \"move_to\", \"parameters\": {\"x\": 100, \"y\": 50, \"speed\": 100}}"}'
```

**Patrol between waypoints:**
```bash
ros2 topic pub /sphero/SB-3660/task std_msgs/msg/String \
  '{data: "{\"task_type\": \"patrol\", \"parameters\": {\"waypoints\": [{\"x\": 0, \"y\": 0}, {\"x\": 100, \"y\": 0}, {\"x\": 100, \"y\": 100}, {\"x\": 0, \"y\": 100}], \"speed\": 80, \"loop\": true}}"}'
```

**Move in a circle (uses differential motor speeds for precise radius control):**
```bash
ros2 topic pub /sphero/SB-3660/task std_msgs/msg/String \
  '{data: "{\"task_type\": \"circle\", \"parameters\": {\"radius\": 50, \"speed\": 100, \"duration\": 10.0, \"direction\": \"ccw\"}}"}'
```
Note: The circle task uses raw motor control to achieve precise circular motion based on the desired radius. The `direction` parameter can be "cw" (clockwise) or "ccw" (counter-clockwise, default).

**LED color sequence:**
```bash
ros2 topic pub /sphero/SB-3660/task std_msgs/msg/String \
  '{data: "{\"task_type\": \"led_sequence\", \"parameters\": {\"sequence\": [{\"red\": 255, \"green\": 0, \"blue\": 0}, {\"red\": 0, \"green\": 255, \"blue\": 0}, {\"red\": 0, \"green\": 0, \"blue\": 255}], \"interval\": 1.0, \"loop\": false}}"}'
```

### Supported Task Types

- **move_to**: Move to specific (x, y) coordinates
- **patrol**: Follow waypoints in sequence
- **circle**: Move in circular pattern using differential motor speeds (supports radius, speed, duration, direction parameters)
- **square**: Move in square pattern
- **led_sequence**: Execute LED color sequence
- **matrix_sequence**: Display matrix pattern sequence (BOLT only)
- **spin**: Spin in place
- **stop**: Stop all movement
- **custom**: Execute custom command sequence
- **set_led**: Set LED color (immediate)
- **roll**: Roll command
- **heading**: Set heading
- **speed**: Set speed
- **matrix**: Display matrix pattern
- **collision**: Configure collision detection
- **reflect**: Reflect/bounce behavior
- **jumping_bean**: Erratic jumping motion

## State Machine Controller Usage

The state machine controller provides dynamic behavior orchestration through configurable states and transitions.

### Configuration Format

Configure the state machine by publishing JSON to `sphero/<sphero_name>/state_machine/config`:

```json
{
  "name": "My State Machine",
  "initial_state": "idle",
  "states": [
    {
      "name": "idle",
      "description": "Robot is idle",
      "entry_condition": {
        "type": "always",
        "params": {}
      },
      "tasks": [
        {
          "task_type": "set_led",
          "parameters": {"red": 0, "green": 0, "blue": 255}
        }
      ]
    },
    {
      "name": "moving",
      "description": "Robot is moving",
      "entry_condition": {
        "type": "timer",
        "params": {"duration": 3.0}
      },
      "tasks": [
        {
          "task_type": "roll",
          "parameters": {"heading": 0, "speed": 100, "duration": 2.0}
        }
      ],
      "timeout": 5.0
    }
  ],
  "transitions": [
    {
      "source": "idle",
      "destination": "moving",
      "trigger": "timer_based",
      "condition": {
        "type": "timer",
        "duration": 3.0
      }
    },
    {
      "source": "moving",
      "destination": "idle",
      "trigger": "auto",
      "condition": {
        "type": "auto"
      }
    }
  ]
}
```

### Entry Condition Types

1. **always**: Always allow entry (default)
2. **timer**: Wait for duration in current state
   - `params`: `{"duration": 5.0}`
3. **topic_value**: Check sensor value
   - `params`: `{"key": "battery_percentage", "operator": "<", "value": 20.0}`
   - Operators: `==`, `!=`, `>`, `<`, `>=`, `<=`

### Transition Condition Types

1. **auto**: Use destination state's entry condition (backward compatible)
2. **timer**: Transition after duration in source state
   - `{"type": "timer", "duration": 5.0}`
3. **topic_value**: Transition when topic value matches condition
   - `{"type": "topic_value", "topic": "command", "msg_type": "std_msgs/String", "field_path": "data", "operator": "==", "value": "stop"}`
4. **topic_message**: Transition when message received on topic
   - `{"type": "topic_message", "topic": "emergency", "msg_type": "std_msgs/Bool", "timeout": 1.0}`

### State Machine Topics

**Subscribe:**
- `sphero/<sphero_name>/state_machine/config` - Configuration messages
- `sphero/<sphero_name>/sensors` - Sensor data for condition evaluation

**Publish:**
- `sphero/<sphero_name>/state_machine/status` - Current state and status
- `sphero/<sphero_name>/state_machine/events` - State machine events
- `sphero/<sphero_name>/task` - Task commands (compatible with task controller)

### Example: Simple Patrol State Machine

```bash
ros2 topic pub /sphero/SB-3660/state_machine/config std_msgs/msg/String \
  '{data: "{\"name\": \"Patrol\", \"initial_state\": \"north\", \"states\": [{\"name\": \"north\", \"entry_condition\": {\"type\": \"always\"}, \"tasks\": [{\"task_type\": \"roll\", \"parameters\": {\"heading\": 0, \"speed\": 80, \"duration\": 2.0}}]}, {\"name\": \"east\", \"entry_condition\": {\"type\": \"timer\", \"params\": {\"duration\": 2.5}}, \"tasks\": [{\"task_type\": \"roll\", \"parameters\": {\"heading\": 90, \"speed\": 80, \"duration\": 2.0}}]}, {\"name\": \"south\", \"entry_condition\": {\"type\": \"timer\", \"params\": {\"duration\": 2.5}}, \"tasks\": [{\"task_type\": \"roll\", \"parameters\": {\"heading\": 180, \"speed\": 80, \"duration\": 2.0}}]}, {\"name\": \"west\", \"entry_condition\": {\"type\": \"timer\", \"params\": {\"duration\": 2.5}}, \"tasks\": [{\"task_type\": \"roll\", \"parameters\": {\"heading\": 270, \"speed\": 80, \"duration\": 2.0}}]}], \"transitions\": [{\"source\": \"north\", \"destination\": \"east\", \"trigger\": \"auto\"}, {\"source\": \"east\", \"destination\": \"south\", \"trigger\": \"auto\"}, {\"source\": \"south\", \"destination\": \"west\", \"trigger\": \"auto\"}, {\"source\": \"west\", \"destination\": \"north\", \"trigger\": \"auto\"}]}"}'
```

### Combined Usage: State Machine + Task Controller

For best results, run both the state machine controller and task controller together:

**Terminal 1 (Task Controller):**
```bash
ros2 run sphero_instance_controller sphero_instance_task_controller_node.py --ros-args -p sphero_name:=SB-3660
```

**Terminal 2 (State Machine Controller):**
```bash
ros2 run sphero_instance_controller sphero_instance_statemachine_controller_node.py --ros-args -p sphero_name:=SB-3660
```

The state machine will publish tasks to the task controller, which executes them on the Sphero.

### Monitoring State Machine

**Watch status:**
```bash
ros2 topic echo /sphero/SB-3660/state_machine/status
```

**Watch events:**
```bash
ros2 topic echo /sphero/SB-3660/state_machine/events
```

## License

TODO: License declaration
