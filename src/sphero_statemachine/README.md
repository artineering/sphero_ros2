# Sphero State Machine Package

A dynamic, configurable state machine controller for Sphero robots using the python-statemachine library and ROS2.

## Overview

This package provides a flexible state machine framework that allows you to define robot behaviors without modifying code. Configure states, transitions, and tasks via JSON, and monitor execution in real-time through a web interface.

## Features

- **Dynamic Configuration**: Define states and transitions via JSON messages
- **Entry Conditions**: Control when states can be entered
  - Always (immediate entry)
  - Timer-based (wait for duration)
  - Sensor-based (check topic values)
- **Task Execution**: Execute tasks when entering states
- **Web Interface**: Visual configuration and monitoring
- **Real-time Events**: Track state transitions and events
- **Example Templates**: Pre-built configurations for common patterns

## Installation

This package is part of the ROS2 workspace and requires:

- ROS2 (tested with Humble)
- python-statemachine
- rclpy
- std_msgs

Already installed in this workspace.

## Quick Start

### 1. Start the State Machine System

**Option A: Via Web Interface (Recommended)**
```bash
# Start the web server (this will start everything you need)
ros2 run sphero_web_interface web_server_node

# Open browser to http://localhost:5000/state_machine
# Click "Start Controller" - this starts both the controller and task executor
```

**Option B: Manual Start**
```bash
# Source the workspace
source ~/ros2_ws_2/install/setup.bash

# Run the state machine controller
ros2 run sphero_statemachine state_machine_controller &

# Run the task executor (REQUIRED for Sphero to respond)
ros2 run sphero_statemachine task_executor &
```

**⚠️ Important:** Both the state machine controller AND task executor must be running for the Sphero to execute tasks!

### 2. Send a Configuration

**Option A: Use the Web Interface**

```bash
# Already running from step 1
# Just select a template and click "Send Configuration"
```

**Option B: Use Example Script**

```bash
cd ~/ros2_ws_2/src/sphero_statemachine/examples
python3 send_config.py simple_state_machine.json
```

**Option C: Use ROS2 Topic**

```bash
ros2 topic pub /state_machine/config std_msgs/msg/String \
  "{data: '$(cat examples/simple_state_machine.json)'}" --once
```

### 3. Monitor the State Machine

```bash
# Watch status updates
ros2 topic echo /state_machine/status

# Watch events
ros2 topic echo /state_machine/events

# Watch task commands
ros2 topic echo /state_machine/task_command
```

## Configuration Format

See [STATE_MACHINE_GUIDE.md](STATE_MACHINE_GUIDE.md) for detailed documentation.

### Basic Structure

```json
{
  "name": "My State Machine",
  "initial_state": "idle",
  "states": [
    {
      "name": "idle",
      "description": "Idle state",
      "entry_condition": {
        "type": "always",
        "params": {}
      },
      "task": {
        "type": "set_led",
        "params": {"color": "blue"}
      }
    }
  ],
  "transitions": [
    {
      "source": "idle",
      "destination": "active",
      "trigger": "start"
    }
  ]
}
```

## Example Configurations

### Simple Two-State Toggle

[examples/simple_state_machine.json](examples/simple_state_machine.json)

Alternates between idle (blue LED) and moving (forward motion) every 3 seconds.

### Patrol Pattern

[examples/patrol_state_machine.json](examples/patrol_state_machine.json)

Moves the robot in a square pattern (north, east, south, west).

## Topics

### Subscribed

- `/state_machine/config` (std_msgs/String): Configuration JSON
- `/state_machine/sensor_data` (std_msgs/String): Sensor data for conditions

### Published

- `/state_machine/status` (std_msgs/String): Current state and statistics
- `/state_machine/events` (std_msgs/String): State machine events
- `/state_machine/task_command` (std_msgs/String): Tasks to execute

## Web Interface

Access the web interface at `http://localhost:5000/state_machine` after starting the web server.

Features:
- Start/stop controller
- Load configuration templates
- Edit and validate JSON
- Real-time state monitoring
- Event log viewer

## Integration with Sphero

To execute tasks on your Sphero, subscribe to `/state_machine/task_command` in your controller:

```python
from std_msgs.msg import String
import json

def task_command_callback(self, msg):
    task = json.loads(msg.data)
    task_type = task.get('task_type')
    params = task.get('params', {})

    if task_type == 'set_led':
        # Set LED color
        pass
    elif task_type == 'roll':
        # Move the robot
        pass
```

## Development

### Package Structure

```
sphero_statemachine/
├── sphero_statemachine/
│   ├── state_machine_controller.py  # Main controller node
│   └── sphero_node.py                # Placeholder node
├── examples/
│   ├── simple_state_machine.json    # Example configuration
│   ├── patrol_state_machine.json    # Patrol example
│   └── send_config.py               # Helper script
├── STATE_MACHINE_GUIDE.md           # Detailed guide
├── README.md                         # This file
├── package.xml
└── setup.py
```

### Building

```bash
colcon build --packages-select sphero_statemachine
source install/setup.bash
```

## Troubleshooting

**State machine not transitioning:**
- Check entry conditions are satisfied
- Verify sensor data is being published (if using topic_value conditions)
- Check event log for condition evaluation

**Tasks not executing:**
- Ensure a subscriber is listening to `/state_machine/task_command`
- Verify task parameters are correct
- Check controller logs

**Web interface not loading:**
- Verify web server is running
- Check that templates directory is installed
- Look for errors in browser console

## Future Enhancements

- Visual state diagram editor
- State history tracking and analytics
- Parallel state execution
- Action integration
- Custom condition plugins
- Configuration versioning

## Documentation

- [STATE_MACHINE_GUIDE.md](STATE_MACHINE_GUIDE.md) - Complete usage guide
- [examples/](examples/) - Example configurations

## License

Part of the Sphero ROS2 workspace.

---

**🤖 Generated with Claude Code**
