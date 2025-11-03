# Sphero Dynamic State Machine Controller

## Overview

This package provides a dynamic, configurable state machine controller for Sphero robots. The state machine can be configured at runtime via ROS2 topics, allowing you to define states, transitions, and tasks without modifying code.

## Features

- **Dynamic Configuration**: Configure states and transitions via JSON over ROS2 topics
- **Entry Conditions**: Each state supports entry conditions that must be satisfied before entering
- **Task Execution**: Each state can execute an associated task when entered
- **Web Interface**: User-friendly web interface for configuration and monitoring
- **Real-time Monitoring**: Live status updates and event logging

## Architecture

### Components

1. **State Machine Controller Node** (`state_machine_controller.py`)
   - Main controller that manages state machine execution
   - Subscribes to configuration and sensor data topics
   - Publishes status and event information

2. **Web Interface** (integrated into `sphero_web_interface`)
   - Visual configuration editor with templates
   - Real-time state monitoring
   - Event log viewer

## Configuration Format

The state machine is configured using JSON with the following structure:

```json
{
  "name": "My State Machine",
  "initial_state": "state_name",
  "states": [
    {
      "name": "state_name",
      "description": "Description of the state",
      "entry_condition": {
        "type": "condition_type",
        "params": {}
      },
      "task": {
        "type": "task_type",
        "params": {}
      },
      "timeout": 10.0
    }
  ],
  "transitions": [
    {
      "source": "state1",
      "destination": "state2",
      "trigger": "trigger_name"
    }
  ]
}
```

### Entry Condition Types

1. **always**: Always allow entry (default)
   ```json
   {
     "type": "always",
     "params": {}
   }
   ```

2. **timer**: Wait for a duration in the current state
   ```json
   {
     "type": "timer",
     "params": {
       "duration": 5.0
     }
   }
   ```

3. **topic_value**: Check a value from sensor data
   ```json
   {
     "type": "topic_value",
     "params": {
       "key": "battery_percentage",
       "operator": "<",
       "value": 20.0
     }
   }
   ```
   Operators: `==`, `!=`, `>`, `<`, `>=`, `<=`

### Task Types

Tasks are published to `/state_machine/task_command` topic and can be handled by other nodes.

Common task types:
- `set_led`: Set LED color
- `roll`: Move the Sphero
- `matrix`: Display pattern on LED matrix
- `none`: No task (monitoring only)

Example task:
```json
{
  "type": "roll",
  "params": {
    "heading": 90,
    "speed": 100
  }
}
```

## ROS2 Topics

### Subscribed Topics

- `/state_machine/config` (String): Configuration messages
- `/state_machine/sensor_data` (String): Sensor data for condition evaluation

### Published Topics

- `/state_machine/status` (String): Current state machine status
- `/state_machine/events` (String): State machine events
- `/state_machine/task_command` (String): Task commands to execute

## Usage

### 1. Start the Web Server

```bash
ros2 run sphero_web_interface web_server_node
```

Access the web interface at `http://localhost:5000/state_machine`

### 2. Start the State Machine Controller

Via web interface:
- Click "Start Controller" button

Or via command line:
```bash
ros2 run sphero_statemachine state_machine_controller
```

### 3. Configure the State Machine

**Option A: Use Web Interface Templates**
1. Select a template from the dropdown (Simple, Patrol, Timed, or Sensor-Based)
2. Modify as needed
3. Click "Send Configuration"

**Option B: Send Configuration via ROS2**
```bash
ros2 topic pub /state_machine/config std_msgs/msg/String "{data: '{\"name\": \"test\", \"initial_state\": \"idle\", \"states\": [{\"name\": \"idle\", \"entry_condition\": {\"type\": \"always\"}, \"task\": {\"type\": \"none\"}}], \"transitions\": []}'}"
```

### 4. Monitor State Machine

- **Web Interface**: View current state and events in real-time
- **Command Line**:
  ```bash
  # Watch status
  ros2 topic echo /state_machine/status

  # Watch events
  ros2 topic echo /state_machine/events
  ```

## Example Configurations

### Simple Two-State Machine

```json
{
  "name": "Simple Two-State Machine",
  "initial_state": "idle",
  "states": [
    {
      "name": "idle",
      "description": "Robot is idle with blue LED",
      "entry_condition": {
        "type": "always",
        "params": {}
      },
      "task": {
        "type": "set_led",
        "params": {"color": "blue"}
      }
    },
    {
      "name": "active",
      "description": "Robot is moving forward",
      "entry_condition": {
        "type": "timer",
        "params": {"duration": 3.0}
      },
      "task": {
        "type": "roll",
        "params": {"heading": 0, "speed": 100}
      },
      "timeout": 5.0
    }
  ],
  "transitions": [
    {
      "source": "idle",
      "destination": "active",
      "trigger": "auto"
    },
    {
      "source": "active",
      "destination": "idle",
      "trigger": "auto"
    }
  ]
}
```

### Patrol Pattern

```json
{
  "name": "Patrol Pattern",
  "initial_state": "north",
  "states": [
    {
      "name": "north",
      "description": "Moving north",
      "entry_condition": {"type": "always", "params": {}},
      "task": {"type": "roll", "params": {"heading": 0, "speed": 80}},
      "timeout": 3.0
    },
    {
      "name": "east",
      "description": "Moving east",
      "entry_condition": {"type": "timer", "params": {"duration": 3.0}},
      "task": {"type": "roll", "params": {"heading": 90, "speed": 80}},
      "timeout": 3.0
    },
    {
      "name": "south",
      "description": "Moving south",
      "entry_condition": {"type": "timer", "params": {"duration": 3.0}},
      "task": {"type": "roll", "params": {"heading": 180, "speed": 80}},
      "timeout": 3.0
    },
    {
      "name": "west",
      "description": "Moving west",
      "entry_condition": {"type": "timer", "params": {"duration": 3.0}},
      "task": {"type": "roll", "params": {"heading": 270, "speed": 80}},
      "timeout": 3.0
    }
  ],
  "transitions": [
    {"source": "north", "destination": "east", "trigger": "auto"},
    {"source": "east", "destination": "south", "trigger": "auto"},
    {"source": "south", "destination": "west", "trigger": "auto"},
    {"source": "west", "destination": "north", "trigger": "auto"}
  ]
}
```

## Integration with Sphero

To integrate with actual Sphero control, you need to subscribe to the `/state_machine/task_command` topic in your Sphero controller and execute the tasks:

```python
def task_command_callback(self, msg):
    """Handle state machine task commands."""
    try:
        task = json.loads(msg.data)
        task_type = task.get('task_type')
        params = task.get('params', {})

        if task_type == 'set_led':
            self.set_led(params.get('color', 'white'))
        elif task_type == 'roll':
            self.roll(params.get('heading', 0), params.get('speed', 0))
        # Add more task handlers...
    except Exception as e:
        self.get_logger().error(f'Error executing task: {e}')
```

## Troubleshooting

### State Machine Not Transitioning

- Check that entry conditions are properly configured
- Verify sensor data is being published to `/state_machine/sensor_data`
- Check event log for condition evaluation details

### Tasks Not Executing

- Ensure a handler is subscribed to `/state_machine/task_command`
- Verify task parameters are correct
- Check logs for error messages

### Web Interface Not Loading

- Verify web server is running on port 5000
- Check browser console for JavaScript errors
- Ensure Flask templates are installed correctly

## Advanced Usage

### Custom Entry Conditions

You can extend the state machine controller to support custom condition types by modifying the `check_entry_condition` method in `state_machine_controller.py`.

### Sensor Data Publishing

Publish sensor data for condition evaluation:

```bash
ros2 topic pub /state_machine/sensor_data std_msgs/msg/String "{data: '{\"battery_low\": false, \"obstacle_detected\": true}'}"
```

## Future Enhancements

- [ ] Add visual state diagram editor
- [ ] Support for parallel states
- [ ] State history and analytics
- [ ] Export/import configurations
- [ ] Custom condition plugins
- [ ] Integration with ROS2 actions

## License

This package is part of the Sphero ROS2 workspace.

---

**Generated with Claude Code** 🤖
