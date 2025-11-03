# Sphero Task Controller

High-level task-based controller for Sphero robots. This package provides a task queue system that accepts complex tasks via a ROS2 topic and executes them using the `sphero_controller_node`.

**Created with assistance from Claude Code (Anthropic)**

## Features

- **Task Queue System**: Queue multiple tasks for sequential execution
- **9 Built-in Task Types**: Move, patrol, circle, square, LED sequences, matrix sequences, spin, stop, and custom
- **State Feedback**: Monitors Sphero state for closed-loop control
- **Task Status Publishing**: Real-time task status updates
- **JSON-based Task Format**: Easy to create and send tasks programmatically

## Supported Task Types

### 1. **move_to** - Move to a specific position
Move the Sphero to a target (x, y) coordinate.

```json
{
    "task_type": "move_to",
    "parameters": {
        "x": 100.0,
        "y": 50.0,
        "speed": 100
    }
}
```

### 2. **patrol** - Patrol between waypoints
Move through a series of waypoints, optionally looping.

```json
{
    "task_type": "patrol",
    "parameters": {
        "waypoints": [
            {"x": 0, "y": 0},
            {"x": 100, "y": 0},
            {"x": 100, "y": 100},
            {"x": 0, "y": 100}
        ],
        "speed": 80,
        "loop": true
    }
}
```

### 3. **circle** - Move in a circular pattern
Move in a circle for a specified duration.

```json
{
    "task_type": "circle",
    "parameters": {
        "radius": 50,
        "speed": 100,
        "duration": 10.0
    }
}
```

### 4. **square** - Move in a square pattern
Automatically generate and follow a square path.

```json
{
    "task_type": "square",
    "parameters": {
        "side_length": 100,
        "speed": 80
    }
}
```

### 5. **led_sequence** - Execute LED color sequence
Display a sequence of LED colors with timing control.

```json
{
    "task_type": "led_sequence",
    "parameters": {
        "sequence": [
            {"red": 255, "green": 0, "blue": 0},
            {"red": 0, "green": 255, "blue": 0},
            {"red": 0, "green": 0, "blue": 255}
        ],
        "interval": 1.0,
        "loop": false
    }
}
```

### 6. **matrix_sequence** - Matrix pattern sequence (BOLT only)
Display a sequence of LED matrix patterns.

```json
{
    "task_type": "matrix_sequence",
    "parameters": {
        "sequence": [
            {"pattern": "smile", "red": 255, "green": 255, "blue": 0},
            {"pattern": "heart", "red": 255, "green": 0, "blue": 0},
            {"pattern": "star", "red": 0, "green": 255, "blue": 255}
        ],
        "interval": 2.0,
        "loop": true
    }
}
```

### 7. **spin** - Spin in place
Rotate the Sphero in place for a number of rotations.

```json
{
    "task_type": "spin",
    "parameters": {
        "rotations": 2,
        "speed": 100
    }
}
```

### 8. **stop** - Stop the Sphero
Immediately stop all movement. If a task is currently running, a STOP task with `delay: 0.0` will cancel it immediately.

```json
{
    "task_type": "stop",
    "parameters": {
        "delay": 0.0
    }
}
```

**Parameters:**
- `delay` (optional, default: 0.0): If 0.0, the STOP task will immediately cancel any currently running task. If greater than 0, it will wait for the current task to complete before executing.

**Behavior:**
- When a STOP task with `delay: 0.0` is received, it will cancel any looping tasks (like `led_sequence` with `loop: true` or `patrol` with `loop: true`)
- Essential for interrupting long-running or infinite loop tasks

### 9. **custom** - Custom command sequence
Execute a custom sequence of commands with precise timing.

```json
{
    "task_type": "custom",
    "parameters": {
        "commands": [
            {
                "type": "led",
                "red": 255,
                "green": 0,
                "blue": 0,
                "duration": 1.0
            },
            {
                "type": "roll",
                "heading": 0,
                "speed": 100,
                "duration": 2.0
            },
            {
                "type": "matrix",
                "pattern": "smile",
                "red": 255,
                "green": 255,
                "blue": 0,
                "duration": 2.0
            },
            {
                "type": "stop",
                "duration": 0.5
            }
        ]
    }
}
```

## Installation

### Prerequisites
- ROS2 (Humble/Rolling/Iron)
- `sphero_package` must be installed and built

### Building
```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_task_controller
source install/setup.bash
```

## Usage

### 1. Start the sphero_controller_node
```bash
# Terminal 1: Start controller for your Sphero
ros2 run sphero_package sphero_controller_node.py --ros-args -p toy_name:=SB-3660
```

### 2. Start the task_controller
```bash
# Terminal 2: Start task controller
ros2 run sphero_task_controller task_controller
```

### 3. Send Tasks
```bash
# Terminal 3: Send a task
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"move_to\",
    \"parameters\": {\"x\": 100, \"y\": 50, \"speed\": 100}
}'}"
```

### Monitor Task Status
```bash
# Watch task status updates
ros2 topic echo /sphero/task/status
```

## ROS2 Topics

### Subscribed Topics
- `/sphero/task` (std_msgs/String): Task commands in JSON format
- `/sphero/state` (std_msgs/String): Sphero state for feedback control

### Published Topics
- `/sphero/led` (std_msgs/String): LED color commands
- `/sphero/roll` (std_msgs/String): Roll movement commands
- `/sphero/heading` (std_msgs/String): Heading commands
- `/sphero/speed` (std_msgs/String): Speed commands
- `/sphero/stop` (std_msgs/String): Stop commands
- `/sphero/matrix` (std_msgs/String): Matrix pattern commands
- `/sphero/task/status` (std_msgs/String): Task status updates

## Task JSON Format

### Required Fields
```json
{
    "task_type": "move_to|patrol|circle|...",
    "parameters": {
        // Task-specific parameters
    }
}
```

### Optional Fields
```json
{
    "task_id": "unique_task_identifier",  // Auto-generated if not provided
    "task_type": "move_to",
    "parameters": {...}
}
```

### Task Status Format
The `/sphero/task/status` topic publishes detailed status information for each task:

```json
{
    "task_id": "task_1730123456789",
    "task_type": "move_to",
    "parameters": {"x": 100, "y": 50, "speed": 100},
    "status": "pending|running|completed|failed|cancelled",
    "created_at": 1730123456.789,
    "started_at": 1730123457.123,
    "completed_at": 1730123460.456,
    "error_message": null,
    "queue_length": 2,
    "has_current_task": true,
    "total_pending": 3
}
```

**Status Fields:**
- `task_id`: Unique identifier for the task
- `task_type`: Type of task being executed
- `status`: Current status (pending, running, completed, failed, cancelled)
- `queue_length`: Number of tasks waiting in queue (not including current task)
- `has_current_task`: Whether a task is currently executing
- `total_pending`: Total number of tasks (queue + current task if any)

## Examples

### Example 1: Move to Position
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"move_to\",
    \"parameters\": {\"x\": 100, \"y\": 100, \"speed\": 80}
}'}"
```

### Example 2: LED Rainbow Sequence
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"led_sequence\",
    \"parameters\": {
        \"sequence\": [
            {\"red\": 255, \"green\": 0, \"blue\": 0},
            {\"red\": 255, \"green\": 127, \"blue\": 0},
            {\"red\": 255, \"green\": 255, \"blue\": 0},
            {\"red\": 0, \"green\": 255, \"blue\": 0},
            {\"red\": 0, \"green\": 0, \"blue\": 255},
            {\"red\": 75, \"green\": 0, \"blue\": 130},
            {\"red\": 148, \"green\": 0, \"blue\": 211}
        ],
        \"interval\": 0.5,
        \"loop\": true
    }
}'}"
```

### Example 3: Square Patrol
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"patrol\",
    \"parameters\": {
        \"waypoints\": [
            {\"x\": 0, \"y\": 0},
            {\"x\": 100, \"y\": 0},
            {\"x\": 100, \"y\": 100},
            {\"x\": 0, \"y\": 100}
        ],
        \"speed\": 80,
        \"loop\": true
    }
}'}"
```

### Example 4: Python Script
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class TaskSender(Node):
    def __init__(self):
        super().__init__('task_sender')
        self.publisher = self.create_publisher(String, 'sphero/task', 10)

    def send_task(self, task_type, parameters):
        task = {
            'task_type': task_type,
            'parameters': parameters
        }
        msg = String()
        msg.data = json.dumps(task)
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent task: {task_type}')

def main():
    rclpy.init()
    sender = Task Sender()

    # Send move task
    sender.send_task('move_to', {'x': 100, 'y': 50, 'speed': 100})

    # Send LED sequence
    sender.send_task('led_sequence', {
        'sequence': [
            {'red': 255, 'green': 0, 'blue': 0},
            {'red': 0, 'green': 255, 'blue': 0},
            {'red': 0, 'green': 0, 'blue': 255}
        ],
        'interval': 1.0,
        'loop': False
    })

    rclpy.spin(sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration

### Default Parameters
- `default_speed`: 100 (0-255)
- `position_tolerance`: 10.0 cm
- `heading_tolerance`: 5 degrees
- Task execution rate: 10 Hz

These can be modified in the `SpheroTaskController.__init__()` method.

## Architecture

```
Task Publisher → /sphero/task → Task Controller → /sphero/* commands → sphero_controller_node → Sphero Robot
                                      ↓
                                 Task Queue
                                      ↓
                               Task Executor
                                      ↓
                            /sphero/task/status
```

## Troubleshooting

### Task Not Executing
1. **Check controller is running**: `ros2 node list | grep sphero_controller`
2. **Check task topic**: `ros2 topic echo /sphero/task`
3. **Monitor task status**: `ros2 topic echo /sphero/task/status`
4. **Check logs**: Look at task_controller terminal output

### Position-based Tasks Not Working
- Ensure Sphero location tracking is enabled in sphero_controller
- Position is estimated from velocity integration
- Reset position by restarting sphero_controller

### Tasks Fail Immediately
- Check JSON format is valid
- Ensure all required parameters are provided
- Check task status for error_message field

## Advanced Usage

### Interactive Task Sender
The package includes an interactive Python script for easily sending tasks:

```bash
# Terminal 3: Run the interactive task sender
cd ~/ros2_ws_2/src/sphero_task_controller/examples
python3 task_sender.py
```

**Features:**
- Menu-driven interface for all 9 task types
- Real-time task status monitoring
- Send multiple tasks consecutively without restarting
- Visual feedback with colored output
- Background ROS2 spinning for continuous operation

**Usage:**
1. Start the task sender
2. Select a task from the menu (0-9)
3. The menu will reappear after sending, allowing you to queue more tasks
4. Monitor task status in real-time
5. Use option 9 to send STOP command (cancels current task)
6. Use option 0 to exit

### Queue Multiple Tasks
Tasks are automatically queued and executed sequentially:

```bash
# Send task 1
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{\"task_type\": \"circle\", \"parameters\": {\"duration\": 5}}'}"

# Send task 2 (will execute after task 1)
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{\"task_type\": \"square\", \"parameters\": {\"side_length\": 100}}'}"

# Send task 3
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{\"task_type\": \"stop\", \"parameters\": {}}'}"
```

### Monitor All Topics
```bash
# Launch in separate terminals
ros2 topic echo /sphero/task
ros2 topic echo /sphero/task/status
ros2 topic echo /sphero/state
```

## Future Enhancements

Potential additions:
- Task priority system
- ~~Task cancellation~~ ✓ Implemented (STOP task with delay=0.0)
- Task groups (parallel execution)
- Conditional task execution
- Path planning integration
- Obstacle avoidance
- Multi-robot coordination

## License

MIT License

## Author

Siddharth Vaghela (siddharth.vaghela@tufts.edu)

**Created with assistance from Claude Code (Anthropic)**

## See Also

- [sphero_package](../sphero_package/README.md): Core Sphero controller
- [sphero_web_interface](../sphero_web_interface/README.md): Web-based control interface
- [Workspace README](../../README.md): Overall workspace documentation
