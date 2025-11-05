# Collision Avoidance State Machine

This state machine demonstrates obstacle detection and avoidance behavior for the Sphero robot.

## States

### 1. Idle State
- **LED**: Green (main LED)
- **Behavior**: Robot is stopped and waiting for commands
- **Transitions**: Waits for "forward" command on `/robot_command`

### 2. Moving State
- **Collision Detection**: Enabled (obstacle mode)
- **LED**: Main LED changes based on movement
- **Matrix**: Green fill pattern to indicate forward movement
- **Behavior**: Robot rolls forward at heading 0° and speed 80
- **Transitions**:
  - To `collision` state when obstacle detected on `/sphero/obstacle`
  - To `idle` state when "stop" command received

### 3. Collision State
- **LED**: Red (warning)
- **Matrix**: Yellow "X" pattern to indicate obstacle
- **Behavior**:
  - Stops immediately
  - Shows warning display for 2 seconds
  - Automatically returns to moving state (which will need heading adjustment)
- **Transitions**:
  - Auto-return to `moving` after 2 seconds
  - To `idle` state when "stop" command received

## Usage

### Start the System
```bash
# Terminal 1: Sphero Task Controller
ros2 run sphero_task_controller task_controller_node

# Terminal 2: State Machine Controller
ros2 run sphero_statemachine state_machine_controller

# Terminal 3: Task Executor
ros2 run sphero_statemachine task_executor

# Terminal 4: Sphero Node (or simulator)
ros2 run sphero_package sphero_node
```

### Load Configuration
Via web interface or:
```bash
ros2 topic pub --once /state_machine/config std_msgs/String \
  "{data: '$(cat collision_avoidance_state_machine.json)'}"
```

### Send Commands
```bash
# Start moving forward
ros2 topic pub --once /robot_command std_msgs/String "{data: 'forward'}"

# Stop
ros2 topic pub --once /robot_command std_msgs/String "{data: 'stop'}"

# Simulate obstacle detection (for testing)
ros2 topic pub --once /sphero/obstacle std_msgs/String "{data: 'detected'}"
```

### Monitor
```bash
# Watch state changes
ros2 topic echo /state_machine/status

# Watch events
ros2 topic echo /state_machine/events
```

## Limitations & Future Enhancements

### Current Limitations
1. **No heading adjustment**: The collision state doesn't calculate a new heading. When it returns to moving, it continues in the same direction.
2. **No random offset**: Cannot generate random values within the state machine configuration.
3. **No command publishing**: State machine tasks cannot publish to `/robot_command` topic.

### Possible Solutions

#### Option 1: External Collision Handler Node
Create a dedicated `collision_handler_node` that:
- Subscribes to `/sphero/obstacle` events
- Calculates reverse heading with random offset
- Publishes new heading to a topic like `/collision/new_heading`
- State machine can read this heading and use it

#### Option 2: Enhanced Task Types
Add new task types to support:
```json
{
  "type": "calculate_reverse_heading",
  "params": {
    "offset_range": [-45, 45],
    "store_as": "new_heading"
  }
}
```

#### Option 3: Parameterized Moving State
Make the moving state accept heading from a topic:
```json
{
  "name": "moving",
  "tasks": [
    {
      "type": "roll",
      "params": {
        "heading_topic": "/desired_heading",
        "speed": 80
      }
    }
  ]
}
```

## Example Flow

```
[Initial] → idle (Green LED, stopped)
          ↓ (receive "forward")
          moving (Green matrix, rolling forward)
          ↓ (obstacle detected)
          collision (Red LED, Yellow X, stopped)
          ↓ (after 2s)
          moving (continues forward - same heading)
          ↓ (receive "stop")
          idle (Green LED, stopped)
```

## Notes

- The `/sphero/obstacle` topic needs to be published by the sphero_node when collision detection is active
- Matrix patterns available: "smile", "frown", "x", "heart", "arrow_up", "arrow_down", "arrow_left", "arrow_right", "fill"
- Speed range: 0-255 (recommended: 50-100 for indoor use)
- Heading range: 0-359 degrees (0=forward from calibrated position)

## Testing Without Real Collisions

You can test the collision avoidance by manually publishing obstacle events:

```bash
# Start the robot moving
ros2 topic pub --once /robot_command std_msgs/String "{data: 'forward'}"

# Wait a few seconds, then simulate collision
ros2 topic pub --once /sphero/obstacle std_msgs/String "{data: 'detected'}"

# Watch the robot stop, show warning, then continue
```
