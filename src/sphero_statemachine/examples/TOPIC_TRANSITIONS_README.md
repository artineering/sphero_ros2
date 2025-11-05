# Topic-Based Transitions in State Machine Controller

This document explains the new topic-based transition capabilities added to the Dynamic State Machine Controller.

## Overview

The state machine controller now supports transitions triggered by ROS topic messages. This allows the robot's behavior to respond dynamically to external commands, sensor data, or any other ROS topic.

## Transition Condition Types

### 1. Auto (Default)
The original behavior - uses the destination state's entry condition.

```json
{
  "source": "idle",
  "destination": "moving",
  "trigger": "auto"
}
```

Or simply omit the `condition` field:
```json
{
  "source": "idle",
  "destination": "moving",
  "trigger": "auto"
}
```

### 2. Timer
Transition after a specified duration in the source state.

```json
{
  "source": "turning",
  "destination": "idle",
  "trigger": "timer_return",
  "condition": {
    "type": "timer",
    "duration": 3.0
  }
}
```

**Parameters:**
- `duration` (float): Seconds to wait in source state before transitioning

### 3. Topic Value
Transition when a specific field in a topic message matches a value.

```json
{
  "source": "idle",
  "destination": "moving_forward",
  "trigger": "command_received",
  "condition": {
    "type": "topic_value",
    "topic": "/robot_command",
    "msg_type": "std_msgs/String",
    "field_path": "data",
    "operator": "==",
    "value": "forward"
  }
}
```

**Parameters:**
- `topic` (string): ROS topic to subscribe to
- `msg_type` (string): Message type in format "package/Type" (e.g., "std_msgs/String")
- `field_path` (string, optional): Dot-notation path to field (e.g., "data", "linear.x", "pose.position.x")
- `operator` (string): Comparison operator: `==`, `!=`, `>`, `<`, `>=`, `<=`
- `value`: Value to compare against

**Examples:**

Numeric comparison:
```json
{
  "condition": {
    "type": "topic_value",
    "topic": "/battery_level",
    "msg_type": "std_msgs/Float32",
    "field_path": "data",
    "operator": "<",
    "value": 20.0
  }
}
```

Nested field access:
```json
{
  "condition": {
    "type": "topic_value",
    "topic": "/robot_pose",
    "msg_type": "geometry_msgs/PoseStamped",
    "field_path": "pose.position.x",
    "operator": ">",
    "value": 5.0
  }
}
```

### 4. Topic Message
Transition when any message is received on a topic (useful for triggers/events).

```json
{
  "source": "idle",
  "destination": "emergency_stop",
  "trigger": "emergency",
  "condition": {
    "type": "topic_message",
    "topic": "/emergency_stop",
    "msg_type": "std_msgs/Bool",
    "timeout": 0.5
  }
}
```

**Parameters:**
- `topic` (string): ROS topic to subscribe to
- `msg_type` (string): Message type in format "package/Type"
- `timeout` (float, optional): Maximum age of message in seconds (if not specified, any received message counts)

## Complete Example

See [topic_triggered_state_machine.json](./topic_triggered_state_machine.json) for a complete example that demonstrates:

1. **Command-based transitions**: Robot moves based on commands published to `/robot_command`
2. **Timer-based transitions**: Robot returns to idle after turning for 3 seconds
3. **Emergency stop**: Any message on `/emergency_stop` immediately triggers emergency state
4. **Multiple transitions from same source**: Different commands trigger different states

### Running the Example

1. Start the state machine controller:
```bash
ros2 run sphero_statemachine state_machine_controller
```

2. Load the configuration:
```bash
ros2 topic pub --once /state_machine/config std_msgs/String \
  "{data: '$(cat topic_triggered_state_machine.json)'}"
```

3. Send commands to control the robot:
```bash
# Move forward
ros2 topic pub /robot_command std_msgs/String "{data: 'forward'}"

# Turn
ros2 topic pub /robot_command std_msgs/String "{data: 'turn'}"

# Stop
ros2 topic pub /robot_command std_msgs/String "{data: 'stop'}"

# Emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}"

# Reset after emergency
ros2 topic pub /robot_command std_msgs/String "{data: 'reset'}"
```

4. Monitor state machine status:
```bash
ros2 topic echo /state_machine/status
```

## Implementation Details

### Dynamic Topic Subscriptions

The state machine controller automatically:
- Creates subscriptions for all topics referenced in transition conditions
- Caches the latest value from each topic
- Tracks message receive timestamps
- Cleans up subscriptions when configuration changes

### Message Type Resolution

The controller uses Python's `importlib` to dynamically import message types at runtime. This means:
- Any standard or custom ROS message type can be used
- The message type must be available in your ROS workspace
- Format is always "package_name/MessageType" (e.g., "geometry_msgs/Twist")

### Field Path Extraction

For `topic_value` conditions, the `field_path` parameter supports nested field access:
- Simple: `"data"` for std_msgs
- Nested: `"linear.x"` for Twist messages
- Deep nesting: `"pose.pose.position.x"` for Odometry messages

### Performance Considerations

- Topic subscriptions are created once during configuration
- Message callbacks are lightweight (just store latest value)
- Transition conditions are checked at 10 Hz (configurable via update timer)
- Only creates subscriptions for topics actually used in conditions

## Backward Compatibility

All existing state machine configurations continue to work without modification:
- Transitions without `condition` field use "auto" behavior
- State entry conditions still work as before
- Old timer-based state transitions remain functional

## Common Patterns

### Voice/Text Commands
```json
{
  "condition": {
    "type": "topic_value",
    "topic": "/voice_command",
    "msg_type": "std_msgs/String",
    "field_path": "data",
    "operator": "==",
    "value": "go"
  }
}
```

### Button Press Detection
```json
{
  "condition": {
    "type": "topic_message",
    "topic": "/button_pressed",
    "msg_type": "std_msgs/Empty"
  }
}
```

### Threshold-Based Transitions
```json
{
  "condition": {
    "type": "topic_value",
    "topic": "/distance_sensor",
    "msg_type": "sensor_msgs/Range",
    "field_path": "range",
    "operator": "<",
    "value": 0.3
  }
}
```

### Velocity-Based State Changes
```json
{
  "condition": {
    "type": "topic_value",
    "topic": "/cmd_vel",
    "msg_type": "geometry_msgs/Twist",
    "field_path": "linear.x",
    "operator": ">",
    "value": 0.0
  }
}
```

## Troubleshooting

### Topic not triggering transition
- Verify topic is being published: `ros2 topic echo <topic_name>`
- Check message type matches: `ros2 topic info <topic_name>`
- Enable debug logging to see condition checks
- Verify field_path is correct for the message type

### Message type import errors
- Ensure the message package is installed
- Check the format is "package/Type" not "package/msg/Type"
- Verify the package is sourced in your ROS environment

### Validation errors
- Check all required parameters are present
- Verify operators are valid (==, !=, >, <, >=, <=)
- Ensure source and destination states exist
- Review logs for specific validation failures
