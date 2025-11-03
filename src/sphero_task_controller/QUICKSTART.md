# Sphero Task Controller - Quick Start Guide

**Created with assistance from Claude Code (Anthropic)**

## Get Started in 3 Steps

### Step 1: Build the Package
```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_task_controller
source install/setup.bash
```

### Step 2: Start the Required Nodes

**Terminal 1 - Sphero Controller:**
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run sphero_package sphero_controller_node.py --ros-args -p toy_name:=SB-3660
```
Replace `SB-3660` with your Sphero's name.

**Terminal 2 - Task Controller:**
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run sphero_task_controller task_controller
```

### Step 3: Send Tasks

**Option A - Command Line:**
```bash
# Terminal 3
source ~/ros2_ws_2/install/setup.bash

# Move to position
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"move_to\",
    \"parameters\": {\"x\": 100, \"y\": 50, \"speed\": 100}
}'}"
```

**Option B - Interactive Python Script:**
```bash
# Terminal 3
source ~/ros2_ws_2/install/setup.bash
cd ~/ros2_ws_2/src/sphero_task_controller/examples
python3 task_sender.py
```

## Common Tasks

### Move to a Position
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"move_to\",
    \"parameters\": {\"x\": 100, \"y\": 100, \"speed\": 80}
}'}"
```

### Circle Pattern
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"circle\",
    \"parameters\": {\"radius\": 50, \"speed\": 100, \"duration\": 10.0}
}'}"
```

### Square Pattern
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"square\",
    \"parameters\": {\"side_length\": 100, \"speed\": 80}
}'}"
```

### LED Rainbow
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"led_sequence\",
    \"parameters\": {
        \"sequence\": [
            {\"red\": 255, \"green\": 0, \"blue\": 0},
            {\"red\": 255, \"green\": 127, \"blue\": 0},
            {\"red\": 255, \"green\": 255, \"blue\": 0},
            {\"red\": 0, \"green\": 255, \"blue\": 0},
            {\"red\": 0, \"green\": 0, \"blue\": 255}
        ],
        \"interval\": 0.5,
        \"loop\": true
    }
}'}"
```

### Stop
```bash
ros2 topic pub --once /sphero/task std_msgs/String "{data: '{
    \"task_type\": \"stop\",
    \"parameters\": {}
}'}"
```

## Monitor Task Status

```bash
# Watch task status updates
ros2 topic echo /sphero/task/status
```

You'll see output like:
```
data: '{"task_id": "task_1730123456789", "task_type": "move_to", "status": "running", ...}'
data: '{"task_id": "task_1730123456789", "task_type": "move_to", "status": "completed", ...}'
```

## Troubleshooting

### Task Not Executing
1. Check sphero_controller is running: `ros2 node list | grep sphero_controller`
2. Check task_controller is running: `ros2 node list | grep task_controller`
3. Monitor task topic: `ros2 topic echo /sphero/task`

### Sphero Not Moving
1. Ensure Sphero is connected (check sphero_controller terminal)
2. Check battery level is sufficient (>20%)
3. Verify task parameters are valid

### Position Tasks Not Working
- Position tracking requires velocity integration
- Start from a known position (e.g., after restart)
- Use relative movements or waypoints

## Next Steps

- Read the full [README.md](README.md) for all task types
- Check out [example_tasks.json](examples/example_tasks.json) for more examples
- Write your own task scripts in Python

## Quick Reference

| Task Type | Purpose | Key Parameters |
|-----------|---------|----------------|
| move_to | Move to position | x, y, speed |
| patrol | Follow waypoints | waypoints[], loop |
| circle | Circular motion | radius, duration |
| square | Square pattern | side_length |
| led_sequence | LED colors | sequence[], interval |
| matrix_sequence | Matrix patterns | sequence[], interval |
| spin | Rotate in place | rotations |
| stop | Stop movement | none |
| custom | Custom commands | commands[] |

---

**Created with assistance from Claude Code (Anthropic)**
