# Sphero Instance Controller - Architecture Diagram

## Overview

This document provides a visual representation of the ROS2 node and topic architecture for the `sphero_instance_controller` package.

## Node and Topic Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Sphero Instance Controller Architecture                  │
│                            (for Sphero: SB-3660)                             │
└─────────────────────────────────────────────────────────────────────────────┘

                            External Publishers
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
                    ▼               ▼               ▼
            /sphero/SB-3660/led      /sphero/SB-3660/roll      /sphero/SB-3660/task
            /sphero/SB-3660/spin     /sphero/SB-3660/stop      /sphero/SB-3660/state_machine/config
            /sphero/SB-3660/raw_motor
                    │               │               │
                    └───────────────┼───────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────────┐      ┌────────────────────┐     ┌──────────────────────┐
│  Device Controller│      │  Task Controller   │     │ State Machine        │
│      Node         │      │      Node          │     │ Controller Node      │
├───────────────────┤      ├────────────────────┤     ├──────────────────────┤
│                   │      │                    │     │                      │
│ Provides:         │      │ Provides:          │     │ Provides:            │
│ • LED control     │      │ • High-level tasks │     │ • State management   │
│ • Roll commands   │      │ • move_to          │     │ • Transitions        │
│ • Heading control │      │ • patrol           │     │ • Entry conditions   │
│ • Speed control   │      │ • circle           │     │ • Dynamic behaviors  │
│ • Raw motor ctrl  │      │ • LED sequences    │     │ • Orchestration      │
│ • Matrix display  │      │ • Matrix sequences │     │                      │
│ • Collision det.  │      │ • Complex moves    │     │                      │
│ • Stabilization   │      │                    │     │                      │
│                   │      │                    │     │                      │
│ Subscribes to:    │      │ Subscribes to:     │     │ Subscribes to:       │
│ • /led            │      │ • /task            │     │ • /state_machine/    │
│ • /roll           │      │ • /state           │     │   config             │
│ • /spin           │      │ • /reset_aim       │     │ • /sensors           │
│ • /heading        │      │                    │     │ • Dynamic topics     │
│ • /speed          │      │                    │     │                      │
│ • /raw_motor      │      │                    │     │                      │
│ • /stop           │      │                    │     │                      │
│ • /reset_aim      │      │                    │     │                      │
│ • /matrix         │      │                    │     │                      │
│ • /collision      │      │                    │     │                      │
│ • /stabilization  │      │                    │     │                      │
│                   │      │                    │     │                      │
│ Publishes:        │      │ Publishes:         │     │ Publishes:           │
│ • /sensors        │      │ • /task/status     │     │ • /state_machine/    │
│ • /state          │◄─────┼────────────────────┼─────┤   status             │
│ • /battery        │      │                    │     │ • /state_machine/    │
│ • /status         │      │                    │     │   events             │
│ • /tap            │      │                    │     │ • /task ────────────►│
│ • /obstacle       │      │                    │     │                      │
└─────┬─────────────┘      └──────┬─────────────┘     └──────────────────────┘
      │                           │                             │
      │                           │                             │
      ▼                           ▼                             ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                    Sphero Hardware (SpheroToy + SpheroEduAPI)              │
│                                                                            │
│  • Motor control (high-level + raw motor control)                          │
│  • LED control (main + back)                                               │
│  • LED matrix (8x8 for BOLT)                                               │
│  • IMU sensors (accelerometer, gyroscope, orientation)                     │
│  • Position tracking (x, y coordinates)                                    │
│  • Velocity tracking                                                       │
│  • Battery monitoring                                                      │
│  • Collision detection                                                     │
└───────────────────────────────────────────────────────────────────────────┘
```

## Topic Hierarchy

All topics are namespaced under `sphero/<sphero_name>/` to support multi-robot setups.

### Device Controller Topics

**Subscribed (Commands):**
```
/sphero/SB-3660/led              (String)  - LED color control
/sphero/SB-3660/roll             (String)  - Roll movement commands
/sphero/SB-3660/spin             (String)  - Spin commands
/sphero/SB-3660/heading          (String)  - Heading control
/sphero/SB-3660/speed            (String)  - Speed control
/sphero/SB-3660/raw_motor        (String)  - Raw motor control (left/right independent)
/sphero/SB-3660/stop             (String)  - Stop movement
/sphero/SB-3660/reset_aim        (String)  - Reset orientation
/sphero/SB-3660/matrix           (String)  - LED matrix display
/sphero/SB-3660/collision        (String)  - Collision detection config
/sphero/SB-3660/stabilization    (String)  - Stabilization control
```

**Published (Status/Sensors):**
```
/sphero/SB-3660/sensors          (SpheroSensor)  - Sensor data
/sphero/SB-3660/state            (String)        - Complete state JSON
/sphero/SB-3660/battery          (BatteryState)  - Battery status
/sphero/SB-3660/status           (String)        - Health heartbeat
/sphero/SB-3660/tap              (String)        - Tap detection events
/sphero/SB-3660/obstacle         (String)        - Collision events
```

### Task Controller Topics

**Subscribed:**
```
/sphero/SB-3660/task             (String)  - Task commands (JSON)
/sphero/SB-3660/state            (String)  - State feedback
/sphero/SB-3660/reset_aim        (String)  - Position reset
```

**Published:**
```
/sphero/SB-3660/task/status      (String)  - Task execution status
```

### State Machine Controller Topics

**Subscribed:**
```
/sphero/SB-3660/state_machine/config  (String)        - Configuration (JSON)
/sphero/SB-3660/sensors               (SpheroSensor)  - Sensor data for conditions
/sphero/SB-3660/<dynamic>             (Various)       - Dynamic topic subscriptions
```

**Published:**
```
/sphero/SB-3660/state_machine/status  (String)  - Current state/status
/sphero/SB-3660/state_machine/events  (String)  - State transition events
/sphero/SB-3660/task                  (String)  - Task commands to executor
```

## Data Flow Examples

### Example 1: Direct LED Control
```
User Command
    │
    └──> /sphero/SB-3660/led (JSON: {"red": 255, "green": 0, "blue": 0})
            │
            └──> Device Controller Node
                    │
                    └──> Sphero Hardware (LED turns red)
```

### Example 2: High-Level Task Execution
```
User Command
    │
    └──> /sphero/SB-3660/task (JSON: {"task_type": "move_to", "parameters": {...}})
            │
            └──> Task Controller Node
                    │
                    ├──> Calculates trajectory
                    ├──> Monitors position via /state
                    └──> Publishes status to /task/status
```

### Example 3: State Machine Orchestration
```
User Configuration
    │
    └──> /sphero/SB-3660/state_machine/config (JSON state machine definition)
            │
            └──> State Machine Controller Node
                    │
                    ├──> Monitors sensor data from /sensors
                    ├──> Evaluates entry conditions
                    ├──> Triggers transitions
                    ├──> Publishes events to /state_machine/events
                    └──> Publishes tasks to /task
                            │
                            └──> Task Controller Node
                                    │
                                    └──> Executes tasks on Sphero
```

### Example 4: Raw Motor Control
```
User Command
    │
    └──> /sphero/SB-3660/raw_motor (JSON: {
            "left_mode": "forward",
            "left_speed": 200,
            "right_mode": "forward",
            "right_speed": 150
         })
            │
            └──> Device Controller Node
                    │
                    └──> Sphero Hardware (Independent left/right motor control)
                            │
                            └──> Results in curved trajectory or rotation
```

### Example 5: Multi-Robot Setup
```
┌─────────────────────────────────────────────────────────────┐
│                     Multiple Spheros                         │
└─────────────────────────────────────────────────────────────┘

Sphero SB-3660:                    Sphero SB-1234:
    │                                  │
    ├─ /sphero/SB-3660/led            ├─ /sphero/SB-1234/led
    ├─ /sphero/SB-3660/task           ├─ /sphero/SB-1234/task
    ├─ /sphero/SB-3660/sensors        ├─ /sphero/SB-1234/sensors
    └─ /sphero/SB-3660/state          └─ /sphero/SB-1234/state

No topic cross-talk! Each robot has its own namespace.
```

## Core Class Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    Core Classes (ROS-independent)             │
└──────────────────────────────────────────────────────────────┘

┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Sphero         │     │  TaskExecutor    │     │  StateMachine   │
├─────────────────┤     ├──────────────────┤     ├─────────────────┤
│ • set_led()     │     │ • add_task()     │     │ • configure()   │
│ • roll()        │◄────┤ • process_tasks()│     │ • process()     │
│ • spin()        │     │ • execute_task() │     │ • transition()  │
│ • set_heading() │     │                  │     │ • check_cond()  │
│ • set_speed()   │     │ Task Types:      │     │                 │
│ • set_raw_motor │     │ • move_to        │     │ Conditions:     │
│   _speed()      │     │ • patrol         │     │ • always        │
│ • stop()        │     │ • circle         │     │ • timer         │
│ • reset_aim()   │     │ • square         │     │ • topic_value   │
│ • set_matrix()  │     │ • led_sequence   │     │                 │
│ • collision()   │     │ • matrix_seq     │     │ Transitions:    │
│ • sensors()     │     │ • spin           │     │ • auto          │
│ • state()       │     │ • stop           │     │ • timer         │
└─────────────────┘     │ • custom         │     │ • topic_value   │
                        └──────────────────┘     │ • topic_message │
                                                 └─────────────────┘

        ▲                       ▲                       ▲
        │                       │                       │
        └───────────────────────┴───────────────────────┘
                                │
                    Used by ROS2 Nodes (thin wrappers)
```

## Message Types

### Custom Messages

```
SpheroSensor.msg:
  - float32 pitch, roll, yaw
  - float32 accel_x, accel_y, accel_z
  - float32 gyro_x, gyro_y, gyro_z
  - float32 x, y
  - float32 velocity_x, velocity_y
  - float32 battery_percentage

SpheroCommand.msg:
  - string command_type
  - string parameters (JSON)

SpheroLED.msg:
  - uint8 red, green, blue
  - string led_type

SpheroRoll.msg:
  - int16 heading
  - int16 speed
  - float32 duration

... (and 5 more message types)
```

## Deployment Scenarios

### Scenario 1: Basic Control
```bash
# Launch device controller only
ros2 run sphero_instance_controller sphero_instance_device_controller_node.py \
  --ros-args -p sphero_name:=SB-3660
```

### Scenario 2: High-Level Tasks
```bash
# Launch task controller (automatically communicates with device controller)
ros2 run sphero_instance_controller sphero_instance_task_controller_node.py \
  --ros-args -p sphero_name:=SB-3660
```

### Scenario 3: State Machine Behaviors
```bash
# Terminal 1: Task controller
ros2 run sphero_instance_controller sphero_instance_task_controller_node.py \
  --ros-args -p sphero_name:=SB-3660

# Terminal 2: State machine controller
ros2 run sphero_instance_controller sphero_instance_statemachine_controller_node.py \
  --ros-args -p sphero_name:=SB-3660
```

### Scenario 4: Multi-Robot Fleet
```bash
# Launch multiple instances with different sphero_name parameters
ros2 launch multi_sphero.launch.py
```

## Key Design Principles

1. **Namespacing**: All topics prefixed with `sphero/<sphero_name>/` for multi-robot support
2. **Modularity**: Core logic separated from ROS interface
3. **Reusability**: Core classes can be used independently of ROS
4. **Layering**: Device → Task → State Machine (increasing abstraction)
5. **Independence**: Each controller can run standalone or combined
6. **Self-contained**: Package includes own message definitions

## Performance Characteristics

- **Sensor Publishing Rate**: 10 Hz (configurable)
- **Heartbeat Rate**: 5 seconds (configurable)
- **Task Processing**: 10 Hz
- **State Machine Updates**: 10 Hz
- **Status Publishing**: 1 Hz

## Dependencies

```
ROS2 Packages:
  - rclpy
  - std_msgs
  - sensor_msgs
  - builtin_interfaces

Python Packages:
  - spherov2 (Sphero SDK)

Build System:
  - ament_cmake
  - ament_cmake_python
  - rosidl_default_generators
```
