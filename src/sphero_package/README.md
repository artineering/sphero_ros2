# Sphero Package

A ROS 2 package for controlling Sphero robots using the spherov2 Python library.

## Overview

This package provides a ROS 2 node that connects to and controls a Sphero robot. It uses the spherov2 library to communicate with Sphero devices via Bluetooth.

## Dependencies

### System Dependencies
- ROS 2 (tested on your current ROS 2 distribution)
- Python 3
- Bluetooth support

### Python Dependencies
- `rclpy` - ROS 2 Python client library
- `spherov2` - Sphero robot control library

## Installation

1. Clone this package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws_2/src
   ```

2. Install the spherov2 Python library:
   ```bash
   pip install spherov2
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws_2
   colcon build --packages-select sphero_package
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Node

To run the Sphero node:

```bash
ros2 run sphero_package sphero_node
```

### Configuration

The node currently connects to a Sphero robot named "SB-3660". To change the target robot, modify the `toy_name` parameter in the `main()` function of `sphero_node.py`:

```python
robot = scanner.find_toy(toy_name="YOUR-ROBOT-NAME")
```

### Stopping the Node

Press `Ctrl+C` to gracefully shutdown the node. The cleanup process will:
1. Turn off the Sphero's LED
2. Disconnect from the robot
3. Shutdown the ROS 2 node

## Features

- Automatic connection to Sphero robot by name
- LED control (currently sets LED to red on startup, turns off on shutdown)
- Graceful shutdown with proper cleanup
- ROS 2 logging support

## Package Structure

```
sphero_package/
├── sphero_package/
│   ├── __init__.py
│   └── sphero_node.py      # Main node implementation
├── resource/
│   └── sphero_package
├── test/
├── package.xml             # Package manifest
├── setup.py                # Python package setup
├── setup.cfg               # Setup configuration
└── README.md               # This file
```

## Node Details

### sphero_node

**Description:** Main ROS 2 node for Sphero robot control

**Published Topics:** None (currently)

**Subscribed Topics:** None (currently)

**Parameters:** None (currently)

## Troubleshooting

### Cannot find Sphero robot
- Ensure Bluetooth is enabled on your system
- Make sure the Sphero is powered on and in pairing mode
- Verify the robot name matches the one specified in the code

### Connection issues
- Try resetting the Sphero robot
- Check that no other applications are connected to the robot
- Ensure you have the necessary Bluetooth permissions

### Keyboard interrupt not working
- The node uses signal handling to ensure graceful shutdown
- If the node hangs, you may need to force kill it with `Ctrl+Z` followed by `kill %1`

## Future Development

Potential features to add:
- ROS 2 topics for velocity control
- Service calls for LED control
- Odometry publishing
- IMU data publishing
- Parameter configuration for robot name and connection settings

## License

TODO: License declaration

## Maintainer

siddharth.vaghela@tufts.edu
