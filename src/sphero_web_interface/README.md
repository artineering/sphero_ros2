# Sphero Web Interface

A comprehensive web-based interface for controlling and monitoring Sphero robots through ROS2.

## Features

- **Connection Management**: Connect to Sphero robots by entering their name
- **Real-time State Monitoring**: View current state including battery, motion, and connection status
- **Sensor Readings**: Monitor accelerometer, gyroscope, velocity, and location data
- **LED Matrix Control**: Display various patterns on Sphero BOLT's LED matrix
- **Motion Control**: Control heading, speed, and movement with an intuitive interface

## Prerequisites

- ROS2 (tested with Humble/Iron)
- Python 3.8+
- Flask and Flask-SocketIO
- sphero_package (the ROS2 package with sphero_controller_node)

## Installation

### 1. Install Python Dependencies

```bash
pip3 install flask flask-socketio python-socketio
```

### 2. Build the Package

Navigate to your ROS2 workspace and build:

```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_web_interface
source install/setup.bash
```

## Usage

### Starting the Web Interface

Simply run the web server node:

```bash
ros2 run sphero_web_interface web_server
```

The web interface will be available at: **http://localhost:5000**

### Connecting to a Sphero

1. Open your web browser and navigate to http://localhost:5000
2. In the **Connection** tab, enter your Sphero's name (e.g., "SB-3660")
3. Click the **Connect** button
4. Wait for the controller to initialize and connect to your Sphero
5. Once connected, all other tabs will become enabled

### Using the Interface

#### State Tab
- View real-time connection status, battery level, and motion state
- Monitor the current LED color
- See raw state data in JSON format

#### Sensors Tab
- View live sensor readings:
  - Accelerometer (X, Y, Z axes in Gs)
  - Gyroscope (X, Y, Z axes in °/s)
  - Velocity (X, Y in cm/s)
  - Location (X, Y in cm)

#### Matrix Tab (for Sphero BOLT)
- Select from 15 predefined patterns:
  - Emotions: Smile, Frown
  - Symbols: Heart, Star, Circle, Square
  - Arrows: Up, Down, Left, Right
  - Icons: Checkmark, Cross, Plus, Exclamation, Question
- Customize the matrix color using RGB sliders
- See a live preview of your selected color

#### Motion Tab
- **LED Control**: Set the main LED color using RGB sliders
- **Heading Control**:
  - Use the visual compass to set direction
  - Drag the slider or click to set heading (0-359°)
- **Speed Control**: Set movement speed (0-255)
- **Quick Move**: Use the directional pad for instant movement:
  - ↑ Forward (0°)
  - ← Left (270°)
  - → Right (90°)
  - ↓ Backward (180°)
  - Center button to STOP

### Disconnecting

Click the **Disconnect** button in the Connection tab to safely disconnect from the Sphero and stop the controller node.

## Architecture

### Components

1. **web_server_node.py**: ROS2 node that:
   - Runs a Flask web server with WebSocket support
   - Manages sphero_controller_node lifecycle
   - Bridges web interface with ROS2 topics
   - Publishes commands and subscribes to state/sensor data

2. **index.html**: Main web interface with tabbed navigation

3. **app.js**: JavaScript client that:
   - Manages WebSocket connections for real-time updates
   - Handles user interactions
   - Sends API requests to the Flask server

4. **style.css**: Modern, responsive styling

### ROS2 Topics

#### Subscribed Topics
- `/sphero/state` (std_msgs/String): Complete robot state as JSON
- `/sphero/sensors` (std_msgs/String): Sensor data
- `/sphero/battery` (sensor_msgs/BatteryState): Battery information
- `/sphero/status` (std_msgs/String): Heartbeat and health status

#### Published Topics
- `/sphero/led` (std_msgs/String): LED color commands
- `/sphero/roll` (std_msgs/String): Roll movement commands
- `/sphero/heading` (std_msgs/String): Heading commands
- `/sphero/speed` (std_msgs/String): Speed commands
- `/sphero/stop` (std_msgs/String): Stop commands
- `/sphero/matrix` (std_msgs/String): LED matrix display commands

### API Endpoints

- `GET /`: Main web interface
- `POST /api/connect`: Connect to a Sphero by name
- `POST /api/disconnect`: Disconnect from current Sphero
- `GET /api/status`: Get current connection status
- `POST /api/led`: Set LED color
- `POST /api/motion/roll`: Send roll command
- `POST /api/motion/heading`: Set heading
- `POST /api/motion/speed`: Set speed
- `POST /api/motion/stop`: Stop the Sphero
- `POST /api/matrix`: Display pattern on LED matrix

### WebSocket Events

- `connect`: Client connected to server
- `disconnect`: Client disconnected from server
- `state_update`: Real-time state updates from Sphero
- `sensor_update`: Real-time sensor data
- `battery_update`: Battery status updates
- `status_update`: Health and heartbeat updates

## Troubleshooting

### Web interface doesn't load
- Ensure the web_server node is running
- Check that port 5000 is not in use by another application
- Verify Flask and Flask-SocketIO are installed

### Cannot connect to Sphero
- Verify the Sphero name is correct
- Ensure your computer's Bluetooth is enabled
- Check that the Sphero is charged and nearby
- Make sure sphero_package is properly built and sourced

### Tabs are disabled
- Tabs only enable after successfully connecting to a Sphero
- Check the connection status in the header
- Look at terminal output for error messages

### Real-time updates not working
- Verify WebSocket connection (check browser console)
- Ensure sphero_controller_node is publishing to topics
- Try refreshing the web page

## Customization

### Adding New Patterns

To add new LED matrix patterns, modify the pattern buttons in [templates/index.html](sphero_web_interface/templates/index.html) and ensure the pattern names match those defined in the `sphero_package/core/sphero/matrix_patterns.py` file.

### Changing Port

To use a different port, modify the `socketio.run()` call in [web_server_node.py](sphero_web_interface/web_server_node.py):

```python
socketio.run(app, host='0.0.0.0', port=YOUR_PORT, debug=False)
```

### Styling

Customize the appearance by editing [static/css/style.css](sphero_web_interface/static/css/style.css).

## Development

### Project Structure

```
sphero_web_interface/
├── sphero_web_interface/
│   ├── __init__.py
│   ├── web_server_node.py       # Main ROS2 node with Flask server
│   ├── static/
│   │   ├── css/
│   │   │   └── style.css        # Styling
│   │   └── js/
│   │       └── app.js           # Client-side JavaScript
│   └── templates/
│       └── index.html           # Main HTML interface
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## License

MIT License

## Author

Siddharth Vaghela (siddharth.vaghela@tufts.edu)

## Acknowledgments

- Built for use with the sphero_package ROS2 package
- Uses spherov2 API for Sphero communication
- Flask and Flask-SocketIO for web server functionality
