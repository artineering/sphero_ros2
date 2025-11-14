# Multi-Robot Web Server

A comprehensive web-based interface for managing and controlling multiple Sphero robots simultaneously. Each Sphero instance gets its own WebSocket server for real-time communication, and all instances are managed through a central web application.

## Features

- 🤖 **Multi-Robot Management**: Control multiple Sphero robots from a single interface
- 🌐 **WebSocket Communication**: Real-time bidirectional communication with each Sphero
- 🎮 **Individual Controllers**: Each Sphero has its own dedicated WebSocket server and controller interface
- 📊 **Status Monitoring**: View connection status, battery level, and state for all robots
- ➕ **Dynamic Addition/Removal**: Add and remove Sphero instances on-the-fly
- 🔄 **Auto-Refresh**: Real-time updates of robot status
- 💳 **Card-Based UI**: Clean, modern interface with robot cards
- 📱 **Responsive Design**: Works on desktop and mobile devices

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                Multi-Robot Web Application                       │
│                    (Port 5000 - Flask)                           │
│                                                                  │
│  • Manages Sphero instances                                      │
│  • Launches/stops WebSocket servers                              │
│  • Provides central dashboard                                    │
└──────────────┬──────────────────────┬──────────────────────────┘
               │                      │
               ▼                      ▼
   ┌──────────────────────┐  ┌──────────────────────┐
   │ WebSocket Server 1   │  │ WebSocket Server 2   │
   │   (Port 5001)        │  │   (Port 5002)        │
   │   Sphero: SB-3660    │  │   Sphero: SB-1234    │
   └──────────┬───────────┘  └──────────┬───────────┘
              │                          │
              ▼                          ▼
   ┌──────────────────────┐  ┌──────────────────────┐
   │  ROS2 Controllers    │  │  ROS2 Controllers    │
   │  • Device            │  │  • Device            │
   │  • Task              │  │  • Task              │
   │  • State Machine     │  │  • State Machine     │
   └──────────┬───────────┘  └──────────┬───────────┘
              │                          │
              ▼                          ▼
        Sphero SB-3660             Sphero SB-1234
```

## Components

### 1. Multi-Robot Web Application (`multirobot_webapp.py`)
- **Type**: Standalone Flask application (not a ROS2 node)
- **Port**: 5000
- **Purpose**: Central dashboard for managing multiple Sphero instances
- **Features**:
  - Add/remove Sphero instances
  - Launch WebSocket servers for each Sphero
  - Display status cards for all robots
  - Provide links to individual controller interfaces

### 2. Sphero Instance WebSocket Server (`sphero_instance_websocket_server.py`)
- **Type**: ROS2 node with WebSocket server
- **Ports**: 5001, 5002, 5003, ... (assigned dynamically)
- **Purpose**: Provide real-time communication for a single Sphero instance
- **Features**:
  - Uses namespaced topics: `sphero/<sphero_name>/*`
  - Launches all three controllers (device, task, state machine)
  - Bi-directional WebSocket communication
  - Real-time state updates
  - Command publishing to ROS2 topics

## Installation

### Prerequisites
```bash
# Install required Python packages
pip install flask flask-socketio flask-cors eventlet --break-system-packages
```

### Build Package
```bash
cd ~/ros2_ws_2
colcon build --packages-select multirobot_webserver
source install/setup.bash
```

## Usage

### Starting the Multi-Robot Web Server

1. **Launch the main web application:**
```bash
ros2 run multirobot_webserver multirobot_webapp
```

2. **Open in browser:**
```
http://localhost:5000
```

### Adding a Sphero

1. Click the "**Add Sphero**" button
2. Enter the Sphero name (e.g., `SB-3660`)
3. Click "**Add Sphero**"

The system will:
- Launch a WebSocket server for that Sphero on a dedicated port
- Automatically start the device, task, and state machine controllers
- Add a card to the dashboard showing the Sphero's status

### Accessing Individual Controller

Click the "**Open Controller**" button on any Sphero card to open its dedicated controller interface in a new tab.

### Removing a Sphero

1. Click the "**Remove**" button on the Sphero card
2. Confirm the removal

This will:
- Stop all controllers for that Sphero
- Shut down the WebSocket server
- Remove the card from the dashboard

## API Endpoints

The multi-robot web application provides a REST API:

### Get All Spheros
```http
GET /api/spheros
```
**Response:**
```json
{
  "success": true,
  "spheros": [
    {
      "name": "SB-3660",
      "port": 5001,
      "status": "running",
      "added_at": 1699876543.21,
      "url": "http://localhost:5001"
    }
  ],
  "count": 1
}
```

### Add Sphero
```http
POST /api/spheros
Content-Type: application/json

{
  "sphero_name": "SB-3660"
}
```

### Remove Sphero
```http
DELETE /api/spheros/{sphero_name}
```

### Get Sphero Status
```http
GET /api/spheros/{sphero_name}
```

### Health Check
```http
GET /health
```

## WebSocket Events (Instance Server)

Each Sphero instance WebSocket server supports these events:

### Client → Server

- **`connect`**: Establish connection
- **`start_controllers`**: Start ROS2 controllers
- **`stop_controllers`**: Stop ROS2 controllers
- **`led_command`**: Set LED color
  ```json
  {"red": 255, "green": 0, "blue": 0}
  ```
- **`roll_command`**: Move robot
  ```json
  {"heading": 90, "speed": 100, "duration": 2.0}
  ```
- **`stop_command`**: Stop movement
- **`task_command`**: Execute high-level task
  ```json
  {"task_type": "move_to", "parameters": {"x": 100, "y": 50, "speed": 100}}
  ```
- **`sm_config`**: Configure state machine
- **`get_status`**: Request current status

### Server → Client

- **`status`**: Full status update
- **`state_update`**: Robot state changed
- **`battery_update`**: Battery level changed
- **`status_update`**: Status changed
- **`task_status_update`**: Task status changed
- **`sm_status_update`**: State machine status changed
- **`controllers_started`**: Controllers launched
- **`controllers_stopped`**: Controllers stopped

## Directory Structure

```
multirobot_webserver/
├── multirobot_webserver/
│   ├── __init__.py
│   ├── multirobot_webapp.py              # Main web application
│   └── sphero_instance_websocket_server.py  # WebSocket server node
├── templates/
│   └── index.html                        # Main dashboard UI
├── static/
│   ├── css/
│   │   └── style.css                     # Styling
│   └── js/
│       └── app.js                        # Frontend logic
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Configuration

### Port Assignment
- **Main Web App**: Port 5000 (fixed)
- **WebSocket Servers**: Ports 5001, 5002, 5003, ... (auto-assigned)

### Topic Namespacing
Each Sphero uses namespaced topics:
```
/sphero/SB-3660/led
/sphero/SB-3660/roll
/sphero/SB-3660/state
/sphero/SB-3660/battery
/sphero/SB-3660/task
/sphero/SB-3660/state_machine/config
... (and more)
```

## Example Workflow

### 1. Start the System
```bash
# Terminal 1: Start main web server
ros2 run multirobot_webserver multirobot_webapp
```

### 2. Add Multiple Spheros
- Open http://localhost:5000
- Click "Add Sphero"
- Add `SB-3660`
- Add `SB-1234`
- Add `SB-9999`

### 3. Control Individual Spheros
- Click "Open Controller" on any Sphero card
- Each opens in a new tab with its own controller interface
- Control each robot independently

### 4. Monitor All Robots
- Main dashboard shows status of all robots
- Real-time updates every 5 seconds
- Color-coded status indicators:
  - 🟢 Green: Running
  - 🔴 Red: Stopped
  - 🟡 Yellow: Starting

## Troubleshooting

### WebSocket Server Won't Start
**Symptom**: Sphero added but status shows "stopped"

**Solutions**:
1. Check if port is already in use
2. Verify ROS2 is sourced: `source ~/ros2_ws_2/install/setup.bash`
3. Check if `sphero_instance_controller` package is built
4. View logs in terminal where main app is running

### Controller Not Connecting to Sphero
**Symptom**: WebSocket server running but no data

**Solutions**:
1. Ensure Bluetooth is enabled
2. Verify Sphero name is correct (case-sensitive)
3. Check if Sphero is charged and powered on
4. Ensure Sphero is not connected to another device

### Cannot Open Individual Controller
**Symptom**: "Open Controller" button doesn't work

**Solutions**:
1. Check browser console for errors
2. Verify WebSocket server is running for that Sphero
3. Try accessing URL directly: `http://localhost:PORT`

### Port Conflicts
**Symptom**: Error about port already in use

**Solutions**:
1. Stop other services using port 5000
2. Change port in `multirobot_webapp.py` if needed
3. Check `lsof -i :5000` to see what's using the port

## Advanced Usage

### Running Behind a Reverse Proxy
If you need to access the interface from other machines, use nginx or Apache as a reverse proxy with WebSocket support enabled.

### Custom Styling
Modify `static/css/style.css` to customize the appearance of the dashboard.

### Adding New Features
The modular architecture makes it easy to extend:
- Add new WebSocket events in `sphero_instance_websocket_server.py`
- Add new API endpoints in `multirobot_webapp.py`
- Extend the UI in `templates/index.html` and `static/js/app.js`

## Performance Notes

- Each Sphero WebSocket server runs in its own process
- ROS2 controllers for each Sphero run independently
- Dashboard auto-refreshes every 5 seconds
- Supports 10+ Spheros simultaneously (tested)

## Security Considerations

⚠️ **Warning**: This is a development tool. For production use:
- Add authentication
- Use HTTPS/WSS
- Implement rate limiting
- Add input validation
- Use environment variables for secrets

## Future Enhancements

Planned features:
- [ ] Coordinated multi-robot control
- [ ] Formation patterns
- [ ] Collision avoidance between robots
- [ ] Fleet-wide state machines
- [ ] Recording and replay of movements
- [ ] Live video streaming
- [ ] Advanced analytics dashboard

## Dependencies

- **ROS2**: Humble or later
- **Python**: 3.10+
- **Flask**: Web framework
- **Flask-SocketIO**: WebSocket support
- **Flask-CORS**: Cross-origin support
- **sphero_instance_controller**: Sphero control package

## License

TODO: License declaration

## Support

For issues or questions:
1. Check the troubleshooting section
2. Review the logs in the terminal
3. Verify all dependencies are installed
4. Test with a single Sphero first

---

**Generated with Claude Code** 🤖
