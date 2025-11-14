# Multi-Robot Webserver - Quick Start Guide

## Prerequisites

```bash
# Install required Python packages
pip install flask flask-socketio flask-cors eventlet --break-system-packages
```

## Build

```bash
cd ~/ros2_ws_2
colcon build --packages-select multirobot_webserver sphero_instance_controller
source install/setup.bash
```

## Launch

### Step 1: Start the Multi-Robot Web Server

```bash
ros2 run multirobot_webserver multirobot_webapp
```

Output:
```
============================================================
Multi-Robot Sphero Web Server
============================================================
Starting server on http://localhost:5000
Press Ctrl+C to shutdown
============================================================
```

### Step 2: Open in Browser

Navigate to: **http://localhost:5000**

You should see the Multi-Robot Sphero Controller dashboard.

### Step 3: Add Your First Sphero

1. Click the "**➕ Add Sphero**" button
2. Enter your Sphero's name (e.g., `SB-3660`)
3. Click "**Add Sphero**"

**What happens:**
- A WebSocket server launches on port 5001
- Device controller starts and connects to Sphero
- Task controller starts
- State machine controller starts
- A card appears on the dashboard

### Step 4: Open the Controller Interface

Click "**🎮 Open Controller**" on the Sphero card.

This opens the individual controller interface for that Sphero in a new tab.

### Step 5: Add More Spheros

Repeat steps 3-4 for additional Spheros:
- `SB-1234` → Port 5002
- `SB-9999` → Port 5003
- etc.

## Example: Adding Two Spheros

```bash
# Terminal 1: Main server
ros2 run multirobot_webserver multirobot_webapp
```

Then in browser (http://localhost:5000):
1. Add Sphero `SB-3660`
2. Add Sphero `SB-1234`

Now you have:
- Main dashboard: http://localhost:5000
- Sphero SB-3660: http://localhost:5001
- Sphero SB-1234: http://localhost:5002

## Dashboard Features

### Sphero Cards

Each card shows:
- **Name**: Sphero identifier
- **Status Indicator**:
  - 🟢 Running (connected and operational)
  - 🔴 Stopped (not running)
  - 🟡 Starting (launching controllers)
- **WebSocket Port**: Port number for this instance
- **Added**: Time since added
- **URL**: Direct link to controller

### Actions

- **🎮 Open Controller**: Opens dedicated controller interface
- **🗑️ Remove**: Stops controllers and removes from dashboard

### Top Bar

- **➕ Add Sphero**: Add new Sphero instance
- **🔄 Refresh**: Manually refresh status (auto-refreshes every 5s)
- **Count Badge**: Shows total number of connected Spheros

## Common Commands

### Check if Package is Built
```bash
ros2 pkg executables multirobot_webserver
```

Expected output:
```
multirobot_webserver multirobot_webapp
multirobot_webserver sphero_instance_websocket_server
```

### Manually Start WebSocket Server
```bash
ros2 run multirobot_webserver sphero_instance_websocket_server SB-3660 5001
```

### Check Running Processes
```bash
ps aux | grep sphero
```

### Stop All
Press `Ctrl+C` in the terminal running the main webapp. This will:
1. Stop all WebSocket servers
2. Stop all controllers
3. Shutdown the main application

## Troubleshooting

### "pip install" Fails

Use `--break-system-packages`:
```bash
pip install flask flask-socketio flask-cors eventlet --break-system-packages
```

### WebSocket Server Won't Start

Check if another process is using the port:
```bash
lsof -i :5001
```

Kill if needed:
```bash
kill -9 <PID>
```

### Controllers Can't Find Sphero

1. Check Bluetooth is on
2. Verify Sphero name (case-sensitive!)
3. Ensure Sphero is charged
4. Make sure Sphero isn't connected elsewhere

### Dashboard Shows "0 Sphero(s) Connected"

1. Refresh the page
2. Click the Refresh button
3. Check browser console for errors (F12)

## Testing Without Hardware

You can test the dashboard without actual Sphero hardware:
1. Start the main webapp
2. Add a Sphero (it will launch the WebSocket server)
3. The WebSocket server will try to connect but fail
4. Dashboard will still show the card with "stopped" status

This is useful for UI development and testing.

## Next Steps

- Read [README.md](README.md) for complete documentation
- Explore the API endpoints
- Customize the styling in `static/css/style.css`
- Check out individual controller features

## Architecture Overview

```
Browser (Port 5000)
        │
        ├─── Dashboard UI (HTML/CSS/JS)
        │
        └─── REST API (/api/spheros)
                │
                ├─── Manages WebSocket Servers
                │
                ├─── Port 5001: Sphero SB-3660
                │         │
                │         └─── ROS2 Controllers
                │
                └─── Port 5002: Sphero SB-1234
                          │
                          └─── ROS2 Controllers
```

## Key Files

- `multirobot_webapp.py`: Main Flask application
- `sphero_instance_websocket_server.py`: Instance WebSocket server
- `templates/index.html`: Dashboard HTML
- `static/css/style.css`: Styling
- `static/js/app.js`: Frontend JavaScript

## Tips

1. **Start Simple**: Add one Sphero first, test it, then add more
2. **Use Browser DevTools**: Press F12 to see console logs and network activity
3. **Check Terminal**: Main app terminal shows all activity and errors
4. **Port Management**: Each Sphero gets the next available port (5001, 5002, ...)
5. **Clean Shutdown**: Always use Ctrl+C to properly shutdown and clean up

## Happy Controlling! 🤖🎮
