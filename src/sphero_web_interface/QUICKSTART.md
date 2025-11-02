# Quick Start Guide - Sphero Web Interface

## Getting Started in 3 Steps

### Step 1: Install Dependencies (First Time Only)

```bash
# Install Python web framework dependencies
pip3 install flask flask-socketio python-socketio
```

### Step 2: Build the Package (First Time Only)

```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_web_interface
source install/setup.bash
```

### Step 3: Launch the Web Interface

**Option A - Using ROS2 command:**
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run sphero_web_interface web_server
```

**Option B - Using the launch script:**
```bash
cd ~/ros2_ws_2/src/sphero_web_interface
./launch_web_interface.sh
```

### Step 4: Open Your Browser

Navigate to: **http://localhost:5000**

## Using the Web Interface

### Connecting to Your Sphero

1. In the **Connection** tab, enter your Sphero's name
   - Example: `SB-3660`
   - Find your Sphero's name on the robot or in the Sphero Edu app

2. Click **Connect**
   - The server will start the controller node
   - Wait for "Connected" status in the header (green badge)
   - Battery level will appear once connected

3. Once connected, all tabs become available!

### Quick Controls Overview

#### 🔵 State Tab
- View battery level, voltage, and health
- See current heading and speed
- Monitor connection status
- View complete raw state data

#### 📊 Sensors Tab
- Real-time accelerometer readings (X, Y, Z)
- Gyroscope data (X, Y, Z)
- Velocity information
- Location tracking

#### 🎨 Matrix Tab (Sphero BOLT only)
- Click any pattern button to display it on the LED matrix
- Use RGB sliders to change the color
- 15 built-in patterns available:
  - Emotions: Smile 😊, Frown ☹
  - Symbols: Heart ❤️, Star ⭐
  - Arrows: ⬆️ ⬇️ ⬅️ ➡️
  - And more!

#### 🎮 Motion Tab

**LED Control:**
- Adjust RGB sliders for custom LED color
- Click "Set LED Color" to apply

**Heading Control:**
- Use the visual compass
- Drag the slider (0-359°)
- Click "Set Heading" to aim the Sphero

**Speed Control:**
- Set speed from 0-255
- Click "Set Speed" to start moving
- Click "STOP" to immediately halt

**Quick Move Pad:**
- ↑ Move forward (0°)
- ← Move left (270°)
- → Move right (90°)
- ↓ Move backward (180°)
- Center: Emergency STOP

### Disconnecting

Click the **Disconnect** button to:
- Stop the Sphero
- Close the controller node
- Return to connection screen

## Troubleshooting

**Problem: "Cannot connect to Sphero"**
- Solution: Check Bluetooth is enabled and Sphero is charged and nearby

**Problem: "Web interface won't load"**
- Solution: Make sure the web_server node is running and nothing else is using port 5000

**Problem: "Tabs are disabled"**
- Solution: You must successfully connect to a Sphero first

**Problem: "No real-time updates"**
- Solution: Refresh the page or check WebSocket connection in browser console

## Tips

1. **Battery Warning**: The web interface will show battery percentage in the header. Charge when it drops below 20%

2. **Emergency Stop**: The red STOP button in the center of the Quick Move pad will immediately halt the Sphero

3. **Heading Calibration**: Use the heading control to calibrate which direction is "forward" for your Sphero

4. **Matrix Patterns**: The matrix tab only works with Sphero BOLT. Other models will ignore these commands.

5. **Multiple Spheros**: To switch between Spheros, disconnect the current one first, then connect to a different one

## Next Steps

- Check out the full [README.md](README.md) for architecture details
- Explore the ROS2 topics being published
- Customize the interface styling in `static/css/style.css`
- Add your own matrix patterns

## Need Help?

- Check terminal output for error messages
- Use `ros2 topic list` to verify topics are active
- Use `ros2 topic echo /sphero/state` to see raw state data
- Review the [sphero_controller_node.py](../sphero_package/sphero_package/sphero_controller_node.py) for available commands

Happy robot controlling! 🤖
