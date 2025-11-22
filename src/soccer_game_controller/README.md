# Soccer Game Controller

A ROS2 package for orchestrating a multi-robot soccer game using Sphero robots with ArUco SLAM-based field calibration and positioning.

## Overview

This package provides an automated workflow for setting up a soccer game with multiple Sphero robots:

1. **Field Calibration**: Uses ArUco markers to define a virtual soccer field
2. **Robot Detection**: Detects Sphero robots via their ArUco markers
3. **Controller Activation**: Automatically launches Sphero controllers via the multi-robot webserver
4. **Interactive Calibration**: Guides user through heading and position calibration for each robot
5. **Positioning**: Moves robots to designated starting positions
6. **Demo**: Performs a synchronized dance routine

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Soccer Game Controller                          │
│                                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │
│  │   ArUco      │  │  Multi-Robot │  │    Game      │             │
│  │   SLAM       │  │  Webserver   │  │  Controller  │             │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘             │
│         │                  │                  │                      │
│         │ Field Coords     │ Sphero Mgmt     │ State Machine        │
│         └──────────────────┴──────────────────┘                     │
│                             │                                        │
│                  ┌──────────┴──────────┐                            │
│                  ▼                      ▼                            │
│         ┌─────────────────┐   ┌─────────────────┐                  │
│         │  Sphero 1       │   │  Sphero 2-4     │                  │
│         │  Controllers    │   │  Controllers    │                  │
│         └─────────────────┘   └─────────────────┘                  │
└─────────────────────────────────────────────────────────────────────┘
```

## Requirements

### Hardware
- Webcam for ArUco marker detection
- 4 Sphero robots (BOLT or RVR)
- 8 ArUco markers (4x4_50 dictionary):
  - IDs 4, 5, 6, 7: Field corner markers
  - IDs 3, 1, 2, 20: Sphero markers

### Software
- ROS2 (Humble or later)
- Python 3.8+
- OpenCV with ArUco support
- Dependencies:
  - `aruco_slam` package
  - `sphero_instance_controller` package
  - `multirobot_webserver` package

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# Package should already be in your workspace
```

2. Install Python dependencies:
```bash
pip3 install opencv-contrib-python requests
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select soccer_game_controller
source install/setup.bash
```

## Usage

### Quick Start

Launch the entire soccer game setup with a single command:

```bash
ros2 launch soccer_game_controller soccer_game.launch.py
```

This will start:
- ArUco SLAM node for field calibration
- Multi-robot webserver for Sphero management
- Soccer game controller node

### Manual Start (for debugging)

If you prefer to start each component separately:

#### Terminal 1: ArUco SLAM
```bash
ros2 run aruco_slam aruco_slam_node
```

#### Terminal 2: Multi-Robot Webserver
```bash
ros2 run multirobot_webserver multirobot_webapp.py
```

#### Terminal 3: Soccer Game Controller
```bash
ros2 run soccer_game_controller soccer_game_controller
```

## Workflow

### Step 1: Field Calibration

1. Print ArUco markers 4, 5, 6, 7 (use `ros2 run aruco_slam marker_generator` to generate)
2. Place markers at the 4 corners of your soccer field:
   - **Marker 4**: Top-Left corner
   - **Marker 5**: Top-Right corner
   - **Marker 6**: Bottom-Right corner
   - **Marker 7**: Bottom-Left corner
3. Ensure all 4 markers are visible to the camera
4. ArUco SLAM will automatically calibrate the field

### Step 2: Sphero Setup (Repeated for each robot)

For each Sphero robot, the controller will:

1. **Prompt for Placement**:
   - Attach the corresponding ArUco marker to the Sphero
     - Sphero SB-3660 → Marker 3
     - Sphero SB-74FB → Marker 1
     - Sphero SB-3716 → Marker 2
     - Sphero SB-58EF → Marker 20
   - Place Sphero on the left-bottom field marker
   - Ensure marker is visible to camera

2. **Detection**:
   - Controller detects the Sphero via ArUco marker
   - Displays field coordinates

3. **Activation**:
   - Automatically launches Sphero controllers
   - Provides web interface URL (e.g., http://localhost:5001)

4. **Calibration**:
   - Open the web interface in your browser
   - Go to the **Motion** tab
   - Adjust heading so:
     - 0° points toward the **left-top field marker**
     - 90° points toward the **right-bottom field marker**
   - Click **"Reset Aim"** to set heading to 0°
   - Click **"Reset to Origin"** to set position to (0, 0)
   - Press **Enter** in the terminal to confirm calibration

5. **Positioning**:
   - Sphero automatically moves to its designated position near center field

6. **Next Robot**:
   - Repeat steps 1-5 for remaining Spheros

### Step 3: Dance Routine

Once all 4 Spheros are calibrated and positioned:
- All Spheros perform a synchronized dance routine
- Spinning motion with cycling LED colors
- Duration: ~15 seconds

### Step 4: Complete

The soccer game setup is complete! All Spheros are:
- Calibrated with correct heading and position
- Positioned on the field
- Ready for soccer gameplay

## Configuration

You can customize the setup by modifying parameters in the launch file or passing them as arguments:

### Field Dimensions
```python
'field_width_cm': 300.0,   # Width in centimeters
'field_height_cm': 200.0,  # Height in centimeters
```

### Marker IDs
```python
'corner_marker_ids': [4, 5, 6, 7],     # Field corners
'sphero_marker_ids': [3, 1, 2, 20],    # Sphero markers
```

### Sphero Names
```python
'sphero_names': ['SB-3660', 'SB-74FB', 'SB-3716', 'SB-58EF']
```

### Webserver URL
```python
'multirobot_webserver_url': 'http://localhost:5000'
```

## Topics

### Subscribed Topics
- `/aruco_slam/calibration_status` (String): Field calibration status
- `/aruco_slam/all_markers` (String): All detected ArUco markers with positions
- `/aruco_slam/{sphero_name}/position` (PoseStamped): Sphero positions in field coordinates

### Published Topics
None (controller uses REST API for Sphero commands)

## API Endpoints Used

The controller interacts with the multi-robot webserver via REST API:

- `POST /api/spheros`: Add/activate a Sphero
- `POST /api/led`: Control Sphero LED color
- `POST /api/task`: Send high-level tasks (move_to, spin, etc.)
- `POST /api/motion/reset`: Reset aim/heading

## Troubleshooting

### Field Won't Calibrate
- Ensure all 4 corner markers are visible to the camera
- Check marker IDs match configuration
- Verify camera has clear view of the field
- Check ArUco SLAM visualization window

### Sphero Not Detected
- Ensure Sphero's ArUco marker is visible to camera
- Verify marker ID matches configuration
- Check marker is properly attached to Sphero
- Ensure Sphero is within field boundaries

### Controllers Won't Activate
- Check multi-robot webserver is running (http://localhost:5000/health)
- Ensure Sphero is powered on and discoverable via Bluetooth
- Verify Sphero name matches Bluetooth advertised name
- Check terminal output for error messages

### Calibration Issues
- Make sure web interface is accessible (http://localhost:5001)
- Verify Sphero is receiving motion commands
- Use the Motion tab controls to test heading and movement
- Ensure camera can track the Sphero's marker

### Dance Routine Doesn't Start
- Verify all 4 Spheros completed calibration
- Check that all Spheros are still connected
- Ensure controllers are running (check webserver status)

## Development

### Project Structure
```
soccer_game_controller/
├── soccer_game_controller/
│   ├── __init__.py
│   └── soccer_game_controller_node.py  # Main controller
├── launch/
│   └── soccer_game.launch.py           # Launch file
├── test/                                # Unit tests
├── package.xml                          # ROS2 package metadata
├── setup.py                             # Python package setup
└── README.md                            # This file
```

### State Machine

The controller uses a finite state machine with the following states:

1. **INIT**: Initialize controller
2. **FIELD_CALIBRATION**: Wait for field calibration
3. **WAIT_FOR_SPHERO_PLACEMENT**: Prompt user to place Sphero
4. **SPHERO_DETECTION**: Detect Sphero via ArUco marker
5. **SPHERO_ACTIVATION**: Activate Sphero controllers
6. **SPHERO_CALIBRATION**: Interactive calibration
7. **MOVE_TO_POSITION**: Move Sphero to starting position
8. **ALL_CALIBRATED**: All Spheros ready (transition to dance)
9. **DANCE**: Perform dance routine
10. **DONE**: Complete

## Future Enhancements

Potential improvements for future versions:

- [ ] Automated heading calibration using camera feedback
- [ ] Visual feedback for calibration progress
- [ ] Game logic implementation (ball tracking, scoring)
- [ ] Multiple game modes (practice, tournament, etc.)
- [ ] Replay/recording functionality
- [ ] Web-based monitoring dashboard
- [ ] Obstacle detection and avoidance
- [ ] Multi-camera support for larger fields

## License

MIT License

## Author

Siddharth Vaghela (siddharth.vaghela@tufts.edu)

## Acknowledgments

- ArUco SLAM package for field calibration
- Sphero Instance Controller for robot management
- Multi-Robot Webserver for fleet management
