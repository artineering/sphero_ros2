# Soccer Game Controller - Quick Start Guide

## Overview

The Soccer Game Controller provides an **interactive, terminal-based workflow** for setting up a 4-robot soccer game with ArUco marker-based field calibration and positioning.

## Prerequisites

- ✅ ArUco SLAM package built and installed
- ✅ Sphero Instance Controller package built and installed
- ✅ Multi-Robot Webserver package built and installed
- ✅ Soccer Game Controller package built and installed
- ✅ 4 Sphero robots (SB-3660, SB-74FB, SB-3716, SB-58EF)
- ✅ 8 ArUco markers printed (IDs: 4, 5, 6, 7, 3, 1, 2, 20)
- ✅ Webcam connected and working

## Quick Start

### 1. Print ArUco Markers

Generate markers using:
```bash
ros2 run aruco_slam marker_generator
```

This creates markers in the `~/aruco_markers/` directory. Print:
- **Field markers**: IDs 4, 5, 6, 7
- **Sphero markers**: IDs 3, 1, 2, 20

### 2. Launch Soccer Game Controller

Start the interactive controller with one command:
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run soccer_game_controller soccer_game_controller
```

**Note:** You need to start the multirobot webserver first in a separate terminal:
```bash
source ~/ros2_ws_2/install/setup.bash
ros2 run multirobot_webserver multirobot_webapp
```

**Or use the convenient startup script** (starts everything automatically):
```bash
~/ros2_ws_2/install/soccer_game_controller/lib/soccer_game_controller/start_soccer_game.sh
```

### 3. Follow Interactive Prompts

The controller will guide you through the entire setup with clear terminal prompts.

---

## Interactive Workflow

### Step 1: Field Configuration

The controller will ask you for field dimensions:

```
Field Configuration
----------------------------------------------------------------------

Enter field width in cm (e.g., 300): 300
Enter field height in cm (e.g., 200): 200

✓ Field dimensions set: 300.0 x 200.0 cm
```

**Action Required:** Enter your actual field dimensions in centimeters.

---

### Step 2: Field Calibration

```
Field Calibration
----------------------------------------------------------------------

Place ArUco markers at the 4 corners of the soccer field:
  • Top-Left corner:     Marker 4
  • Top-Right corner:    Marker 5
  • Bottom-Right corner: Marker 6
  • Bottom-Left corner:  Marker 7

Waiting for field calibration...
```

**Action Required:**
1. Place the 4 corner markers as shown
2. Ensure all markers are visible to the camera
3. Wait for automatic detection

```
✓ Field calibrated successfully!
```

---

### Step 3: Sphero Setup (Repeated 4 times)

For each Sphero, you'll see:

```
======================================================================
SPHERO 1 of 4: SB-3660
======================================================================

Setup Instructions:
  1. Attach ArUco marker 3 to SB-3660
  2. Place SB-3660 on the left-bottom field marker
  3. Ensure marker is visible to camera

Press Enter when ready...
```

**Action Required:**
1. Attach the marker
2. Place the Sphero
3. Press **Enter**

```
Detecting SB-3660...
✓ Sphero detected!

Activating controllers for SB-3660...
✓ Controllers activated (Port: 5001)

Initializing controllers...
✓ Controllers ready
```

---

### Step 4: Calibration (For each Sphero)

```
Calibration
----------------------------------------------------------------------

Follow these steps:
  1. Open web interface: http://localhost:5001
  2. Go to Motion tab
  3. Adjust heading so:
     • 0° points toward TOP-LEFT field marker
     • 90° points toward BOTTOM-RIGHT field marker
  4. Click 'Reset Aim' button
  5. Click 'Reset to Origin' button

Press Enter when calibration is complete...
```

**Action Required:**
1. Open the web interface in your browser
2. Navigate to Motion tab
3. Use the heading slider to orient the Sphero
4. Click **Reset Aim**
5. Click **Reset to Origin**
6. Press **Enter** in the terminal

```
✓ SB-3660 calibrated!

Moving SB-3660 to position (120, 70) cm...
✓ Move command sent

Moving...
✓ Position reached
```

The controller **automatically repeats** Steps 3-4 for the remaining 3 Spheros.

---

### Step 5: Dance Routine

After all 4 Spheros are calibrated:

```
======================================================================
✓ ALL SPHEROS CALIBRATED AND POSITIONED!
======================================================================

Starting dance routine...

Dancing...
✓ Dance complete!
```

---

### Step 6: Complete

```
======================================================================
✓ SOCCER GAME SETUP COMPLETE!
======================================================================
```

All Spheros are now ready for soccer gameplay! ⚽

---

## Sphero to Marker Mapping

| Sphero     | Marker ID | Web Port |
|------------|-----------|----------|
| SB-3660    | 3         | 5001     |
| SB-74FB    | 1         | 5002     |
| SB-3716    | 2         | 5003     |
| SB-58EF    | 20        | 5004     |

---

## Terminal-Only Interaction

**Clean Output:** The controller outputs **only user-facing messages** to the terminal.

- ✅ No ROS2 logging spam
- ✅ No debug messages
- ✅ Clear prompts and confirmations
- ✅ Progress indicators

**What you'll see:**
- Field dimension prompts
- Calibration instructions
- Progress updates
- Success/error messages

**What you won't see:**
- ROS2 node lifecycle messages
- Topic subscription debug info
- Internal state transitions
- Backend API calls

---

## Common Issues

### Field dimensions not accepting input
→ Make sure to enter positive numbers (e.g., 300, not -300)

### Field won't calibrate
→ Check all 4 corner markers are visible to camera

### Sphero not detected
→ Ensure marker is visible and within field boundaries

### Controllers won't start
→ Verify Sphero is powered on and Bluetooth is enabled

### Can't open web interface
→ Check the port number shown in terminal output

---

## Example Session

Here's what a complete session looks like:

```bash
$ ros2 launch soccer_game_controller soccer_game.launch.py

======================================================================
         SOCCER GAME CONTROLLER
======================================================================

Field Configuration
----------------------------------------------------------------------

Enter field width in cm (e.g., 300): 300
Enter field height in cm (e.g., 200): 200

✓ Field dimensions set: 300.0 x 200.0 cm

Starting ArUco SLAM node...
✓ ArUco SLAM started

Field Calibration
----------------------------------------------------------------------

Place ArUco markers at the 4 corners of the soccer field:
  • Top-Left corner:     Marker 4
  • Top-Right corner:    Marker 5
  • Bottom-Right corner: Marker 6
  • Bottom-Left corner:  Marker 7

Waiting for field calibration...
✓ Field calibrated successfully!

======================================================================
SPHERO 1 of 4: SB-3660
======================================================================

Setup Instructions:
  1. Attach ArUco marker 3 to SB-3660
  2. Place SB-3660 on the left-bottom field marker
  3. Ensure marker is visible to camera

Press Enter when ready...

Detecting SB-3660...
✓ Sphero detected!

Activating controllers for SB-3660...
✓ Controllers activated (Port: 5001)

Initializing controllers...
✓ Controllers ready

Calibration
----------------------------------------------------------------------

Follow these steps:
  1. Open web interface: http://localhost:5001
  2. Go to Motion tab
  3. Adjust heading so:
     • 0° points toward TOP-LEFT field marker
     • 90° points toward BOTTOM-RIGHT field marker
  4. Click 'Reset Aim' button
  5. Click 'Reset to Origin' button

Press Enter when calibration is complete...

✓ SB-3660 calibrated!

Moving SB-3660 to position (120, 70) cm...
✓ Move command sent

Moving...
✓ Position reached

[Repeats for SB-74FB, SB-3716, SB-58EF...]

======================================================================
✓ ALL SPHEROS CALIBRATED AND POSITIONED!
======================================================================

Starting dance routine...

Dancing...
✓ Dance complete!

======================================================================
✓ SOCCER GAME SETUP COMPLETE!
======================================================================
```

---

## Tips

- **Field Dimensions**: Measure your actual field to get accurate positioning
- **Lighting**: Ensure good lighting for reliable marker detection
- **Camera Position**: Mount camera above field for best view
- **Marker Size**: Use appropriately sized markers for your field
- **Calibration**: Take your time during heading calibration for accuracy
- **Patience**: Wait for each step to complete before proceeding

---

## Next Steps

After setup is complete, you can:
- Control Spheros via web interfaces (http://localhost:5001-5004)
- Implement game logic (ball tracking, scoring, etc.)
- Experiment with different formations
- Create custom tasks and routines

Good luck with your soccer game! ⚽🤖
