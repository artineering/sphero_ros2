# Soccer Game Controller - Workflow Diagram

## Complete Workflow

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SOCCER GAME SETUP                            │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 1: INITIALIZATION                                              │
└─────────────────────────────────────────────────────────────────────┘
    │
    │  Launch: ros2 launch soccer_game_controller soccer_game.launch.py
    │
    ├─► Start ArUco SLAM Node
    │   └─► Initialize camera
    │   └─► Start marker detection
    │
    ├─► Start Multi-Robot Webserver
    │   └─► Listen on port 5000
    │   └─► Ready to manage Spheros
    │
    └─► Start Soccer Game Controller
        └─► Initialize state machine
        └─► Subscribe to topics
        │
        ▼

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 2: FIELD CALIBRATION                                           │
└─────────────────────────────────────────────────────────────────────┘
    │
    │  USER ACTION: Place 4 corner markers
    │
    │  Marker 4 (TL) ──┬─────────────┬── Marker 5 (TR)
    │                  │             │
    │                  │   Field     │
    │                  │             │
    │  Marker 7 (BL) ──┴─────────────┴── Marker 6 (BR)
    │
    ├─► ArUco SLAM detects all 4 corners
    ├─► Computes perspective transformation
    ├─► Publishes calibration status: "CALIBRATED"
    │
    └─► Controller receives confirmation
        │
        ▼

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 3: SPHERO SETUP (Repeated 4 times)                             │
└─────────────────────────────────────────────────────────────────────┘
    │
    │  ┌─────────────────────────────────────────────────────────────┐
    │  │ SPHERO 1: SB-3660 (Marker 3)                                 │
    │  └─────────────────────────────────────────────────────────────┘
    │
    ├─► STEP 1: PLACEMENT
    │   │
    │   │  USER ACTION:
    │   │  - Attach Marker 3 to SB-3660
    │   │  - Place on left-bottom field marker
    │   │
    │   ├─► ArUco SLAM detects Marker 3
    │   ├─► Transforms to field coordinates
    │   ├─► Publishes position on /aruco_slam/SB_3660/position
    │   │
    │   └─► Controller confirms detection ✓
    │       │
    │       ▼
    │
    ├─► STEP 2: ACTIVATION
    │   │
    │   ├─► Controller calls POST /api/spheros
    │   ├─► Webserver launches instance on port 5001
    │   │   └─► Device Controller (Bluetooth connection)
    │   │   └─► Task Controller (High-level tasks)
    │   │   └─► State Machine Controller (Behavior)
    │   │
    │   ├─► Controller subscribes to position topic
    │   │
    │   └─► Web interface ready: http://localhost:5001 ✓
    │       │
    │       ▼
    │
    ├─► STEP 3: CALIBRATION (Interactive)
    │   │
    │   │  USER ACTION (via web interface):
    │   │
    │   ├─► Open http://localhost:5001
    │   ├─► Navigate to Motion tab
    │   │
    │   ├─► Adjust Heading:
    │   │   └─► Use heading slider
    │   │   └─► Align 0° → top-left corner
    │   │   └─► Align 90° → bottom-right corner
    │   │   └─► Click "Reset Aim"
    │   │
    │   ├─► Reset Position:
    │   │   └─► Click "Reset to Origin"
    │   │   └─► Sets position to (0, 0)
    │   │
    │   │  USER ACTION (in terminal):
    │   │   └─► Press Enter to confirm
    │   │
    │   └─► Calibration confirmed ✓
    │       │
    │       ▼
    │
    ├─► STEP 4: POSITIONING
    │   │
    │   ├─► Controller calculates target position
    │   │   └─► SB-3660: (center_x - 30, center_y - 30) cm
    │   │
    │   ├─► Sends move_to task via POST /api/task
    │   │   {
    │   │     "task_type": "move_to",
    │   │     "parameters": {"x": 120, "y": 70, "speed": 100}
    │   │   }
    │   │
    │   ├─► Sphero moves to position
    │   │
    │   └─► Position reached ✓
    │       │
    │       ▼
    │
    │  ┌─────────────────────────────────────────────────────────────┐
    │  │ SPHERO 2: SB-74FB (Marker 1)                                 │
    │  └─────────────────────────────────────────────────────────────┘
    │       │
    │       └─► [Repeat Steps 1-4 with port 5002]
    │
    │  ┌─────────────────────────────────────────────────────────────┐
    │  │ SPHERO 3: SB-3716 (Marker 2)                                 │
    │  └─────────────────────────────────────────────────────────────┘
    │       │
    │       └─► [Repeat Steps 1-4 with port 5003]
    │
    │  ┌─────────────────────────────────────────────────────────────┐
    │  │ SPHERO 4: SB-58EF (Marker 20)                                │
    │  └─────────────────────────────────────────────────────────────┘
    │       │
    │       └─► [Repeat Steps 1-4 with port 5004]
    │           │
    │           ▼

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 4: FINAL POSITIONING                                           │
└─────────────────────────────────────────────────────────────────────┘
    │
    │  All 4 Spheros positioned in square formation:
    │
    │     SB-3660 ────────── SB-74FB
    │        │                  │
    │        │    (center)      │
    │        │                  │
    │     SB-58EF ────────── SB-3716
    │
    ├─► All positions confirmed ✓
    │
    └─► Ready for dance routine
        │
        ▼

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 5: DANCE ROUTINE                                               │
└─────────────────────────────────────────────────────────────────────┘
    │
    ├─► FOR EACH Sphero (parallel):
    │   │
    │   ├─► Send spin task (5 rotations, speed 150)
    │   │   POST /api/task: {
    │   │     "task_type": "spin",
    │   │     "parameters": {"rotations": 5, "speed": 150}
    │   │   }
    │   │
    │   └─► Cycle LED colors (every 2 seconds):
    │       └─► Red → Green → Blue → Yellow → Magenta → Cyan
    │           POST /api/led: {
    │             "red": R, "green": G, "blue": B, "type": "main"
    │           }
    │
    ├─► Duration: ~15 seconds
    │
    └─► Dance complete ✓
        │
        ▼

┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 6: COMPLETE                                                    │
└─────────────────────────────────────────────────────────────────────┘
    │
    ├─► All Spheros calibrated ✓
    ├─► All Spheros positioned ✓
    ├─► Dance routine complete ✓
    │
    └─► READY FOR SOCCER GAMEPLAY! ⚽
```

## Data Flow

```
┌─────────────┐
│   Camera    │
└──────┬──────┘
       │ Video Frame
       ▼
┌─────────────────────┐
│   ArUco SLAM        │
│   - Detect markers  │
│   - Transform coords│
└──────┬──────────────┘
       │
       ├─► /aruco_slam/calibration_status
       │   (Field calibration state)
       │
       ├─► /aruco_slam/all_markers
       │   (All detected markers with camera/field coords)
       │
       └─► /aruco_slam/{sphero}/position
           (Individual Sphero positions in field coords)
           │
           ▼
┌──────────────────────────┐
│  Soccer Game Controller  │
│  - Monitor calibration   │
│  - Detect Spheros        │
│  - Orchestrate setup     │
└────────┬─────────────────┘
         │
         │ HTTP REST API
         ▼
┌──────────────────────────┐
│  Multi-Robot Webserver   │
│  - Manage instances      │
│  - Port allocation       │
└────────┬─────────────────┘
         │ Launches
         ▼
┌──────────────────────────┐
│  Sphero Instance Server  │
│  (One per Sphero)        │
└────────┬─────────────────┘
         │
         ├─► Device Controller (ROS2)
         │   └─► Bluetooth ↔ Sphero
         │
         ├─► Task Controller (ROS2)
         │   └─► High-level behaviors
         │
         └─► State Machine (ROS2)
             └─► Sphero state management
```

## State Machine

```
INIT
  │
  ▼
FIELD_CALIBRATION ────► (Wait for ArUco SLAM calibration)
  │
  │ Field calibrated
  ▼
WAIT_FOR_SPHERO_PLACEMENT ────► (Display instructions)
  │
  │ User places Sphero
  ▼
SPHERO_DETECTION ──────► (Monitor /aruco_slam/all_markers)
  │
  │ Sphero detected
  ▼
SPHERO_ACTIVATION ─────► (POST /api/spheros)
  │
  │ Controllers launched
  ▼
SPHERO_CALIBRATION ────► (Interactive calibration via web UI)
  │
  │ User confirms calibration
  ▼
MOVE_TO_POSITION ──────► (Send move_to task)
  │
  │ Position reached
  ├─► More Spheros? ──Yes──► WAIT_FOR_SPHERO_PLACEMENT
  │                            (Next Sphero)
  │
  │ All done
  ▼
ALL_CALIBRATED ────────► (All 4 Spheros ready)
  │
  ▼
DANCE ─────────────────► (Synchronized dance routine)
  │
  │ Dance complete
  ▼
DONE ──────────────────► (Setup complete)
```

## User Interaction Points

Throughout the workflow, users interact at these points:

1. **Field Setup**: Place 4 corner markers
2. **Sphero Placement** (4x): Place each Sphero with its marker
3. **Heading Calibration** (4x): Adjust heading via web interface
4. **Calibration Confirmation** (4x): Press Enter to confirm
5. **Observation**: Watch dance routine

All other steps are fully automated!
