# ArUco SLAM for Soccer Field Tracking

This ROS2 package provides ArUco marker-based SLAM (Simultaneous Localization and Mapping) for tracking Sphero robots on a virtual soccer field.

## Features

- **Field Calibration**: Uses 4 ArUco markers at field corners to establish coordinate mapping
- **Perspective Transformation**: Maps camera pixel coordinates to real-world field coordinates (cm)
- **Multi-Robot Tracking**: Tracks up to 4 Sphero robots simultaneously
- **Real-time Visualization**: Live camera feed with marker detection and field overlay
- **ROS2 Integration**: Publishes Sphero positions as Pose2D messages

## Marker ID Assignment

- **IDs 0-3**: Field corner markers
  - 0: Top-Left corner
  - 1: Top-Right corner
  - 2: Bottom-Right corner
  - 3: Bottom-Left corner

- **IDs 10-13**: Sphero robot markers
  - 10: SB-3660 (Leonardo)
  - 11: SB-74FB (Raphael)
  - 12: SB-3716 (Donatello)
  - 13: SB-58EF (Michelangelo)

## Installation

### Dependencies

Install required Python packages:

```bash
pip install opencv-python opencv-contrib-python numpy
```

### Build

```bash
cd ~/ros2_ws_2
colcon build --packages-select aruco_slam
source install/setup.bash
```

## Usage

### 1. Generate ArUco Markers

Generate printable markers:

```bash
ros2 run aruco_slam marker_generator
```

This creates markers in the `markers/` directory. Print these markers and place them:
- Corner markers (0-3) at the four corners of your playing field
- Robot markers (10-13) attached to each Sphero

### 2. Run the SLAM Node

```bash
ros2 run aruco_slam aruco_slam_node
```

### 3. Configuration Parameters

Customize the node with parameters:

```bash
ros2 run aruco_slam aruco_slam_node --ros-args \
  -p camera_id:=0 \
  -p field_width_cm:=600.0 \
  -p field_height_cm:=400.0 \
  -p publish_rate_hz:=10.0
```

Available parameters:
- `camera_id` (int): Camera device ID (default: 0)
- `field_width_cm` (float): Field width in centimeters (default: 600.0)
- `field_height_cm` (float): Field height in centimeters (default: 400.0)
- `corner_marker_ids` (list): IDs for corner markers (default: [0,1,2,3])
- `sphero_marker_ids` (list): IDs for Sphero markers (default: [10,11,12,13])
- `sphero_names` (list): Names for Spheros (default: ['SB-3660','SB-74FB','SB-3716','SB-58EF'])
- `publish_rate_hz` (float): Publishing frequency (default: 10.0)
- `auto_calibrate` (bool): Auto-calibrate when all corners detected (default: true)
- `show_visualization` (bool): Show camera feed window (default: true)

## Published Topics

### Sphero Positions

Each Sphero publishes its position on a dedicated topic:

- `/aruco_slam/SB-3660/position` (geometry_msgs/PoseStamped)
- `/aruco_slam/SB-74FB/position` (geometry_msgs/PoseStamped)
- `/aruco_slam/SB-3716/position` (geometry_msgs/PoseStamped)
- `/aruco_slam/SB-58EF/position` (geometry_msgs/PoseStamped)

PoseStamped format:
- `header.frame_id`: "field" (coordinate frame)
- `pose.position.x`: X coordinate in meters (0 = left edge)
- `pose.position.y`: Y coordinate in meters (0 = top edge)
- `pose.position.z`: Always 0 (2D field)
- `pose.orientation`: Identity quaternion (orientation not yet computed)

### System Status

- `/aruco_slam/calibration_status` (std_msgs/String): Calibration status text
- `/aruco_slam/all_markers` (std_msgs/String): JSON with all detected markers
- `/aruco_slam/camera_feed` (sensor_msgs/Image): Annotated camera feed

## Coordinate System

The virtual field uses a standard 2D coordinate system:
- **Origin**: Top-left corner (marker 0)
- **+X axis**: Points right (toward marker 1)
- **+Y axis**: Points down (toward marker 3)
- **Units**: Centimeters

Example field (600cm × 400cm):
```
(0,0)──────────────(600,0)
  │                    │
  │    Playing Field   │
  │                    │
(0,400)────────────(600,400)
```

## Calibration Process

1. **Place corner markers**: Position ArUco markers 0-3 at the four corners of your playing area
2. **Start the node**: The system will automatically detect all four corners
3. **Auto-calibration**: Once all corners are visible, the system calibrates and displays "CALIBRATED"
4. **Track robots**: Place marker-equipped Spheros in the field to see their positions

## Troubleshooting

### Camera not opening
- Check camera permissions: `ls -l /dev/video*`
- Try different camera IDs: `-p camera_id:=1`

### Markers not detected
- Ensure adequate lighting
- Print markers at sufficient size (recommended: 5cm×5cm minimum)
- Check marker orientation and flatness
- Verify correct ArUco dictionary (4x4_50)

### Inaccurate positions
- Ensure corner markers are placed accurately at field boundaries
- Check that camera has a clear view of the entire field
- Avoid lens distortion by using a camera with wider field of view
- Re-calibrate if markers are moved

## Implementation Details

### Architecture

- **ArucoDetector**: OpenCV-based marker detection using cv2.aruco
- **FieldMapper**: Perspective transformation using cv2.getPerspectiveTransform
- **ArucoMarker**: Position tracking and history management
- **ArucoSLAMNode**: ROS2 integration and publishing

### Algorithm

1. Capture camera frame
2. Detect ArUco markers using OpenCV
3. Extract marker centers
4. If not calibrated: attempt field calibration using corner markers
5. Transform Sphero marker positions to field coordinates
6. Publish positions as Pose2D messages
7. Visualize with overlays

## Example: Monitor Sphero Position

```bash
# Terminal 1: Start SLAM node
ros2 run aruco_slam aruco_slam_node

# Terminal 2: Echo Sphero position
ros2 topic echo /aruco_slam/SB-3660/position
```

## Future Enhancements

- [ ] Orientation (theta) calculation from marker rotation
- [ ] Kalman filtering for smoother position estimates
- [ ] Multiple camera support for larger fields
- [ ] Camera calibration for lens distortion correction
- [ ] Velocity estimation from position history
- [ ] Integration with Sphero control for closed-loop navigation

## References

- Based on concepts from: https://github.com/M-Murdock/Sphero_Starter_Code
- OpenCV ArUco documentation: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
