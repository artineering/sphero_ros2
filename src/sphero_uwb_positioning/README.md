# Sphero UWB Positioning

ROS2 package for real-time 2D positioning of Sphero robots using UWB (Ultra-Wideband) ranging with WiFi-enabled anchors.

## Overview

This package receives UWB range measurements from 4 WiFi-enabled anchors (Arduino Portenta C33 + UWB Shield) via UDP and computes 2D positions for UWB tags (Arduino Stella boards) attached to Sphero robots using multilateration. It provides:

- Real-time 2D position estimation via multilateration
- Tag-to-Sphero name mapping
- Arena boundary detection (inside/outside)
- Position and status publishing per Sphero
- RViz2 visualization with markers
- Comprehensive diagnostics

## Hardware Setup

### Anchors (4x)
- **Hardware:** Arduino Portenta C33 + UWB Shield
- **Network:** WiFi-enabled, all anchors send UDP packets to the same port
- **Placement:** Positioned at known locations spanning the arena (typically rectangular arrangement)
- **Coordinate System:** "sphero_arena" frame with positions configured in `uwb_config.yaml`

### Tags (up to 12)
- **Hardware:** Arduino Stella boards with UWB
- **Attachment:** Mounted on Sphero robots
- **Mapping:** Tag IDs mapped to Sphero names in configuration

### Network Configuration
- All anchors send UDP packets to the ROS2 host on a single port (default: 5000)
- Packet format: `{"anchor_id": N, "tag_id": M, "distance_cm": X, "timestamp_ms": T}`

## Installation

1. Clone into your ROS2 workspace:
```bash
cd ~/ros2_ws_2/src
# Package already exists at: sphero_uwb_positioning/
```

2. Install Python dependencies (if needed):
```bash
pip3 install scipy shapely
```

3. Build the package:
```bash
cd ~/ros2_ws_2
colcon build --packages-select sphero_uwb_positioning
source install/setup.bash
```

## Configuration

Edit `config/uwb_config.yaml` to configure:

### 1. Anchor Positions
Define the 4 anchor positions in meters (arena coordinate frame):
```yaml
anchors:
  - id: 1
    x: 0.0
    y: 0.0
  - id: 2
    x: 6.0
    y: 0.0
  - id: 3
    x: 6.0
    y: 4.0
  - id: 4
    x: 0.0
    y: 4.0
```

### 2. Tag-to-Sphero Mapping
Map UWB tag IDs to Sphero robot names:
```yaml
tag_sphero_mapping:
  1: "SB-3660"
  2: "SB-74FB"
  3: "SB-3716"
  # Add more as needed
```

### 3. UDP and Processing Parameters
```yaml
udp_port: 5000              # UDP port to receive range packets
publish_rate: 10.0          # Publishing rate in Hz
range_timeout: 1.0          # Max age for range measurements (seconds)
min_anchors_for_fix: 3      # Minimum anchors for position estimate
ema_alpha: 0.3              # Smoothing factor (0.0-1.0, higher=less smoothing)
outlier_threshold: 0.5      # Residual threshold for outlier rejection
```

## Usage

### Launch the Node

Basic launch:
```bash
ros2 launch sphero_uwb_positioning sphero_uwb.launch.py
```

With custom config:
```bash
ros2 launch sphero_uwb_positioning sphero_uwb.launch.py config:=/path/to/custom_config.yaml
```

With debug logging:
```bash
ros2 launch sphero_uwb_positioning sphero_uwb.launch.py log_level:=debug
```

### Published Topics

#### Per-Sphero Topics
For each Sphero (e.g., "SB-3660"):

- **`sphero_uwb/SB-3660/position`** (`geometry_msgs/PoseStamped`)
  - 2D position in "sphero_arena" frame
  - Published at configured rate (default 10 Hz)

- **`sphero_uwb/SB-3660/status`** (`sphero_uwb_positioning/SpheroUWBStatus`)
  - Full status including:
    - `sphero_name`: Robot name
    - `tag_id`: UWB tag ID
    - `in_arena`: Boolean (inside/outside boundary)
    - `num_anchors_visible`: Number of anchors with fresh data
    - `x, y`: Position in meters
    - `quality`: Residual RMSE (lower is better)

#### Global Topics

- **`/sphero_uwb/markers`** (`visualization_msgs/MarkerArray`)
  - RViz2 markers for visualization:
    - Green cubes: Anchors
    - Green spheres: Tags inside arena
    - Red spheres: Tags outside arena
    - Text labels: Sphero names and coordinates

- **`/sphero_uwb/diagnostics`** (`diagnostic_msgs/DiagnosticArray`)
  - System health diagnostics:
    - Per-anchor: online status, packet count, last seen
    - Per-tag: tracking status, position quality, visible anchors
    - Overall system: anchor/tag counts, UDP statistics

### Visualization in RViz2

1. Launch RViz2:
```bash
rviz2
```

2. Configure:
   - Set **Fixed Frame** to `sphero_arena`
   - Add **MarkerArray** display
   - Set topic to `/sphero_uwb/markers`

3. You should see:
   - Green cubes at anchor positions
   - Green/red spheres at tag positions (color indicates arena status)
   - Text labels with Sphero names and coordinates

### Testing with Simulated Data

Send test UDP packets to verify the node is working:

```bash
# Send a test range measurement from anchor 1 to tag 1 (1.5m)
echo '{"anchor_id":1,"tag_id":1,"distance_cm":150,"timestamp_ms":1000}' | nc -u localhost 5000

# Send measurements from all 4 anchors for tag 1
for i in {1..4}; do
  echo "{\"anchor_id\":$i,\"tag_id\":1,\"distance_cm\":$((100+i*50)),\"timestamp_ms\":1000}" | nc -u localhost 5000
done

# Monitor position output
ros2 topic echo sphero_uwb/SB-3660/position
```

## Integration with Other Packages

### sphero_instance_controller
The UWB positioning system is compatible with the multi-robot controller:
- Topic namespacing follows the `sphero_uwb/<sphero_name>/` convention
- Subscribe to `sphero_uwb/<sphero_name>/position` for external localization
- Use for closed-loop control or position feedback

### aruco_slam
Both packages provide external position estimates:
- **ArUco:** Camera-based visual localization
- **UWB:** Radio-based ranging localization
- Can be used independently or fused for improved accuracy

### multirobot_webserver
UWB positions can be integrated into the web dashboard:
- Display real-time UWB positions on the web UI
- Show arena boundary status
- Monitor anchor and tag health via diagnostics

## Algorithm Details

### 2D Multilateration
Uses linearized least-squares approach:
1. Requires at least 3 anchors with valid range measurements
2. Linearizes the system by subtracting the last equation:
   ```
   A * [x, y]^T = b
   where:
     A[i] = [2*(x_N - x_i), 2*(y_N - y_i)]
     b[i] = (d_i² - d_N²) - (x_i² - x_N²) - (y_i² - y_N²)
   ```
3. Solves via `np.linalg.lstsq`
4. Computes residual RMSE as quality metric

### EMA Smoothing
Exponential Moving Average applied to positions:
- `x_new = x_old + alpha * (x_raw - x_old)`
- Higher `alpha` = less smoothing, faster response
- Lower `alpha` = more smoothing, slower response

### Arena Boundary Detection
Uses convex hull of anchor positions:
- Computes convex hull from 4 anchor coordinates
- Checks if tag position is inside polygon
- Requires `scipy` and `shapely` libraries
- Fallback: assumes all positions are "in arena" if libraries unavailable

### Outlier Rejection
Rejects measurements with high residual errors:
- Threshold: `outlier_threshold * num_anchors`
- Helps filter NLOS (Non-Line-Of-Sight) measurements
- Logged at debug level for troubleshooting

## Troubleshooting

### No position estimates published
1. Check that anchors are sending UDP packets:
   ```bash
   sudo tcpdump -i any -n udp port 5000
   ```
2. Verify tag-to-Sphero mapping in config
3. Check diagnostics:
   ```bash
   ros2 topic echo /sphero_uwb/diagnostics
   ```
4. Ensure at least 3 anchors have fresh data

### Poor position accuracy
1. Check anchor positions are correctly configured
2. Verify anchors span the workspace (not collinear)
3. Monitor residual in status messages (should be < 0.5m)
4. Adjust `ema_alpha` for more/less smoothing
5. Check for NLOS conditions or multipath interference

### Tags jumping outside arena
1. Verify anchor positions form correct boundary
2. Check arena boundary computation logs
3. Install `scipy` and `shapely` if not available
4. Inspect convex hull vertices in debug logs

### High latency
1. Reduce `publish_rate` if CPU-bound
2. Check network for UDP packet loss
3. Verify range measurements are arriving (check diagnostics)

### Anchor offline warnings
1. Verify WiFi connectivity for all anchors
2. Check anchor is sending to correct IP and port
3. Inspect anchor firmware and network configuration
4. Monitor anchor diagnostics for "last seen" timestamps

## Dependencies

### ROS2 Packages
- `rclpy`
- `geometry_msgs`
- `visualization_msgs`
- `diagnostic_msgs`
- `std_msgs`
- `tf2_ros`

### Python Libraries
- `numpy` (required)
- `scipy` (optional, for arena boundary detection)
- `shapely` (optional, for arena boundary detection)

## Coordinate Frames

- **sphero_arena**: Fixed arena frame, origin at anchor 1
  - X-axis: From anchor 1 towards anchor 2
  - Y-axis: From anchor 1 towards anchor 4
  - Z-axis: Up (right-hand rule)
  - Units: meters

## Parameters Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `udp_port` | int | 5000 | UDP port for receiving range packets |
| `publish_rate` | float | 10.0 | Publishing rate in Hz |
| `range_timeout` | float | 1.0 | Max age for range measurements (seconds) |
| `min_anchors_for_fix` | int | 3 | Minimum anchors for position estimate |
| `ema_alpha` | float | 0.3 | EMA smoothing factor (0.0-1.0) |
| `outlier_threshold` | float | 0.5 | Residual threshold for outlier rejection |
| `anchors` | list | - | List of anchor configurations (id, x, y) |
| `tag_sphero_mapping` | dict | - | Mapping of tag IDs to Sphero names |

## Message Definitions

### SpheroUWBStatus.msg
```
std_msgs/Header header
string sphero_name           # Sphero name (e.g., "SB-3660")
uint8 tag_id                 # UWB tag ID (1-12)
bool in_arena                # True if inside arena boundary
uint8 num_anchors_visible    # Number of anchors with fresh data
float64 x                    # X position in meters
float64 y                    # Y position in meters
float64 quality              # Position quality (residual RMSE)
```

## License

MIT

## Authors

- svaghela

## Version

1.0.0
