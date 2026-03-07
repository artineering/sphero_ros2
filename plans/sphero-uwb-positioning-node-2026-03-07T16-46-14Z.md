# Sphero UWB Positioning ROS2 Package

**Created:** 2026-03-07T16:46:14Z
**Status:** Pending Approval

## Task Description

Create a NEW ROS2 package `sphero_uwb_positioning` that receives UWB range measurements from 4 WiFi-enabled anchors (Arduino Portenta C33 + UWB Shield) via UDP and computes 2D positions for 10-12 UWB tags (Arduino Stella boards) attached to Sphero robots. This replaces the serial-based approach with a UDP-based WiFi transport system.

## Analysis

### Current State
- Reference implementation exists at `/tmp/uwb_extracted/` using serial transport
- UWB localization node uses 4 anchors with serial connections
- Implements 2D multilateration with EMA smoothing
- Publishes to `/uwb/tag_<id>/pose` and `/uwb/markers`
- Uses custom messages `UWBRange.msg` and `TagPose2D.msg`

### Requirements for New Package
1. UDP receiver replacing serial reader (single socket for all 4 anchors)
2. Tag-to-Sphero name mapping (UWB tag IDs → Sphero names like "SB-3660")
3. Arena boundary detection (inside vs outside convex hull)
4. Namespaced topics matching workspace convention: `sphero_uwb/<sphero_name>/position`
5. Custom status message with arena detection
6. Integration with existing `sphero_instance_controller` and `aruco_slam` packages
7. Visualization markers with color-coding for arena status

### Key Design Decisions
1. **Transport:** UDP unicast (all 4 anchors send to single port on ROS2 host)
2. **Frame:** "sphero_arena" coordinate frame (consistent with workspace)
3. **Message format:** JSON over UDP: `{"anchor_id":N,"tag_id":M,"distance_cm":X,"timestamp_ms":T}`
4. **Namespacing:** `sphero_uwb/<sphero_name>/` (multi-robot convention)
5. **Build system:** ament_python (consistent with workspace)
6. **Position updates:** Real-time as measurements arrive, published at configurable rate (default 10 Hz)

## Detailed Plan

### Step 1: Create Package Structure
**Action:** Create new ament_python package with proper directory structure

**Files to create:**
```
src/sphero_uwb_positioning/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── msg/
│   └── SpheroUWBStatus.msg
├── config/
│   └── uwb_config.yaml
├── launch/
│   └── sphero_uwb.launch.py
└── sphero_uwb_positioning/
    ├── __init__.py
    ├── sphero_uwb_positioning_node.py
    └── udp_range_reader.py
```

**Commands:**
```bash
cd /home/svaghela/ros2_ws_2/src
mkdir -p sphero_uwb_positioning/{sphero_uwb_positioning,msg,config,launch}
cd sphero_uwb_positioning
touch sphero_uwb_positioning/{__init__.py,sphero_uwb_positioning_node.py,udp_range_reader.py}
touch msg/SpheroUWBStatus.msg
touch config/uwb_config.yaml
touch launch/sphero_uwb.launch.py
touch setup.py setup.cfg package.xml README.md
```

**Expected outcome:** Directory structure matches ament_python package layout

### Step 2: Define Custom Message
**Action:** Create `SpheroUWBStatus.msg` for status publishing

**File:** `msg/SpheroUWBStatus.msg`

**Content:**
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

**Expected outcome:** Message definition ready for colcon build

### Step 3: Create UDP Range Reader Module
**Action:** Implement thread-based UDP listener adapting from `serial_reader.py`

**File:** `sphero_uwb_positioning/udp_range_reader.py`

**Key components:**
```python
@dataclass
class RangeMeasurement:
    """Reuse dataclass structure from reference implementation"""
    anchor_id: int
    tag_id: int
    distance_m: float
    timestamp: float
    device_ts_ms: int

class UDPRangeReader(threading.Thread):
    """UDP listener for UWB range packets from all anchors"""

    def __init__(self, port, output_queue, max_queue_size, logger):
        # Initialize UDP socket on 0.0.0.0:port
        # Track per-anchor statistics

    def run(self):
        # Main loop: recvfrom(1024) with 1s timeout
        # Parse JSON: {"anchor_id":N,"tag_id":M,"distance_cm":X,"timestamp_ms":T}
        # Convert cm to meters
        # Create RangeMeasurement and append to queue
        # Handle JSONDecodeError gracefully

    def stop(self):
        # Set stop event

    def close(self):
        # Stop thread and close socket
```

**Expected outcome:** UDP receiver can parse JSON packets from multiple anchors

### Step 4: Create Main Positioning Node
**Action:** Implement `sphero_uwb_positioning_node.py` with multilateration and Sphero mapping

**File:** `sphero_uwb_positioning/sphero_uwb_positioning_node.py`

**Core functionality:**

1. **Initialization:**
   - Declare parameters: udp_port, publish_rate, range_timeout, ema_alpha, etc.
   - Load anchor configurations (ID, x, y positions)
   - Load tag-to-Sphero mapping from parameters
   - Compute arena boundary from anchor positions
   - Initialize UDP reader thread

2. **Anchor Management:**
   - Wait for all 4 anchors to be online (at least 1 packet from each)
   - Publish static TF2 transforms for anchor positions
   - Track per-anchor diagnostics (last seen, packet count)

3. **Tag Tracking:**
   - Maintain tag state per tag_id (range table, position, initialized flag)
   - Only track tags that are in the Sphero mapping
   - Apply range timeout (default 1.0s) to filter stale data

4. **2D Multilateration:**
   - Reuse least-squares algorithm from reference implementation
   - Linearize by subtracting last equation: A*[x,y]^T = b
   - Solve via np.linalg.lstsq
   - Compute residual RMSE as quality metric
   - Apply outlier rejection based on residual threshold

5. **EMA Smoothing:**
   - Apply exponential moving average to x, y positions
   - alpha parameter (default 0.3) controls smoothing strength

6. **Arena Boundary Detection:**
   - Determine if tag position is inside convex hull of anchors
   - Use scipy.spatial.ConvexHull or shapely.geometry.Polygon.contains
   - Alternative: rectangular boundary check if convex hull fails

7. **Position Publishing:**
   - **Topic:** `sphero_uwb/<sphero_name>/position`
   - **Type:** `geometry_msgs/PoseStamped`
   - **Frame:** "sphero_arena"
   - **Rate:** Configurable (default 10 Hz)

8. **Status Publishing:**
   - **Topic:** `sphero_uwb/<sphero_name>/status`
   - **Type:** `SpheroUWBStatus`
   - **Content:** sphero_name, tag_id, in_arena, num_anchors_visible, x, y, quality

9. **Visualization:**
   - **Topic:** `/sphero_uwb/markers`
   - **Type:** `visualization_msgs/MarkerArray`
   - **Markers:**
     - Anchor cubes (green, at fixed positions)
     - Tag spheres (green=in arena, red=out of arena)
     - Text labels with Sphero name and coordinates

10. **Diagnostics:**
    - **Topic:** `/sphero_uwb/diagnostics`
    - **Type:** `diagnostic_msgs/DiagnosticArray`
    - **Content:**
      - Per-anchor: last seen timestamp, packet count, online status
      - Per-tag: position quality, last update time, arena status

**Expected outcome:** Full positioning node with all required features

### Step 5: Create Configuration File
**Action:** Define YAML configuration with anchor positions and tag mapping

**File:** `config/uwb_config.yaml`

**Content:**
```yaml
sphero_uwb_positioning_node:
  ros__parameters:
    # UDP settings
    udp_port: 5000

    # Publishing rate (Hz)
    publish_rate: 10.0

    # Range measurement timeout (seconds)
    range_timeout: 1.0

    # Minimum anchors for position fix
    min_anchors_for_fix: 3

    # EMA filter alpha (0.0-1.0, higher = less smoothing)
    ema_alpha: 0.3

    # Outlier rejection threshold (meters)
    outlier_threshold: 0.5

    # Anchor configurations (positions in meters)
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

    # Tag-to-Sphero mapping
    # Maps UWB tag IDs to Sphero names
    tag_sphero_mapping:
      1: "SB-3660"
      2: "SB-74FB"
      3: "SB-3716"
      # Add more as needed (up to 12 tags)
```

**Expected outcome:** Configuration file with all tunable parameters

### Step 6: Create Launch File
**Action:** Implement launch file for easy startup

**File:** `launch/sphero_uwb.launch.py`

**Content:**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('sphero_uwb_positioning'),
        'config',
        'uwb_config.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to UWB configuration file'
        ),

        Node(
            package='sphero_uwb_positioning',
            executable='sphero_uwb_positioning_node',
            name='sphero_uwb_positioning_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
```

**Expected outcome:** Single-command launch with parameter loading

### Step 7: Create package.xml
**Action:** Define package metadata and dependencies

**File:** `package.xml`

**Key dependencies:**
```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>visualization_msgs</depend>
<depend>diagnostic_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2_ros</depend>

<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Expected outcome:** Package dependencies declared for colcon

### Step 8: Create setup.py
**Action:** Configure ament_python build with entry points and data files

**File:** `setup.py`

**Key sections:**
```python
entry_points={
    'console_scripts': [
        'sphero_uwb_positioning_node = sphero_uwb_positioning.sphero_uwb_positioning_node:main',
    ],
},

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
```

**Expected outcome:** Executable and resources installed correctly

### Step 9: Create README.md
**Action:** Document package usage and integration

**File:** `README.md`

**Sections:**
- Package overview
- Hardware setup (anchors, tags, network)
- Configuration guide (anchor positions, tag mapping)
- Launch instructions
- Topic reference
- Integration with sphero_instance_controller
- Troubleshooting

**Expected outcome:** Complete documentation for users

### Step 10: Build and Test
**Action:** Build package and verify compilation

**Commands:**
```bash
cd /home/svaghela/ros2_ws_2
colcon build --packages-select sphero_uwb_positioning
source install/setup.bash
```

**Verification:**
```bash
# Check node executable
ros2 pkg executables sphero_uwb_positioning

# Check message definition
ros2 interface show sphero_uwb_positioning/msg/SpheroUWBStatus

# Test launch file
ros2 launch sphero_uwb_positioning sphero_uwb.launch.py
```

**Expected outcome:** Package builds successfully, node runs without errors

### Step 11: Integration Testing
**Action:** Test UDP reception and position computation

**Test plan:**
1. **UDP packet test:**
   ```bash
   # Send test packet
   echo '{"anchor_id":1,"tag_id":1,"distance_cm":150,"timestamp_ms":1000}' | nc -u localhost 5000

   # Verify node receives it (check logs)
   ```

2. **Position publishing test:**
   ```bash
   # Echo position topic
   ros2 topic echo sphero_uwb/SB-3660/position

   # Echo status topic
   ros2 topic echo sphero_uwb/SB-3660/status
   ```

3. **Visualization test:**
   ```bash
   # Echo markers
   ros2 topic echo /sphero_uwb/markers

   # View in RViz2
   rviz2
   # Add MarkerArray display, set topic to /sphero_uwb/markers
   # Set fixed frame to "sphero_arena"
   ```

4. **Diagnostics test:**
   ```bash
   ros2 topic echo /sphero_uwb/diagnostics
   ```

**Expected outcome:** All topics publish correctly, visualization works in RViz2

## Implementation Details

### Multilateration Algorithm
Reuse the linearized least-squares approach from reference implementation:

```python
def _multilaterate(self, anchors, distances):
    """
    Given N >= 3 anchors at (x_i, y_i) with distances d_i,
    linearize by subtracting last equation:

    A[i] = [2*(x_N - x_i), 2*(y_N - y_i)]
    b[i] = (d_i² - d_N²) - (x_i² - x_N²) - (y_i² - y_N²)

    Solve A*[x,y]^T = b via least squares
    Returns (x, y, residual_rmse)
    """
    n = len(anchors)
    if n < 3:
        return None

    x_ref, y_ref = anchors[-1].x, anchors[-1].y
    d_ref = distances[-1]

    A = np.zeros((n-1, 2))
    b = np.zeros(n-1)

    for i in range(n-1):
        xi, yi = anchors[i].x, anchors[i].y
        di = distances[i]
        A[i, 0] = 2.0 * (x_ref - xi)
        A[i, 1] = 2.0 * (y_ref - yi)
        b[i] = (di**2 - d_ref**2) - (xi**2 - x_ref**2) - (yi**2 - y_ref**2)

    result, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

    if rank < 2:
        return None

    x_est, y_est = result[0], result[1]

    # Compute RMSE
    total_error = 0.0
    for i in range(n):
        xi, yi = anchors[i].x, anchors[i].y
        di = distances[i]
        d_est = np.sqrt((x_est - xi)**2 + (y_est - yi)**2)
        total_error += (d_est - di)**2
    rmse = np.sqrt(total_error / n)

    return (x_est, y_est, rmse)
```

### Arena Boundary Detection
Use convex hull approach:

```python
from scipy.spatial import ConvexHull
from shapely.geometry import Point, Polygon

def _compute_arena_boundary(self):
    """Compute convex hull from anchor positions"""
    anchor_points = np.array([[a.x, a.y] for a in self.anchors])
    hull = ConvexHull(anchor_points)
    hull_points = anchor_points[hull.vertices]
    self.arena_polygon = Polygon(hull_points)

def _is_in_arena(self, x, y):
    """Check if point is inside arena boundary"""
    point = Point(x, y)
    return self.arena_polygon.contains(point)
```

### Tag State Tracking
```python
@dataclass
class TagState:
    tag_id: int
    sphero_name: str
    ranges: Dict[int, Tuple[float, float]]  # anchor_id -> (distance_m, timestamp)
    x: float = 0.0
    y: float = 0.0
    initialized: bool = False
    last_update: float = 0.0
```

### UDP Reader Thread Safety
- Use `collections.deque` for thread-safe queue (same as reference)
- Single producer (UDP thread), single consumer (main node timer)
- No locks needed due to deque's thread-safety for single-producer/single-consumer

### QoS Configuration
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Use for position topics (real-time data)
self.create_publisher(PoseStamped, topic, qos_profile=sensor_qos)
```

## Expected Outcomes

1. **New package:** `sphero_uwb_positioning` successfully builds and installs
2. **UDP reception:** Node receives JSON packets from 4 anchors on single port
3. **Position computation:** 2D multilateration works with ≥3 anchors
4. **Sphero mapping:** Tag IDs correctly map to Sphero names
5. **Arena detection:** Boundary detection identifies tags inside/outside arena
6. **Topic publishing:**
   - `sphero_uwb/<sphero_name>/position` (PoseStamped)
   - `sphero_uwb/<sphero_name>/status` (SpheroUWBStatus)
   - `/sphero_uwb/markers` (MarkerArray)
   - `/sphero_uwb/diagnostics` (DiagnosticArray)
7. **Visualization:** RViz2 displays anchors and tags with color-coding
8. **Integration:** Compatible with `sphero_instance_controller` namespacing

## Potential Risks & Considerations

### Risk 1: Network Packet Loss
**Issue:** UDP is unreliable, packets may be lost
**Mitigation:**
- Range timeout handles missing measurements
- Require ≥3 anchors for position fix (can tolerate 1 missing)
- Track per-anchor last-seen diagnostics

### Risk 2: Tag-to-Sphero Mapping Maintenance
**Issue:** Manual mapping in config file may become outdated
**Mitigation:**
- Document mapping clearly in README
- Log warnings for unmapped tags
- Consider future auto-discovery feature

### Risk 3: Arena Boundary Accuracy
**Issue:** Convex hull may not match physical boundaries
**Mitigation:**
- Allow manual boundary override in config
- Fallback to rectangular boundary if convex hull fails
- Document boundary assumptions

### Risk 4: Clock Synchronization
**Issue:** Device timestamps from different anchors may drift
**Mitigation:**
- Use ROS2 host timestamp on receipt (not device timestamp)
- Track device_ts_ms for debugging only
- Document clock assumptions

### Risk 5: Coordinate Frame Compatibility
**Issue:** "sphero_arena" frame may conflict with ArUco SLAM
**Mitigation:**
- Document coordinate system (origin, axes, units)
- Provide TF2 transforms to ArUco frame if needed
- Keep frames separate initially

### Risk 6: Python Dependencies
**Issue:** scipy, shapely may not be installed
**Mitigation:**
- Document Python requirements in README
- Add to package.xml as dependencies
- Graceful fallback if scipy unavailable

## Testing Plan

### Unit Tests
1. **UDP reader:** Mock socket, verify JSON parsing
2. **Multilateration:** Known anchor/distance inputs, verify position outputs
3. **Arena detection:** Test points inside/outside known boundary
4. **EMA filter:** Verify smoothing behavior with mock data

### Integration Tests
1. **UDP to position:** Send UDP packets, verify published positions
2. **Multi-anchor:** Test with 3, 4 anchors, verify graceful degradation
3. **Timeout handling:** Stop sending packets, verify stale data rejection
4. **Outlier rejection:** Send high-residual data, verify rejection

### System Tests
1. **End-to-end:** 4 Arduino anchors + 1 tag → RViz2 visualization
2. **Multi-tag:** 4 anchors + 3 tags → verify independent tracking
3. **Performance:** Measure latency, publish rate, CPU usage
4. **Reliability:** 1-hour stress test with continuous ranging

### Manual Testing
```bash
# 1. Launch node
ros2 launch sphero_uwb_positioning sphero_uwb.launch.py

# 2. Send test UDP packets
for i in {1..4}; do
  echo "{\"anchor_id\":$i,\"tag_id\":1,\"distance_cm\":$((100+i*50)),\"timestamp_ms\":1000}" | nc -u localhost 5000
done

# 3. Verify position published
ros2 topic echo sphero_uwb/SB-3660/position

# 4. Check visualization
rviz2
# Add MarkerArray, topic=/sphero_uwb/markers, frame=sphero_arena

# 5. Monitor diagnostics
ros2 topic echo /sphero_uwb/diagnostics
```

## Integration with Existing Packages

### sphero_instance_controller
- **Compatibility:** Topic namespacing matches `sphero/<sphero_name>/` convention
- **Use case:** UWB positioning provides external localization
- **Integration:** Subscribe to `sphero_uwb/<sphero_name>/position` for closed-loop control

### aruco_slam
- **Compatibility:** Both provide external position estimates
- **Comparison:** ArUco is camera-based, UWB is radio-based
- **Integration:** Sensor fusion possible (weighted average or Kalman filter)

### multirobot_webserver
- **Compatibility:** UWB positions can be displayed on dashboard
- **Integration:** Add UWB position overlay to web UI
- **Future work:** Real-time position visualization

## File Summary

### Files to Create (13 total)
1. `src/sphero_uwb_positioning/package.xml`
2. `src/sphero_uwb_positioning/setup.py`
3. `src/sphero_uwb_positioning/setup.cfg`
4. `src/sphero_uwb_positioning/README.md`
5. `src/sphero_uwb_positioning/resource/sphero_uwb_positioning` (marker file)
6. `src/sphero_uwb_positioning/msg/SpheroUWBStatus.msg`
7. `src/sphero_uwb_positioning/config/uwb_config.yaml`
8. `src/sphero_uwb_positioning/launch/sphero_uwb.launch.py`
9. `src/sphero_uwb_positioning/sphero_uwb_positioning/__init__.py`
10. `src/sphero_uwb_positioning/sphero_uwb_positioning/sphero_uwb_positioning_node.py`
11. `src/sphero_uwb_positioning/sphero_uwb_positioning/udp_range_reader.py`

### Files to Reference (No Modifications)
- `/tmp/uwb_extracted/uwb_localization_node.py` (multilateration algorithm)
- `/tmp/uwb_extracted/serial_reader.py` (thread-based reader pattern)
- `/tmp/uwb_extracted/rtls_config.yaml` (configuration structure)
- `/home/svaghela/ros2_ws_2/uwb_wifi_foxglove_plan.md` (UDP protocol spec)

## Approval Checklist

- [ ] Package structure matches workspace conventions
- [ ] Namespacing aligns with multi-robot approach (`sphero_uwb/<sphero_name>/`)
- [ ] UDP transport properly specified (port, JSON format)
- [ ] Tag-to-Sphero mapping approach acceptable
- [ ] Arena boundary detection method approved
- [ ] Message definitions complete
- [ ] Configuration parameters comprehensive
- [ ] Integration points identified
- [ ] Testing plan adequate
- [ ] Documentation plan sufficient

## Approval Status

- [X] Waiting for user approval
- [ ] Approved
- [ ] Executed

---

**Note:** This plan follows the ROS2 Expert agent workflow. Implementation will only proceed after user approval.
