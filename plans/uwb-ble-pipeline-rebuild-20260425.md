# Plan: UWB BLE Pipeline Rebuild

**Created:** 2026-04-25
**Status:** Pending Approval
**Author:** ros2_expert

## Context

New tag firmware (`arduino/UWB_MulticastTag/UWB_MulticastTag.ino`) computes 2D position on-tag and broadcasts it via BLE manufacturer data:
- Local name: `UWB-T<TAG_ID>`
- Manufacturer payload (7 bytes): Company ID `0xFFFF` (LE) + `tag_id` (u8) + `x_cm` (i16 LE) + `y_cm` (i16 LE)
- Tag firmware updates the BLE adv only when position changes (10 cm quantization), and stops updating when fewer than 3 anchors are valid (`STALE_TIMEOUT_MS = 2500` ms in firmware).

The old WiFi/UDP/JSON range pipeline + ROS-side multilateration is dead. The host now passively scans BLE and republishes positions on ROS topics. No ranging math runs on the host.

## Investigation Findings

1. `bleak==1.1.1` is already installed system-wide (`pip show bleak`).
2. `bluez 5.72` is installed; `svaghela` is in `plugdev` (typical BLE userland group on Ubuntu). No special capabilities are set on `python3`. Standard scan should work as user; if it doesn't we'll fall back to `setcap cap_net_raw,cap_net_admin+eip /usr/bin/python3.12` or running with `sudo`.
3. **No other workspace package consumes `sphero_uwb/*` topics or `SpheroUWBStatus`.** Verified via `grep` — only `sphero_uwb_positioning` itself references either. Safe to remove/redesign without breaking consumers.
4. `sphero_instance_device_controller_node.py` line 120 subscribes to `/aruco_slam/<sphero_name_safe>/position` (PoseStamped) when `external_localization=True`. **This is the integration target** — if we publish UWB positions on a parallel topic with the same shape, `sphero_instance_device_controller` can switch sources by parameter.
5. `aruco_slam_node.py` publishes `/aruco_slam/<sphero_name_safe>/position` as `geometry_msgs/PoseStamped`, frame `sphero_arena`, units **cm** (per CLAUDE.md "units in cm" note for ArUco). The new firmware also reports cm. Consistent.
6. Package is `ament_cmake` style (no `setup.py`), has `rosidl_generate_interfaces` for the custom message, installs Python executables via `install(PROGRAMS …)`. Build style stays the same.
7. Workspace-root `tcp_range_reader.py` is dead — was a TCP fallback for the old anchor JSON path. Should be deleted along with the rest of the old pipeline.
8. Sphero name normalization rule (from CLAUDE.md and the existing code): hyphens → underscores in topic segments and node names (`SB-3660` → `SB_3660`). Reuse this rule.

## 1. Files to Delete / Gut

### Inside `src/sphero_uwb_positioning/`
- **Delete:** `sphero_uwb_positioning/sphero_uwb_positioning_node.py` (old multilateration node — entire file replaced)
- **Delete:** `sphero_uwb_positioning/udp_range_reader.py` (dead transport)
- **Delete:** `sphero_uwb_positioning/__pycache__/` (stale bytecode)
- **Delete:** `config/uwb_config.yaml` (replaced by new BLE-oriented config)
- **Delete:** `launch/sphero_uwb.launch.py` (replaced)
- **Delete:** `README.md` (will be rewritten — describes the dead UDP system)
- **Decision required (open question 1):** `msg/SpheroUWBStatus.msg` — keep + revise, or delete entirely. Recommended: **revise** (see §3 below).

### At workspace root
- **Delete:** `/home/svaghela/ros2_ws_2/tcp_range_reader.py` (dead TCP fallback for the old anchor pipeline; not referenced by any package)

### In `CMakeLists.txt` / `package.xml`
- `CMakeLists.txt` — replace `sphero_uwb_positioning_node.py` install entry with the new node filename.
- `CMakeLists.txt` — `rosidl_generate_interfaces` block: keep if revising the message, drop entirely if dropping the message.
- `package.xml` — current `<depend>` set covers what we still need (`rclpy`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `visualization_msgs`, `diagnostic_msgs`). **Add** `<exec_depend>python3-bleak</exec_depend>` (rosdep key for the bleak Debian/PyPI package; may need to fall back to a pip install note in README if rosdep key is absent on Rolling).

## 2. New Node Design

### Name and file
- **Module:** `sphero_uwb_positioning/ble_position_node.py`
- **Console executable:** `ble_position_node` (installed via `install(PROGRAMS … RENAME ble_position_node)` in CMakeLists.txt, mirroring the existing pattern)
- **Node name (logical):** `uwb_ble_position_node`

### Responsibilities
1. Run a long-lived `bleak.BleakScanner` with a `detection_callback`.
2. Filter advertisements by name prefix (`UWB-T`) and the `0xFFFF` company-id manufacturer record.
3. Decode the 7-byte payload, look up `tag_id` in the configured `tag_id → sphero_name` map, drop unknown tags (with throttled debug log).
4. For each known tag, publish:
   - `geometry_msgs/PoseStamped` on `sphero/<sphero_name_safe>/uwb/position` (mirrors `sphero_instance_controller` namespacing convention)
   - `sphero_uwb_positioning/UwbTagStatus` (revised message — see §3) on `sphero/<sphero_name_safe>/uwb/status`
5. Maintain an in-memory liveness table; if a tag is not seen within `tag_stale_timeout_s` (default 3.0 s — slightly above the firmware's 2.5 s `STALE_TIMEOUT_MS`), mark stale and stop emitting fresh `position` messages but continue emitting `status` with `is_stale=True`.
6. Publish anchor positions as a `MarkerArray` on `/uwb/anchors_markers` for RViz (positions read from config purely for visualization — see open question 2).
7. Publish `diagnostic_msgs/DiagnosticArray` on `/diagnostics` at 1 Hz with per-tag last-seen, RSSI, and overall scanner health.

### Threading / asyncio bridge to rclpy
The cleanest pattern for a node that owns an asyncio scanner is **one background thread that owns the asyncio loop**, with thread-safe handoff to the ROS thread:

```
main()                         (rclpy thread)
  ├─ rclpy.init()
  ├─ node = BlePositionNode()
  │     ├─ creates publishers, params, timers
  │     └─ starts BleScannerThread(target=_run_scanner_loop)
  │           └─ asyncio.new_event_loop().run_until_complete(scanner_main())
  ├─ rclpy.spin(node)         ← timer callbacks publish from latest_state
  └─ shutdown: node.stop_scanner() then rclpy.shutdown()
```

- The asyncio detection callback writes the latest decoded sample for each tag into a `dict[int, TagSample]` guarded by `threading.Lock`.
- A ROS timer (default 20 Hz) reads the dict, publishes pose + status. **Publishing happens on the ROS thread**, not from inside the bleak callback — this avoids needing `MultiThreadedExecutor` or worrying about thread affinity in `rclpy`.
- This decouples advertisement rate (driven by tag motion) from publish rate (steady cadence consumers can rely on). It also means `last_seen` timestamps come from the BLE callback wall-clock, while `header.stamp` is the ROS clock at publish time.
- Shutdown: signal handler / `destroy_node()` calls `loop.call_soon_threadsafe(loop.stop)` then `thread.join(timeout=2.0)`.

### Convention parallel
The shape mirrors `sphero_instance_device_controller_node.py`:
- Hyphenated Sphero names normalized to underscores for topic segments.
- Per-instance topic prefix `sphero/<name_safe>/...` (matches multi-robot convention).
- Single ROS node owning all tags (different from `sphero_instance_controller` which is one-process-per-Sphero) — justified because the BLE scanner is a singleton system resource; multiple processes scanning compete for the same HCI socket.

## 3. Message + Topic Design

### Recommendation: revise `SpheroUWBStatus` → rename to `UwbTagStatus`
Drop fields that no longer apply (`num_anchors_visible`, `quality` — both were host-side multilateration concerns), add fields that fit the BLE pipeline:

```
# msg/UwbTagStatus.msg
std_msgs/Header header        # frame_id="sphero_arena", stamp = publish time
string sphero_name            # e.g., "SB-3660"
uint8 tag_id                  # 1..12
int16 x_cm                    # raw position from tag firmware (10 cm quantized)
int16 y_cm
int8 rssi_dbm                 # last advertisement RSSI
float64 last_seen_age_s       # seconds since last BLE adv
bool is_stale                 # true if last_seen_age_s > tag_stale_timeout_s
string ble_address            # MAC of advertising tag (debug aid)
```

Keep the message in this package (already wired through `rosidl_generate_interfaces`). Renaming the message file means consumers get a clean break — there are no consumers, so the cost is zero.

### Topics

| Topic | Type | Notes |
|---|---|---|
| `sphero/<name_safe>/uwb/position` | `geometry_msgs/PoseStamped` | **Mirrors ArUco's `/aruco_slam/<name_safe>/position` shape exactly**, so `sphero_instance_device_controller` can be parameterized to switch sources. Frame `sphero_arena`. **Position published in cm** to match ArUco SLAM (CLAUDE.md line 145). `pose.position.z = 0.0`, `orientation.w = 1.0`. |
| `sphero/<name_safe>/uwb/status` | `sphero_uwb_positioning/UwbTagStatus` | Per-tag liveness/diagnostics. |
| `/uwb/anchors_markers` | `visualization_msgs/MarkerArray` | Static anchor cubes for RViz. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Standard diagnostics topic name. |

**Naming rationale:** Putting `uwb` *under* `sphero/<name>/` (rather than the old `sphero_uwb/<name>/...` flat namespace) keeps everything about a single robot under that robot's namespace, which is what CLAUDE.md prescribes for multi-robot packages. Consumers can subscribe to either `/aruco_slam/<name>/position` or `sphero/<name>/uwb/position` interchangeably (same message type, same units, same frame).

## 4. Config Schema

`config/ble_uwb_config.yaml`:

```yaml
uwb_ble_position_node:
  ros__parameters:
    # ─── Publishing ───
    publish_rate_hz: 20.0              # ROS publish cadence (independent of BLE adv rate)
    tag_stale_timeout_s: 3.0           # Mark tag stale after this many seconds without an adv
    frame_id: "sphero_arena"

    # ─── BLE Scanner ───
    ble_adapter: "hci0"                # bleak adapter argument; null = default
    scan_active: true                  # active scan = better RSSI + faster discovery
    scan_restart_interval_s: 60.0      # Periodic scanner restart for robustness; 0 = disabled

    # ─── Tag → Sphero mapping (parallel arrays, matches existing convention) ───
    tag_ids:     [1, 2, 3, 4]
    sphero_names: ["SB-3660", "SB-74FB", "SB-3716", "SB-58EF"]

    # ─── Anchor positions (cm) — for RViz visualization only ───
    # The tag firmware computes its own position from anchor positions baked into ANCHOR_X/Y.
    # These values must match arduino/UWB_MulticastTag/UWB_MulticastTag.ino for visualization to be honest.
    anchor_ids: [0, 1, 2, 3]
    anchor_x_cm: [0.0, 305.0, 305.0, 0.0]
    anchor_y_cm: [0.0,   0.0, 305.0, 305.0]
```

Notes on config:
- Uses the same parallel-array idiom (`tag_ids` / `sphero_names`) the old config used and the old node parsed — minimizes friction for Siddharth.
- Anchor positions are config-only for RViz purposes. The firmware is the source of truth (open question 2).

## 5. Launch File

`launch/uwb_ble.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('sphero_uwb_positioning'),
        'config', 'ble_uwb_config.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='sphero_uwb_positioning',
            executable='ble_position_node',
            name='uwb_ble_position_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
```

Single launch — same shape as the existing `sphero_uwb.launch.py`.

## 6. Dependencies

### Python runtime
- `bleak >= 0.21` (currently 1.1.1 installed system-wide). Declared in `package.xml` via `<exec_depend>python3-bleak</exec_depend>`.
- If rosdep doesn't recognize `python3-bleak` on Rolling, fall back to:
  - Note in README: `pip3 install bleak` (or `sudo apt install python3-bleak`), and
  - Add `bleak` to a `requirements.txt` at the package root for documentation.

### System
- `bluez` (already installed: 5.72-0ubuntu5.5).
- BLE scan permission: `bleak` on Linux uses BlueZ via D-Bus. Standard install lets users in `bluetooth`/`plugdev` scan without root. If `BleakError: ... not authorized` shows up at runtime, the README should document:
  ```bash
  sudo setcap 'cap_net_raw,cap_net_admin+eip' $(readlink -f $(which python3))
  ```
  (caveat: applies to the system Python, not a venv) **or** add user to `bluetooth` group:
  ```bash
  sudo usermod -aG bluetooth $USER && newgrp bluetooth
  ```

### `package.xml` additions
```xml
<exec_depend>python3-bleak</exec_depend>
<!-- existing depends keep working: rclpy, geometry_msgs, std_msgs, tf2_ros, visualization_msgs, diagnostic_msgs -->
```

`tf2_ros` can be dropped if we decide not to publish anchor TFs (the old node did; the new design uses MarkerArray instead, so TF is no longer needed unless we want the anchor frames available to other nodes — open question 3).

## 7. Testing Strategy

### Without hardware
**Fake BLE broadcaster:** Trying to fake BLE advertisements on Linux without a second radio is painful. Instead, ship a **fake-publisher mode** in the node itself, gated by a parameter:

- `--ros-args -p fake_mode:=true -p fake_tag_ids:='[1,2]'` makes the node skip `BleakScanner.start()` and instead run a timer that synthesizes `TagSample` records (e.g., tag 1 oscillating in a circle, tag 2 random walk inside the anchor footprint).
- Lets Siddharth verify topic shape, namespacing, RViz visualization, and `sphero_instance_device_controller` localization integration end-to-end without the radio.

### With one tag
Reuse `test_ble_uwb_scanner.py` as the smoke-test baseline (it already prints decoded payloads). Power on one tag and run both scripts in sequence:
1. `python3 test_ble_uwb_scanner.py` — confirm payload format unchanged from what the new firmware produces.
2. `ros2 launch sphero_uwb_positioning uwb_ble.launch.py` — confirm `ros2 topic echo sphero/SB-3660/uwb/position` shows the same x/y as the standalone scanner.

### With ArUco
With ArUco SLAM running and a robot with a tag:
- `ros2 topic hz /aruco_slam/SB-3660/position` and `ros2 topic hz sphero/SB-3660/uwb/position` should both tick.
- In RViz, switch the PoseStamped display source between the two topics — they should agree to within ~10 cm (tag firmware's quantization).

### Acceptance criteria
- All configured tags publish `position` within 5 s of being powered on.
- `is_stale=True` appears within 4 s of a tag being powered off.
- Scanner survives BLE adapter reset (`sudo hciconfig hci0 down && sudo hciconfig hci0 up`) — `scan_restart_interval_s` covers this.

## 8. Open Questions (block execution)

1. **Keep or drop `SpheroUWBStatus` message file?** Recommended: **revise + rename** to `UwbTagStatus` with the new schema in §3. Alternative: drop the custom message entirely and use only `geometry_msgs/PoseStamped` + `diagnostic_msgs/DiagnosticArray`. Per-tag rich status is useful enough that keeping a custom message wins on UX, but it's Siddharth's call.
2. **Anchor positions in YAML — duplicate or skip?** The tag firmware bakes anchor coordinates into `#define ANCHOR_X/Y`. Putting them in YAML again is a source-of-truth split. Recommended: **keep in YAML for RViz markers only**, with a README note that the firmware constants are authoritative. Alternative: skip RViz anchor markers and don't track anchor positions on the host at all.
3. **Drop `tf2_ros` dependency?** Old node published static TFs for anchors. New design uses MarkerArray. If no other consumer needs `uwb_anchor_<id>` TF frames (none currently do — verified by grep), we can drop the dependency. Recommended: **drop**.
4. **Fused position topic?** When both ArUco and UWB are publishing for the same Sphero, do we want a separate fusion node / topic (e.g., `sphero/<name>/position` as fused output)? Recommended: **out of scope for this rebuild** — wire them as parallel sources first, fuse later. `sphero_instance_device_controller`'s `external_localization` parameter already lets the user pick one source.
5. **Stale behavior — stop publishing or publish-with-flag?** Two options:
   - (a) When tag is stale, stop emitting `position` PoseStamped (consumers see topic go silent).
   - (b) Keep publishing the last known position with the status message setting `is_stale=True`.

   Recommended: **(a) stop emitting `position`, but keep emitting `status` with `is_stale=True`**. This matches how ArUco SLAM behaves (no marker → no PoseStamped) and avoids consumers acting on stale data.
6. **Scanner singleton enforcement.** Should the node refuse to start if another BLE scanner is already running on the same adapter? Recommended: log a warning and let bleak surface the error; don't add custom interlock.

## 9. Step-by-Step Execution Order

After approval:

1. **Delete dead files** (one commit):
   - `git rm src/sphero_uwb_positioning/sphero_uwb_positioning/sphero_uwb_positioning_node.py`
   - `git rm src/sphero_uwb_positioning/sphero_uwb_positioning/udp_range_reader.py`
   - `git rm src/sphero_uwb_positioning/config/uwb_config.yaml`
   - `git rm src/sphero_uwb_positioning/launch/sphero_uwb.launch.py`
   - `git rm src/sphero_uwb_positioning/README.md`
   - `git rm tcp_range_reader.py` (workspace root)
   - `rm -rf src/sphero_uwb_positioning/sphero_uwb_positioning/__pycache__ build/sphero_uwb_positioning install/sphero_uwb_positioning`

2. **Update message** (depends on Q1 answer):
   - If revising: replace `msg/SpheroUWBStatus.msg` with `msg/UwbTagStatus.msg` per §3, update `CMakeLists.txt` `rosidl_generate_interfaces` line, then build to confirm message generates.
   - If dropping: remove `msg/` directory and the `rosidl_generate_interfaces` block from `CMakeLists.txt`, drop `rosidl_default_generators` and `rosidl_default_runtime` from `package.xml`, and remove `<member_of_group>rosidl_interface_packages</member_of_group>`.

3. **Write new node** at `sphero_uwb_positioning/ble_position_node.py`:
   - Param declaration block
   - `BlePositionNode(Node)` class with the asyncio-thread bridge described in §2
   - `BleScannerThread` helper
   - `detection_callback` (decode 7 bytes, lookup in `tag_to_sphero`, atomically update shared dict)
   - `_publish_callback` ROS timer (drain shared dict, emit pose + status)
   - `_diagnostics_callback` ROS timer
   - `_publish_anchor_markers` (one-shot at startup, latched)
   - Fake mode (Q5 — for testing strategy §7)
   - `main()` entry

4. **Write new launch + config**:
   - `launch/uwb_ble.launch.py` per §5
   - `config/ble_uwb_config.yaml` per §4

5. **Update build wiring**:
   - `CMakeLists.txt`: rename installed program to `ble_position_node`
   - `package.xml`: add `<exec_depend>python3-bleak</exec_depend>`; drop `tf2_ros` if Q3 = drop

6. **Write fresh README** at `src/sphero_uwb_positioning/README.md` documenting:
   - New BLE pipeline architecture
   - Topic/message reference
   - Config sample
   - BLE permission notes
   - Test instructions (incl. fake mode)

7. **Build + verify**:
   ```bash
   colcon build --packages-select sphero_uwb_positioning
   source install/setup.bash
   ros2 launch sphero_uwb_positioning uwb_ble.launch.py
   ros2 topic list | grep uwb
   ros2 topic echo sphero/SB-3660/uwb/status
   ```

8. **Hardware integration test** (Siddharth, with one tag powered):
   - Compare topics against `test_ble_uwb_scanner.py` baseline
   - Verify `sphero_instance_device_controller` with `external_localization:=true` accepts UWB poses (note: requires a small param indirection or topic remap to point it at `sphero/<name>/uwb/position` instead of `/aruco_slam/<name>/position`; flag as a follow-up if needed).

9. **Commit** in clear chunks (delete commit, scaffolding commit, node commit, README/config commit) for reviewability.
