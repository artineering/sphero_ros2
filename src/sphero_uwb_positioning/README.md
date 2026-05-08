# sphero_uwb_positioning

Passive BLE scanner that republishes UWB tag positions as ROS topics.

## Architecture

The tag firmware (`arduino/UWB_MulticastTag/UWB_MulticastTag.ino`) runs the
multilateration on-device and broadcasts its computed position via BLE
manufacturer data:

- **Local name:** `UWB-T<TAG_ID>`
- **Manufacturer payload (7 bytes):** Company ID `0xFFFF` (LE) + `tag_id` (u8) +
  `x_cm` (i16 LE) + `y_cm` (i16 LE)

This package runs a single ROS node (`uwb_ble_position_node`) that owns a
`bleak.BleakScanner` on a background asyncio thread, decodes adverts, and
publishes positions on the ROS thread.

```
bleak (asyncio thread) ── lock ── dict[tag_id, TagSample] ── ROS timer ──> publish
```

No ranging or multilateration runs on the host. Position units are **cm** to
match the existing ArUco SLAM convention.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `sphero/<name_safe>/uwb/position` | `geometry_msgs/PoseStamped` | One per configured tag. Frame `sphero_arena`, units cm. `pose.position.z=0`, `orientation.w=1`. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | One `DiagnosticStatus` per tag at 1 Hz. Level OK / WARN (stale) / ERROR (never seen). |

`<name_safe>` substitutes hyphens with underscores (e.g. `SB-3660` → `SB_3660`),
matching the convention used in `sphero_instance_controller`.

### Stale handling

When a tag stops advertising for longer than `tag_stale_timeout_s`:

- `position` keeps emitting at `publish_rate_hz` with the **last known** x/y and
  a fresh `header.stamp` — downstream consumers see a live topic, not a gap.
- The matching `DiagnosticStatus` flips to `WARN` with the age in its message.

Tags that have never been seen do not publish `position` at all and report
`ERROR` in `/diagnostics`.

## Config

`config/ble_uwb_config.yaml`:

```yaml
uwb_ble_position_node:
  ros__parameters:
    publish_rate_hz: 20.0
    tag_stale_timeout_s: 3.0
    frame_id: "sphero_arena"
    ble_adapter: "hci0"
    scan_active: true
    scan_restart_interval_s: 60.0
    tag_ids:     [1, 2, 3, 4]
    sphero_names: ["SB-3660", "SB-74FB", "SB-3716", "SB-58EF"]
    fake_mode: false
    fake_tag_ids: [1, 2]
```

`tag_ids` and `sphero_names` are parallel arrays. `scan_restart_interval_s` cycles
the scanner periodically for robustness against BLE adapter glitches; set to 0 to
disable.

## BLE permissions

`bleak` uses BlueZ via D-Bus. On most Ubuntu setups, scanning works as a normal
user. If you see `BleakError: ... not authorized`, grant raw socket caps to
`python3`:

```bash
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(readlink -f $(which python3))
```

The cap is set on the system Python (e.g. `/usr/bin/python3.12`), not on a venv
interpreter. You may need to re-run after Python upgrades.

Verify with:

```bash
getcap $(readlink -f $(which python3))
```

## Run

```bash
colcon build --packages-select sphero_uwb_positioning
source install/setup.bash
ros2 launch sphero_uwb_positioning uwb_ble.launch.py
```

### Fake mode (no radio required)

For pipeline testing without a tag, run the node directly with `fake_mode:=true`.
It synthesizes oscillating positions for the `fake_tag_ids` so all downstream
consumers (RViz, `sphero_instance_device_controller`) get plausible PoseStamped
traffic:

```bash
ros2 run sphero_uwb_positioning ble_position_node --ros-args \
  -p fake_mode:=true \
  -p tag_ids:='[1,2]' \
  -p sphero_names:='[SB-3660,SB-74FB]'
```

## Smoke test

1. Confirm BLE adverts decode correctly with the standalone scanner:
   ```bash
   python3 test_ble_uwb_scanner.py
   ```
2. Power on at least one tag, then:
   ```bash
   ros2 launch sphero_uwb_positioning uwb_ble.launch.py
   ros2 topic echo sphero/SB_3660/uwb/position
   ros2 topic echo /diagnostics
   ```
3. The x/y in the PoseStamped should match what the standalone scanner prints.

## Dependencies

- `bleak >= 0.21` (system: `pip3 install bleak` or `apt install python3-bleak`)
- `bluez` (system)
- `rclpy`, `geometry_msgs`, `diagnostic_msgs`
