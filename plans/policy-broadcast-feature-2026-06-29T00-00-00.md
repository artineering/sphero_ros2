# Policy Broadcast Feature — Implementation Plan (Phase 1: ROS2 message + controller support)

**Status:** APPROVED 2026-06-29. Decisions locked (see below). Phase 1 = ROS2 message + controller support, published via broadcast topic / CLI. Web UI is a later phase.

## Locked decisions
- **Transport:** single latched fleet-wide topic `/fleet/policy` (QoS `transient_local`, depth 1).
- **Similarity cue type:** selectable per policy (Identity = fixed matrix color | Behavior = blinking), carried in the message.
- **Common Fate leader:** named explicitly in the policy.
- **Proximity:** real Kinect-based (subscribe to `/localization/<name>/position`, compute distance, react under `threshold_cm`). Silently no-ops when the Kinect tracker is not running.
- **Proximity reaction:** STOP (halt motion). Keep the `reaction` field {FLASH, STOP} in the message for future flexibility; default/use STOP in Phase 1.
- **Cue stacking:** flags are independent; S and CF can both apply to the same member simultaneously.
- **members[] form:** raw callsigns (e.g. `SB-33E3`) with tolerant normalization on the controller side.
- **Concurrency:** single active fleet policy in Phase 1 (replace by republish, revoke via `active=false`).
- **Similarity render surface:** LED matrix (BOLT); non-BOLT units no-op.

## Grounding (verified file:line)
- Controller: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_device_controller_node.py`
  - Subscribers built in `_create_subscribers()` (lines 231-279), all `std_msgs/String` JSON on `sphero/<name_safe>/<cmd>`.
  - `self.sphero_name` (raw, e.g. `SB-33E3`) + `self.topic_name_safe` (`SB_33E3`) at lines 183-186.
  - Reusable callbacks: `matrix_callback` (511-548), `ir_callback` (618-650).
  - Optional own-position sub `/localization/<name_safe>/position` at lines 233-238 (gated on `external_localization` param).
- Core API: `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero.py`
  - `set_matrix(...)` (370-418), `clear_matrix()` (420-435), `flash_led(...)` (123-142), `stop()` (279-294).
  - IR: `start_ir_broadcast/stop` (566-604), `start_ir_follow/stop` (606-644) — all `Processors.SECONDARY`, `(near,far)` clamped 0-7.
- Blink reference: `scripts/blink_fleet.py` fills the 8x8 with `matrix=[1]*64` at a color.
- IR dispatch (central, web): `multirobot_webserver/.../multirobot_webapp.py:898 dispatch_ir_formation`; `IR_CHANNEL_PAIRS = [(0,1),(2,3),(4,5),(6,7)]` at line 78.
- Localization: `kinect_field_tracking/.../field_tracker_node.py:1141` publishes `PoseStamped` on `/localization/<name_safe>/position`, frame `field`, units **cm**, ~10 Hz.
- Fleet msgs: `multirobot_msgs/msg/` built via `rosidl_generate_interfaces` in `multirobot_msgs/CMakeLists.txt` (deps `std_msgs`, `geometry_msgs`, `builtin_interfaces`).
- **No proximity/distance-threshold logic exists today.** Collision detection (439-492) is contact-only.

## 1. Policy message schema
New fleet-level messages in `multirobot_msgs/msg/`. Nested sub-messages so the later web layer maps cleanly.

`FleetPolicy.msg`
```
std_msgs/Header header
string   policy_id          # opaque id; lets web/CLI track/replace a policy
bool     active             # false => REVOKE: every member clears ALL cues
string[] members            # callsigns, raw form e.g. "SB-33E3"
bool     enable_similarity  # S
bool     enable_proximity   # P
bool     enable_common_fate # CF
SimilarityCue similarity
ProximityRule proximity
CommonFate    common_fate
```
`SimilarityCue.msg`
```
uint8 IDENTITY=0     # fixed color on LED matrix
uint8 BEHAVIOR=1     # blinking
uint8 cue_type
uint8 red
uint8 green
uint8 blue           # cue color (Identity steady; Behavior on-phase)
float32 blink_hz     # used when cue_type==BEHAVIOR
```
`ProximityRule.msg`
```
float32 threshold_cm   # react when a counted peer is within this distance
string[] peers         # callsigns that count; empty => all other members
uint8 FLASH=0
uint8 STOP=1
uint8 reaction
```
`CommonFate.msg`
```
string leader        # callsign of leader; all other members follow
uint8  base_channel  # 0..3 -> index into IR_CHANNEL_PAIRS (near,far)
```

- **Role determination:** controller is *leader* iff its own callsign == `common_fate.leader`; a CF member that is not the leader is a *follower*. Fully decentralized — no central assignment.
- **Clearing semantics:** on every message the controller recomputes its desired cue set. Any cue not in the new set — or `active=false`, or self no longer in `members[]` — is torn down via the matching stop/clear API. "Flag off", "left policy", and "revoked" share one code path.
- **Topic:** single latched `/fleet/policy` (QoS `transient_local`, depth 1). Every node needs the whole member list + leader name to compute its own role; latched QoS means restarted/reconnected controllers immediately get the current policy.

## 2. Controller changes
File: `sphero_instance_device_controller_node.py`.
- Import `from multirobot_msgs.msg import FleetPolicy` (near line 28).
- In `_create_subscribers()` (231): add `transient_local` depth-1 subscription `/fleet/policy` -> `policy_callback` (explicit `QoSProfile` with `durability=TRANSIENT_LOCAL`).
- New state in `__init__` (after 206): `_policy_blink_timer`, `_active_policy_id`, `_policy_cf_role`, `_proximity_subs`, `_peer_positions`, `_my_position`.
- `policy_callback(msg)`: normalize membership (`m.replace('-','_') == self.topic_name_safe`); if `not active` or not member -> `_clear_all_policy_cues()`; else apply/clear each of S/P/CF per its flag.
- `_apply_similarity(cue)`:
  - IDENTITY -> `self.sphero.set_matrix(custom_matrix=[1]*64, red, green, blue)` (reuses `set_matrix` sphero.py:370).
  - BEHAVIOR -> ROS timer at `1/(2*blink_hz)` toggling matrix fill on/off.
  - Clear -> cancel timer + `clear_matrix()`.
- `_apply_common_fate(msg)`: `near,far = IR_CHANNEL_PAIRS[base_channel]`; leader -> `start_ir_broadcast(near,far)`, follower -> `start_ir_follow(near,far)`; on role change/clear call matching stop first.
- `_apply_proximity(msg)`: subscribe to each counted peer's `/localization/<peer_safe>/position` + own position, compute Euclidean distance on a ~5 Hz timer, fire STOP reaction when `< threshold_cm`. Gate: if positions missing/stale, throttled warn + no-op.
- `_clear_all_policy_cues()`: cancel blink timer + clear_matrix; stop IR broadcast+follow; tear down proximity subs; reset state.

## 3. IR channel assignment (decentralized)
Reuse `IR_CHANNEL_PAIRS = [(0,1),(2,3),(4,5),(6,7)]`. Policy carries `common_fate.base_channel` (0..3); every controller computes `(near,far)=IR_CHANNEL_PAIRS[base_channel]` identically, so leader and followers agree with no handshake. Single-policy Phase 1 with `base_channel=0` is fine; concurrent CF policies would need distinct `base_channel` (no central allocator in Phase 1 — risk).

## 4. Build / test
- Add `<depend>multirobot_msgs</depend>` to `src/sphero_instance_controller/package.xml`.
- `colcon build --packages-select multirobot_msgs` then `--packages-select sphero_instance_controller`; `source install/setup.bash`.
- Identity: `ros2 topic pub --once /fleet/policy multirobot_msgs/msg/FleetPolicy "{active: true, members: ['SB-33E3'], enable_similarity: true, similarity: {cue_type: 0, red: 0, green: 0, blue: 255}}"` -> matrix fills blue.
- Behavior: same with `cue_type: 1, blink_hz: 2.0` -> blinks ~2 Hz. Revoke: `{active: false, members: ['SB-33E3']}` -> clears.
- CF: two BOLTs, `enable_common_fate: true, common_fate: {leader: 'SB-33E3', base_channel: 0}` -> leader broadcasts, follower follows; flip/`active:false` stops IR.
- Proximity: with kinect running, `enable_proximity: true, proximity: {threshold_cm: 30.0, peers: [], reaction: 1}`; drive units close -> stop fires; kill kinect -> graceful no-op.
- Echo: `ros2 topic echo /fleet/policy --qos-durability transient_local`.
- Hardware: matrix/IR need real BOLTs; proximity needs the kinect tracker. Membership/role/clear branching smoke-tests in sim via logs.

## 5. Residual risks
- Concurrent CF channel uniqueness — no central allocator in Phase 1 (acceptable for single policy).
- Untracked proximity peers contribute no distance (silent no-op) — acceptable per decision.
- Non-BOLT units no-op on Similarity matrix render.
