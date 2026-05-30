# Distributed BLE Worker Nodes (16 Spheros across 4 RPi4s)

**Created:** 2026-05-29T02:10:00Z
**Updated:** 2026-05-29T02:45:00Z (Phases 1 & 2 approved and in progress; Phase 0 skipped per user — multi-host CycloneDDS baseline assumed working; launcher agent built as a new package `sphero_worker_agent`)
**Status:** Approved — Phases 1 & 2 In Progress

## Finalized Decisions (from user)

- **O1 — DDS: RESOLVED.** Run **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
  with **multicast on a dedicated switched LAN**. Cross-host discovery already
  works on this setup. Design straight for CycloneDDS multicast — **no Fast DDS
  Discovery Server fallback**. DDS reachability is treated as a given.
- **O3 — BLE cap: RESOLVED.** ~4 simultaneous links/Pi is validated. Per-Pi cap
  locked at **4** (kept as a config constant, but 4 is the committed value).
- **O4 — Capacity: RESOLVED.** Exactly-16 is **not** hard. Degraded-to-12 when a
  Pi is offline is acceptable. **No 5th Pi, no redundancy hardware in v1.**
- **O2 — Worker host/IP mapping (v1 default):** small config (`config/workers.yaml`)
  per worker: `name`, `host`/IP, `ssh_user` (deploy only), agent `port`, `capacity`.
  **User must fill in real hostnames/IPs at deploy time** — left as placeholders.
- **O5 — Workspace sync (v1 default):** **git pull `deploy` branch + `colcon build`
  per Pi**, driven by a small sync script. Treated as an **operational
  prerequisite**, not automated by the coordinator in v1.
- **O6 — Manual worker override:** **Not in v1.** Least-loaded auto-assignment
  only. Listed as a future add.

## Task Description

Spread up to 16 Spheros across 4 RPi4 worker nodes (~4 per Pi) so each Pi's
BLE radio drives ~4 concurrent Sphero links instead of the single coordinator
host (rpi5-main) trying to hold all 16. Motivation is BLE
bandwidth/connection limits: one Bluetooth adapter saturates well before 16
concurrent Sphero links. The coordinator/webserver stays on rpi5-main; the 4
RPi4s become remote BLE worker nodes. This document is a **design plan only**
— no production code is written yet.

## Analysis — Current State (verified against the code)

**Per-Sphero process tree.** `add_sphero()`
(`multirobot_webapp.py:248`) spawns exactly one process today:

```
subprocess.Popen(['ros2','run','sphero_instance_controller',
                  'sphero_instance_websocket_server.py', name, port[, 'true']])
```

That websocket-server process is the **root** of the per-Sphero tree. On
startup it auto-starts (`main()` → `auto_start_controllers`) and
`start_controllers()` (`sphero_instance_websocket_server.py:333`) spawns **three
more** local children, each in its own process group (`os.setpgrp`):

- `sphero_instance_device_controller_node.py` — **holds the BLE link**
  (`device_controller_node.py:27` `from spherov2 import scanner`, `:572`
  `scanner.find_toy(toy_name=...)`, `:603` `with SpheroEduAPI(toy=robot)`).
- `sphero_instance_task_controller_node.py`
- `sphero_instance_statemachine_controller_node.py`

So per Sphero there are **4 processes**: websocket server + 3 controllers.
The BLE radio is touched only by the device controller. For 4 Spheros/Pi that
is 4 BLE links and ~16 processes per RPi4.

**Implication:** the entire 4-process tree for a Sphero must run on the **same**
worker Pi (the websocket server spawns its controllers as local subprocesses;
that spawn model cannot be split across hosts without restructuring). So the
unit of placement is "one Sphero → one Pi," and the existing websocket-server
self-launch design is reused as-is on the worker.

**Localization contract.** The device controller subscribes to
`/localization/<name_safe>/position` (`device_controller_node.py:121-123`,
plain `PoseStamped`, depth-10 default QoS). The active source
(aruco/matrix/uwb) and the FleetNode both run on rpi5-main. So this topic
**must cross hosts** to reach each worker's device controller. Same for the
command topics `sphero/<name_safe>/{led,roll,task,...}` published by the
websocket server (which will now run on the worker) — those are intra-worker
(publisher and subscriber both on the worker), so only `/localization/...`
truly needs to fan out from rpi5 to the workers. FleetNode also **consumes**
`/sphero/<name_safe>/sensors` and `/sphero/<name_safe>/battery` published by the
worker's device controller — so telemetry must flow worker → rpi5 as well.

**Lifecycle assumptions that break for remote workers.** Three spots assume a
**local** `subprocess.Popen` handle stored as `instance['process']`:

- `add_sphero` stores `instance_info['process'] = process` and polls
  `process.poll()` after a 2s sleep to confirm startup
  (`multirobot_webapp.py:302,320`).
- `remove_sphero` calls `process.terminate()` / `process.kill()` on that handle
  (`:379,386`).
- `get_all_instances` / `get_instance` poll `instance['process'].poll()` for
  liveness (`:417,441`).

A process on a remote Pi has no local `Popen` handle, so all three need a
remote-aware equivalent (remote start, remote stop, remote liveness).

**Logical assignment already done at add-time** (keep all of it): web port
(`self.next_port++`), UWB `tag_id` (1-16), marker slot. The instance `url` is
hardcoded `http://localhost:{port}` (`:311`) — **breaks** once the websocket
server runs on a worker; must become `http://<rpiN>:{port}`.

**Assumptions:**
- A1: All 5 boards (rpi5-main + 4× rpi4) sit on the **dedicated switched LAN**
  and reach each other; CycloneDDS multicast discovery already works there
  (O1 resolved). Keep rpi5 on the wired/switched link (not the WLAN AP-fallback
  path) during fleet operation.
- A2: Each RPi4 has its own Bluetooth adapter and the `spherov2` stack + the
  built `sphero_ros2` workspace available.
- A3: Target is 4 Spheros/Pi (cap locked at 4, O3 resolved); 16 total is the
  ceiling (matches the 16-tag, 16-marker pools). 4 Pis × 4 = 16 exactly, so
  there is no spare capacity — one Pi offline caps the fleet at 12, which is
  **accepted** (O4 resolved).
- A4: rpi5-main keeps running the webserver, the active positioning source, and
  FleetNode. Workers run only Sphero instance trees.

## Detailed Plan (Design Decisions per Required Area)

### 1. Node-assignment strategy

**Decision: least-loaded-by-count with a per-Pi cap of 4. Auto-assignment only —
no manual override in v1 (O6).**

- Maintain a worker registry on the coordinator: `workers = {'rpi4-1': {host,
  capacity=4, count, online}, ...}` loaded from `config/workers.yaml`. The cap
  stays a config constant per worker, but **4 is the committed value** (O3).
- On `add_sphero`, pick the online worker with the fewest assigned Spheros that
  is below capacity. Ties broken by config order (deterministic). This balances
  BLE load better than round-robin when Spheros are removed/re-added unevenly.
- **Pi full:** if all online workers are at capacity, reject with a clear
  message (`"all workers at capacity (N/16 BLE slots used)"`). Do not silently
  overload a radio — that defeats the purpose.
- **Pi offline:** excluded from selection (fleet degrades to 12, accepted — O4).
  Spheros already assigned to an offline Pi are marked `status: 'worker_offline'`
  (see §5), not auto-moved.
- Why not round-robin: with add/remove churn, round-robin can pile multiple
  live links on one radio while another sits idle. Least-loaded directly
  optimizes the thing we care about (per-radio link count).
- **Future (not v1):** optional manual `worker` override (add-request field / UI
  dropdown) honored when that worker is online and under cap.

### 2. Remote launch mechanism

**Recommendation: a lightweight per-Pi launcher agent (small HTTP service),
NOT raw SSH-spawn, NOT a custom ROS lifecycle layer.**

Compared options:

- **SSH-spawn** (`subprocess` → `ssh pi@rpiN ros2 run ...`): lowest setup, but
  weak on the things that matter here. Process lifetime is tied to the SSH
  channel; clean teardown of the whole 4-process tree (the websocket server +
  its 3 setpgrp children) over SSH is fiddly; liveness = "is the SSH PID
  alive," which doesn't see a crashed grandchild; log capture means scraping
  SSH stdout; needs passwordless keys to every worker. Workable but brittle for
  long-running fleets.
- **Per-Pi launcher agent (HTTP)** *(recommended)*: a tiny Flask/uvicorn service
  on each RPi4 exposing `POST /spawn` (name, port, external_localization),
  `DELETE /spawn/<name>`, `GET /status`, `GET /health`. It runs the **same**
  `ros2 run ... sphero_instance_websocket_server.py` locally (so the existing
  self-launch of the 3 controllers is unchanged), tracks the local `Popen`
  handle, does process-group teardown locally (reusing the proven
  `os.killpg(os.getpgid(pid), SIGTERM)` pattern already in
  `stop_controllers`), captures logs to per-Sphero files on the worker, and
  reports real liveness via `poll()`. The coordinator's `add/remove/list`
  become HTTP calls. This keeps the messy local-process management **on the
  same host as the processes**, which is exactly where the current code already
  does it well.
- **ROS 2-native** (lifecycle nodes / a launch service / `launch_ros` remote):
  most "correct" ROS-wise but highest build cost and a large rewrite of the
  websocket-server self-launch model; overkill for spawning a known fixed
  process per Sphero. Defer.

Rationale: the launcher agent isolates remote concerns behind a clean,
testable API; reuses the existing process-tree + teardown logic verbatim on the
worker; gives honest liveness and real log capture; and only needs HTTP
reachability (no SSH key sprawl). Security: bind the agent to the LAN, add a
shared bearer token in a header (workers + coordinator share a secret in env);
this is a closed lab LAN so we keep it minimal.

### 3. ROS 2 multi-host setup

**Decision (O1 resolved): single shared `ROS_DOMAIN_ID` across all 5 hosts on
CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) using multicast discovery on
the dedicated switched LAN. No Discovery Server fallback.**

- Every board exports the **same** `ROS_DOMAIN_ID` (e.g. 42) and
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. CycloneDDS multicast discovery on the
  switched LAN already works on this setup (same configuration behind the earlier
  `eth0` CycloneDDS error that was fixed by enabling the switch), so all nodes
  discover each other automatically. `/localization/<name>/position` published on
  rpi5 reaches the worker device controllers, and worker-published
  `/sphero/<name>/sensors` + `/battery` reach FleetNode — DDS handles the fan-out.
  **DDS reachability is treated as a given; this design does not re-solve it.**
- **Required matching env on every Pi (and the coordinator):**
  - `ROS_DOMAIN_ID` — identical on all hosts.
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
  - CycloneDDS interface/peer config so it binds the switched-LAN NIC and not a
    stray interface — typically `CYCLONEDDS_URI` pointing at a small XML that
    sets `General/Interfaces/NetworkInterface[@name]` (or `name=...`) to the
    LAN device. (This is the knob that the earlier `eth0` error exercised.)
  - The launcher agent (§2) **injects this env into every spawned Sphero
    process**, so worker children inherit the correct domain/RMW/interface.
- Per-Pi prerequisite: sourced workspace and firewall open for CycloneDDS UDP on
  the LAN (default discovery/data ports ~7400+). Closed switched LAN, so minimal.
- Why not per-node domains + bridging: adds a domain-bridge process and config
  for zero benefit here — all nodes are ours and want to see the same small set
  of topics.

### 4. Coordinator state & web UI impact (design-level)

- **Worker registry** (new): static config (e.g. `config/workers.yaml`) listing
  each worker `name`, `host`/IP, agent `port`, `capacity`. Loaded by
  `SpheroInstanceManager.__init__`. Live counts tracked in memory.
- **`instance` dict** gains: `worker` (which Pi), `host` (IP/hostname),
  and the `url` becomes `http://{host}:{port}` instead of `localhost`. The
  local `process` handle is **replaced** by remote identity (`worker` + `name`);
  liveness comes from the agent's `GET /status`, not `process.poll()`.
- **`add_sphero`**: after computing port/tag/marker, call assignment (§1) to
  pick a worker, then `POST` to that worker's agent to spawn; on success store
  the remote-aware instance dict; on failure roll back the tag/marker
  allocation. FleetNode registration (`add_robot`) is unchanged (it keys on
  name/tag/marker, host-agnostic).
- **`remove_sphero`**: `DELETE` to the owning worker's agent instead of
  `process.terminate()`; then free port/tag/marker and `fleet_node.remove_robot`.
- **`get_all_instances` / `get_instance`**: liveness via cached agent status
  (refreshed by the health poller in §5) instead of `process.poll()`.
- **API responses** (`/api/spheros`): add `worker` and corrected remote `url`
  to each instance payload. Optional new `GET /api/workers` returning per-worker
  online/capacity/count for the UI.
- **Web UI**: show each Sphero's worker badge; the "open controller" link uses
  the remote `url`; an optional worker-status panel (online, 3/4 used).
  (Manual worker-override dropdown is deferred to a future version, per §1/O6.)

### 5. Health / failure handling

- **Worker health poller** on the coordinator: periodic `GET /health` +
  `GET /status` per worker (e.g. every few seconds). Marks workers
  online/offline and reconciles each Sphero's reported liveness.
- **Dead Sphero process** (agent reports a Sphero's tree exited, e.g. toy not
  found — the existing `_shutdown_after_delay` path makes the websocket server
  exit non-zero): coordinator marks that instance `status: 'stopped'` and
  surfaces it in the UI; does **not** auto-respawn (matches today's behavior —
  toy-not-found is terminal until user retries).
- **Unreachable Pi**: mark worker offline; its Spheros become
  `status: 'worker_offline'`. **No auto-rebalance in v1** (moving a live BLE
  link mid-task is disruptive and the fleet is at exact capacity). Provide a
  manual "reassign" action that removes from the dead Pi's records and re-adds
  on a healthy Pi when one has a free slot.
- **Clean teardown on remove**: the agent does the local process-group kill
  (reuse the SIGTERM→SIGKILL-after-timeout pattern from `stop_controllers`),
  so BLE links are released deterministically on the worker. Coordinator frees
  the logical resources only after the agent confirms.
- **Coordinator shutdown**: iterate instances and `DELETE` each on its worker
  (replaces today's local loop at `:826`), so workers don't leak Sphero trees.

### 6. Deployment prerequisites (per RPi4)

- Built/synced `sphero_ros2` workspace at a known path; `source install/setup.bash`
  in the agent's environment so `ros2 run sphero_instance_controller ...`
  resolves. **Workspace sync (O5 decision): git pull the `deploy` branch +
  `colcon build` locally on each Pi**, driven by a small sync script. Chosen
  over rsync-the-`install/` or NFS because arch/Python ABI must match exactly and
  the device controller imports `spherov2` natively; building on each Pi
  guarantees a matching ABI. Tradeoff: builds are slower per Pi and require the
  toolchain on every worker, but correctness is unambiguous. **This is an
  operational prerequisite — the coordinator does not automate sync in v1**; a
  `make deploy`-style fan-out script (Phase 4) keeps the 4 workers + coordinator
  in lockstep.
- `spherov2` Python package + BLE stack (BlueZ) installed and the Bluetooth
  adapter usable by the run user (often needs `bluetooth` group / capabilities).
- The launcher agent installed and started on boot (systemd unit), bound to the
  LAN, with the shared token in its env.
- Same `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + CycloneDDS
  interface/peer config (`CYCLONEDDS_URI`) exported in the agent's environment so
  spawned Sphero processes inherit them (§3).
- Firewall: CycloneDDS UDP ports + the agent's HTTP port open on the LAN.

## Expected Outcomes

- Each RPi4 holds ~4 BLE links; no single radio exceeds its connection budget.
- Coordinator transparently places, starts, stops, and monitors Spheros on
  remote workers; UI shows which Pi each Sphero is on and links to the remote
  controller URL.
- The `/localization/<name>/position` contract and `sphero/<name>/...` +
  telemetry topics work unchanged across hosts via shared-domain DDS.

## Potential Risks & Considerations

- **No spare capacity** (4×4=16 exactly): one Pi down = max 12 Spheros.
  **Accepted for v1** (O4) — no 5th Pi / redundancy hardware.
- **Workspace drift** across 5 boards is the top operational risk; the git-pull
  + per-Pi build sync script (O5) must be run disciplinedly before each fleet run.
- **AP fallback** (rpi5 flips to AP after WLAN stall, per memory): the switched
  LAN (eth) is the DDS path, so keep rpi5 on the wired/switched link during fleet
  runs; a WLAN AP flip should not affect the LAN peers but must not become the
  active DDS interface.
- **Single Bluetooth adapter per Pi** must actually sustain the 4 links under
  task load (validated at design time, O3); re-bench if link behavior degrades.
- **Security is minimal** (shared token, LAN-bound) — fine for a closed lab,
  not for an open network.

## Open Questions / Decisions — Status

- **O1 — RESOLVED.** CycloneDDS multicast on the dedicated switched LAN; cross-host
  discovery already works. No Discovery Server fallback. (§3)
- **O3 — RESOLVED.** 4 BLE links/Pi validated; cap locked at 4 (config constant). (§1)
- **O4 — RESOLVED.** Exactly-16 not required; degraded-to-12 on a Pi outage is
  accepted; no 5th Pi / redundancy in v1. (§1, Risks)
- **O2 — v1 DEFAULT (user must supply values).** `config/workers.yaml` schema:
  per worker `name`, `host`/IP, `ssh_user`, agent `port`, `capacity`.
  **Real hostnames/IPs are placeholders the user fills in at deploy time.**
- **O5 — v1 DEFAULT.** git pull `deploy` + `colcon build` per Pi via a sync
  script; an operational prerequisite, not coordinator-automated. (§6)
- **O6 — DEFERRED (not v1).** Least-loaded auto-assignment only; manual override
  is a future add. (§1)

**Still required from the user before/at deploy:** the actual worker hostnames/IPs
(and ssh users) for `config/workers.yaml`, plus confirmation of the LAN NIC name
to set in the CycloneDDS interface config on each Pi.

## Phased Implementation Outline (for approval — no code until approved)

Each phase is independently verifiable and builds on the prior one. No
production code is written until this outline is approved.

**Phase 0 — Multi-host baseline (infra, mostly verification).**
- Delivers: confirmed CycloneDDS env on rpi5 + at least one RPi4 (same
  `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`
  bound to the LAN NIC); a manually-launched Sphero tree on that worker.
- Verify: `ros2 topic echo /localization/<name>/position` on the worker sees
  rpi5's publisher, and FleetNode on rpi5 sees the worker's
  `/sphero/<name>/sensors` + `/battery`; the worker holds 4 live BLE links
  stably under a simple roll task.

**Phase 1 — Per-Pi launcher agent + systemd.**
- Delivers: a small HTTP agent on each RPi4 — `POST /spawn` (name, port,
  external_localization), `DELETE /spawn/<name>`, `GET /status`, `GET /health`
  — that runs the existing `ros2 run ... sphero_instance_websocket_server.py`
  locally (3 controllers self-launch unchanged), tracks the local `Popen`,
  injects the CycloneDDS env, captures per-Sphero logs, and does process-group
  teardown (reusing the `os.killpg(... SIGTERM)` pattern from `stop_controllers`).
  Shared bearer token; LAN-bound. Installed as a boot systemd unit on all 4 Pis.
- Verify: `curl` spawn → the Sphero's 4-process tree appears on that Pi and the
  BLE link connects; `curl` delete → tree is gone and the link releases; this
  works independently on each Pi; agent survives a reboot (systemd).

**Phase 2 — Coordinator worker registry + assignment.**
- Delivers: `config/workers.yaml` (placeholder hosts/IPs) loaded by
  `SpheroInstanceManager.__init__`; in-memory live counts; least-loaded
  (cap 4) selection with deterministic tie-break and an "all at capacity"
  rejection; offline workers excluded.
- Verify: with stubbed/real agents, repeated `add_sphero` distributes Spheros
  least-loaded across workers, respects the cap, and rejects at 16
  (or 12 with one worker marked offline).

**Phase 3 — Remote add/remove/list + UI host/url.**
- Delivers: `add_sphero` → `POST` to the selected worker's agent (roll back
  tag/marker/port on failure); `remove_sphero` → `DELETE` to the owning agent;
  `get_all_instances`/`get_instance` liveness via cached agent status instead
  of local `process.poll()`; instance `url` becomes `http://{host}:{port}`;
  `/api/spheros` payload gains `worker` + remote `url`; UI shows a worker badge
  and the "open controller" link uses the remote url; coordinator shutdown
  `DELETE`s each instance on its worker.
- Verify: add/remove from the webserver lands on the correct Pi; the controller
  link opens the remote URL; tag/marker/port pools stay consistent across
  add/remove churn; coordinator shutdown leaves no orphaned trees on workers.

**Phase 4 — Health polling + worker_offline handling.**
- Delivers: periodic `GET /health`+`GET /status` poller; workers flipped
  online/offline; per-Sphero `status` reconciled (`stopped` for a crashed tree —
  no auto-respawn; `worker_offline` for an unreachable Pi — no auto-rebalance);
  optional `GET /api/workers` and a worker-status panel in the UI; a manual
  "reassign" action (remove from dead Pi's records, re-add on a healthy Pi with
  a free slot).
- Verify: kill a worker → UI shows it offline and its Spheros as
  `worker_offline`; kill a single Sphero tree → that instance shows `stopped`;
  manual reassign re-places a stranded Sphero on a healthy Pi.

**Phase 5 — Deploy tooling & docs.**
- Delivers: a `make deploy`-style fan-out script that, per worker, git-pulls the
  `deploy` branch and runs `colcon build` (O5), then restarts the agent;
  updates to `doc/package.md` + `doc/development.md`.
- Verify: one command brings all 4 workers to the current build; docs describe
  the worker setup, required CycloneDDS env, and the workers.yaml schema.

## Approval Status
- [x] Waiting for user approval
- [x] Approved (Phases 1 & 2; Phase 0 skipped — CycloneDDS multi-host baseline assumed working)
- [ ] Executed

## Implementation Progress
- **Phase 0** — SKIPPED per user (multi-host CycloneDDS baseline assumed working).
- **Phase 1** — DONE (built, lifecycle/auth/teardown verified). New ament_python package `src/sphero_worker_agent`:
  HTTP launcher agent (`POST /spawn`, `DELETE /spawn/<name>`, `GET /status`,
  `GET /health`), bearer-token auth, LAN-bindable, CycloneDDS env injection,
  per-Sphero log capture, process-group teardown reusing the
  `os.killpg(os.getpgid(pid), SIGTERM→SIGKILL)` pattern. systemd unit template.
  Console script `ros2 run sphero_worker_agent agent`.
- **Phase 2** — DONE (built, selection logic unit-tested — 9/9 pass). `multirobot_webserver`: `config/workers.yaml`
  schema + example; worker registry + least-loaded selection (cap 4 config
  constant, deterministic tie-break, reject-when-full, skip-offline) loaded by
  `SpheroInstanceManager.__init__`. Registry/selection only — `add_sphero`
  still spawns locally (Phase 3 wires the remote agents).
- **Phase 3+** — NOT STARTED (remote add/remove/list wiring + UI host/url).
