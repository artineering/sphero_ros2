# sphero_worker_agent

Per-Pi HTTP launcher agent for the distributed BLE worker fleet. Runs on each
RPi4 worker and lets the coordinator (`multirobot_webserver` on rpi5-main)
spawn, tear down, and inspect Sphero instance trees on that worker over HTTP.

It runs the existing
`ros2 run sphero_instance_controller sphero_instance_websocket_server.py`
invocation locally (the 3 controllers self-launch unchanged), injects the
CycloneDDS env into every spawned process, captures per-Sphero logs, and tears
down the whole process group cleanly (`SIGTERM` -> `SIGKILL` after a timeout)
so BLE links release deterministically.

## API (bearer-token auth on every route)

All requests require `Authorization: Bearer <WORKER_AGENT_TOKEN>`.

| Method | Path            | Body                                         | Description                          |
|--------|-----------------|----------------------------------------------|--------------------------------------|
| POST   | `/spawn`        | `{name, port, external_localization?, fleet_hold?}` | Spawn a Sphero tree. 200 / 409.      |
| DELETE | `/spawn/<name>` | -                                            | Tear down a Sphero tree. 200 / 404.  |
| GET    | `/status`       | -                                            | Running Spheros + liveness.          |
| GET    | `/health`       | -                                            | Liveness check (`{"status":"ok"}`).  |
| GET    | `/scan?duration=<s>` | -                                       | Advertising Spheros. 200 / 409 / 500. |

Every response from `/status`, `/health` and `/scan` includes `"worker"`, the host's
`socket.gethostname()` (matches the `name` in the webserver's `workers.yaml`).

`/scan` (duration default 4, clamped 1-10) runs a bleak discovery and returns
`{"worker", "duration_s", "robots": [{"name", "address", "rssi"}]}`, filtered to
spherov2 toy name prefixes (`SB-`, `BB-`, ...), de-duplicated by name keeping
the strongest RSSI, sorted strongest first. It takes the same per-host flock
the device controllers hold around scan+connect (`SPHERO_BLE_CONNECT_LOCK`,
default `/tmp/sphero_ble_connect.lock`). If the lock isn't free within
`duration + 5` s it returns 409 `{"busy": true, "error": "BLE connect in progress"}`
instead of scanning, so worst-case latency is about `2*duration + 5` s. A BLE
error returns 500 `{"error": ...}`. Connected Spheros stop advertising, so only
unconnected ones show up.

## Configuration (env vars)

- `WORKER_AGENT_TOKEN` - shared bearer token (REQUIRED; agent refuses to start without it).
- `WORKER_AGENT_HOST` - bind address (default `127.0.0.1`; set to the worker's LAN IP).
- `WORKER_AGENT_PORT` - bind port (default `8181`).
- `WORKER_AGENT_LOG_DIR` - per-Sphero log directory (default `/tmp/sphero_worker_agent`).
- `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` - injected into every
  spawned Sphero process. Set once per Pi (typically in the systemd unit).
- `WORKER_AGENT_LAUNCH_CMD` - optional launch-command override (testing only).
  When unset the agent uses the real
  `ros2 run sphero_instance_controller sphero_instance_websocket_server.py`.
- `WORKER_CONNECT_QUEUE` - `1` (default) runs the connect-queue ROS node (below);
  `0` keeps the agent Flask-only.
- `WORKER_CONNECT_ENTRY_TIMEOUT` / `WORKER_CONNECT_FIRST_BEACON` /
  `WORKER_CONNECT_BEACON_STALE` - queue entry limits in seconds (150 / 60 / 10).

## BLE connect queue

The agent publishes the robots it has spawned that have not finished their
initial BLE connect on `/worker/<host_token>/connect_queue` (`std_msgs/String`
JSON, reliable + transient_local, depth 1; `rpi4-node01` -> `rpi4_node01`):

```json
{"host": "rpi4-node01", "seq": 17, "stamp": 1763335130.2,
 "pending": [{"name": "SB-1E2D", "spawn_id": "6f1c...", "since": 1763335120.4,
              "stage": "spawned"}]}
```

A robot is listed at spawn (before `Popen`) and removed when its `/status`
reports `connected`/`disconnected`, on any `device_error`, on teardown or
process exit, when its `initializing`/`connecting` beacon goes silent for 10 s,
if no beacon arrives within 60 s, or after 150 s. Each spawn gets a
`SPHERO_SPAWN_ID` (echoed as `spawn_id` on `/status`); messages from an
earlier process of the same robot are ignored. The device controller's quiet
gate holds connected robots while anyone else is listed. Every change is
logged as `[connect-queue] t=<epoch> host=... add|remove|stage ...`.

```bash
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
  /worker/rpi4_node01/connect_queue
```

### Fleet hold

`POST /spawn {..., "fleet_hold": true}` spawns the robot with
`SPHERO_BLE_FLEET_HOLD=1` (otherwise `0`). After its initial connect it holds
(reporting `connected`, `holding: true`) until `/fleet/ble_release`
(`std_msgs/String` JSON, reliable + transient_local, depth 1)
`{"seq", "stamp", "spawn_ids", "reason"}` lists its `spawn_id`. A
`"spawn_ids": ["*"]` releases every held robot that connected no more than
`SPHERO_BLE_FLEET_RELEASE_SKEW` (2 s) after the release's `stamp`; an older
latched `*` is ignored. Safety cap `SPHERO_BLE_FLEET_HOLD_MAX` (900 s). While
holding, a battery-voltage read every `SPHERO_BLE_HOLD_KEEPALIVE` s (30) keeps
the robot awake.

## Run

```bash
export WORKER_AGENT_TOKEN=changeme
export WORKER_AGENT_HOST=10.0.0.21   # this worker's LAN IP
ros2 run sphero_worker_agent agent
```

## Boot on each RPi4 (systemd)

See `systemd/sphero-worker-agent.service` for a unit template with placeholders
(`<RUN_USER>`, `<WORKSPACE>`, `<LAN_IP>`, `<SHARED_TOKEN>`, `<ROS_DOMAIN_ID>`,
`<LAN_NIC>`) and step-by-step install notes.
