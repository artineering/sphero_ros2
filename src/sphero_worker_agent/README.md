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
| POST   | `/spawn`        | `{name, port, external_localization?}`       | Spawn a Sphero tree. 200 / 409.      |
| DELETE | `/spawn/<name>` | -                                            | Tear down a Sphero tree. 200 / 404.  |
| GET    | `/status`       | -                                            | Running Spheros + liveness.          |
| GET    | `/health`       | -                                            | Liveness check (`{"status":"ok"}`).  |

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
