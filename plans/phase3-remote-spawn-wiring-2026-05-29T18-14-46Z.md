# Phase 3: Coordinator-Side Remote Spawn Wiring

**Created:** 2026-05-29T18-14-46Z
**Status:** Approved (coordinator pre-approved code-only execution)

## Task Description
Wire the coordinator (`multirobot_webserver`) to spawn/remove/monitor Sphero
instances on remote RPi4 worker nodes via each worker's launcher agent HTTP API,
using the existing `WorkerRegistry` for least-loaded selection. Keep the local
single-host spawn path intact as a fallback when no `workers.yaml` is loaded.

## Analysis
Current state:
- `SpheroInstanceManager.add_sphero` always spawns locally via `subprocess.Popen`.
- `WorkerRegistry` already does config load, least-loaded `select_worker()`,
  `assign`/`release`/`set_online` accounting. It is loaded into
  `self.worker_registry` (None if no config).
- Instance dicts currently carry `name/port/process/status/added_at/url`
  (+ `tag_id`, `marker_slot`). There is no `worker`/`host`/remote-url bookkeeping.
- No HTTP client, no token/agent-base-url config.

Decisions (per coordinator):
- Use `requests` (confirmed installed); add `<exec_depend>python3-requests</exec_depend>`.
- Timeouts on every agent HTTP call so a dead Pi can't hang the webserver.
- Token + per-worker agent base URL come from config, not hard-coded.

Config mechanism (where token / base URL are read):
- Bearer token: top-level `agent:` block in `config/workers.yaml`
  (`agent.token`), overridable by env `SPHERO_AGENT_TOKEN`. WorkerRegistry
  exposes it via a new `agent_token` attribute parsed in `from_config`.
- Per-worker agent base URL: derived from each `Worker.host` + `Worker.port`
  already in the registry: `http://{host}:{port}`. A `Worker.base_url`
  property is added. No new per-worker config field needed (host/port exist).

## Detailed Plan

### Step 1: package.xml dependency
- Add `<exec_depend>python3-requests</exec_depend>`.

### Step 2: WorkerRegistry — token + base_url
- `Worker.base_url` property -> `http://{host}:{port}`.
- `WorkerRegistry` stores `agent_token`; parse `agent.token` in `from_config`,
  env `SPHERO_AGENT_TOKEN` overrides. Keep existing API unchanged otherwise.

### Step 3: workers.yaml — agent block
- Add commented `agent:` block documenting `token` (placeholder), so deploy
  knows where to set it. Real value via env at runtime.

### Step 4: Remote agent HTTP client (in multirobot_webapp.py)
- Small helper methods on SpheroInstanceManager:
  - `_agent_headers()` -> bearer auth header from registry token (omit if empty).
  - `_agent_spawn(worker, sphero_name, tag_id)` -> POST {base}/spawn, returns
    remote payload (must include port/url) or raises on failure/timeout.
  - `_agent_remove(worker, sphero_name)` -> DELETE {base}/spawn/<name>,
    graceful on agent-unreachable.
  - `_agent_status(worker)` -> GET {base}/status for liveness (cached).
- Timeouts: connect+read tuple `(3, 5)` on every call.

### Step 5: add_sphero — remote path with rollback
- If `self.worker_registry` is not None: select_worker(); on success POST /spawn.
- On any failure (timeout, non-2xx, bad payload): rollback — release the
  worker accounting, do NOT add to instances, do NOT add fleet robot, return
  failure. (registry.assign only after a confirmed spawn.)
- On success: store instance dict with `worker` (name), `host`, remote `url`
  (from agent payload), `port`, `status`, `added_at`, `tag_id`, `marker_slot`;
  `process=None` for remote. Add fleet robot + marker slot as today.
- If `self.worker_registry` is None: existing local subprocess path unchanged.

### Step 6: remove_sphero — remote DELETE
- If instance is remote (`worker` set): DELETE /spawn/<name> on that worker;
  graceful if agent unreachable (log, still tear down local bookkeeping).
  registry.release(worker). Remove fleet robot + instance entry.
- If local: existing process.terminate path unchanged.

### Step 7: liveness — cached agent /status
- `get_all_instances` / `get_instance`: for remote instances, derive status
  from a cached agent `GET /status` (cache TTL ~2s to avoid hammering / hangs),
  rather than `process.poll()`. Local instances keep `process.poll()`.
- Update `WorkerRegistry.set_online` based on /status reachability.

### Step 8: API surface
- `/api/spheros` GET: each entry gains `worker` (None for local) and the
  correct (remote or local) `url`.
- New `GET /api/workers`: returns registry snapshot (per-worker
  name/host/port/capacity/count/free/online) + totals, or empty when no
  registry loaded.

### Step 9: shutdown_all — DELETE remote instances
- shutdown_all already calls remove_sphero per instance; ensure remote DELETE
  fires for remote instances (covered by Step 6) and is best-effort/graceful.

## Expected Outcomes
- With workers.yaml loaded: add/remove/list route through remote agents;
  least-loaded selection + capacity accounting honored; failures roll back cleanly.
- Without workers.yaml: behavior identical to today (local subprocess spawn).
- No agent call can hang the webserver (timeouts everywhere).

## Potential Risks & Considerations
- Agent /spawn payload shape: assume `{success, port, url}`; if `url` absent,
  synthesize `http://{host}:{port}`. Documented in code.
- /status cache prevents per-request hangs but means up to ~2s stale liveness.
- Token empty -> no auth header sent (dev mode); agent may 401 if it requires one.

## Testing Plan (coordinator runs)
- Build + source; launch fake local agents; add/remove/list; kill an agent to
  verify graceful handling + rollback; confirm local fallback with no config.

## Approval Status
- [x] Approved (coordinator pre-approved code-only execution)
- [x] Executed
