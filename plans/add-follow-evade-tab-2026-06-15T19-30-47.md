# Add Follow/Evade Feature to Multi-Robot Coordinator Dashboard

**Created:** 2026-06-15T19:30:47Z
**Status:** Pending Approval
**Complexity:** Medium
**Technologies:** Flask, vanilla JS (ControlStation class), HTML/CSS (no new framework), ThreadPoolExecutor fan-out, per-unit `/api/task` dispatch

## Task Description
Add a "FOLLOW/EVADE" capability to the coordinator dashboard (`:5000`). The
operator designates 1-4 connected Spheros as IR **broadcasters**; each
broadcaster is auto-assigned an IR channel pair; remaining connected units are
assigned per-broadcaster as **followers** or **evaders**. APPLY dispatches the
correct `ir_broadcast` / `ir_follow` / `ir_evade` task to each unit; STOP ALL IR
clears all IR behavior across the fleet.

## Requirements Analysis
- User interface: a new panel ("// FOLLOW/EVADE") in the coordinator dashboard.
  Lists connected units, lets operator pick broadcasters (max 4) and assign
  every other unit a role under one broadcaster.
- Backend integration: two new coordinator endpoints that fan DIFFERENT tasks
  to DIFFERENT units via the existing per-unit `POST {url}/api/task` mechanism.
- Real-time: reuse existing `/api/spheros` 5s poll (`this.spheros`); no new poll.
- Browser compatibility: same as existing dashboard (modern evergreen).
- Responsive: panel uses existing `.panel` flow; reuse existing grid utilities.
- Accessibility: semantic controls, labels, keyboard-usable selects/buttons.

## Analysis (current state, verified)
- `multirobot_webapp.py`:
  - `GET /api/spheros` (line ~1397) returns `{success, spheros:[{name, port,
    status, url, worker}], count}`. Frontend polls every 5s into `this.spheros`.
  - `POST /api/broadcast_task` (line ~1512) → `manager.broadcast_task(task_core,
    start_offset)` (line ~835). The fan-out helper:
    - snapshots running instances via `self.instances.values()` filtered by
      `self._instance_status(inst) == 'running'`, building `(name, url)` targets;
    - stamps a single `now`, merges `{now, start_offset}` into the payload, and
      POSTs the SAME payload to each `{url}/api/task` in a `ThreadPoolExecutor`
      with `BROADCAST_POST_TIMEOUT`;
    - returns `{success, sent, failed, now, start_offset, start_time, results:
      [{name, success, error?}]}`.
  - Each instance dict has `name`, `url`, and a status resolvable via
    `_instance_status(inst)`. `url` already points at the right host (local or
    coordinator-side relay for remote workers).
- Frontend:
  - `templates/index.html` = stacked `<section class="panel">` blocks (NO tabs at
    coordinator level; tabs exist only inside the per-unit console). New feature
    fits cleanly as another `panel`.
  - `static/js/app.js` = single `ControlStation` class; `this.spheros` holds the
    fleet; `fetch` + `this.toast(msg, kind, tag)` is the universal call+notify
    pattern; `renderFleet()` shows the established render approach.
  - `static/css/style.css` = JetBrains-Mono industrial theme; tokens `--amber`
    (accent/primary), `--green` (ok), `--red` (danger), `.panel`, `.action`,
    `.action--primary/--accent/--ghost/--danger`, `.readout`, `.field`,
    `select.field`. We reuse these — no new CSS framework, minimal new rules.

### Decision: panel, not literal "tab"
The coordinator dashboard has no tab strip — it is a vertical stack of panels.
The task says "add a tab/panel ... in a way that fits the existing layout."
The cleanest, most consistent integration is a new `<section class="panel">`
titled `// FOLLOW/EVADE`, matching FLEET / LOCALIZATION / POSITIONING. This is
called out as an assumption below for approval.

## UI/UX Design

### Wireframe
```
+--------------------------------------------------------------+
| // FOLLOW/EVADE        BROADCASTERS 2/4 · ASSIGNABLE 5       |
+--------------------------------------------------------------+
|  Pick broadcasters (max 4). Each gets an auto channel pair.  |
|                                                              |
|  UNIT        BROADCASTER   ROLE / UNDER                      |
|  SB-3660     [x] CH 0/1    (is a broadcaster)                |
|  SB-3AAC     [ ]           [ none ▾ ] under [ SB-3660 ▾ ]    |
|  SB-33E3     [ ]           [ evade ▾ ] under [ SB-3660 ▾ ]   |
|  SB-1FA8     [ ]           [ follow ▾] under [ SB-3AAC ▾ ]   |  <- broadcaster
|  ...                                                         |
|                                                              |
|  [ APPLY FORMATION ]                  [ STOP ALL IR ]        |
+--------------------------------------------------------------+
```
- Each connected unit is one row.
- A **broadcaster checkbox**. When checked: the role/under controls for that row
  are hidden/disabled and it shows its auto-assigned `CH near/far` chip.
- For non-broadcaster rows: a **role select** (`none` / `follow` / `evade`) and
  an **"under" select** listing the current broadcasters. The under-select is
  disabled while role is `none`.
- A live header readout: broadcaster count (`N/4`) and assignable count.
- Broadcaster checkbox disabled when 4 already selected (and this row isn't one).

### User Flow
1. Operator checks 1-4 units as broadcasters → each shows `CH near/far`.
2. For each remaining unit, operator picks `follow`/`evade` and the broadcaster.
3. APPLY → builds `{broadcasters:[...]}`, POST `/api/ir_formation` → toast with
   per-unit results.
4. STOP ALL IR → POST `/api/ir_stop` → toast.

Channel pairs are assigned **client-side for display only** in broadcaster
check order (0/1, 2/3, 4/5, 6/7); the **server re-derives** the authoritative
pairs in payload order (server is the source of truth — display is advisory).

## Detailed Plan

### Step 1: Backend — channel-pair constant + helper (multirobot_webapp.py)
- Add module constant near the other pools (after `MARKER_POOL`):
  `IR_CHANNEL_PAIRS = [(0, 1), (2, 3), (4, 5), (6, 7)]  # max 4 broadcasters`.
- Verify outcome: import still succeeds.

### Step 2: Backend — `manager.dispatch_ir_formation(broadcasters)` method
On `SpheroInstanceManager`, mirroring `broadcast_task`'s fan-out style:
- Validate `broadcasters` is a non-empty list, `len <= 4` (else error message
  "max 4 broadcasters").
- Build the set of running unit names from `self.instances` +
  `_instance_status(...) == 'running'` (same snapshot approach as broadcast).
- Validation pass (no dispatch until all pass):
  - each broadcaster entry is a dict with non-empty string `name`;
  - every referenced name (broadcaster, follower, evader) is currently running;
  - a unit is not both a broadcaster and a follower/evader;
  - a unit appears as follower/evader under at most one broadcaster (and not as
    both follower and evader);
  - broadcaster names are unique.
  On any failure return `{success: False, message: <clear reason>}`.
- Build a per-unit task map `{name: {task_type, parameters}}`:
  - broadcaster i (0-based) → pair `(near, far) = IR_CHANNEL_PAIRS[i]`;
    `{task_type:'ir_broadcast', parameters:{near, far}}`.
  - each follower → `{task_type:'ir_follow', parameters:{near, far}}` (that
    broadcaster's pair).
  - each evader → `{task_type:'ir_evade', parameters:{near, far}}`.
- Fan out: reuse the SAME per-POST mechanism as `broadcast_task` — a small
  internal `_post_task_to_unit(name, url, payload)` posting to `{url}/api/task`
  with `BROADCAST_POST_TIMEOUT`, run via `ThreadPoolExecutor`. Each unit gets its
  OWN payload (not a shared one), so we look up `url` per name from the instance
  dict. Return `{success: True, dispatched, failed, results:[{name, role,
  task_type, success, error?}]}`.
- NOTE on timing: `ir_*` tasks are continuous-behavior toggles, not a synced
  one-shot, so NO `now`/`start_offset` is added (unlike `broadcast_task`). Each
  unit just receives `{task_type, parameters}`. Flagged as an assumption below.

### Step 3: Backend — `manager.stop_all_ir()` method
- Snapshot running `(name, url)` targets (same as broadcast).
- For each unit, POST three tasks: `ir_broadcast_stop`, `ir_follow_stop`,
  `ir_evade_stop`, each `{parameters:{}}`. Fan out across units in parallel; per
  unit, the three stops are sent sequentially (3 small POSTs). Harmless no-op if
  a unit wasn't doing that behavior (per task contract).
- Return `{success, results:[{name, success, error?}]}` (a unit is `success` if
  all three stop POSTs returned 2xx; otherwise carries the first error).

### Step 4: Backend — two Flask routes
- `POST /api/ir_formation`:
  - parse JSON; require `broadcasters` to be a list; else 400.
  - `result = manager.dispatch_ir_formation(data['broadcasters'])`.
  - 200 on `success`, 400 otherwise (matches existing route conventions).
- `POST /api/ir_stop`:
  - `result = manager.stop_all_ir()`; 200 (no body needed).
- Placement: alongside `/api/broadcast_task` in the routes section.

### Step 5: Frontend — HTML panel (index.html)
- Add a `<section class="panel">` after the FLEET panel (most logically grouped
  with fleet ops) with:
  - `panel__head`: title `// FOLLOW/EVADE`, a `panel__meta` readout
    (`#irBroadcasterCount`, `#irAssignableCount`).
  - a hint line (max-4 rule).
  - `<div id="irFormationGrid">` (JS-rendered rows).
  - `<div id="irFormationEmpty">` empty-state ("NO UNITS DEPLOYED ...").
  - footer with `#applyIrBtn` (`action--primary`) and `#stopIrBtn`
    (`action--danger`).

### Step 6: Frontend — JS (app.js, in ControlStation)
- `renderFleet()` already runs every poll; add a call to `this.renderIrFormation()`
  at its end (and in the empty-fleet early-return) so the IR panel tracks the
  same `this.spheros` data — NO new polling.
- State: `this.irRoles = {}` (`name -> {role:'none'|'follow'|'evade',
  under:<broadcasterName|null>}`) and `this.irBroadcasters = new Set()`,
  pruned to current units each render (same pattern as `this.selected`).
- `renderIrFormation()`:
  - compute broadcaster list in stable order (fleet order), assign display pairs.
  - render one row per running unit: broadcaster checkbox (disabled if 4 chosen
    and not this unit, or if this unit currently has a non-none role), channel
    chip for broadcasters, role select + under-select for the rest.
  - keep a structural signature like `renderFleet`'s `_lastFleetSig` so we don't
    clobber half-made selections on every 5s poll when the fleet is unchanged;
    re-render only on fleet shape change OR local interaction.
  - wire change handlers updating `this.irRoles`/`this.irBroadcasters`, then
    re-render (local interaction path).
  - update header readouts; enforce max-4 in the UI.
- `buildIrPayload()`: assemble `{broadcasters:[{name, followers:[...],
  evaders:[...]}]}` from state. Skip roles whose `under` isn't a current
  broadcaster.
- `applyIrFormation()`: validate >=1 broadcaster client-side; POST
  `/api/ir_formation`; toast success / per-unit failures (reuse
  `truncateNames`, the same shape as `sendBroadcast`).
- `stopAllIr()`: POST `/api/ir_stop`; toast result.
- Bind `#applyIrBtn` / `#stopIrBtn` in `bindActions()`.

### Step 7: CSS (style.css)
- Minimal additions reusing tokens: a row grid for `#irFormationGrid` rows
  (`.ir-row`), a channel chip (`.ir-chip`, amber-tinted like existing chips),
  reuse `select.field` for the selects. ~25-40 lines, no new framework.

## API Endpoints

### REST Endpoints (new)
| Method | Path | Request | Response |
|--------|------|---------|----------|
| POST | /api/ir_formation | `{broadcasters:[{name, followers:[...], evaders:[...]}]}` | `{success, dispatched, failed, results:[{name, role, task_type, success, error?}]}` or `{success:false, message}` |
| POST | /api/ir_stop | `{}` (none) | `{success, results:[{name, success, error?}]}` |

### Per-unit dispatch (unchanged contract, reused)
| Target | Body |
|--------|------|
| `{url}/api/task` (broadcaster) | `{task_type:'ir_broadcast', parameters:{near, far}}` |
| `{url}/api/task` (follower) | `{task_type:'ir_follow', parameters:{near, far}}` |
| `{url}/api/task` (evader) | `{task_type:'ir_evade', parameters:{near, far}}` |
| `{url}/api/task` (stop) | `ir_broadcast_stop` / `ir_follow_stop` / `ir_evade_stop`, `parameters:{}` |

## Data Flow
```
Operator picks broadcasters + roles
  → app.js buildIrPayload() → POST /api/ir_formation
    → manager.dispatch_ir_formation() validates + assigns pairs
      → ThreadPoolExecutor: POST {url}/api/task (DIFFERENT task per unit)
        → instance controller runs ir_broadcast / ir_follow / ir_evade
  ← per-unit results ← aggregated ← toast on dashboard
```

## Expected Outcomes
- New `// FOLLOW/EVADE` panel lists connected units, enforces max-4 broadcasters,
  shows auto channel pairs, lets operator assign follow/evade per broadcaster.
- APPLY dispatches the right task to each unit; per-unit success/error surfaced.
- STOP ALL IR clears IR fleet-wide.
- Existing behavior (fleet, broadcast, localization, UWB) untouched.

## Potential Risks & Considerations
- Security: validate/normalize all names server-side; only dispatch to units
  already in `self.instances` (no arbitrary URL). No HTML built from unit names
  without escaping (reuse the existing `safe()` escaping pattern in row render).
- Performance: fan-out is bounded by `BROADCAST_POST_TIMEOUT` per POST and capped
  workers — a dead unit can't stall others (same guarantee as broadcast).
- UX: 5s poll must not wipe in-progress selections → signature guard like
  `renderFleet`.
- Compatibility: vanilla JS + fetch, same as rest of app.

## Testing Plan
- Lightweight static checks (no live Spheros):
  - [ ] `python3 -c "import ast; ast.parse(open('multirobot_webapp.py').read())"`
        (syntax) and a route-registration check via Flask test client `GET /`
        renders + `OPTIONS`/`POST` on the two new routes are registered.
  - [ ] `node --check static/js/app.js` (JS syntax).
- Manual (operator, live fleet):
  - [ ] Pick 1 broadcaster + 1 follower + 1 evader → APPLY → observe behaviors.
  - [ ] Exceed 4 broadcasters → UI blocks; server rejects if forced.
  - [ ] Unit both broadcaster and follower → server rejects with clear message.
  - [ ] STOP ALL IR with nothing running → harmless success.
  - [ ] 5s poll doesn't clear half-made selections.

## Assumptions (please confirm)
1. **Panel, not literal tab strip** — the coordinator dashboard is a panel stack;
   I will add a `// FOLLOW/EVADE` `<section class="panel">` matching the others
   (no tab UI exists at the coordinator level to extend).
2. **No timing fields on ir_* tasks** — `broadcast_task` injects `now`/
   `start_offset` for synchronized one-shots. IR behaviors are continuous
   toggles, so `dispatch_ir_formation` posts bare `{task_type, parameters}` to
   `/api/task` (no `now`/`start_offset`). If the instance controller's
   `/api/task` REQUIRES those fields even for ir_* types, I'll add them.
3. **Per-unit `/api/task` accepts a single `{task_type, parameters}`** (the same
   single-task shape `broadcast_task` sends, minus timing). This is the shape the
   fan-out helper already uses, so I'm reusing it verbatim.
4. **`stop_all_ir` sends 3 separate POSTs per unit** (one per stop type) rather
   than a bundle, since the contract lists three distinct stop task_types and a
   bundle endpoint shape for stops isn't specified.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
