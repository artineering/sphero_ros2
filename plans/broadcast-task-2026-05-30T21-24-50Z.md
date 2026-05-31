# Broadcast Task to All Deployed Spheros

**Created:** 2026-05-30T21:24:50Z
**Status:** Pending Approval
**Complexity:** Medium
**Technologies:** Flask, `requests`, `concurrent.futures.ThreadPoolExecutor`, vanilla JS (retro-terminal dashboard)

## Task Description
Add a dashboard control that composes ONE task and broadcasts the SAME payload to ALL currently
deployed Sphero instances at once. The webapp stamps a single coordinator timestamp `now` and a
`start_offset`; every unit fires at `now + start_offset` on its NTP-synced clock, so they begin in
sync. Fan-out to the instances MUST be parallel.

## Verification of current code (done before planning)

- **`url` field present on BOTH paths** — confirmed:
  - Local instance: `multirobot_webapp.py:535` → `'url': f'http://localhost:{port}'`
  - Remote instance: `multirobot_webapp.py:639` → `'url': url` where `url = f'http://{worker.host}:{port}'` (line 624).
  - So `f"{instance['url']}/api/task"` is reachable server-side from rpi5 for every instance regardless of local/remote. CONFIRMED.
- **Per-instance `POST /api/task`** — `sphero_instance_websocket_server.py:669-675`: accepts arbitrary JSON, calls `node.publish_task_command(data)`, returns `{'status': 'success'}`. Extra `now`/`start_offset` keys ride through untouched into the published task JSON. CONFIRMED.
- **`requests` + `time` already imported** — `multirobot_webapp.py:21` (`import requests`), `:15` (`import time`). `_agent_spawn` (~line 381) shows the existing `requests.post(url, json=..., timeout=...)` pattern to mirror. `concurrent.futures` is stdlib, no new dep.
- **Manager pattern** — `SpheroInstanceManager` (`:311`), instances in `self.instances[name]` dict with `name`, `port`, `status`, `url`. Existing thin-route → manager-method pattern: `add_spheros_batch` (`:742`) returns `{success, deployed, failed, results:[...]}`; route at `:1391` validates body then `return jsonify(manager.add_spheros_batch(names)), 200`. We mirror this shape.
- **Status helper** — `_instance_status(instance)` is used in `get_all_instances` (`:836-838`) to derive live status. We will broadcast to instances whose derived status is `running`.
- **Task-type surface** — registered handlers in `sphero_task_executor.py:42-57`:
  `move_to, patrol, circle, square, led_sequence, matrix_sequence, spin, stop, custom, set_led, roll, heading, speed, matrix, collision, reflect`.
  The per-instance UI (`sphero_instance_controller/.../static/js/app.js:955+`) builds typed forms for `roll, move_to, patrol, circle, square`. Because the type space is large (16 handlers) and parameter shapes vary, the broadcast composer uses a **type selector + raw-JSON parameters field** (decision below) rather than re-implementing 16 typed forms.
- **UI patterns** — ribbon `<section class="ribbon">` with `<button class="action action--primary" id="addSpheroBtn">` (`index.html:61-66`); DEPLOY modal `#addSpheroModal` (`:262-280`) with `.modal__panel/.modal__head/.modal__body/.modal__foot`, `.modal__tag`, `.field`, `.field__label`, `.field__hint`; helpers `this.openModal(id, focusId)`, `this.closeModal(id)`, `this.toast(msg, level, tag)` (levels `success|error|info`); fleet state in `this.spheros` refreshed by `renderFleet()` (`:133`).

## Requirements Analysis
- **UI needs:** Operator opens a broadcast composer, picks a task type, supplies parameters, sets lead seconds, hits BROADCAST. Sees a countdown to start and a summary toast.
- **Backend integration:** New `POST /api/broadcast_task` → manager `broadcast_task()` → parallel `POST {url}/api/task` to each running instance.
- **Real-time:** No new websocket. Synchronization is achieved by the shared `now + start_offset` reference, NOT by tight delivery timing — parallel fan-out only needs to beat `start_offset`.
- **Browser compatibility:** Same as existing dashboard (modern evergreen Chrome/Firefox/Safari/Edge). Uses `fetch`, template literals, `setInterval` — all already in `app.js`.
- **Responsive/Accessibility:** Reuse existing `.modal`/`.action`/`.field` classes (already styled + responsive). Modal has `role="dialog" aria-modal="true"`; button gets `disabled` + `aria-disabled` when zero units deployed.

## UI/UX Design

### Wireframe (ribbon + modal)
```
RIBBON (primary actions):
[ DEPLOY UNIT ] [ REFRESH ] [ BROADCAST TASK ] [ ARUCO… ] [ UWB… ]
                              ^ new, action--accent, disabled when 0 units

BROADCAST MODAL (#broadcastModal):
+--------------------------------------------------+
| [ BROADCAST ]   FAN TASK TO FLEET           [×] |
+--------------------------------------------------+
| [ TASK TYPE ]  ( roll  v )                        |
| [ PARAMETERS (JSON) ]                             |
|  +--------------------------------------------+   |
|  | {"heading": 0, "speed": 100, "duration":0}|   |
|  +--------------------------------------------+   |
| [ LEAD SECONDS ]  ( 3.0 )                          |
| hint: 8 units deployed · fires at now + lead       |
| (countdown appears here after BROADCAST)           |
+--------------------------------------------------+
|                [ CANCEL ]   [ BROADCAST ALL ]    |
+--------------------------------------------------+
```

### User Flow
1. Operator clicks **BROADCAST TASK** (ribbon) → modal opens (only enabled if `this.spheros` has ≥1 running unit).
2. Selects task type from `<select>` (populated from the known handler list). Default `roll`.
3. Edits the **parameters** JSON textarea (pre-filled with a sensible default per selected type; changing the type swaps the default).
4. Sets **lead seconds** (default `3.0`, min `0.5`).
5. Clicks **BROADCAST ALL** → JS validates JSON, POSTs to `/api/broadcast_task`.
6. On response: show a live **countdown** ("START IN 3 … 2 … 1 … GO") in the modal, plus a summary `toast()` e.g. `"Broadcast to 8 units, start in 3.0s"` or `"7 sent, 1 failed: SB-58EF"`. Modal auto-closes after GO.

## Detailed Plan

### Step 1 — Manager method `broadcast_task` (parallel fan-out)
- File: `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py` (in `SpheroInstanceManager`, place after `remove_spheros_batch`, ~line 826).
- Add module constant near the other tuning constants (~line 57):
  `BROADCAST_POST_TIMEOUT = (3, 5)` (connect, read) and `BROADCAST_MAX_WORKERS = 16` (fleet caps at 16 units / UWB tags).
- Signature: `def broadcast_task(self, task_type: str, parameters: dict, start_offset: float) -> dict:`
- Behavior:
  1. Snapshot target instances = those with derived `_instance_status(inst) == 'running'`. Capture `(name, url)` pairs into a list (snapshot so a concurrent add/remove can't mutate mid-fan-out).
  2. If no targets → return `{'success': False, 'message': 'No units deployed'}` (route turns this into 400; defensive even though route also checks).
  3. **Stamp once:** `now = time.time()` BEFORE dispatch. Build `payload = {'task_type': task_type, 'parameters': parameters, 'now': now, 'start_offset': start_offset}`.
  4. Define a worker `_post_one(name, url)` that does
     `requests.post(f'{url}/api/task', json=payload, timeout=BROADCAST_POST_TIMEOUT)` inside try/except,
     returning `{'name': name, 'success': True}` on 2xx else `{'name': name, 'success': False, 'error': <reason>}`.
     (Catch `requests.RequestException` and non-2xx `r.status_code` → error string; a slow/dead unit yields a failure result, never raises.)
  5. **Parallel dispatch:** `with ThreadPoolExecutor(max_workers=min(BROADCAST_MAX_WORKERS, len(targets))) as ex:` submit `_post_one` for every target, gather via `concurrent.futures.as_completed`. Each future's own `timeout` is the per-POST `requests` timeout (≤5s), so one stuck unit cannot block the others — they ran concurrently. (Do NOT pass a timeout to `as_completed`; rely on per-POST timeout so we always collect every result.)
  6. Tally `sent` (success) / `failed`, return:
     ```python
     {'success': True, 'sent': sent, 'failed': failed,
      'now': now, 'start_offset': start_offset,
      'start_time': now + start_offset, 'results': results}
     ```
- Add `from concurrent.futures import ThreadPoolExecutor, as_completed` to imports (top of file, near line 18). `requests`/`time` already imported.
- Keep it surgical: no retries, no caching, no per-instance threading beyond the pool.

### Step 2 — Route `POST /api/broadcast_task` (thin)
- File: same file, place beside the batch routes (after `/api/spheros/batch_delete`, ~line 1437).
- Body: `{task_type: str, parameters: dict (optional, default {}), start_offset: float (optional, default 3.0)}`.
- Validation (return 400 with `{success:False, message}` on failure):
  - `data` is JSON object.
  - `task_type` present and a non-empty `str`.
  - `parameters` if present is a `dict` (reject list/str).
  - `start_offset` if present is a number; coerce `float`, clamp/reject if `< 0` (reject negative; allow 0+). Default `3.0`.
  - Reject if `not manager.instances` (no units) → 400 `{success:False, message:'No units deployed'}` (fast path; manager re-checks running count).
- On valid: `return jsonify(manager.broadcast_task(task_type, parameters, start_offset)), 200`.
- (Note: we validate `task_type` is a non-empty string but do NOT whitelist against the 16 handlers — the per-instance controller is the authority on unknown types, matching how single `/api/task` behaves today. Mention in code comment.)

### Step 3 — Ribbon button (HTML)
- File: `src/multirobot_webserver/templates/index.html`, in the primary-actions ribbon (~line 66, after REFRESH button).
- Markup mirroring existing `.action`:
  ```html
  <button class="action action--accent" id="broadcastBtn" disabled aria-disabled="true">
      <span class="action__glyph">⇶</span>
      <span class="action__label">BROADCAST TASK</span>
  </button>
  ```
  (Match whatever glyph span the neighboring buttons use; if they use an SVG/`action__glyph`, copy that structure.)

### Step 4 — Broadcast modal (HTML)
- File: `index.html`, after the DETACH modal (~line 300).
- Reuse `.modal/.modal__panel/.modal__head/.modal__body/.modal__foot/.modal__tag/.field/.field__label/.field__hint/.action` classes:
  ```html
  <div id="broadcastModal" class="modal" role="dialog" aria-modal="true">
    <div class="modal__panel">
      <div class="modal__head">
        <span class="modal__tag">[ BROADCAST ]</span>
        <h3 class="modal__title">FAN TASK TO FLEET</h3>
        <button class="modal__close close" aria-label="Close">×</button>
      </div>
      <div class="modal__body">
        <label for="broadcastType" class="field__label">[ TASK TYPE ]</label>
        <select id="broadcastType" class="field"> … options … </select>
        <label for="broadcastParams" class="field__label">[ PARAMETERS (JSON) ]</label>
        <textarea id="broadcastParams" class="field" rows="5" spellcheck="false"></textarea>
        <label for="broadcastLead" class="field__label">[ LEAD SECONDS ]</label>
        <input type="number" id="broadcastLead" class="field" value="3.0" min="0.5" step="0.5">
        <p class="field__hint"><span id="broadcastSummary">0 units · fires at now + lead</span></p>
        <p class="field__hint" id="broadcastCountdown" hidden></p>
      </div>
      <div class="modal__foot">
        <button id="cancelBroadcastBtn" class="action action--ghost">CANCEL</button>
        <button id="confirmBroadcastBtn" class="action action--primary">BROADCAST ALL</button>
      </div>
    </div>
  </div>
  ```
- `<option>`s = the known handler list (`roll, move_to, patrol, circle, square, spin, set_led, stop, heading, speed, matrix, led_sequence, matrix_sequence, collision, reflect, custom`), default `roll` selected.

### Step 5 — Frontend wiring (JS)
- File: `src/multirobot_webserver/static/js/app.js`.
- **Param defaults map** (module-level const, mirrors the per-instance forms' defaults):
  ```js
  const BROADCAST_DEFAULTS = {
    roll:    {heading:0, speed:100, duration:0},
    move_to: {x:100, y:100, speed:100},
    circle:  {radius:50, speed:100, duration:10, direction:'ccw'},
    patrol:  {waypoints:[{x:50,y:50},{x:100,y:50},{x:100,y:100}], speed:100, loop:false},
    spin:    {speed:120, duration:3},
    set_led: {red:0, green:128, blue:255},
    stop:    {},
    // others → {}  (operator fills JSON manually)
  };
  ```
- In `bindEvents()` (near line 46-90):
  - `$('#broadcastBtn').addEventListener('click', () => this.openBroadcast());`
  - `$('#broadcastType').addEventListener('change', () => this.fillBroadcastDefault());`
  - `$('#confirmBroadcastBtn').addEventListener('click', () => this.sendBroadcast());`
  - `$('#cancelBroadcastBtn').addEventListener('click', () => this.closeModal('broadcastModal'));`
- New methods:
  - `openBroadcast()` — guard `if (!this.runningCount()) { toast('No units deployed','info','BROADCAST'); return; }`; `fillBroadcastDefault()`; update `#broadcastSummary` with count; `openModal('broadcastModal','broadcastType')`.
  - `fillBroadcastDefault()` — set `#broadcastParams` value to `JSON.stringify(BROADCAST_DEFAULTS[type] ?? {}, null, 2)`.
  - `sendBroadcast()` —
    1. Parse `#broadcastParams` with `try{JSON.parse}` → on error `toast('Parameters must be valid JSON','error','BROADCAST')`, abort.
    2. Validate parsed is a plain object (not array).
    3. Read `start_offset = parseFloat(#broadcastLead)` (fallback 3.0, min 0.5).
    4. `fetch('/api/broadcast_task', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({task_type, parameters, start_offset})})`.
    5. On `data.success`: call `this.startCountdown(data.start_offset)`; toast summary:
       `failed===0 ? `Broadcast to ${sent} units, start in ${start_offset}s` : `${sent} sent, ${failed} failed: ${truncateNames(failedNames)}``
       (reuse existing `this.truncateNames`).
    6. On non-success/HTTP error: `toast('Broadcast failed: '+msg,'error','BROADCAST')`. On network throw: `toast('Broadcast uplink lost.','error','BROADCAST')` (mirror existing catch style).
  - `startCountdown(offset)` — show `#broadcastCountdown`, tick down with `setInterval` from `Math.ceil(offset)` to GO, then clear interval and `closeModal('broadcastModal')`.
  - `runningCount()` — `this.spheros.filter(s => s.status === 'running').length`.
- **Enable/disable button:** in `renderFleet()` / wherever `this.spheros` updates (after line 131-133), set
  `$('#broadcastBtn').disabled = this.runningCount() === 0;` and matching `aria-disabled`. Initialize disabled in HTML.

### Step 6 — CSS
- File: `src/multirobot_webserver/static/css/style.css`.
- Expected to need ZERO new layout CSS (modal/field/action classes already cover it). Add only if the countdown needs emphasis — a small `.field__hint--countdown { color: var(--accent…); letter-spacing… }` reusing existing custom properties. Decide during impl; keep minimal.

## API Endpoints

| Method | Path | Body | Response |
|--------|------|------|----------|
| POST | `/api/broadcast_task` | `{task_type, parameters?, start_offset?}` | `{success, sent, failed, now, start_offset, start_time, results:[{name, success, error?}]}` |

## Data Flow
```
Operator → BROADCAST modal → fetch POST /api/broadcast_task
   → route validates → manager.broadcast_task()
       → now = time.time()  (stamped ONCE)
       → payload {task_type, parameters, now, start_offset}
       → ThreadPoolExecutor: POST payload → every running instance['url']/api/task  (PARALLEL)
           → each instance: publish_task_command(payload) → /sphero/<name>/task topic
       → controller fires at now + start_offset on NTP clock (in sync)
   ← {success, sent, failed, now, start_offset, start_time, results}
Operator ← countdown + summary toast
```

## Expected Outcomes
- One composed task reaches every running unit via concurrent POSTs.
- All units share one `now`; they start together at `now + start_offset`.
- A slow/unreachable unit fails its own POST without delaying others (parallel), and is reported in `results` + summary toast.
- Button disabled when zero units deployed.

## Potential Risks & Considerations
- **Security:** `task_type` not whitelisted server-side (matches single `/api/task`); controller validates. `parameters` is opaque JSON forwarded as-is — no eval, no shell. Input is type-checked (str/dict/number). XSS: failed-unit names rendered via existing `toast()`/`truncateNames` (already used for batch; confirm it text-sets, not `innerHTML` of raw names — reuse same sink as DEPLOY/DETACH which already handle server-returned names).
- **Performance:** ≤16 units → pool of ≤16 threads, each ≤5s timeout; worst case ~5s wall regardless of fleet size (parallel). Negligible.
- **Sync correctness:** Synchronization comes from `now + start_offset` on NTP-synced controllers, NOT delivery timing. `start_offset` default 3s >> parallel-delivery spread (a few ms). Requires controllers to honor `now`/`start_offset` (the locked task-controller upgrade) — out of scope here but the payload contract is exactly as specified.
- **Concurrency:** Instance list snapshotted before fan-out so add/remove during broadcast can't corrupt iteration.
- **UX:** If operator sets lead too low (<delivery spread) units could desync; min 0.5s in the input, default 3s.

## Testing Plan
- Manual:
  - [ ] Deploy 2–3 local instances; BROADCAST `roll` → confirm each instance logs a task with identical `now` and the same `start_offset` (inspect `/sphero/<name>/task`).
  - [ ] Stop one instance's process (or block its port) → broadcast → that unit appears in `results` as failed, others succeed, wall time still ~≤5s (proves parallel).
  - [ ] Invalid JSON in params → client toast, no POST.
  - [ ] Zero units → button disabled; direct POST returns 400.
  - [ ] `curl -X POST localhost:5000/api/broadcast_task -H 'Content-Type: application/json' -d '{"task_type":"roll","parameters":{"heading":0,"speed":100,"duration":0},"start_offset":3}'` → inspect response shape.
- Accessibility: [ ] modal reachable by keyboard, button `disabled`/`aria-disabled` honored.
- Parallelism proof: [ ] temporarily point one instance url at a sink that sleeps >read-timeout; confirm total broadcast time ≈ one timeout, not N×timeout.

## Browser Compatibility
Same as existing dashboard — Chrome/Firefox/Safari/Edge (evergreen). No new browser APIs.

## Files Touched
- `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py` — import, constants, `broadcast_task()` method, `/api/broadcast_task` route.
- `src/multirobot_webserver/templates/index.html` — ribbon button + broadcast modal.
- `src/multirobot_webserver/static/js/app.js` — defaults map, event bindings, `openBroadcast/fillBroadcastDefault/sendBroadcast/startCountdown/runningCount`, button enable/disable in fleet render.
- `src/multirobot_webserver/static/css/style.css` — optional minor countdown style only.

No new dependencies (`requests` + stdlib `concurrent.futures`/`time`).

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
