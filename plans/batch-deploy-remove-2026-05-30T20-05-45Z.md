# Batch Deploy + Batch Remove of Spheros

**Created:** 2026-05-30T20:05:45Z
**Status:** Pending Approval
**Complexity:** Medium
**Technologies:** Flask, vanilla JS (ES6), CSS (existing retro-terminal theme), ROS2 fleet node

## Task Description
Add two operator workflows to the multirobot_webserver dashboard:
1. **Batch deploy** — paste N callsigns (one per line); server auto-assigns the next-free UWB tags (1–16) and spawns each unit. Returns per-item results (success/fail + assigned tag + reason).
2. **Batch remove** — per-card checkboxes + "DETACH SELECTED", plus a separate "DETACH ALL". Each guarded by a confirm.

## Requirements Analysis
- UI: multi-line callsign entry, per-card selection checkboxes, selection counter, two batch-detach buttons, confirm modals, summary toasts.
- Backend: two new POST routes returning per-item results; tag auto-assignment authoritative on the server.
- Real-time: none new — existing 5s `/api/spheros` poll already re-renders the fleet. Selection state must survive re-render.
- Browser compat / responsive / a11y: match existing app (no new deps, same modal + `action--*` patterns, ARIA on checkboxes).

## Analysis (verified against code)

**Frontend (`static/js/app.js`, `templates/index.html`):**
- Single deploy: `#addSpheroBtn` → `openModal('addSpheroModal')` + `refreshUwbTags()`. Modal has `#tagIdSelect` (populated by `populateTagSelect`, app.js:258) and `#spheroNameInput`. `#confirmAddBtn` → `deploySphero(name, tagId)` (app.js:137) → `POST /api/spheros {sphero_name, tag_id}`.
- Single detach: per-card `detach-<name>` button → `requestDetach(name)` (app.js:624) → `confirmRemoveModal` → `detachSphero(name)` (app.js:159) → `DELETE /api/spheros/<name>`.
- Fleet render: `renderFleet()` (app.js:470) diffs a signature `name|port|status|added_at|url`; rebuilds `#spheroGrid` innerHTML via `unitTile(s,i)` (app.js:526) only when the signature changes, then re-binds `open-`/`detach-` listeners. **Implication:** any per-card checkbox DOM is destroyed on every structural re-render, so selection must live in a JS `Set<name>`, reconciled and re-applied after each rebuild.
- `toast(message, kind, tag)` (app.js:632) — reuse for summaries. `kind ∈ {success,error,info}`.

**Backend (`multirobot_webserver/multirobot_webapp.py`):**
- `UWB_TAG_IDS = list(range(1,17))` (line 36).
- `add_sphero(name, tag_id)` (line 465): rejects duplicate name (476), tag out of range (486), tag already assigned (492, checks `fleet_node.free_tag_ids()`). Distributed path `_add_sphero_remote` (582) selects least-loaded worker via `worker_registry.select_worker()` which **raises RuntimeError when all workers are at capacity** (worker_registry.py:124). So a large batch realistically partial-fails on capacity.
- `remove_sphero(name)` (line 663): handles local + remote; returns `{success, message}`.
- `fleet_node.free_tag_ids()` (line 236) and `tag_assignments()` (242) are the authoritative tag source, guarded by `fleet_node._lock`. `GET /api/uwb/tags` (1349) exposes them.
- Flask runs the dev server; request handlers are effectively serialized for our purposes, but **tag assignment must be computed per-item inside the loop** (after each successful add updates `fleet_node`), not all up front, so two callsigns in one batch never collide on a tag.

## UI/UX Design

### Decision #4 (UI integration) — RECOMMENDATION: Option (a), converge on one modal
Convert `#addSpheroModal` into the batch modal: replace the `#tagIdSelect` picker + single `#spheroNameInput` with a multi-line `<textarea>` (one callsign per line) and drop the manual tag picker. One line = one unit; tags are always auto-assigned server-side.

**Why (a) over (b):**
- The manual tag picker exists only to satisfy the current required `tag_id`. With server-side auto-assignment, manual tag choice is no longer needed for the normal flow — removing it deletes `populateTagSelect`/`#tagIdSelect` plumbing rather than maintaining two parallel deploy paths.
- One modal = one code path = less surface area, matches "Simplicity First."
- Single deploy is just a 1-line batch, so no capability is lost.
- Tradeoff: an operator can no longer pin a *specific* tag to a unit from the UI. Given the spec explicitly says "operator does NOT pick tags for batch," and tags 1–16 are otherwise interchangeable, this is acceptable. The single-add `POST /api/spheros` route (which still requires `tag_id`) is **left intact** for API/back-compat; only the front-end stops using it.

If you'd rather keep a manual-tag escape hatch, say so and I'll switch to Option (b) (keep `addSpheroModal` as-is, add a separate `batchDeployModal`); the backend plan is identical either way.

### Wireframe — Deploy modal (after)
```
+--------------------------------------------+
| [ DEPLOY ]   BIND NEW UNITS            [x]  |
+--------------------------------------------+
| [ CALLSIGNS ]  (one per line)               |
| +----------------------------------------+  |
| | SB-3660                                |  |
| | SB-58EF                                |  |
| | SB-...                                 |  |
| +----------------------------------------+  |
| Tags 1-16 auto-assigned to free slots.      |
| 3 callsigns · 13 tags free                  |
+--------------------------------------------+
|                      [CANCEL]  [DEPLOY ALL] |
+--------------------------------------------+
```

### Wireframe — Fleet card (checkbox added) + selection bar
```
// FLEET     [ 2 SELECTED ]  [DETACH SELECTED]  [DETACH ALL]
+---------------------------+
| [x] SB-3660      ● ONLINE |   <- checkbox top-left of unit__head
| [CH] 5001  [UPT] 2M 4S    |
| [URL] http://...          |
| [ ▸ CONSOLE ] [ × DETACH ] |
+---------------------------+
```

### User flows
- Deploy: DEPLOY UNIT → textarea → DEPLOY ALL → POST batch → summary toast ("6 deployed, 2 failed: SB-X no free tags; SB-Y worker full") → modal closes on full success, stays open (showing failures) on partial.
- Detach selected: tick cards → bar shows count + enables DETACH SELECTED → confirm modal ("Release 3 units?") → POST batch_delete → summary toast.
- Detach all: DETACH ALL (always enabled when units>0) → confirm ("Release ALL 8 units?") → POST batch_delete with every name.

## Detailed Plan

### Step 1 — Backend: `POST /api/spheros/batch`
- File: `multirobot_webserver/multirobot_webapp.py` (new route near line 1291; small helper on `SpheroInstanceManager`).
- Request: `{ "names": ["SB-3660", "SB-58EF", ...] }`
- Server logic (new `manager.add_spheros_batch(names)`):
  1. Normalize: strip each, drop blanks, dedupe preserving order (first wins).
  2. For each name, **in order**:
     - If already in `self.instances` → fail `{reason: "already deployed"}`.
     - Compute `free = fleet_node.free_tag_ids()` (or `UWB_TAG_IDS` minus locally-tracked when no fleet_node) **fresh each iteration**; if empty → fail `{reason: "no free tags"}`.
     - `tag_id = free[0]`; call existing `self.add_sphero(name, tag_id)` (reuses dup/capacity/remote logic + rollback). On success the fleet_node now owns that tag, so the next iteration's `free_tag_ids()` excludes it — no in-batch collision.
     - Map `add_sphero` result → item `{name, success, tag_id?, port?, reason?}` (reason = `result['message']` on failure).
- Response `200`:
  ```json
  { "success": true,
    "deployed": 6, "failed": 2,
    "results": [
      {"name":"SB-3660","success":true,"tag_id":1,"port":5001},
      {"name":"SB-XXXX","success":false,"reason":"all workers at capacity (8/8 BLE slots used)"}
    ] }
  ```
- `400` only if `names` missing/not a list/empty-after-normalize. Per-item failures are **not** HTTP errors (overall request succeeded).
- **Justification for a dedicated batch endpoint vs. JS looping single POSTs:** (1) tag assignment stays atomic & authoritative on the side that owns `fleet_node` — JS can't race-free pick free tags; (2) one round-trip; (3) clean per-item result shape for the summary toast. Recommended.

### Step 2 — Backend: `POST /api/spheros/batch_delete`
- File: same. New `manager.remove_spheros_batch(names)`.
- Request: `{ "names": [...] }`. Loops `self.remove_sphero(name)`, collecting `{name, success, reason?}`.
- Response mirrors Step 1: `{success, removed, failed, results:[...]}`.
- **Justification vs. client-looping `DELETE`:** a single endpoint gives one summary round-trip and uniform partial-failure reporting, symmetric with batch deploy. Low complexity (thin loop over existing `remove_sphero`). Recommended over client loop.

### Step 3 — Frontend: convert deploy modal (`templates/index.html`)
- In `#addSpheroModal` `.modal__body`: remove `#tagIdSelect` + its label/hint and the single `#spheroNameInput`; add:
  - `<label class="field__label">[ CALLSIGNS ]</label>`
  - `<textarea id="spheroNamesInput" class="field" rows="6" ... placeholder="SB-3660&#10;SB-58EF"></textarea>`
  - hint paragraph + a live `<span id="batchSummary">` ("N callsigns · M tags free").
- Change foot button label to `DEPLOY ALL` (keep `id="confirmAddBtn"`, classes unchanged).
- Modal title → `BIND NEW UNITS`.

### Step 4 — Frontend: deploy logic (`static/js/app.js`)
- Remove `#tagIdSelect`/`populateTagSelect` usage and the tag branch in the `confirmAdd` handler (app.js:63-73). Delete `populateTagSelect` (258-289) and the `populateTagSelect()` call in `refreshUwbTags` (250); keep `refreshUwbTags` itself (still feeds the UWB panel counts) — confirm it's not otherwise needed by tag select before deleting.
- New `parseCallsigns(text)`: split on `\n`, trim, drop blanks, dedupe (case-sensitive, preserve order). Optional client pre-check: warn if a name is already in `this.spheros` (friendly only; server is authoritative).
- `confirmAdd` → gather names → if empty, `toast('Enter at least one callsign.', 'error', 'DEPLOY')` → else `deploySpheros(names)`.
- New `async deploySpheros(names)` → `POST /api/spheros/batch {names}` → on response, build summary via `summarizeBatch(data, 'DEPLOY', 'deployed')`; `refresh()` + `refreshUwb()`/`refreshUwbTags()`; close modal only if `data.failed === 0`, else keep open and leave failed lines for retry.
- New `summarizeBatch(data, tag, verb)` helper → emits a `toast`. Success-only: `success` toast "8 deployed". Any failures: `error` toast "6 deployed, 2 failed: SB-X (no free tags); SB-Y (worker full)" (truncate to first ~3 names + "+N more").
- `<textarea>` input listener updates `#batchSummary` live (count of parsed lines, free-tag count from `this.uwbTags.free.length`).

### Step 5 — Frontend: multi-select state model (`static/js/app.js`)
- Add `this.selected = new Set()` in constructor.
- `unitTile` gains a checkbox in `unit__head`:
  `<input type="checkbox" class="unit__select" id="sel-${safe(name)}" aria-label="Select ${safe(name)}">`.
- In `renderFleet`, **two paths must reconcile selection**:
  - When signature unchanged (only `[UPT]` updates) — checkboxes still exist; nothing to do beyond pruning `selected` of names no longer present.
  - When grid rebuilt — after `grid.innerHTML = ...` and the existing open/detach binding loop, also: for each unit set `checkbox.checked = this.selected.has(name)` and bind `change` → toggle `selected` + `updateSelectionBar()`.
  - Always: `this.selected = new Set([...this.selected].filter(n => currentNames.has(n)))` (drop detached units).
- `updateSelectionBar()`: set counter text `[ N SELECTED ]`; enable/disable `#detachSelectedBtn` (`disabled = selected.size===0`); show/hide the whole selection bar + `#detachAllBtn` based on `this.spheros.length>0`. Call it at the end of `renderFleet`.

### Step 6 — Frontend: selection bar markup (`templates/index.html`)
- In the FLEET panel `.panel__head` (near line 113), add a controls row:
  ```html
  <div class="fleet__select" id="fleetSelectBar" hidden>
    <span class="fleet__count" id="selectedCount">[ 0 SELECTED ]</span>
    <button class="action action--danger" id="detachSelectedBtn" disabled>
      <span class="action__glyph">×</span><span class="action__label">DETACH SELECTED</span></button>
    <button class="action action--danger" id="detachAllBtn">
      <span class="action__glyph">⊗</span><span class="action__label">DETACH ALL</span></button>
  </div>
  ```

### Step 7 — Frontend: batch detach logic + confirm reuse (`static/js/app.js`)
- `bindActions`: wire `#detachSelectedBtn` → `requestBatchDetach([...this.selected])`; `#detachAllBtn` → `requestBatchDetach(this.spheros.map(s=>s.name))`.
- Generalize the existing confirm modal: add `this.pendingBatchDetach = null`. `requestBatchDetach(names)` sets the confirm copy ("Release N unit(s)? This terminates each unit's WebSocket server…") and opens `confirmRemoveModal`. `#confirmRemoveBtn` handler branches: if `pendingBatchDetach` → `detachSpheros(names)`; else existing single path. Keep single `requestDetach` working (per-card DETACH unchanged).
- New `async detachSpheros(names)` → `POST /api/spheros/batch_delete {names}` → `summarizeBatch(data,'DETACH','removed')` → clear `this.selected` for removed → `refresh()` + `refreshUwb()`.

### Step 8 — CSS (`static/css/style.css`)
- Minimal additions reusing tokens: `.fleet__select` (flex row, gap, wrap), `.fleet__count` (mono, `var(--fg-2)`), `.unit__select` (accent-color: `var(--amber)`; position in `unit__head`). No new colors/fonts.

## API Endpoints

### REST (new)
| Method | Path | Request | Response |
|--------|------|---------|----------|
| POST | /api/spheros/batch | `{names:[...]}` | `{success, deployed, failed, results:[{name,success,tag_id?,port?,reason?}]}` |
| POST | /api/spheros/batch_delete | `{names:[...]}` | `{success, removed, failed, results:[{name,success,reason?}]}` |

### REST (unchanged, retained)
`GET/POST /api/spheros`, `DELETE /api/spheros/<name>`, `GET /api/uwb/tags`. (Single POST keeps its `tag_id` contract; front-end stops calling it.)

## Data Flow
```
Operator paste → textarea → parseCallsigns() → POST /api/spheros/batch
  → manager.add_spheros_batch(): per name → free_tag_ids()[0] → add_sphero()
      → local subprocess OR _add_sphero_remote (least-loaded worker) → fleet_node.add_robot()
  → results[] → summary toast + 5s poll re-render
```

## Expected Outcomes
- Paste many callsigns, one click deploys all; tags auto-assigned, no UI tag picking.
- Per-card checkboxes; DETACH SELECTED (≥1) and DETACH ALL (units>0), each confirmed.
- Selection survives the 5s re-render; detached units drop out of the selection set.
- Partial failures reported clearly via one summary toast.

## Potential Risks & Considerations
- **In-batch tag collision** — mitigated by recomputing `free_tag_ids()` each iteration after each successful add updates `fleet_node`.
- **Capacity partial-fail** (distributed) — surfaced per item via `add_sphero`'s message; `select_worker` RuntimeError text reused.
- **Large batch latency** — local `add_sphero` does `time.sleep(2)` per unit; 16 units ≈ 30s blocking one request. Acceptable for now (operator initiates; dev server). Call out: if this is too slow we can later parallelize or trim the sleep — NOT in scope unless you want it.
- **Selection desync** — reconcile against current unit names every render; prune stale names.
- **XSS** — names already escaped via `safe()` in tiles and `toast()` escaping; keep escaping in the summary list.
- **No new deps**, retro theme preserved.

## Testing Plan
- [ ] Deploy 3 valid callsigns → 3 cards, tags 1–3 (or next free), success toast.
- [ ] Deploy with 1 duplicate + 1 blank line → dedupe/skip; correct counts.
- [ ] Deploy beyond worker capacity (distributed) → partial summary lists failures; modal stays open.
- [ ] Tick 2 cards → counter "2 SELECTED", DETACH SELECTED enabled; confirm → both gone.
- [ ] DETACH ALL → confirm → fleet empties; selection bar hides.
- [ ] Selection persists across a 5s poll (tick a card, wait 5s, still ticked); detach a *different* card out-of-band → stale name pruned.
- [ ] Browsers: Chrome/Firefox; mobile width (textarea + wrapped select bar usable).

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
```
