# Visual Lane-Based Drag-and-Drop Bundle Builder

**Created:** 2026-06-01T21:00:00Z
**Status:** Pending Approval
**Complexity:** High
**Technologies:** HTML5 (drag-and-drop), Vanilla JS (ES6+), CSS3 (Grid/Flex), Flask REST endpoint `/api/broadcast_task` (unchanged)

## Task Description

Replace the broadcast modal's "task-type select + ADD button + raw-JSON textarea" composer with a **visual, lane-based builder** shown ALONGSIDE the existing JSON text view (kept in sync). A Sphero runs one task per actuator "lane" concurrently; placing two *owner* tasks in the same lane produces a bundle the controller silently rejects. The builder must surface lanes as columns/zones, let operators drag task cards into lanes, edit per-card params and `start_offset`, **block + visually flag owner-lane conflicts**, disable BROADCAST while any conflict exists, and stay two-way-synced with the JSON textarea.

Scope of THIS deliverable: design only. No code is written.

## Requirements Analysis

- **User interface needs:** A palette of draggable task-type cards; a lane board (columns) where cards live; each placed card shows type + compact editable param summary + remove control; per-card `start_offset`; a synced JSON view; LEAD seconds input; BROADCAST button.
- **Backend integration:** Unchanged. POST `{ tasks: [{task_type, parameters, start_offset}], start_offset }` to `/api/broadcast_task`.
- **Real-time requirements:** None new. Existing fleet `runningCount()` gates open; existing countdown stays.
- **Browser compatibility:** Desktop Chrome/Firefox/Safari/Edge (current). HTML5 native DnD works on all; mobile/touch is a non-goal but click-to-add gives a touch fallback.
- **Responsive design:** Lane board scrolls horizontally on narrow viewports; modal panel widened from 480px to ~860px for the builder.
- **Accessibility:** Click-to-add as a non-drag path; ARIA roles on lanes (`role="list"`) and cards (`role="listitem"`); conflict state announced via `aria-invalid` + visible text; keyboard-reachable remove buttons.

## Analysis (current state)

**index.html (`#broadcastModal`, lines 309–355):** select `#broadcastType` (16 options), `#broadcastAddBtn`/`#broadcastClearBtn`, `#broadcastParams` textarea (JSON array), `#broadcastLanes` hint (`hidden`), `#broadcastLead` number input, `#broadcastSummary`, `#broadcastCountdown`, footer `#cancelBroadcastBtn` / `#confirmBroadcastBtn`. Panel `max-width:480px`.

**app.js:**
- `BROADCAST_DEFAULTS` — per-type default params object (roll, move_to, patrol, circle, square, spin, set_led, stop, heading, speed, matrix, led_sequence, matrix_sequence, collision, reflect, custom). `jumping_bean` is referenced in `TASK_LANES` but has no default and no `<option>`.
- `ALL_LANES = ['DRIVE','LED','MATRIX']`, `TASK_LANES` — coarse 3-lane map; advisory only.
- `openBroadcast()` gates on `runningCount()`, calls `renderBroadcastLanes()`, sets summary, opens modal.
- `parseBroadcastBundle()` — JSON.parse the textarea; returns array or `null`.
- `addBroadcastTask()` — pushes `{task_type, parameters: defaults, start_offset:0}` and re-serializes the textarea.
- `clearBroadcastBundle()` — resets textarea to `[]`.
- `renderBroadcastLanes()` — recomputes a one-line conflict/stagger hint; never blocks.
- `sendBroadcast()` — parses, validates each entry has a string `task_type`, clamps lead ≥0.5, POSTs, toasts, starts countdown. **This is the only network path and stays intact.**
- Event wiring (lines 150–155): open, add, clear, textarea `input`→renderLanes, confirm→send, cancel→close.

**style.css:** retro-terminal palette via CSS vars (`--bg`,`--bg-1..3`,`--fg`,`--fg-1..3`,`--line`,`--line-strong`,`--amber` [a blue accent #4f9dff],`--amber-soft`,`--green`,`--red`,`--mono`,`--rad`). `.modal__*`, `.field`, `.field__label`, `.field__hint`, `.action`/`--primary`/`--accent`/`--ghost`/`--danger`, `.modal__tag--warn`. We reuse these tokens; no new colors invented (conflict = `--red`, ok = `--green`, accent = `--amber`).

**Key design tension:** the lane taxonomy here is *provisional*; the FINAL owner/modifier classification comes from a parallel controller-side design. Therefore the builder must be **data-driven off a single config object** so the lane list and per-type class can be reconciled later by editing data only — no structural rewrite.

## UI/UX Design

### Wireframe / Mockup

```
+--------------------------------------------------------------------------+
| [ BROADCAST ]  FAN TASK TO FLEET                                      [x] |
+--------------------------------------------------------------------------+
|  PALETTE (drag a card into a lane, or click to drop into its home lane)   |
|  +--------+ +--------+ +--------+ +--------+ +--------+ +--------+         |
|  | roll  ◆| | circle◆| |move_to◆| | spin  ◆| |heading○| | speed ○|  ...    |
|  +--------+ +--------+ +--------+ +--------+ +--------+ +--------+         |
|     ◆ = owner (one per lane)        ○ = modifier (always allowed)         |
|--------------------------------------------------------------------------|
|  LANE BOARD                                            (scrolls →)        |
|  DRIVE      AIM      THROTTLE  LED-MAIN  LED-FRONT LED-BACK MATRIX  CONFIG |
| +--------+ +------+ +-------+ +-------+ +--------+ +------+ +------+ +----+|
| | roll  ◆| |head ○| | spd  ○| |seq   ◆| | led  ○ | |led  ◆| |mtx  ◆| |col ||
| | hd 0   | |hd 90 | | 100   | |3 step | | r0g0b0 | |...   | |smile | |... ||
| | sp 100 | |@2s   | | @0s   | |@0s    | | @0s  ✕ | |@0s  ✕| |@0s ✕ | |✕   ||
| | @0s  ✕ | | ✕    | | ✕     | | ✕     | +--------+ +------+ +------+ +----+|
| +--------+ +------+ +-------+ +-------+                                    |
| | (drop) | |(drop)| |(drop) | |(drop) |   ← empty lanes show a drop hint  |
| +--------+ +------+ +-------+ +-------+                                    |
|  ^ a lane with TWO owners turns RED and shows "⚠ 2 owners — pick one"     |
|--------------------------------------------------------------------------|
| [ BUNDLE TASKS (JSON) ]   (advanced — edits on blur re-parse the board)   |
| +----------------------------------------------------------------------+ |
| | [ {"task_type":"roll","parameters":{...},"start_offset":0}, ... ]     | |
| +----------------------------------------------------------------------+ |
|  lanes: DRIVE + LED + MATRIX · DRIVE@0s, AIM@2s …    (or red conflict)    |
|  [ LEAD SECONDS ]  [ 3.0 ]   N units · fires at now + lead               |
+--------------------------------------------------------------------------+
|                                    [ CANCEL ]   [ BROADCAST ALL ]         |
+--------------------------------------------------------------------------+
```

### User Flow

1. Operator clicks BROADCAST TASK → modal opens, board rebuilt from current JSON (`[]` on first open).
2. Drags a "roll" card from palette → drops on DRIVE lane → card appears; JSON updates live; lane summary updates.
3. Drags a second owner ("circle") onto DRIVE → lane already has an owner → lane flashes red, shows "⚠ 2 owners", BROADCAST disabled. (Drop is *accepted but flagged* — see "Conflict policy" — so the operator can see both and remove one.)
4. Removes "circle" (✕) → DRIVE valid again → BROADCAST re-enabled.
5. Edits a card's `start_offset` field → JSON + lane stagger readout update.
6. (Optional) edits JSON directly → on blur, board re-parses and re-renders; invalid JSON shows a warning and leaves the board untouched.
7. Sets LEAD → clicks BROADCAST ALL → existing `sendBroadcast()` POSTs → countdown.

## Detailed Plan

### Step 1: Lane/class CONFIG object (data-driven core)

- **File:** `static/js/app.js` (new constants near `BROADCAST_DEFAULTS`).
- Replace the coarse `ALL_LANES`/`TASK_LANES` advisory map with a richer config. Keep names generic so the controller team's final taxonomy is a data edit only.

```js
// Lane definitions — ORDER here is the column order on the board.
const LANES = [
  { id: 'DRIVE',     label: 'DRIVE' },
  { id: 'AIM',       label: 'AIM' },        // heading
  { id: 'THROTTLE',  label: 'THROTTLE' },   // speed
  { id: 'LED_MAIN',  label: 'LED·MAIN' },
  { id: 'LED_FRONT', label: 'LED·FRONT' },
  { id: 'LED_BACK',  label: 'LED·BACK' },
  { id: 'MATRIX',    label: 'MATRIX' },
  { id: 'CONFIG',    label: 'CONFIG' },
];

// Per-type class. role: 'owner' (one per lane, conflicts) | 'modifier' (never conflicts).
// lane: the card's HOME lane (where click-to-add drops it, and the only lane it accepts
// when role==='owner'). For modifiers whose lane depends on a param (set_led's led_type),
// `laneFor(task)` resolves it dynamically; otherwise the static `lane` is used.
const TASK_CLASS = {
  // ---- DRIVE owners (continuous) ----
  roll:        { role: 'owner',    lane: 'DRIVE' },
  move_to:     { role: 'owner',    lane: 'DRIVE' },
  circle:      { role: 'owner',    lane: 'DRIVE' },
  square:      { role: 'owner',    lane: 'DRIVE' },
  patrol:      { role: 'owner',    lane: 'DRIVE' },
  spin:        { role: 'owner',    lane: 'DRIVE' },
  reflect:     { role: 'owner',    lane: 'DRIVE' },
  jumping_bean:{ role: 'owner',    lane: 'DRIVE' },
  // ---- LED / MATRIX owners ----
  led_sequence:   { role: 'owner', lane: 'LED_MAIN' },
  matrix_sequence:{ role: 'owner', lane: 'MATRIX' },
  // ---- modifiers (instantaneous, never conflict) ----
  heading: { role: 'modifier', lane: 'AIM' },
  speed:   { role: 'modifier', lane: 'THROTTLE' },
  set_led: { role: 'modifier', lane: 'LED_MAIN',  laneFor: (t) => LED_LANE_BY_TYPE[t?.parameters?.led_type] || 'LED_MAIN' },
  matrix:  { role: 'modifier', lane: 'MATRIX' },
  stop:    { role: 'modifier', lane: 'DRIVE' },   // provisional; controller design may reclass
  collision:{ role: 'modifier', lane: 'CONFIG' },
  custom:  { role: 'modifier', lane: 'CONFIG' },  // provisional: spans many lanes → treat as non-conflicting CONFIG card
};

const LED_LANE_BY_TYPE = { main: 'LED_MAIN', front: 'LED_FRONT', back: 'LED_BACK' };

// Compact param fields surfaced as inline editors per card. Anything not listed
// stays editable only via the card's "{…}" JSON popover (or the global JSON view).
const CARD_FIELDS = {
  roll:    [['heading','#'], ['speed','#'], ['duration','#']],
  move_to: [['x','#'], ['y','#'], ['speed','#']],
  circle:  [['radius','#'], ['speed','#'], ['duration','#']],
  spin:    [['duration','#'], ['speed','#']],
  heading: [['heading','#']],
  speed:   [['speed','#']],
  set_led: [['led_type','sel:main,front,back'], ['red','#'], ['green','#'], ['blue','#']],
  matrix:  [['pattern','txt'], ['red','#'], ['green','#'], ['blue','#']],
  // others (patrol, square, reflect, led_sequence, matrix_sequence, collision, custom)
  // expose no inline fields → edited via the JSON popover only.
};
```

- **Helper** `classOf(type)` returns `TASK_CLASS[type] || { role:'modifier', lane:'CONFIG' }` (unknown types degrade to a non-conflicting CONFIG card so nothing breaks if the controller adds a type).
- **Helper** `laneOf(task)` = `cls.laneFor ? cls.laneFor(task) : cls.lane`.
- **Outcome:** all UI structure derives from `LANES` + `TASK_CLASS`; the final taxonomy is a data swap.

### Step 2: State model

- **File:** `static/js/app.js`, in `ControlStation`.
- Single source of truth: `this.bundle = []` — an array of task objects exactly matching the wire format `{ task_type, parameters, start_offset }`, **plus** a non-serialized `_uid` (stable id for DOM keying and drag identification). `_uid` is stripped before send/serialize.
- Derived views (never stored): the lane board DOM and the JSON textarea are both *renderings* of `this.bundle`.
- Reconciliation rule: **`this.bundle` is canonical.** Builder edits mutate `bundle` then call `renderBundle()` (rebuild board) + `syncJsonFromBundle()`. JSON edits go bundle-ward only on `blur`/parse via `syncBundleFromJson()`.

```
this.bundle: [ {_uid, task_type, parameters, start_offset}, ... ]
        │  mutate (drag/drop, add, remove, field edit)
        ▼
   renderBundle()  ──► board DOM (cards grouped by laneOf)
   syncJsonFromBundle() ──► #broadcastParams textarea (clean JSON, _uid stripped)
        ▲
   syncBundleFromJson()  ◄── textarea blur (parse; assign fresh _uids; bad JSON → keep old bundle + warn)
```

### Step 3: Functions (replace/extend in app.js)

| Function | Purpose |
|----------|---------|
| `openBroadcast()` | (edit) build `this.bundle` from current JSON via `syncBundleFromJson()` once, then `renderBuilder()`. Keep running-count gate + summary + countdown reset. |
| `renderBuilder()` | Render palette (once) + `renderBundle()` + `syncJsonFromBundle()` + `updateConflicts()`. |
| `renderPalette()` | Build palette cards from `TASK_CLASS` keys; each gets `draggable=true`, `data-type`, owner/modifier glyph; click = `addTaskToHomeLane(type)`. |
| `renderBundle()` | Clear lane `__items` containers; for each task compute `laneOf(task)`, append a card built by `buildCard(task)` to that lane. |
| `buildCard(task)` | Returns a card element: header (type + role glyph + ✕), inline `CARD_FIELDS` editors, a `start_offset` field, and a `{…}` button opening a small JSON popover for full `parameters`. `draggable=true`, `data-uid`. |
| `addTaskToHomeLane(type)` | Push `{_uid, task_type:type, parameters: structuredClone(BROADCAST_DEFAULTS[type]||{}), start_offset:0}`; re-render. (Click-to-add / a11y path.) |
| `addTaskToLane(type, laneId)` | Drag-drop path: same as above but, for a *modifier* with a per-lane variant (`set_led`), seed `led_type` from `laneId`; owners ignore `laneId` (always home lane). |
| `moveTask(uid, laneId)` | Drag an existing card to another lane. Owners can't change lane (no-op + toast). Modifiers with `laneFor` (set_led) update the param that determines their lane; pure modifiers keep their type (lane is fixed by type, so this is a no-op except for set_led). |
| `removeTask(uid)` | Splice from bundle; re-render. |
| `editCardField(uid, key, value)` | Coerce (`#`→Number) and set `task.parameters[key]` (or `start_offset`); re-render lane membership if `set_led.led_type` changed; `syncJsonFromBundle()`; `updateConflicts()`. |
| `clearBroadcastBundle()` | (edit) `this.bundle = []`; re-render. |
| `syncJsonFromBundle()` | `#broadcastParams.value = JSON.stringify(bundle.map(strip_uid), null, 2)`. |
| `syncBundleFromJson()` | Parse textarea; on success map to bundle with fresh `_uid`s and `renderBundle()`+`updateConflicts()`; on failure set the warning hint and leave bundle/board unchanged. |
| `laneConflicts()` | Returns `Set` of lane ids holding ≥2 **owners**. (Modifiers excluded entirely.) |
| `updateConflicts()` | Mark each lane el `data-conflict`; write `#broadcastLanes` summary (reuse element); enable/disable `#confirmBroadcastBtn` based on `laneConflicts().size===0`. |
| `sendBroadcast()` | (edit) build `tasks` from `this.bundle.map(strip_uid)` instead of re-parsing JSON; keep validation, lead clamp, POST, toast, countdown. **Guard:** abort + toast if `laneConflicts().size>0`. |

`renderBroadcastLanes()` (old advisory one-liner) is **removed**; its textarea `input` listener is replaced by a `blur` listener calling `syncBundleFromJson()`. Inline card-field `input` listeners drive `editCardField`.

### Step 4: HTML markup (index.html `#broadcastModal` body rewrite)

Replace lines 317–342 (select + add/clear + textarea + old hint) with a builder block; keep `#broadcastParams` (now an "advanced" view), `#broadcastLead`, `#broadcastSummary`, `#broadcastCountdown`, and the footer.

```html
<!-- PALETTE -->
<label class="field__label">[ PALETTE — DRAG OR CLICK TO ADD ]</label>
<div id="broadcastPalette" class="palette" role="list"></div>

<!-- LANE BOARD -->
<label class="field__label">[ LANES — ONE OWNER ◆ PER LANE ]</label>
<div id="broadcastBoard" class="board" role="group" aria-label="Bundle lanes"></div>
<!-- each lane injected as:
  <div class="lane" data-lane="DRIVE" role="list" aria-label="DRIVE lane">
    <div class="lane__head"><span class="lane__name">DRIVE</span>
         <span class="lane__flag" hidden>⚠ 2 owners</span></div>
    <div class="lane__items"></div>            (drop target)
    <div class="lane__drop">drop task</div>    (empty-state hint)
  </div>
-->

<details class="bundle-json">
  <summary class="field__label">[ BUNDLE TASKS (JSON) — ADVANCED ]</summary>
  <textarea id="broadcastParams" class="field" rows="6" spellcheck="false">[]</textarea>
  <p class="field__hint" id="broadcastLanes" hidden></p>
</details>
```

- The `<select id="broadcastType">` is removed (the palette replaces it); `#broadcastAddBtn`/`#broadcastClearBtn` are removed (palette = add; a small CLEAR button moves next to the JSON `<details>` summary). Card template for `buildCard` is created in JS (no `<template>` needed but a `<template id="cardTpl">` is acceptable).
- Panel widened: add class to `#broadcastModal .modal__panel` or a modal modifier `modal__panel--wide` → `max-width: 860px`.

### Step 5: Drag-and-drop wiring (HTML5 native DnD)

Palette cards and placed cards are `draggable`. Lanes are drop targets.

- **dragstart** (palette card): `dataTransfer.setData('text/x-task-type', type)`; add `.dragging`.
- **dragstart** (placed card): `dataTransfer.setData('text/x-task-uid', uid)`.
- **lane dragover:** `preventDefault()` (to allow drop); compute would-be validity and toggle `.lane--drop-ok` / `.lane--drop-block` for live feedback (owner into occupied owner-lane → block styling, but drop still allowed → see policy).
- **lane drop:** read uid first (move) else type (add). `addTaskToLane(type, laneId)` or `moveTask(uid, laneId)`.
- **dragend / dragleave:** clear hover classes.

All listeners attached once via delegation on `#broadcastBoard` and `#broadcastPalette` (board is rebuilt, so delegate on the static containers, not per-card).

### Step 6: Conflict detection + policy

- **Policy:** dropping a second owner into an owner-occupied lane is **accepted but flagged red**, BROADCAST disabled, lane shows "⚠ N owners — keep one". Rationale: the user explicitly asked to "block + show what's in each lane" — showing both offenders and disabling send is clearer than silently refusing the drop (which hides *what* conflicts). Modifiers are never flagged.
- `laneConflicts()` groups `bundle` by `laneOf`, counts `role==='owner'` per lane, returns lanes with count ≥2.
- `updateConflicts()` runs after every mutation: sets `lane[data-conflict="true"]`, shows `.lane__flag`, sets `#confirmBroadcastBtn.disabled`, and `aria-invalid` on conflicting lanes. Writes the `#broadcastLanes` summary (ok → "lanes: … · stagger"; conflict → "⚠ lane conflict: DRIVE …").

### Step 7: CSS (style.css additions)

All new rules reuse existing tokens; append a `/* broadcast builder */` block.

```
.modal__panel--wide { max-width: 860px; }

.palette { display:flex; flex-wrap:wrap; gap:6px; margin-bottom:14px; }
.palette__card { /* small chip: bg-2, line border, --rad, mono 12px, cursor:grab */ }
.palette__card[data-role="owner"]    { border-left:3px solid var(--amber); }
.palette__card[data-role="modifier"] { border-left:3px solid var(--green); }

.board { display:flex; gap:8px; overflow-x:auto; padding-bottom:6px; }
.lane  { flex:0 0 150px; background:var(--bg-1); border:1px solid var(--line); border-radius:var(--rad); display:flex; flex-direction:column; min-height:140px; }
.lane__head { font-size:10px; letter-spacing:.12em; color:var(--fg-2); padding:6px 8px; border-bottom:1px solid var(--line); display:flex; justify-content:space-between; }
.lane__items { flex:1; padding:6px; display:flex; flex-direction:column; gap:6px; }
.lane__drop  { color:var(--fg-3); font-size:10px; text-align:center; padding:8px; }
.lane--drop-ok    { outline:1px dashed var(--green); }
.lane--drop-block { outline:1px dashed var(--red); }
.lane[data-conflict="true"] { border-color:var(--red); }
.lane[data-conflict="true"] .lane__head { color:var(--red); }
.lane__flag { color:var(--red); }

.card { background:var(--bg-2); border:1px solid var(--line-strong); border-radius:var(--rad); padding:6px; font-size:11px; cursor:grab; }
.card--owner    { border-left:3px solid var(--amber); }
.card--modifier { border-left:3px solid var(--green); }
.card__head { display:flex; justify-content:space-between; align-items:center; gap:4px; }
.card__type { font-weight:600; color:var(--fg); }
.card__rm   { color:var(--fg-2); }   /* ✕ — :hover color:var(--red) */
.card__field { display:flex; gap:4px; align-items:center; margin-top:3px; }
.card__field label { color:var(--fg-3); font-size:9px; width:42px; }
.card__field input, .card__field select { /* compact --bg-3 inputs, padding 2px 4px, font 11px */ }
.card.dragging { opacity:.5; }
.bundle-json summary { cursor:pointer; }
```

- Conflict color = `--red`; owner accent = `--amber`; modifier = `--green`. No new palette values.

## API Endpoints / WebSocket Events

### REST Endpoints (UNCHANGED)
| Method | Path | Request | Response |
|--------|------|---------|----------|
| POST | `/api/broadcast_task` | `{ tasks:[{task_type,parameters,start_offset}], start_offset }` | `{ success, sent, failed, results[], start_offset }` |

No WebSocket changes. No new endpoints.

## Data Flow

```
Drag/click/edit ─► this.bundle (canonical) ─► renderBundle() ─► lane board DOM
                                            └► syncJsonFromBundle() ─► JSON textarea
JSON textarea (blur) ─► syncBundleFromJson() ─► this.bundle ─► re-render
BROADCAST ─► sendBroadcast() ─► POST /api/broadcast_task ─► fleet ─► countdown
            (guarded: blocked while laneConflicts().size > 0)
```

## Expected Outcomes

- Operators build bundles visually; lane occupancy is always visible.
- Two owners in a lane → red lane + disabled BROADCAST → no more silent controller rejections.
- Modifiers stack freely; per-card params and `start_offset` editable inline.
- JSON view remains for power users and stays in sync.
- Send path, endpoint, payload shape, and countdown unchanged.
- Lane taxonomy reconcilable with the controller design by editing `LANES`/`TASK_CLASS` only.

## Potential Risks & Considerations

- **Security/XSS:** card fields and type labels must be set via `textContent`/`value`, never `innerHTML` with user/param data. JSON popover uses `<textarea>.value`. (Low risk — same-origin operator console, but enforce it.)
- **Two-way sync loops:** guard against feedback (JSON sync only on `blur`, not `input`; builder mutations don't trigger the JSON listener). Use a `this._syncing` flag if needed.
- **`_uid` leakage:** must strip `_uid` before serialize/send (controller would ignore it, but keep the wire clean).
- **Taxonomy drift:** lane map is provisional; unknown/reclassed types degrade to a CONFIG modifier so nothing crashes. Add a code comment pointing to the controller-side design as the source of truth.
- **DnD on touch:** native HTML5 DnD doesn't fire on touch; click-to-add is the documented fallback. Acceptable (desktop operator console).
- **Modifier "lane" semantics:** `set_led` lane depends on `led_type`; editing that field must move the card between LED lanes — covered by `editCardField` re-render. Verify no orphaned cards.
- **`jumping_bean`/`stop`/`custom` provisional classing:** flagged in comments; final classes from controller design.

## Testing Plan

- Manual:
  - [ ] Chrome/Firefox/Safari/Edge: drag palette→lane, drag card lane→lane, click-to-add, remove.
  - [ ] Two owners same lane → lane red, flag shown, BROADCAST disabled; remove one → re-enabled.
  - [ ] Modifiers never conflict (stack 3 `set_led` of different `led_type` → 3 different LED lanes; same type → same lane, no conflict since modifiers excluded).
  - [ ] Edit inline field → JSON updates; edit `start_offset` → stagger readout updates.
  - [ ] Edit JSON → blur → board rebuilds; invalid JSON → warning + board unchanged.
  - [ ] BROADCAST posts identical payload as before (verify in Network tab); countdown still runs.
  - [ ] Open with a pre-existing non-empty JSON bundle → board reflects it.
- Accessibility:
  - [ ] Tab to palette cards / remove buttons; Enter on a palette card adds it (click path).
  - [ ] Conflicting lane exposes `aria-invalid`; flag text readable.
- Performance:
  - [ ] Re-render under ~16ms for typical bundles (<20 cards).

## Browser Compatibility

- Chrome 90+, Firefox 90+, Safari 14+, Edge 90+ (native HTML5 DnD, `structuredClone` — Safari 15.4+; fall back to `JSON.parse(JSON.stringify())` if older Safari matters).
- Mobile: click-to-add works; drag is desktop-only (non-goal).

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
