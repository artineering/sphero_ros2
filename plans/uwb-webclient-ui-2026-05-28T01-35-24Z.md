# UWB Positioning — Web Client UI

**Created:** 2026-05-28T01:35:24Z
**Status:** Pending Approval
**Complexity:** Medium
**Technologies:** Vanilla JS (ES6 class), HTML5, CSS3 (existing design-token system), Flask/Jinja2 template, Fetch API polling (NO Socket.IO — matches existing stack)

## Task Description
Add the front-end UI for the already-implemented UWB positioning backend in
`multirobot_webapp.py`. Three pieces:
1. Required UWB tag-id selector in the existing add-Sphero ("DEPLOY UNIT") modal.
2. An arena / anchor configuration section (4 anchors A0–A3, x/y in cm).
3. UWB positioning start/stop control + status indicator, mirroring the existing
   ArUco SLAM start/stop/status pattern.

No backend changes. The REST contract is authoritative and already live.

## Requirements Analysis
- **User interface needs:**
  - Operator must pick a free tag id (1–16) when deploying a Sphero; already-used
    ids are visibly unavailable.
  - Operator can enter/edit the 4 anchor coordinates (cm) and save them.
  - Operator can start/stop UWB positioning and see running / configured / fake /
    assigned-count status at a glance.
- **Backend integration:** plain Flask REST (the 7 endpoints below); no Socket.IO.
- **Real-time requirements:** front-end polling only (matches existing ArUco 4s
  poll and fleet 5s poll). No new transport.
- **Browser compatibility:** modern evergreen browsers (Chrome/Firefox/Safari/Edge);
  the existing code already uses `fetch`, optional chaining, template literals, and
  CSS custom properties, so no new baseline is introduced.
- **Responsive design:** reuse existing `@media (max-width: 720px)` panel/flex rules;
  the new anchor form is a simple grid that stacks naturally.
- **Accessibility:** `<label for>` on every new input/select, `aria-modal` already on
  modal, buttons keep visible focus via existing `.action` styling, status conveyed
  with text (not color alone) via the existing readout pattern.

## Analysis (current state)

### Files in scope
- `src/multirobot_webserver/templates/index.html` — markup (ribbon, panels, modal).
- `src/multirobot_webserver/static/js/app.js` — single `ControlStation` ES6 class.
- `src/multirobot_webserver/static/css/style.css` — design-token stylesheet.

### Existing patterns confirmed (to mirror, not reinvent)
- **Add-Sphero flow:** ribbon button `#addSpheroBtn` → `openModal('addSpheroModal',
  'spheroNameInput')`; `#confirmAddBtn` reads `#spheroNameInput`, calls
  `deploySphero(name)` which `POST /api/spheros` with `{ sphero_name }` only.
  Toasts on success/failure; `refresh()` re-pulls the fleet.
- **ArUco controls:** ribbon buttons `#arucoStartBtn` / `#arucoStopBtn` (toggle
  `hidden`), a LOCALIZATION `.panel` with `.readout` rows, polled by `refreshAruco()`
  every 4s in `init()`; `startAruco()` / `stopAruco()` POST and re-poll.
- **Status indicator pattern:** a `.chip--state` in the topbar with a `.dot`
  (`data-state="online|offline|error|starting"`) + a text label (ARUCO chip).
- **Styling system:** CSS custom properties (`--amber`, `--green`, `--red`, `--fg-*`,
  `--bg-*`), reusable classes `.action(.--primary/.--accent/.--ghost/.--danger)`,
  `.panel`, `.readout`, `.readout__input`, `.field`, `.field__label`, `.field__hint`,
  `.chip`, `.dot`, `.toast`. No build step, no bundler, no framework, no jQuery.

### Backend contract (verified against multirobot_webapp.py — authoritative)
- `POST /api/spheros` — body `{ sphero_name, tag_id:int(1-16) }` BOTH required.
  201 `{success:true, instance:{name,port,status,added_at,url,tag_id}}`.
  400 messages: "Missing sphero_name in request" / "Missing tag_id in request" /
  "tag_id must be an integer (1-16)" / "tag_id N out of range (1-16)" /
  "tag_id N already assigned to SB-XXXX" / "Sphero X already exists".
- `GET /api/uwb/tags` → `{success:true, all:[1..16], free:[...], assigned:{"3":"SB-3660"}}`
  (assigned keys are STRINGS).
- `GET /api/uwb/anchors` → `{success:true, anchors_cm:[{x,y}*4]|null, configured:bool}`.
- `POST /api/uwb/anchors` — body `{anchors_cm:[{x,y}*4]}` (A0..A3, cm).
  200 `{success:true, message:"Anchor map stored" | "Stored; stop/start positioning to apply"}`.
  400: "expected exactly 4 anchors" / "each anchor needs numeric x and y".
- `POST /api/uwb/start` — opt body `{fake_mode:bool}`. 200 success.
  400: "anchors not configured" / "UWB positioning already running" /
  "UWB positioning process died on startup".
- `POST /api/uwb/stop` — 200 success / 400 "UWB positioning is not running".
- `GET /api/uwb/status` → `{success:true, running, anchors_configured, fake_mode, assigned_count}`.

## UI/UX Design

### Where things live
```
+----------------------------------------------------------------+
| TOPBAR  SCS·01   [UPT][FLEET][ARUCO ●][UWB ●][LINK ●]          |  <- add UWB chip
+----------------------------------------------------------------+
| RIBBON  [DEPLOY UNIT] [REFRESH] [START ARUCO]/[STOP ARUCO]      |
|         [START UWB]/[STOP UWB]            ...hint               |  <- add UWB buttons
+----------------------------------------------------------------+
| // FLEET                                                       |
|   unit tiles ...                                               |
+----------------------------------------------------------------+
| // LOCALIZATION   (ARUCO·SLAM)   existing                      |
+----------------------------------------------------------------+
| // POSITIONING    (UWB)                            NEW PANEL    |
|   [STATE][ANCHORS][MODE][ASSIGNED] readouts                    |
|   ---- ARENA ANCHORS (cm) -------------------------            |
|   A0  x[__] y[__]    A1  x[__] y[__]                           |
|   A2  x[__] y[__]    A3  x[__] y[__]   [SAVE ANCHORS]          |
|   hint: configure anchors before starting positioning         |
+----------------------------------------------------------------+
| FOOTER                                                         |
+----------------------------------------------------------------+

DEPLOY MODAL (existing) gains a [ UWB TAG ] <select> above CALLSIGN-or-below,
populated from /api/uwb/tags; assigned ids shown disabled.
```

### User flows
1. **Deploy with tag:** click DEPLOY UNIT → modal opens, tag `<select>` auto-populated
   from `GET /api/uwb/tags` (assigned ids disabled, labelled with owner) → operator
   types callsign + picks a free tag → DEPLOY → `POST /api/spheros {sphero_name, tag_id}`
   → success toast + fleet refresh + tags refresh; on 400, error toast, modal stays open.
2. **Configure anchors:** operator edits A0–A3 x/y → SAVE ANCHORS →
   `POST /api/uwb/anchors {anchors_cm:[...]}` → success toast (surfacing the
   "stop/start positioning to apply" message verbatim when running) → status re-poll
   so START UWB becomes enabled.
3. **Start/stop UWB:** START UWB (enabled only when `anchors_configured`) →
   `POST /api/uwb/start` → poll reflects RUNNING; STOP UWB → `POST /api/uwb/stop`.
   Status chip + panel readouts update every ~4s.

## Detailed Plan (smallest set of changes)

### Step 1 — Add a UWB status chip to the topbar (index.html)
- Action: insert one `.chip.chip--state` after the ARUCO chip, before the LINK chip.
- Markup (mirrors ARUCO chip exactly):
  ```html
  <div class="chip chip--state" title="UWB positioning">
      <span class="chip__label">UWB</span>
      <span class="dot" id="uwbDot" data-state="offline"></span>
      <span class="chip__value" id="uwbLabel">OFFLINE</span>
  </div>
  ```
- Verify: chip renders OFFLINE on load, flips to ONLINE when UWB running.

### Step 2 — Add UWB start/stop buttons to the ribbon (index.html)
- Action: add two buttons after the existing ArUco stop button, mirroring it.
  ```html
  <button class="action action--accent" id="uwbStartBtn">
      <span class="action__glyph">◎</span>
      <span class="action__label">START UWB</span>
  </button>
  <button class="action action--ghost" id="uwbStopBtn" hidden>
      <span class="action__glyph">■</span>
      <span class="action__label">STOP UWB</span>
  </button>
  ```
- Verify: buttons appear; START disabled (greyed via `[disabled]`) until anchors configured.

### Step 3 — Add the // POSITIONING (UWB) panel (index.html)
- Action: add one new `<section class="panel panel--narrow">` after the LOCALIZATION
  panel. Contains: (a) a `.loc__readout` row of four `.readout` cells
  (STATE / ANCHORS / MODE / ASSIGNED), and (b) an anchor form.
- Readout cells reuse existing `.readout` / `.readout__k` / `.readout__v`:
  `#uwbStateText`, `#uwbAnchorsText`, `#uwbModeText`, `#uwbAssignedText`.
- Anchor form (new minimal block, IDs `anchorA0x … anchorA3y`):
  ```html
  <div class="anchors">
    <div class="anchors__grid">
      <!-- repeated A0..A3 -->
      <div class="anchor">
        <span class="anchor__id">A0</span>
        <label class="anchor__field"><span>X</span>
          <input type="number" id="anchorA0x" step="0.1" inputmode="decimal" class="readout__input"></label>
        <label class="anchor__field"><span>Y</span>
          <input type="number" id="anchorA0y" step="0.1" inputmode="decimal" class="readout__input"></label>
      </div>
      ...A1,A2,A3...
    </div>
    <div class="anchors__foot">
      <span class="loc__hint">Anchor coordinates in cm. Configure before starting positioning.</span>
      <button class="action action--primary" id="saveAnchorsBtn">SAVE ANCHORS</button>
    </div>
  </div>
  ```
- Verify: panel renders, inputs editable, prefilled from `GET /api/uwb/anchors`
  when configured.

### Step 4 — Add the tag-id selector to the DEPLOY modal (index.html)
- Action: add a labelled `<select>` to `#addSpheroModal .modal__body`, placed above
  the CALLSIGN field, reusing `.field__label`; give the select the `.field` class so it
  inherits modal field styling (plus a tiny `select.field` rule in Step 7 for the arrow/padding).
  ```html
  <label for="tagIdSelect" class="field__label">[ UWB TAG ]</label>
  <select id="tagIdSelect" class="field" required></select>
  <p class="field__hint">Each unit binds to one UWB tag (1–16). Assigned tags are disabled.</p>
  ```
- Verify: opening modal triggers a tags fetch; options 1–16 present; assigned ones
  disabled with "· SB-XXXX" suffix; first free id preselected.

### Step 5 — JS: extend ControlStation state + init polling (app.js)
- Action:
  - In `constructor`, add `this.uwb = { running:false, anchorsConfigured:false,
    fakeMode:false, assignedCount:0 };` and `this.uwbTags = { all:[], free:[], assigned:{} };`.
  - In `init()`, after the ArUco poll, add a UWB poll mirroring it:
    `this.refreshUwb(); setInterval(() => this.refreshUwb(), 4000);`
  - In `#refreshBtn` handler, also call `this.refreshUwb()`.
- Verify: UWB poll fires every ~4s (Network tab shows `/api/uwb/status`).

### Step 6 — JS: new methods (app.js)
Add methods mirroring the ArUco/deploy patterns:
- `async refreshUwbTags()` — `GET /api/uwb/tags`; store; if modal open, repopulate
  `#tagIdSelect`.
- `populateTagSelect()` — build `<option>` per id in `all`; if id is a key in
  `assigned`, set `disabled` + text `${id} · ${assigned[String(id)]}`; else text `id`;
  preselect first `free` id; if none free, leave a disabled "no free tags" placeholder
  and disable `#confirmAddBtn`.
- `async refreshUwb()` — `GET /api/uwb/status`; store into `this.uwb`; call
  `renderUwb()`; also call `refreshUwbTags()` so the selector + ASSIGNED stay current.
- `async refreshAnchors()` — `GET /api/uwb/anchors`; if `configured`, fill the 8 inputs;
  called once on load and after a successful save.
- `async saveAnchors()` — read 8 inputs, build `anchors_cm:[{x,y}*4]`; validate all are
  finite numbers client-side (toast + abort if not); `POST /api/uwb/anchors`; on success
  toast `data.message` verbatim (so the "stop/start positioning to apply" note shows),
  then `refreshUwb()`; on 400 toast `data.message`.
- `async startUwb()` — guard: if `!this.uwb.anchorsConfigured` toast "Configure anchors
  first" and return; `POST /api/uwb/start` (body `{}` — fake_mode omitted/false); toast;
  `refreshUwb()`. (Decision: no fake-mode toggle in UI for now — see Open Questions.)
- `async stopUwb()` — `POST /api/uwb/stop`; toast; `refreshUwb()`.
- `renderUwb()` — mirror `renderAruco()`:
  - dot `#uwbDot` / label `#uwbLabel` online↔offline by `running`.
  - `#uwbStateText` RUNNING(green)/OFFLINE(fg-2); `#uwbAnchorsText`
    CONFIGURED(green)/UNSET(amber); `#uwbModeText` FAKE(amber)/LIVE(fg) only meaningful
    when running else "—"; `#uwbAssignedText` = `assignedCount`.
  - toggle `#uwbStartBtn` / `#uwbStopBtn` `hidden` by `running`.
  - `#uwbStartBtn.disabled = !anchorsConfigured` (only when not running).

### Step 7 — JS: wire up the new controls (app.js, in bindActions)
- Action: add listeners:
  - `#uwbStartBtn` → `startUwb()`, `#uwbStopBtn` → `stopUwb()`,
    `#saveAnchorsBtn` → `saveAnchors()`.
  - Change DEPLOY button opening: when opening `addSpheroModal`, also call
    `refreshUwbTags()` then `populateTagSelect()` so the list is fresh.
- Modify `deploySphero`: change signature to `deploySphero(name, tagId)` and include
  `tag_id: tagId` in the POST body. On success also `refreshUwbTags()`.
- Modify `#confirmAddBtn` handler: read both `#spheroNameInput` and `#tagIdSelect`;
  require a non-empty name AND a selected, non-disabled tag value (else toast + keep
  modal open); pass both to `deploySphero`; close modal only after a successful POST
  (move close into the success branch of `deploySphero`, or keep current close + rely on
  error toast — see Open Questions on close timing).
- Verify: deploying with no tag selected is blocked; valid deploy posts both fields;
  400 errors surface as toasts.

### Step 8 — CSS: minimal additions (style.css)
- Action: add a small block (≈25 lines) at the end, reusing tokens:
  - `select.field` — appearance reset, padding, and a CSS caret so the native select
    matches the modal field look.
  - `.anchors__grid` — `display:grid; grid-template-columns:repeat(2,1fr); gap:12px;`
    (stacks to 1 col under the existing 720px breakpoint).
  - `.anchor` — flex row: `.anchor__id` (amber, monospace), two `.anchor__field`
    label/input pairs; reuse `.readout__input` for the number boxes (already styled).
  - `.anchors__foot` — flex space-between for hint + SAVE button.
- No changes to existing selectors; purely additive. Verify visually that the panel
  matches the LOCALIZATION panel aesthetic.

## API Endpoints / Polling
| Method | Path | When called |
|--------|------|-------------|
| GET | /api/uwb/status | every ~4s + after any UWB action |
| GET | /api/uwb/tags | on load, on modal open, after status poll, after deploy |
| GET | /api/uwb/anchors | on load, after successful save |
| POST | /api/uwb/anchors | SAVE ANCHORS click |
| POST | /api/uwb/start | START UWB click |
| POST | /api/uwb/stop | STOP UWB click |
| POST | /api/spheros | DEPLOY click (now includes tag_id) |

## Data Flow
```
Operator → app.js fetch() → Flask REST → SpheroInstanceManager / FleetNode → ROS2
Operator UI ← app.js render ← JSON ← Flask REST ← poll (status/tags/anchors)
```

## Expected Outcomes
- DEPLOY modal requires a free tag; assigned tags visibly unavailable; tag_id sent.
- All four add-Sphero 400 cases surface as readable toasts; modal stays open on error.
- Anchor form loads existing values, saves 4×{x,y} cm, surfaces the apply-while-running note.
- START UWB guarded until anchors configured; start/stop reflected in chip + readouts;
  ASSIGNED count and tag list stay current via polling.

## Potential Risks & Considerations
- **Stale tag list / race:** two operators could pick the same free id between poll and
  submit; backend rejects with 400 "already assigned" — we surface it and re-pull tags.
  Acceptable; no locking added.
- **assigned keys are strings:** `assigned["3"]` — JS lookups must stringify the id.
  Called out explicitly in Step 6.
- **XSS:** `assigned` values are Sphero names from operator input. Build options via
  `document.createElement`/`textContent` (or reuse the existing `safe()` escaper) — do
  NOT inject names via innerHTML. Toaster already escapes.
- **Number parsing:** anchor inputs validated as finite numbers client-side before POST;
  empty fields blocked with a toast (avoids sending NaN).
- **Disabled START:** guard both via `disabled` attribute and a JS early-return so a
  programmatic/edge click can't bypass it.
- **Modal close timing:** if we close on click before the POST resolves, a 400 hides the
  error context. Plan moves modal close to the success branch (see Open Questions).
- **Performance:** three small GET polls now on a 4–5s cadence; negligible.

## Testing Plan
- Manual:
  - [ ] Load page: UWB chip OFFLINE, panel renders, anchors prefilled if configured.
  - [ ] Open DEPLOY: tag list 1–16, assigned ones disabled w/ owner; first free preselected.
  - [ ] Deploy with no free tags → confirm blocked / placeholder.
  - [ ] Deploy valid → 201, tile appears, that tag now disabled in a re-opened modal.
  - [ ] Force each 400 (duplicate name, taken tag) → correct toast, modal stays open.
  - [ ] Save anchors (valid) while stopped → "Anchor map stored"; while running →
        "Stored; stop/start positioning to apply".
  - [ ] Save anchors with a blank field → client-side toast, no POST.
  - [ ] START UWB disabled until anchors configured; after configure, START works;
        chip→ONLINE, STATE→RUNNING, STOP shows; STOP returns to OFFLINE.
  - [ ] Kill the UWB process externally → within ~4s status poll flips to OFFLINE.
  - [ ] Chrome + Firefox; mobile width (<720px) stacks panel/anchor grid.
- Accessibility:
  - [ ] Tab reaches select, anchor inputs, SAVE, START/STOP; focus rings visible.
  - [ ] Labels associated (`for`/`id`); status text not color-only.

## Browser Compatibility
- Chrome 90+, Firefox 90+, Safari 15+, Edge 90+ (matches existing fetch/optional-chaining/
  CSS-vars usage; no new APIs introduced). Mobile Safari / Chrome Mobile via existing
  responsive breakpoint.

## Files Touched (exact)
1. `src/multirobot_webserver/templates/index.html` — UWB chip, 2 ribbon buttons,
   POSITIONING panel + anchor form, tag `<select>` in DEPLOY modal.
2. `src/multirobot_webserver/static/js/app.js` — UWB/tags/anchors state, polling,
   `refreshUwb/refreshUwbTags/populateTagSelect/refreshAnchors/saveAnchors/startUwb/
   stopUwb/renderUwb` methods, deploy-with-tag wiring.
3. `src/multirobot_webserver/static/css/style.css` — additive `select.field`,
   `.anchors__grid`, `.anchor*`, `.anchors__foot` rules only.

No backend file is modified. No new dependency, framework, or build step.

## Approval Status
- [x] Waiting for user approval
- [ ] Approved
- [ ] Executed
