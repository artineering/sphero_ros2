# Add "none" positioning source (start server with no localization)

**Created:** 2026-05-29T17:18:36Z
**Status:** Pending Approval
**Complexity:** Low
**Technologies:** Flask, vanilla JS (ES6), ROS2 (rclpy), HTML/CSS

## Task Description
Add a `'none'` positioning source so the multirobot webserver can start with NO
localization node running (nothing grabbing the camera), and make `'none'` the
default. Operator can switch to/from a real source (aruco/matrix/uwb) at runtime.

## Requirements Analysis
- UI: add a "None" button to the existing positioning-source ribbon; default
  selection renders as "None".
- Backend: accept `'none'` as a valid source; `set_positioning_source('none')`
  stops all three real sources and starts nothing.
- Default: `__init__` default changes `'matrix'` -> `'none'`; env override still
  works and accepts `'none'`.
- Startup: if source is `'none'`, print a hint and start nothing (no camera).
- GET response shape unchanged; `sources` list includes `'none'` first.

## Analysis
Current single-active-publisher logic lives in `set_positioning_source`. Adding
`'none'` fits cleanly: it already stops non-selected sources, so for `'none'` we
just stop all three and return success without starting anything. The UI source
buttons are wired generically via `[data-source]` -> `setSource(btn.dataset.source)`
and styled by `renderSource()`, so a new `data-source="none"` button is picked up
automatically. The GET endpoint feeds the `sources` list; the buttons are static
HTML (not generated from the list), so I add the None button in the template.

## Detailed Plan

### Step 1: Backend constant
- File: `multirobot_webapp.py` (~line 46)
- Add `ALL_SOURCES = ('none',) + POSITIONING_SOURCES` after `POSITIONING_SOURCES`.
- Keep `POSITIONING_SOURCES` as the three real sources (unchanged).

### Step 2: `__init__` default
- File: `multirobot_webapp.py` (~lines 238-242)
- Default env value `'matrix'` -> `'none'`; validate against `ALL_SOURCES`.

### Step 3: `set_positioning_source`
- File: `multirobot_webapp.py` (~lines 652-691)
- Validate against `ALL_SOURCES`.
- The existing "stop non-selected" block already stops all three when
  `source == 'none'` (none of the equality guards match). Add a `none` branch in
  the start section that sets `result = {'success': True, 'message': '...'}`
  without starting anything.

### Step 4: GET endpoint `sources` list
- File: `multirobot_webapp.py` (~line 1012)
- Change `'sources': list(POSITIONING_SOURCES)` -> `'sources': list(ALL_SOURCES)`
  ('none' first).

### Step 5: Startup block
- File: `multirobot_webapp.py` (~lines 1143-1152)
- Add a `none` branch: print "No positioning source — start one from the UI/API"
  and start nothing. Keep uwb / aruco / matrix behavior.

### Step 6: UI button
- File: `templates/index.html` (~line 93)
- Add `<button class="action" id="srcNoneBtn" data-source="none">` with label
  NONE as the first button in the ribbon. No JS change needed (generic wiring).

## API Endpoints / WebSocket Events
| Method | Path | Change |
|--------|------|--------|
| GET | /api/positioning_source | `sources` now `['none','aruco','matrix','uwb']` |
| POST | /api/positioning_source | accepts `{"source":"none"}` |

## Expected Outcomes
- Server starts with `source: 'none'`, no positioning node, camera free.
- POST a real source starts it; POST `'none'` stops everything.

## Potential Risks & Considerations
- Low risk; additive. No topic-contract or node-implementation changes.
- Input validation widened only to include the literal `'none'`.

## Testing Plan
- `colcon build --packages-select multirobot_webserver` clean.
- Start server; `fuser /dev/video0` shows no holder.
- `GET /api/positioning_source` -> `source:'none'`, `'none'` in sources.
- POST aruco/matrix starts a node; POST none stops it.

## Approval Status
- [ ] Waiting for user approval
