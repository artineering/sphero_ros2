/* ============================================================
   SCS·01 — Sphero Control Station
   Operator console front-end logic
   ============================================================ */

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

// Per-type default parameters for the BROADCAST composer. Mirrors the
// per-instance forms' defaults; types absent here default to {} (operator
// fills the JSON manually).
const BROADCAST_DEFAULTS = {
    roll:    { heading: 0, speed: 100, duration: 0 },
    move_to: { x: 100, y: 100, speed: 100 },
    patrol:  { waypoints: [{ x: 50, y: 50 }, { x: 100, y: 50 }, { x: 100, y: 100 }], speed: 100, loop: false },
    circle:  { radius: 50, speed: 100, duration: 10, direction: 'ccw' },
    square:  { side_length: 100 },
    spin:    { duration: 5, speed: 120 },
    set_led: { red: 0, green: 128, blue: 255 },
    stop:    {},
    heading: { heading: 0 },
    speed:   { speed: 100 },
    matrix:  { pattern: 'smile', red: 255, green: 255, blue: 255 },
    led_sequence: {
        sequence: [
            { red: 255, green: 0, blue: 0 },
            { red: 0, green: 255, blue: 0 },
            { red: 0, green: 0, blue: 255 },
        ],
        interval: 1.0,
        loop: false,
    },
    matrix_sequence: {
        sequence: [
            { pattern: 'smile', red: 255, green: 255, blue: 0 },
            { pattern: 'heart', red: 255, green: 0, blue: 0 },
            { pattern: 'arrow', red: 0, green: 255, blue: 255 },
        ],
        interval: 2.0,
        loop: false,
    },
    collision: { action: 'start', mode: 'obstacle', sensitivity: 'HIGH' },
    reflect: { offset_min: -45, offset_max: 45, speed: 80 },
    custom: {
        commands: [
            { type: 'led', red: 0, green: 128, blue: 255, duration: 1.0 },
            { type: 'roll', heading: 90, speed: 100, duration: 2.0 },
            { type: 'stop', duration: 0.5 },
        ],
    },
};

// ============================================================
// Lane / task-class config (DATA-DRIVEN CORE)
// ------------------------------------------------------------
// A Sphero runs one OWNER task per actuator "lane" concurrently; two owners in
// one lane is a bundle the controller silently rejects. The visual builder
// derives ALL of its structure (columns, conflict rules, click-to-add homes)
// from the objects below, so reconciling with the controller-side taxonomy is a
// DATA edit only — no structural rewrite.
//
// SOURCE OF TRUTH for the final taxonomy is the parallel controller design. If
// the controller's lane names differ, rename the `id`s in LANES and the `lane`
// fields in TASK_CLASS to match; nothing else needs to change. (See the
// reconciliation note in plans/visual-lane-bundle-builder-*.md.)

// Lane definitions — ORDER here is the column order on the board.
const LANES = [
    { id: 'DRIVE',     label: 'DRIVE' },
    { id: 'AIM',       label: 'AIM' },        // heading modifier
    { id: 'THROTTLE',  label: 'THROTTLE' },   // speed modifier
    { id: 'LED_MAIN',  label: 'LED·MAIN' },
    { id: 'LED_FRONT', label: 'LED·FRONT' },
    { id: 'LED_BACK',  label: 'LED·BACK' },
    { id: 'MATRIX',    label: 'MATRIX' },
    { id: 'CONFIG',    label: 'CONFIG' },
];

// set_led's home lane depends on its `led_type` param.
const LED_LANE_BY_TYPE = { main: 'LED_MAIN', front: 'LED_FRONT', back: 'LED_BACK' };

// Per-type class.
//   role: 'owner' (one per lane → conflicts when doubled) | 'modifier' (never
//         conflicts, always addable, applies live).
//   lane: the card's HOME lane (click-to-add target; the only lane an owner
//         accepts). Modifiers whose lane depends on a param define `laneFor`.
const TASK_CLASS = {
    // ---- DRIVE owners ----
    roll:         { role: 'owner', lane: 'DRIVE' },
    move_to:      { role: 'owner', lane: 'DRIVE' },
    patrol:       { role: 'owner', lane: 'DRIVE' },
    square:       { role: 'owner', lane: 'DRIVE' },
    circle:       { role: 'owner', lane: 'DRIVE' },
    spin:         { role: 'owner', lane: 'DRIVE' },
    reflect:      { role: 'owner', lane: 'DRIVE' },
    // ---- LED / MATRIX owners ----
    led_sequence:    { role: 'owner', lane: 'LED_MAIN' },
    matrix_sequence: { role: 'owner', lane: 'MATRIX' },
    // ---- exclusive owners (occupy ALL lanes — any other owner conflicts) ----
    custom:       { role: 'owner', lane: '*' },
    jumping_bean: { role: 'owner', lane: '*' },
    // ---- modifiers (never conflict) ----
    heading: { role: 'modifier', lane: 'AIM' },
    speed:   { role: 'modifier', lane: 'THROTTLE' },
    set_led: { role: 'modifier', lane: 'LED_MAIN', laneFor: (t) => LED_LANE_BY_TYPE[t?.parameters?.led_type] || 'LED_MAIN' },
    matrix:  { role: 'modifier', lane: 'MATRIX' },
    collision:{ role: 'modifier', lane: 'CONFIG' },
};

// Unknown / reclassed types degrade to a non-conflicting CONFIG modifier so the
// board never crashes if the controller adds a type the front-end hasn't met.
const DEFAULT_CLASS = { role: 'modifier', lane: 'CONFIG' };
const classOf = (type) => TASK_CLASS[type] || DEFAULT_CLASS;
// Resolve a task's effective lane id ('*' = exclusive, occupies every lane).
const laneFor = (task) => {
    const cls = classOf(task && task.task_type);
    return cls.laneFor ? cls.laneFor(task) : cls.lane;
};

// Compact inline param editors per card. Format: [key, kind] where kind is
// '#' (number) | 'txt' (text) | 'sel:a,b,c' (select). Params not listed here
// are still editable via the card's "{…}" JSON popover or the advanced JSON view.
const CARD_FIELDS = {
    roll:    [['heading', '#'], ['speed', '#'], ['duration', '#']],
    move_to: [['x', '#'], ['y', '#'], ['speed', '#']],
    circle:  [['radius', '#'], ['speed', '#'], ['duration', '#']],
    spin:    [['duration', '#'], ['speed', '#']],
    heading: [['heading', '#']],
    speed:   [['speed', '#']],
    set_led: [['led_type', 'sel:main,front,back'], ['red', '#'], ['green', '#'], ['blue', '#']],
    matrix:  [['pattern', 'txt'], ['red', '#'], ['green', '#'], ['blue', '#']],
};

// Defaults for the two exclusive owners that have no BROADCAST_DEFAULTS entry.
BROADCAST_DEFAULTS.jumping_bean = BROADCAST_DEFAULTS.jumping_bean || {};

// Strip the non-serialized _uid before send / serialize (keep the wire clean).
const stripUid = ({ _uid, ...rest }) => rest;
let _uidSeq = 0;
const nextUid = () => `t${++_uidSeq}`;
const cloneParams = (p) => (typeof structuredClone === 'function'
    ? structuredClone(p)
    : JSON.parse(JSON.stringify(p ?? {})));

class ControlStation {
    constructor() {
        this.spheros = [];
        this.pendingDetach = null;
        this.pendingBatchDetach = null;
        this.selected = new Set();
        this.bootedAt = Date.now();
        this.aruco = { running: false, enabled: false };
        this.uwb = { running: false, anchorsConfigured: false, fakeMode: false, assignedCount: 0 };
        this.uwbTags = { all: [], free: [], assigned: {} };
        this.source = { active: null, running: {} };
        this.linkOk = true;
        this.lastSyncAt = null;
        // Broadcast builder: single source of truth (wire-format array + _uid).
        this.bundle = [];
        this._syncing = false;     // guards JSON<->builder sync feedback loops
        this._paletteBuilt = false;
        this.init();
    }

    /* -------------------------------------------------------- init */
    init() {
        this.bindActions();
        this.startMissionClock();
        this.refresh();
        // Fleet polling — every 5s
        setInterval(() => this.refresh(), 5000);
        // ArUco status polling — every 4s
        this.refreshAruco();
        setInterval(() => this.refreshAruco(), 4000);
        // UWB status polling — every 4s
        this.refreshUwb();
        setInterval(() => this.refreshUwb(), 4000);
        // Positioning-source status polling — every 4s
        this.refreshSource();
        setInterval(() => this.refreshSource(), 4000);
        // Prefill anchor inputs once on load
        this.refreshAnchors();
    }

    bindActions() {
        $('#addSpheroBtn').addEventListener('click', () => {
            this.openModal('addSpheroModal', 'spheroNamesInput');
            this.refreshUwbTags();
            this.updateBatchSummary();
        });
        $('#refreshBtn').addEventListener('click', () => { this.refresh(); this.refreshAruco(); this.refreshUwb(); });
        $('#arucoStartBtn').addEventListener('click', () => this.startAruco());
        $('#arucoStopBtn').addEventListener('click', () => this.stopAruco());
        $('#uwbStartBtn').addEventListener('click', () => this.startUwb());
        $('#uwbStopBtn').addEventListener('click', () => this.stopUwb());
        $('#saveAnchorsBtn').addEventListener('click', () => this.saveAnchors());
        $$('[data-source]').forEach((btn) => {
            btn.addEventListener('click', () => this.setSource(btn.dataset.source));
        });

        // Deploy modal
        const confirmAdd = $('#confirmAddBtn');
        const cancelAdd = $('#cancelBtn');
        const namesInput = $('#spheroNamesInput');
        confirmAdd.addEventListener('click', () => {
            const names = this.parseCallsigns(namesInput.value);
            if (!names.length) { this.toast('Enter at least one callsign.', 'error', 'DEPLOY'); return; }
            this.deploySpheros(names);
        });
        cancelAdd.addEventListener('click', () => this.closeModal('addSpheroModal'));
        namesInput.addEventListener('input', () => this.updateBatchSummary());

        // Detach modal
        $('#confirmRemoveBtn').addEventListener('click', () => {
            if (this.pendingBatchDetach) {
                const names = this.pendingBatchDetach;
                this.pendingBatchDetach = null;
                this.detachSpheros(names);
                this.closeModal('confirmRemoveModal');
            } else if (this.pendingDetach) {
                this.detachSphero(this.pendingDetach);
                this.closeModal('confirmRemoveModal');
            }
        });
        $('#cancelRemoveBtn').addEventListener('click', () => this.closeModal('confirmRemoveModal'));

        // Batch detach controls (fleet selection bar)
        $('#detachSelectedBtn').addEventListener('click', () => this.requestBatchDetach([...this.selected]));
        $('#detachAllBtn').addEventListener('click', () => this.requestBatchDetach(this.spheros.map((s) => s.name)));

        // Broadcast modal — visual lane builder
        $('#broadcastBtn').addEventListener('click', () => this.openBroadcast());
        $('#broadcastClearBtn').addEventListener('click', (e) => {
            // Button lives inside the <summary>; don't toggle the details panel.
            e.preventDefault();
            e.stopPropagation();
            this.clearBroadcastBundle();
        });
        // JSON is canonical-ward only on blur (not input) to avoid sync loops.
        $('#broadcastParams').addEventListener('blur', () => this.syncBundleFromJson());
        $('#confirmBroadcastBtn').addEventListener('click', () => this.sendBroadcast());
        $('#cancelBroadcastBtn').addEventListener('click', () => this.closeModal('broadcastModal'));
        this.bindBuilderDnd();

        // Generic close-on-X / outside-click
        $$('.modal .close').forEach((btn) => {
            btn.addEventListener('click', (e) => {
                e.target.closest('.modal').classList.remove('show');
            });
        });
        $$('.modal').forEach((modal) => {
            modal.addEventListener('click', (e) => {
                if (e.target === modal) modal.classList.remove('show');
            });
        });

        // Esc closes any open modal
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                $$('.modal.show').forEach((m) => m.classList.remove('show'));
            }
        });
    }

    /* -------------------------------------------------------- mission clock */
    startMissionClock() {
        const tick = () => {
            const elapsed = Math.floor((Date.now() - this.bootedAt) / 1000);
            const h = String(Math.floor(elapsed / 3600)).padStart(2, '0');
            const m = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
            const s = String(elapsed % 60).padStart(2, '0');
            $('#missionClock').textContent = `${h}:${m}:${s}`;
        };
        tick();
        setInterval(tick, 1000);
    }

    /* -------------------------------------------------------- fleet API */
    async refresh() {
        try {
            const r = await fetch('/api/spheros');
            const data = await r.json();
            this.linkUp();
            if (data.success) {
                const next = data.spheros || [];
                this.flagUnexpectedDrops(this.spheros, next);
                this.spheros = next;
                this.lastSyncAt = Date.now();
                this.renderFleet();
                this.updateTelemetry();
            }
        } catch (err) {
            this.linkDown();
            console.error('refresh error', err);
        }
    }

    // A unit that drops from 'running' to a non-running state while still
    // present in the listing died unexpectedly (BLE link loss / crash) — a
    // clean DETACH removes the unit from the listing entirely, so it never
    // trips this. There is no per-unit device_error channel into the
    // coordinator (instances emit it only to their own console clients), so
    // this liveness transition is the signal the operator gets centrally.
    flagUnexpectedDrops(prev, next) {
        if (!prev || !prev.length) return;
        const wasRunning = new Map(prev.map((s) => [s.name, s.status === 'running']));
        next.forEach((s) => {
            if (wasRunning.get(s.name) && s.status !== 'running' && s.status !== 'starting') {
                this.toast(`Unit ${s.name} link lost (${(s.status || 'down').toUpperCase()})`, 'error', 'LINK');
            }
        });
    }

    // Split a textarea blob into clean callsigns: one per line, trimmed,
    // blanks dropped, deduped preserving first-seen order (server is the
    // authoritative tag/dup arbiter — this is just client-side tidy-up).
    parseCallsigns(text) {
        const seen = new Set();
        const out = [];
        (text || '').split('\n').forEach((line) => {
            const name = line.trim();
            if (name && !seen.has(name)) { seen.add(name); out.push(name); }
        });
        return out;
    }

    updateBatchSummary() {
        const el = $('#batchSummary');
        if (!el) return;
        const n = this.parseCallsigns($('#spheroNamesInput').value).length;
        const free = this.uwbTags.free.length;
        el.textContent = `${n} callsign${n === 1 ? '' : 's'} · ${free} tag${free === 1 ? '' : 's'} free`;
    }

    async deploySpheros(names) {
        // Friendly pre-warn for names already deployed (server stays authoritative).
        const existing = new Set(this.spheros.map((s) => s.name));
        const dupes = names.filter((n) => existing.has(n));
        if (dupes.length) {
            this.toast(`Already deployed: ${this.truncateNames(dupes)}`, 'info', 'DEPLOY');
        }
        try {
            const r = await fetch('/api/spheros/batch', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ names }),
            });
            const data = await r.json();
            if (!data.success) {
                this.toast(`Deploy failed: ${data.message || 'bad request'}`, 'error', 'DEPLOY');
                return;
            }
            this.summarizeBatch(data, 'DEPLOY', 'deployed');
            this.refresh();
            this.refreshUwbTags();
            // Full success → close; partial → keep open with the failed lines.
            if (data.failed === 0) {
                this.closeModal('addSpheroModal');
            } else {
                const failed = (data.results || []).filter((x) => !x.success).map((x) => x.name);
                $('#spheroNamesInput').value = failed.join('\n');
                this.updateBatchSummary();
            }
        } catch (err) {
            this.toast('Deploy uplink lost.', 'error', 'DEPLOY');
        }
    }

    // One summary toast for a batch result. `verb` is the success past-tense
    // word ('deployed'/'removed'); `data.{deployed|removed, failed, results}`.
    summarizeBatch(data, tag, verb) {
        const okCount = verb === 'deployed' ? data.deployed : data.removed;
        if (!data.failed) {
            this.toast(`${okCount} ${verb}`, 'success', tag);
            return;
        }
        const reasons = (data.results || [])
            .filter((x) => !x.success)
            .map((x) => `${x.name} (${x.reason || 'failed'})`);
        this.toast(`${okCount} ${verb}, ${data.failed} failed: ${this.truncateNames(reasons)}`, 'error', tag);
    }

    truncateNames(list, max = 3) {
        if (list.length <= max) return list.join('; ');
        return `${list.slice(0, max).join('; ')} +${list.length - max} more`;
    }

    async detachSpheros(names) {
        try {
            const r = await fetch('/api/spheros/batch_delete', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ names }),
            });
            const data = await r.json();
            if (!data.success) {
                this.toast(`Detach failed: ${data.message || 'bad request'}`, 'error', 'DETACH');
                return;
            }
            (data.results || []).forEach((x) => { if (x.success) this.selected.delete(x.name); });
            this.summarizeBatch(data, 'DETACH', 'removed');
            this.refresh();
            this.refreshUwb();
        } catch (err) {
            this.toast('Detach uplink lost.', 'error', 'DETACH');
        }
    }

    async detachSphero(name) {
        try {
            const r = await fetch(`/api/spheros/${encodeURIComponent(name)}`, { method: 'DELETE' });
            const data = await r.json();
            if (data.success) {
                this.toast(`Unit ${name} released`, 'success', 'DETACH');
                this.refresh();
            } else {
                this.toast(`Detach failed: ${data.message}`, 'error', 'DETACH');
            }
        } catch (err) {
            this.toast('Detach uplink lost.', 'error', 'DETACH');
        }
    }

    /* -------------------------------------------------------- ArUco API */
    async refreshAruco() {
        try {
            const r = await fetch('/api/aruco_slam/status');
            const data = await r.json();
            this.aruco = { running: !!data.running, enabled: !!data.enabled };
            this.renderAruco();
        } catch (err) {
            // Silent — link state is already covered by /api/spheros polling.
        }
    }

    async startAruco() {
        const camId = parseInt($('#cameraIdInput').value, 10) || 0;
        try {
            const r = await fetch('/api/aruco_slam/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ camera_id: camId }),
            });
            const data = await r.json();
            if (data.success) {
                this.toast(`ArUco SLAM online (cam ${camId})`, 'success', 'ARUCO');
                this.refreshAruco();
            } else {
                this.toast(`ArUco start failed: ${data.message}`, 'error', 'ARUCO');
            }
        } catch (err) {
            this.toast('ArUco uplink lost.', 'error', 'ARUCO');
        }
    }

    async stopAruco() {
        try {
            const r = await fetch('/api/aruco_slam/stop', { method: 'POST' });
            const data = await r.json();
            if (data.success) {
                this.toast('ArUco SLAM offline', 'success', 'ARUCO');
                this.refreshAruco();
            } else {
                this.toast(`ArUco stop failed: ${data.message}`, 'error', 'ARUCO');
            }
        } catch (err) {
            this.toast('ArUco uplink lost.', 'error', 'ARUCO');
        }
    }

    /* -------------------------------------------------------- UWB API */
    async refreshUwb() {
        try {
            const r = await fetch('/api/uwb/status');
            const data = await r.json();
            this.uwb = {
                running: !!data.running,
                anchorsConfigured: !!data.anchors_configured,
                fakeMode: !!data.fake_mode,
                assignedCount: data.assigned_count || 0,
            };
            this.renderUwb();
        } catch (err) {
            // Silent — link state is covered by /api/spheros polling.
        }
        this.refreshUwbTags();
    }

    async refreshUwbTags() {
        try {
            const r = await fetch('/api/uwb/tags');
            const data = await r.json();
            if (data.success) {
                this.uwbTags = {
                    all: data.all || [],
                    free: data.free || [],
                    assigned: data.assigned || {},
                };
                if ($('#addSpheroModal').classList.contains('show')) {
                    this.updateBatchSummary();
                }
            }
        } catch (err) {
            // Silent.
        }
    }

    async refreshAnchors() {
        try {
            const r = await fetch('/api/uwb/anchors');
            const data = await r.json();
            if (data.success && data.configured && Array.isArray(data.anchors_cm)) {
                data.anchors_cm.forEach((a, i) => {
                    const xi = $(`#anchorA${i}x`);
                    const yi = $(`#anchorA${i}y`);
                    if (xi && a && a.x != null) xi.value = a.x;
                    if (yi && a && a.y != null) yi.value = a.y;
                });
            }
        } catch (err) {
            // Silent.
        }
    }

    async saveAnchors() {
        const anchors = [];
        for (let i = 0; i < 4; i++) {
            const x = parseFloat($(`#anchorA${i}x`).value);
            const y = parseFloat($(`#anchorA${i}y`).value);
            if (!Number.isFinite(x) || !Number.isFinite(y)) {
                this.toast(`Anchor A${i} needs numeric X and Y.`, 'error', 'UWB');
                return;
            }
            anchors.push({ x, y });
        }
        try {
            const r = await fetch('/api/uwb/anchors', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ anchors_cm: anchors }),
            });
            const data = await r.json();
            if (data.success) {
                this.toast(data.message, 'success', 'UWB');
                this.refreshUwb();
                this.refreshAnchors();
            } else {
                this.toast(`Anchor save failed: ${data.message}`, 'error', 'UWB');
            }
        } catch (err) {
            this.toast('UWB uplink lost.', 'error', 'UWB');
        }
    }

    async startUwb() {
        if (!this.uwb.anchorsConfigured) {
            this.toast('Configure anchors first.', 'error', 'UWB');
            return;
        }
        const fakeMode = $('#uwbFakeMode').checked;
        try {
            const r = await fetch('/api/uwb/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ fake_mode: fakeMode }),
            });
            const data = await r.json();
            if (data.success) {
                this.toast(`UWB positioning online${fakeMode ? ' (fake)' : ''}`, 'success', 'UWB');
                this.refreshUwb();
            } else {
                this.toast(`UWB start failed: ${data.message}`, 'error', 'UWB');
            }
        } catch (err) {
            this.toast('UWB uplink lost.', 'error', 'UWB');
        }
    }

    async stopUwb() {
        try {
            const r = await fetch('/api/uwb/stop', { method: 'POST' });
            const data = await r.json();
            if (data.success) {
                this.toast('UWB positioning offline', 'success', 'UWB');
                this.refreshUwb();
            } else {
                this.toast(`UWB stop failed: ${data.message}`, 'error', 'UWB');
            }
        } catch (err) {
            this.toast('UWB uplink lost.', 'error', 'UWB');
        }
    }

    renderUwb() {
        const dot = $('#uwbDot');
        const label = $('#uwbLabel');
        const stateText = $('#uwbStateText');
        const anchorsText = $('#uwbAnchorsText');
        const modeText = $('#uwbModeText');
        const assignedText = $('#uwbAssignedText');
        const startBtn = $('#uwbStartBtn');
        const stopBtn = $('#uwbStopBtn');

        if (this.uwb.running) {
            dot.dataset.state = 'online';
            label.textContent = 'ONLINE';
            stateText.textContent = 'RUNNING';
            stateText.style.color = 'var(--green)';
            startBtn.hidden = true;
            stopBtn.hidden = false;
        } else {
            dot.dataset.state = 'offline';
            label.textContent = 'OFFLINE';
            stateText.textContent = 'OFFLINE';
            stateText.style.color = 'var(--fg-2)';
            startBtn.hidden = false;
            stopBtn.hidden = true;
            startBtn.disabled = !this.uwb.anchorsConfigured;
        }

        if (this.uwb.anchorsConfigured) {
            anchorsText.textContent = 'CONFIGURED';
            anchorsText.style.color = 'var(--green)';
        } else {
            anchorsText.textContent = 'UNSET';
            anchorsText.style.color = 'var(--amber)';
        }

        if (this.uwb.running) {
            modeText.textContent = this.uwb.fakeMode ? 'FAKE' : 'LIVE';
            modeText.style.color = this.uwb.fakeMode ? 'var(--amber)' : 'var(--fg)';
        } else {
            modeText.textContent = '—';
            modeText.style.color = 'var(--fg-2)';
        }

        assignedText.textContent = this.uwb.assignedCount;
    }

    /* -------------------------------------------------------- positioning source */
    async refreshSource() {
        try {
            const r = await fetch('/api/positioning_source');
            const data = await r.json();
            if (data.success) {
                this.source = { active: data.source, running: data.running || {} };
                this.renderSource();
            }
        } catch (err) {
            // Silent — link state is covered by /api/spheros polling.
        }
    }

    async setSource(source) {
        try {
            const r = await fetch('/api/positioning_source', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ source }),
            });
            const data = await r.json();
            if (data.success) {
                this.toast(`Positioning source → ${source.toUpperCase()}`, 'success', 'SOURCE');
                this.refreshSource();
                this.refreshAruco();
                this.refreshUwb();
            } else {
                this.toast(`Source switch failed: ${data.message}`, 'error', 'SOURCE');
            }
        } catch (err) {
            this.toast('Source uplink lost.', 'error', 'SOURCE');
        }
    }

    renderSource() {
        const label = $('#sourceLabel');
        if (label) {
            label.textContent = this.source.active ? this.source.active.toUpperCase() : '—';
        }
        $$('[data-source]').forEach((btn) => {
            const isActive = btn.dataset.source === this.source.active;
            btn.classList.toggle('action--primary', isActive);
        });
    }

    /* -------------------------------------------------------- render: fleet */
    renderFleet() {
        const grid = $('#spheroGrid');
        const empty = $('#noSpherosMessage');

        // BROADCAST is only valid with at least one running unit.
        const broadcastBtn = $('#broadcastBtn');
        if (broadcastBtn) {
            const disabled = this.runningCount() === 0;
            broadcastBtn.disabled = disabled;
            broadcastBtn.setAttribute('aria-disabled', String(disabled));
        }

        // Prune selection of any units no longer present (detached out-of-band).
        const currentNames = new Set(this.spheros.map((s) => s.name));
        this.selected = new Set([...this.selected].filter((n) => currentNames.has(n)));

        if (this.spheros.length === 0) {
            grid.innerHTML = '';
            empty.classList.add('show');
            this._lastFleetSig = '';
            this.updateSelectionBar();
            return;
        }
        empty.classList.remove('show');

        // Signature of everything that affects tile structure / static data.
        // [UPT] is computed client-side from added_at and is updated in place
        // below, so it must not be part of this signature.
        const sig = this.spheros
            .map((s) => `${s.name}|${s.port}|${s.status}|${s.added_at}|${s.url}`)
            .join('::');

        if (sig === this._lastFleetSig) {
            // Fleet shape unchanged — checkboxes still exist; only refresh
            // dynamic [UPT] cells. (Selection already pruned above.)
            this.spheros.forEach((s) => {
                const tile = document.getElementById(`unit-${s.name}`);
                if (!tile) return;
                const uptCell = tile.querySelector('.unit__upt');
                if (uptCell) uptCell.textContent = this.formatAge(s.added_at);
            });
            this.updateSelectionBar();
            return;
        }
        this._lastFleetSig = sig;

        grid.innerHTML = this.spheros.map((s, i) => this.unitTile(s, i)).join('');

        this.spheros.forEach((s) => {
            const open = document.getElementById(`open-${s.name}`);
            const detach = document.getElementById(`detach-${s.name}`);
            const sel = document.getElementById(`sel-${s.name}`);
            if (open) open.addEventListener('click', () => window.open(this.consoleUrl(s.url), '_blank'));
            if (detach) detach.addEventListener('click', () => this.requestDetach(s.name));
            if (sel) {
                sel.checked = this.selected.has(s.name);
                sel.addEventListener('change', () => {
                    if (sel.checked) this.selected.add(s.name);
                    else this.selected.delete(s.name);
                    this.updateSelectionBar();
                });
            }
        });
        this.updateSelectionBar();
    }

    updateSelectionBar() {
        const bar = $('#fleetSelectBar');
        const count = $('#selectedCount');
        const selBtn = $('#detachSelectedBtn');
        if (!bar) return;
        bar.hidden = this.spheros.length === 0;
        if (count) count.textContent = `[ ${this.selected.size} SELECTED ]`;
        if (selBtn) selBtn.disabled = this.selected.size === 0;
    }

    // The instance url points at the host the WebSocket server runs on (the
    // worker, for remote instances). Browsers only reach the coordinator, which
    // relays remote instances on the same port, so rewrite the host to this
    // page's host while keeping port/path. Harmless for local instances.
    consoleUrl(url) {
        try {
            const u = new URL(url, window.location.href);
            u.protocol = window.location.protocol;
            u.hostname = window.location.hostname;
            return u.toString();
        } catch (err) {
            return url;
        }
    }

    unitTile(s, i) {
        const state = (s.status || 'stopped').toLowerCase();
        const stateLabel = state.toUpperCase();
        const dotState = state === 'running' ? 'online' : (state === 'starting' ? 'starting' : 'error');
        const uptime = this.formatAge(s.added_at);
        const safe = (str) => String(str).replace(/[<>&"]/g, (c) => ({'<':'&lt;','>':'&gt;','&':'&amp;','"':'&quot;'}[c]));
        const delay = (i * 60).toString();

        return `
        <article id="unit-${safe(s.name)}" class="unit" style="animation-delay:${delay}ms">
            <div class="unit__head">
                <input type="checkbox" id="sel-${safe(s.name)}" class="unit__select" aria-label="Select ${safe(s.name)}">
                <span class="unit__name">${safe(s.name)}</span>
                <span class="unit__state" data-state="${state === 'running' ? 'online' : state}">
                    <span class="dot" data-state="${dotState}"></span>
                    ${stateLabel}
                </span>
            </div>
            <div class="unit__rows">
                <span class="unit__k">[CH]</span>      <span class="unit__v">${s.port}</span>
                <span class="unit__k">[UPT]</span>     <span class="unit__v unit__upt">${uptime}</span>
                <span class="unit__k">[URL]</span>     <span class="unit__v">${safe(s.url)}</span>
                <span class="unit__k">[STATE]</span>   <span class="unit__v">${stateLabel}</span>
            </div>
            <div class="unit__actions">
                <button id="open-${safe(s.name)}" class="action action--accent">
                    <span class="action__glyph">▸</span>
                    <span class="action__label">CONSOLE</span>
                </button>
                <button id="detach-${safe(s.name)}" class="action action--danger">
                    <span class="action__glyph">×</span>
                    <span class="action__label">DETACH</span>
                </button>
            </div>
        </article>`;
    }

    /* -------------------------------------------------------- render: header / aruco */
    updateTelemetry() {
        $('#spheroCount').textContent = this.spheros.length;
        $('#capacityFill').textContent = this.spheros.length;
        $('#lastSync').textContent = this.lastSyncAt ? this.formatClock(this.lastSyncAt) : '—';
    }

    renderAruco() {
        const dot = $('#arucoDot');
        const label = $('#arucoLabel');
        const stateText = $('#arucoStateText');
        const enabledText = $('#arucoEnabledText');
        const startBtn = $('#arucoStartBtn');
        const stopBtn = $('#arucoStopBtn');

        if (this.aruco.running) {
            dot.dataset.state = 'online';
            label.textContent = 'ONLINE';
            stateText.textContent = 'RUNNING';
            stateText.style.color = 'var(--green)';
            startBtn.hidden = true;
            stopBtn.hidden = false;
        } else {
            dot.dataset.state = 'offline';
            label.textContent = 'OFFLINE';
            stateText.textContent = 'OFFLINE';
            stateText.style.color = 'var(--fg-2)';
            startBtn.hidden = false;
            stopBtn.hidden = true;
        }
        enabledText.textContent = this.aruco.enabled ? 'TRUE' : 'FALSE';
        enabledText.style.color = this.aruco.enabled ? 'var(--amber)' : 'var(--fg-2)';
    }

    /* -------------------------------------------------------- link health */
    linkUp() {
        if (!this.linkOk) {
            this.toast('Backend reconnected', 'success', 'LINK');
        }
        this.linkOk = true;
        $('#linkDot').dataset.state = 'online';
        $('#linkLabel').textContent = 'NOMINAL';
    }
    linkDown() {
        if (this.linkOk) {
            this.toast('Backend uplink lost', 'error', 'LINK');
        }
        this.linkOk = false;
        $('#linkDot').dataset.state = 'error';
        $('#linkLabel').textContent = 'NO LINK';
    }

    /* -------------------------------------------------------- modal helpers */
    /* -------------------------------------------------------- broadcast */
    runningCount() {
        return this.spheros.filter((s) => s.status === 'running').length;
    }

    openBroadcast() {
        const count = this.runningCount();
        if (count === 0) {
            this.toast('No units deployed.', 'info', 'BROADCAST');
            return;
        }
        // Seed the bundle from whatever JSON is currently in the textarea
        // (`[]` on first open), then render the full builder.
        this.syncBundleFromJson();
        this.renderBuilder();
        $('#broadcastSummary').textContent =
            `${count} unit${count === 1 ? '' : 's'} · fires at now + lead`;
        const cd = $('#broadcastCountdown');
        cd.hidden = true;
        cd.textContent = '';
        this.openModal('broadcastModal');
    }

    /* ---- builder render ---- */

    renderBuilder() {
        this.renderPalette();
        this.renderBundle();
        this.syncJsonFromBundle();
        this.updateConflicts();
    }

    // Palette of draggable task cards, built once from TASK_CLASS.
    renderPalette() {
        if (this._paletteBuilt) return;
        const pal = $('#broadcastPalette');
        if (!pal) return;
        pal.textContent = '';
        Object.keys(TASK_CLASS).forEach((type) => {
            const cls = classOf(type);
            const chip = document.createElement('button');
            chip.type = 'button';
            chip.className = 'palette__card';
            chip.draggable = true;
            chip.dataset.type = type;
            chip.dataset.role = cls.role;
            chip.setAttribute('role', 'listitem');
            chip.title = `${type} (${cls.role}) — drag to a lane or click to add`;
            const glyph = cls.role === 'owner' ? '◆' : '○';
            chip.textContent = `${type} ${glyph}`;   // textContent: no innerHTML
            pal.appendChild(chip);
        });
        this._paletteBuilt = true;
    }

    // Build the lane columns (once) then (re)distribute cards into them.
    renderBundle() {
        const board = $('#broadcastBoard');
        if (!board) return;
        if (!board.children.length) {
            LANES.forEach((lane) => {
                const col = document.createElement('div');
                col.className = 'lane';
                col.dataset.lane = lane.id;
                col.setAttribute('role', 'list');
                col.setAttribute('aria-label', `${lane.label} lane`);
                const head = document.createElement('div');
                head.className = 'lane__head';
                const name = document.createElement('span');
                name.className = 'lane__name';
                name.textContent = lane.label;
                const flag = document.createElement('span');
                flag.className = 'lane__flag';
                flag.hidden = true;
                head.append(name, flag);
                const items = document.createElement('div');
                items.className = 'lane__items';
                const drop = document.createElement('div');
                drop.className = 'lane__drop';
                drop.textContent = 'drop task';
                col.append(head, items, drop);
                board.appendChild(col);
            });
        }
        // Clear all lane item containers.
        $$('#broadcastBoard .lane__items').forEach((c) => { c.textContent = ''; });
        // Place each task. '*' (exclusive owners) render into every lane's items
        // so the operator sees the lane is taken; conflict counting handles them.
        for (const task of this.bundle) {
            const lane = laneFor(task);
            const targets = lane === '*'
                ? LANES.map((l) => l.id)
                : [LANES.some((l) => l.id === lane) ? lane : 'CONFIG'];
            targets.forEach((laneId, idx) => {
                const items = $(`#broadcastBoard .lane[data-lane="${laneId}"] .lane__items`);
                if (items) items.appendChild(this.buildCard(task, idx > 0));
            });
        }
    }

    // Build one placed card. `ghost` = a duplicate rendering of an exclusive
    // owner in a secondary lane (read-only, no editors).
    buildCard(task, ghost = false) {
        const cls = classOf(task.task_type);
        const card = document.createElement('div');
        card.className = `card card--${cls.role}`;
        card.dataset.uid = task._uid;
        card.draggable = !ghost;
        card.setAttribute('role', 'listitem');
        if (ghost) card.classList.add('card--ghost');

        const head = document.createElement('div');
        head.className = 'card__head';
        const typeEl = document.createElement('span');
        typeEl.className = 'card__type';
        typeEl.textContent = `${cls.role === 'owner' ? '◆' : '○'} ${task.task_type}`;
        head.appendChild(typeEl);
        if (!ghost) {
            const right = document.createElement('span');
            const jbtn = document.createElement('button');
            jbtn.type = 'button';
            jbtn.className = 'card__json';
            jbtn.textContent = '{…}';
            jbtn.title = 'Edit full parameters as JSON';
            jbtn.dataset.uid = task._uid;
            jbtn.dataset.act = 'json';
            const rm = document.createElement('button');
            rm.type = 'button';
            rm.className = 'card__rm';
            rm.textContent = '✕';
            rm.title = 'Remove task';
            rm.setAttribute('aria-label', `Remove ${task.task_type}`);
            rm.dataset.uid = task._uid;
            rm.dataset.act = 'rm';
            right.append(jbtn, rm);
            head.appendChild(right);
        }
        card.appendChild(head);

        if (ghost) {
            const note = document.createElement('div');
            note.className = 'card__ghostnote';
            note.textContent = '(occupies lane)';
            card.appendChild(note);
            return card;
        }

        // Inline param editors.
        const fields = CARD_FIELDS[task.task_type] || [];
        const params = task.parameters || (task.parameters = {});
        fields.forEach(([key, kind]) => {
            card.appendChild(this.buildField(task._uid, 'param', key, kind, params[key]));
        });
        // Per-card start_offset (always present).
        card.appendChild(this.buildField(task._uid, 'offset', 'start_offset', '#', task.start_offset));
        return card;
    }

    buildField(uid, scope, key, kind, value) {
        const row = document.createElement('label');
        row.className = 'card__field';
        const lbl = document.createElement('span');
        lbl.textContent = key === 'start_offset' ? '@s' : key;
        let input;
        if (kind.startsWith('sel:')) {
            input = document.createElement('select');
            kind.slice(4).split(',').forEach((opt) => {
                const o = document.createElement('option');
                o.value = opt;
                o.textContent = opt;
                if (String(value) === opt) o.selected = true;
                input.appendChild(o);
            });
        } else {
            input = document.createElement('input');
            input.type = kind === '#' ? 'number' : 'text';
            input.value = value == null ? '' : value;
            if (kind === '#') input.step = 'any';
        }
        input.dataset.uid = uid;
        input.dataset.scope = scope;
        input.dataset.key = key;
        input.dataset.kind = kind;
        row.append(lbl, input);
        return row;
    }

    /* ---- mutations ---- */

    addTaskToLane(type, laneId) {
        const cls = classOf(type);
        const task = {
            _uid: nextUid(),
            task_type: type,
            parameters: cloneParams(BROADCAST_DEFAULTS[type] ?? {}),
            start_offset: 0,
        };
        // Modifier with a per-lane variant (set_led): seed the lane-deciding
        // param from the drop target so it lands where it was dropped.
        if (cls.role === 'modifier' && cls.laneFor && type === 'set_led' && laneId) {
            const led = Object.keys(LED_LANE_BY_TYPE).find((k) => LED_LANE_BY_TYPE[k] === laneId);
            if (led) task.parameters.led_type = led;
        }
        this.bundle.push(task);
        this.afterMutate();
    }

    addTaskToHomeLane(type) {
        const cls = classOf(type);
        const home = cls.laneFor ? null : (cls.lane === '*' ? null : cls.lane);
        this.addTaskToLane(type, home);
    }

    moveTask(uid, laneId) {
        const task = this.bundle.find((t) => t._uid === uid);
        if (!task) return;
        const cls = classOf(task.task_type);
        if (cls.role === 'owner') {
            // Owners have a fixed home lane; can't be relocated.
            this.toast(`${task.task_type} is an owner — fixed to its lane.`, 'info', 'BROADCAST');
            return;
        }
        // Modifier with a lane-deciding param (set_led): update that param.
        if (cls.laneFor && task.task_type === 'set_led') {
            const led = Object.keys(LED_LANE_BY_TYPE).find((k) => LED_LANE_BY_TYPE[k] === laneId);
            if (led) { task.parameters.led_type = led; this.afterMutate(); }
            return;
        }
        // Pure modifiers have a fixed lane by type — moving is a no-op.
    }

    removeTask(uid) {
        this.bundle = this.bundle.filter((t) => t._uid !== uid);
        this.afterMutate();
    }

    editCardField(uid, scope, key, kind, raw) {
        const task = this.bundle.find((t) => t._uid === uid);
        if (!task) return;
        let value = raw;
        if (kind === '#') {
            value = raw === '' ? 0 : Number(raw);
            if (!Number.isFinite(value)) value = 0;
        }
        if (scope === 'offset') {
            task.start_offset = value;
        } else {
            task.parameters = task.parameters || {};
            task.parameters[key] = value;
        }
        // set_led.led_type changes the card's lane → full re-render needed.
        if (scope === 'param' && key === 'led_type') {
            this.afterMutate();
        } else {
            this.syncJsonFromBundle();
            this.updateConflicts();
        }
    }

    clearBroadcastBundle() {
        this.bundle = [];
        this.afterMutate();
    }

    afterMutate() {
        this.renderBundle();
        this.syncJsonFromBundle();
        this.updateConflicts();
    }

    /* ---- JSON <-> bundle sync ---- */

    syncJsonFromBundle() {
        if (this._syncing) return;
        this._syncing = true;
        $('#broadcastParams').value = JSON.stringify(this.bundle.map(stripUid), null, 2);
        this._syncing = false;
    }

    // Parse the advanced JSON view; on success replace the bundle (fresh _uids)
    // and re-render. On bad/non-array JSON, leave the bundle untouched + warn.
    syncBundleFromJson() {
        if (this._syncing) return;
        const hint = $('#broadcastLanes');
        const raw = ($('#broadcastParams').value || '').trim();
        let arr;
        if (!raw) {
            arr = [];
        } else {
            try {
                arr = JSON.parse(raw);
            } catch (e) {
                if (hint) { hint.hidden = false; hint.textContent = '⚠ bundle is not valid JSON — board unchanged'; }
                return;
            }
        }
        if (!Array.isArray(arr)) {
            if (hint) { hint.hidden = false; hint.textContent = '⚠ bundle must be a JSON array — board unchanged'; }
            return;
        }
        this.bundle = arr.map((item) => ({
            _uid: nextUid(),
            task_type: (item && item.task_type) || '',
            parameters: (item && item.parameters) || {},
            start_offset: item && Number.isFinite(item.start_offset) ? item.start_offset : 0,
        }));
        this.renderBundle();
        this.updateConflicts();
    }

    /* ---- conflict detection + policy ---- */

    // Set of lane ids holding >=2 OWNERS. Modifiers are excluded entirely.
    // Exclusive owners ('*') count toward every lane.
    laneConflicts() {
        const counts = {};
        for (const task of this.bundle) {
            if (classOf(task.task_type).role !== 'owner') continue;
            const lane = laneFor(task);
            const lanes = lane === '*' ? LANES.map((l) => l.id) : [lane];
            for (const id of lanes) counts[id] = (counts[id] || 0) + 1;
        }
        const out = new Set();
        Object.keys(counts).forEach((id) => { if (counts[id] >= 2) out.add(id); });
        return out;
    }

    updateConflicts() {
        const conflicts = this.laneConflicts();
        const ownerCount = {};
        for (const task of this.bundle) {
            if (classOf(task.task_type).role !== 'owner') continue;
            const lane = laneFor(task);
            const lanes = lane === '*' ? LANES.map((l) => l.id) : [lane];
            for (const id of lanes) ownerCount[id] = (ownerCount[id] || 0) + 1;
        }
        $$('#broadcastBoard .lane').forEach((col) => {
            const id = col.dataset.lane;
            const bad = conflicts.has(id);
            col.dataset.conflict = bad ? 'true' : 'false';
            col.setAttribute('aria-invalid', bad ? 'true' : 'false');
            const flag = col.querySelector('.lane__flag');
            if (flag) {
                flag.hidden = !bad;
                if (bad) flag.textContent = `⚠ ${ownerCount[id]} owners — keep one`;
            }
        });
        // BROADCAST disabled while any conflict exists.
        const confirm = $('#confirmBroadcastBtn');
        if (confirm) confirm.disabled = conflicts.size > 0;
        // Summary line in the advanced JSON details.
        const hint = $('#broadcastLanes');
        if (hint) {
            if (this.bundle.length === 0) {
                hint.hidden = true;
                hint.textContent = '';
            } else if (conflicts.size > 0) {
                hint.hidden = false;
                hint.textContent = `⚠ lane conflict: ${[...conflicts].join(', ')} · controller will reject`;
            } else {
                const stagger = this.bundle.map((t) => {
                    const lane = laneFor(t);
                    const off = Number.isFinite(t.start_offset) ? t.start_offset : 0;
                    return `${lane === '*' ? 'ALL' : lane} @${off}s`;
                });
                hint.hidden = false;
                hint.textContent = `lanes: ${stagger.join(', ')}`;
            }
        }
    }

    /* ---- drag-and-drop (delegated, attached once) ---- */

    bindBuilderDnd() {
        const palette = $('#broadcastPalette');
        const board = $('#broadcastBoard');
        if (!palette || !board) return;

        // Palette: click-to-add (a11y/touch) + dragstart.
        palette.addEventListener('click', (e) => {
            const chip = e.target.closest('.palette__card');
            if (chip) this.addTaskToHomeLane(chip.dataset.type);
        });
        palette.addEventListener('dragstart', (e) => {
            const chip = e.target.closest('.palette__card');
            if (!chip) return;
            e.dataTransfer.setData('text/x-task-type', chip.dataset.type);
            e.dataTransfer.effectAllowed = 'copy';
        });

        // Board: card actions (remove / JSON popover) + inline field edits.
        board.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-act]');
            if (!btn) return;
            if (btn.dataset.act === 'rm') this.removeTask(btn.dataset.uid);
            else if (btn.dataset.act === 'json') this.openCardJson(btn.dataset.uid);
        });
        board.addEventListener('input', (e) => {
            const input = e.target.closest('[data-key]');
            if (!input) return;
            this.editCardField(input.dataset.uid, input.dataset.scope,
                input.dataset.key, input.dataset.kind, input.value);
        });
        board.addEventListener('dragstart', (e) => {
            const card = e.target.closest('.card');
            if (!card || !card.draggable) return;
            e.dataTransfer.setData('text/x-task-uid', card.dataset.uid);
            e.dataTransfer.effectAllowed = 'move';
            card.classList.add('dragging');
        });
        board.addEventListener('dragend', (e) => {
            const card = e.target.closest('.card');
            if (card) card.classList.remove('dragging');
            $$('#broadcastBoard .lane').forEach((l) => l.classList.remove('lane--drop-ok', 'lane--drop-block'));
        });
        board.addEventListener('dragover', (e) => {
            const lane = e.target.closest('.lane');
            if (!lane) return;
            e.preventDefault();
            const type = e.dataTransfer.getData('text/x-task-type');
            // Owner into an occupied owner-lane → block styling (drop still
            // accepted per policy, but flagged).
            const occupied = lane.dataset.conflict === 'true'
                || this.bundle.some((t) => classOf(t.task_type).role === 'owner' && laneFor(t) === lane.dataset.lane);
            const wouldBeOwner = type && classOf(type).role === 'owner';
            lane.classList.toggle('lane--drop-block', !!(wouldBeOwner && occupied));
            lane.classList.toggle('lane--drop-ok', !(wouldBeOwner && occupied));
        });
        board.addEventListener('dragleave', (e) => {
            const lane = e.target.closest('.lane');
            if (lane) lane.classList.remove('lane--drop-ok', 'lane--drop-block');
        });
        board.addEventListener('drop', (e) => {
            const lane = e.target.closest('.lane');
            if (!lane) return;
            e.preventDefault();
            lane.classList.remove('lane--drop-ok', 'lane--drop-block');
            const laneId = lane.dataset.lane;
            const uid = e.dataTransfer.getData('text/x-task-uid');
            if (uid) { this.moveTask(uid, laneId); return; }
            const type = e.dataTransfer.getData('text/x-task-type');
            if (type) this.addTaskToLane(type, laneId);
        });
    }

    // Minimal JSON popover for a card's full parameters (edit-on-confirm).
    openCardJson(uid) {
        const task = this.bundle.find((t) => t._uid === uid);
        if (!task) return;
        const current = JSON.stringify(task.parameters || {}, null, 2);
        const edited = window.prompt(`Parameters for ${task.task_type} (JSON):`, current);
        if (edited == null) return;
        let parsed;
        try {
            parsed = JSON.parse(edited);
        } catch (e) {
            this.toast('Invalid JSON — parameters unchanged.', 'error', 'BROADCAST');
            return;
        }
        if (typeof parsed !== 'object' || parsed === null || Array.isArray(parsed)) {
            this.toast('Parameters must be a JSON object.', 'error', 'BROADCAST');
            return;
        }
        task.parameters = parsed;
        this.afterMutate();
    }

    async sendBroadcast() {
        // Guard: never send while any owner-lane conflict exists.
        if (this.laneConflicts().size > 0) {
            this.toast('Resolve lane conflicts before broadcasting.', 'error', 'BROADCAST');
            return;
        }
        const tasks = this.bundle.map(stripUid);
        if (tasks.length === 0) {
            this.toast('Bundle is empty — add at least one task.', 'info', 'BROADCAST');
            return;
        }
        for (const item of tasks) {
            if (typeof item.task_type !== 'string' || !item.task_type.trim()) {
                this.toast('Each bundle entry needs a task_type.', 'error', 'BROADCAST');
                return;
            }
        }

        let startOffset = parseFloat($('#broadcastLead').value);
        if (!Number.isFinite(startOffset) || startOffset < 0.5) startOffset = 0.5;

        try {
            const r = await fetch('/api/broadcast_task', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ tasks, start_offset: startOffset }),
            });
            const data = await r.json();
            if (!data.success) {
                this.toast(`Broadcast failed: ${data.message || 'bad request'}`, 'error', 'BROADCAST');
                return;
            }
            const sent = data.sent;
            const failed = data.failed;
            if (failed === 0) {
                this.toast(`Broadcast to ${sent} unit${sent === 1 ? '' : 's'}, start in ${data.start_offset}s`, 'success', 'BROADCAST');
            } else {
                const failedNames = (data.results || []).filter((x) => !x.success).map((x) => x.name);
                this.toast(`${sent} sent, ${failed} failed: ${this.truncateNames(failedNames)}`, 'error', 'BROADCAST');
            }
            this.startCountdown(data.start_offset);
        } catch (e) {
            this.toast('Broadcast uplink lost.', 'error', 'BROADCAST');
        }
    }

    startCountdown(offset) {
        const cd = $('#broadcastCountdown');
        if (this._countdownTimer) clearInterval(this._countdownTimer);
        let remaining = Math.max(1, Math.ceil(offset));
        cd.hidden = false;
        cd.textContent = `START IN ${remaining}…`;
        this._countdownTimer = setInterval(() => {
            remaining -= 1;
            if (remaining > 0) {
                cd.textContent = `START IN ${remaining}…`;
            } else {
                cd.textContent = 'GO';
                clearInterval(this._countdownTimer);
                this._countdownTimer = null;
                setTimeout(() => this.closeModal('broadcastModal'), 600);
            }
        }, 1000);
    }

    openModal(id, focusId) {
        const m = document.getElementById(id);
        m.classList.add('show');
        if (focusId) {
            const f = document.getElementById(focusId);
            if (f) { f.value = ''; setTimeout(() => f.focus(), 30); }
        }
    }
    closeModal(id) { document.getElementById(id).classList.remove('show'); }
    requestDetach(name) {
        this.pendingBatchDetach = null;
        this.pendingDetach = name;
        $('#removeConfirmText').textContent =
            `Release unit "${name}"? This terminates the WebSocket server on its channel and stops every attached controller.`;
        this.openModal('confirmRemoveModal');
    }
    requestBatchDetach(names) {
        if (!names.length) { this.toast('No units selected.', 'error', 'DETACH'); return; }
        this.pendingDetach = null;
        this.pendingBatchDetach = names;
        const n = names.length;
        $('#removeConfirmText').textContent =
            `Release ${n} unit${n === 1 ? '' : 's'}? This terminates each unit's WebSocket server and stops every attached controller.`;
        this.openModal('confirmRemoveModal');
    }

    /* -------------------------------------------------------- toast */
    toast(message, kind = 'info', tag = 'INFO') {
        const t = document.createElement('div');
        t.className = `toast toast--${kind}`;
        t.innerHTML = `<span class="toast__tag">[${tag}]</span>${message.replace(/[<>&]/g, (c) => ({'<':'&lt;','>':'&gt;','&':'&amp;'}[c]))}`;
        document.body.appendChild(t);
        setTimeout(() => {
            t.classList.add('toast--out');
            setTimeout(() => t.remove(), 220);
        }, 3200);
    }

    /* -------------------------------------------------------- format helpers */
    formatAge(unixSec) {
        if (!unixSec) return '—';
        const sec = Math.max(0, Math.floor(Date.now() / 1000 - unixSec));
        if (sec < 60)      return `${sec}S`;
        if (sec < 3600)    return `${Math.floor(sec / 60)}M ${sec % 60}S`;
        if (sec < 86400)   return `${Math.floor(sec / 3600)}H ${Math.floor((sec % 3600) / 60)}M`;
        return `${Math.floor(sec / 86400)}D ${Math.floor((sec % 86400) / 3600)}H`;
    }
    formatClock(ms) {
        const d = new Date(ms);
        const hh = String(d.getHours()).padStart(2, '0');
        const mm = String(d.getMinutes()).padStart(2, '0');
        const ss = String(d.getSeconds()).padStart(2, '0');
        return `${hh}:${mm}:${ss}`;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.scs = new ControlStation();
});
