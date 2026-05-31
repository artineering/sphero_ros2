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

// Which actuator lane(s) each task type occupies. The per-instance controller
// allows one task per lane concurrently; a bundle whose sub-tasks share a lane
// is rejected server-side. This map drives a lightweight client-side warning
// only (it does NOT block sending). 'custom' and 'jumping_bean' touch ALL lanes.
const ALL_LANES = ['DRIVE', 'LED', 'MATRIX'];
const TASK_LANES = {
    move_to: ['DRIVE'], patrol: ['DRIVE'], square: ['DRIVE'], circle: ['DRIVE'],
    spin: ['DRIVE'], roll: ['DRIVE'], heading: ['DRIVE'], speed: ['DRIVE'],
    reflect: ['DRIVE'], stop: ['DRIVE'],
    set_led: ['LED'], led_sequence: ['LED'],
    matrix: ['MATRIX'], matrix_sequence: ['MATRIX'],
    custom: ALL_LANES, jumping_bean: ALL_LANES,
};

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

        // Broadcast modal
        $('#broadcastBtn').addEventListener('click', () => this.openBroadcast());
        $('#broadcastAddBtn').addEventListener('click', () => this.addBroadcastTask());
        $('#broadcastClearBtn').addEventListener('click', () => this.clearBroadcastBundle());
        $('#broadcastParams').addEventListener('input', () => this.renderBroadcastLanes());
        $('#confirmBroadcastBtn').addEventListener('click', () => this.sendBroadcast());
        $('#cancelBroadcastBtn').addEventListener('click', () => this.closeModal('broadcastModal'));

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
                this.spheros = data.spheros || [];
                this.lastSyncAt = Date.now();
                this.renderFleet();
                this.updateTelemetry();
            }
        } catch (err) {
            this.linkDown();
            console.error('refresh error', err);
        }
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
        this.renderBroadcastLanes();
        $('#broadcastSummary').textContent =
            `${count} unit${count === 1 ? '' : 's'} · fires at now + lead`;
        const cd = $('#broadcastCountdown');
        cd.hidden = true;
        cd.textContent = '';
        this.openModal('broadcastModal', 'broadcastType');
    }

    // Parse the bundle editor as a JSON array. Returns the array on success, or
    // null on parse error / non-array (caller decides how to react).
    parseBroadcastBundle() {
        const raw = ($('#broadcastParams').value || '').trim();
        if (!raw) return [];
        let arr;
        try {
            arr = JSON.parse(raw);
        } catch (e) {
            return null;
        }
        return Array.isArray(arr) ? arr : null;
    }

    // ADD: append the selected type (with its default params) to the bundle.
    addBroadcastTask() {
        const type = $('#broadcastType').value;
        const arr = this.parseBroadcastBundle();
        if (arr === null) {
            this.toast('Bundle must be a valid JSON array.', 'error', 'BROADCAST');
            return;
        }
        // start_offset is an explicit per-task lane stagger (additive on the
        // bundle lead). Inject 0 so it's visible in the JSON and easy to edit.
        arr.push({ task_type: type, parameters: BROADCAST_DEFAULTS[type] ?? {}, start_offset: 0 });
        $('#broadcastParams').value = JSON.stringify(arr, null, 2);
        this.renderBroadcastLanes();
    }

    clearBroadcastBundle() {
        $('#broadcastParams').value = '[]';
        this.renderBroadcastLanes();
    }

    // Lightweight live hint: which lanes the bundle occupies + a conflict
    // warning when two entries share a lane. Never blocks sending.
    renderBroadcastLanes() {
        const hint = $('#broadcastLanes');
        if (!hint) return;
        const arr = this.parseBroadcastBundle();
        if (arr === null) {
            hint.hidden = false;
            hint.textContent = '⚠ bundle is not valid JSON';
            return;
        }
        if (arr.length === 0) {
            hint.hidden = true;
            hint.textContent = '';
            return;
        }
        const seen = new Set();
        const conflicts = new Set();
        const stagger = [];
        for (const item of arr) {
            const type = item && item.task_type;
            const lanes = TASK_LANES[type] ?? [];
            for (const lane of lanes) {
                if (seen.has(lane)) conflicts.add(lane);
                seen.add(lane);
            }
            // Per-task stagger readout, e.g. "DRIVE @0s, LED @2s".
            const off = item && Number.isFinite(item.start_offset) ? item.start_offset : 0;
            const laneTag = lanes.length ? lanes.join('+') : (type || '?');
            stagger.push(`${laneTag} @${off}s`);
        }
        const lanesTxt = seen.size ? [...seen].join(' + ') : '(unknown)';
        hint.hidden = false;
        hint.textContent = conflicts.size
            ? `⚠ lane conflict: ${[...conflicts].join(', ')} · controller will reject`
            : `lanes: ${lanesTxt} · ${stagger.join(', ')}`;
    }

    async sendBroadcast() {
        const tasks = this.parseBroadcastBundle();
        if (tasks === null) {
            this.toast('Bundle must be a valid JSON array.', 'error', 'BROADCAST');
            return;
        }
        if (tasks.length === 0) {
            this.toast('Bundle is empty — ADD at least one task.', 'info', 'BROADCAST');
            return;
        }
        for (const item of tasks) {
            if (typeof item !== 'object' || item === null || Array.isArray(item)
                || typeof item.task_type !== 'string' || !item.task_type.trim()) {
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
