/* ============================================================
   SCS·01 — Sphero Control Station
   Operator console front-end logic
   ============================================================ */

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

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
