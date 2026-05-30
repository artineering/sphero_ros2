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
            this.openModal('addSpheroModal', 'spheroNameInput');
            this.refreshUwbTags();
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
        const nameInput = $('#spheroNameInput');
        const tagSelect = $('#tagIdSelect');
        confirmAdd.addEventListener('click', () => {
            const name = nameInput.value.trim();
            if (!name) { this.toast('Enter a callsign first.', 'error', 'DEPLOY'); return; }
            const opt = tagSelect.options[tagSelect.selectedIndex];
            if (!tagSelect.value || (opt && opt.disabled)) {
                this.toast('Select a free UWB tag.', 'error', 'DEPLOY');
                return;
            }
            const tagId = parseInt(tagSelect.value, 10);
            this.deploySphero(name, tagId);
        });
        cancelAdd.addEventListener('click', () => this.closeModal('addSpheroModal'));
        nameInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') confirmAdd.click(); });

        // Detach modal
        $('#confirmRemoveBtn').addEventListener('click', () => {
            if (this.pendingDetach) {
                this.detachSphero(this.pendingDetach);
                this.closeModal('confirmRemoveModal');
            }
        });
        $('#cancelRemoveBtn').addEventListener('click', () => this.closeModal('confirmRemoveModal'));

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

    async deploySphero(name, tagId) {
        try {
            const r = await fetch('/api/spheros', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sphero_name: name, tag_id: tagId }),
            });
            const data = await r.json();
            if (data.success) {
                this.toast(`Unit ${name} bound to channel ${data.instance?.port ?? '?'} (tag ${data.instance?.tag_id ?? tagId})`, 'success', 'DEPLOY');
                this.closeModal('addSpheroModal');
                this.refresh();
                this.refreshUwbTags();
            } else {
                // Keep the modal open so the operator can correct the error.
                this.toast(`Deploy failed: ${data.message}`, 'error', 'DEPLOY');
            }
        } catch (err) {
            this.toast('Deploy uplink lost.', 'error', 'DEPLOY');
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
                    this.populateTagSelect();
                }
            }
        } catch (err) {
            // Silent.
        }
    }

    populateTagSelect() {
        const select = $('#tagIdSelect');
        const confirmBtn = $('#confirmAddBtn');
        select.innerHTML = '';

        if (!this.uwbTags.all.length || !this.uwbTags.free.length) {
            const ph = document.createElement('option');
            ph.value = '';
            ph.disabled = true;
            ph.selected = true;
            ph.textContent = this.uwbTags.all.length ? 'no free tags' : 'loading…';
            select.appendChild(ph);
            confirmBtn.disabled = true;
            return;
        }

        confirmBtn.disabled = false;
        const firstFree = this.uwbTags.free[0];
        this.uwbTags.all.forEach((id) => {
            const opt = document.createElement('option');
            opt.value = String(id);
            const owner = this.uwbTags.assigned[String(id)];
            if (owner) {
                opt.disabled = true;
                opt.textContent = `${id} · ${owner}`;
            } else {
                opt.textContent = String(id);
            }
            if (id === firstFree) opt.selected = true;
            select.appendChild(opt);
        });
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

        if (this.spheros.length === 0) {
            grid.innerHTML = '';
            empty.classList.add('show');
            this._lastFleetSig = '';
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
            // Fleet shape unchanged — only refresh dynamic [UPT] cells.
            this.spheros.forEach((s) => {
                const tile = document.getElementById(`unit-${s.name}`);
                if (!tile) return;
                const uptCell = tile.querySelector('.unit__upt');
                if (uptCell) uptCell.textContent = this.formatAge(s.added_at);
            });
            return;
        }
        this._lastFleetSig = sig;

        grid.innerHTML = this.spheros.map((s, i) => this.unitTile(s, i)).join('');

        this.spheros.forEach((s) => {
            const open = document.getElementById(`open-${s.name}`);
            const detach = document.getElementById(`detach-${s.name}`);
            if (open) open.addEventListener('click', () => window.open(this.consoleUrl(s.url), '_blank'));
            if (detach) detach.addEventListener('click', () => this.requestDetach(s.name));
        });
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
        this.pendingDetach = name;
        $('#removeConfirmText').textContent =
            `Release unit "${name}"? This terminates the WebSocket server on its channel and stops every attached controller.`;
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
