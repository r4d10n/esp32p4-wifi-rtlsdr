'use strict';

/* ═══════════════════════════════════════════════════════════════
   ESP32-P4 SDR Portal — portal.js
   Vanilla JS, no external dependencies
   ═══════════════════════════════════════════════════════════════ */

/* ── Utilities ────────────────────────────────────────────────── */
function esc(s) {
    return String(s)
        .replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')
        .replace(/"/g,'&quot;').replace(/'/g,'&#39;');
}

function debounce(fn, ms) {
    let t;
    return (...args) => { clearTimeout(t); t = setTimeout(() => fn(...args), ms); };
}

/* ── Toast ────────────────────────────────────────────────────── */
function toast(msg, type) {
    const container = document.getElementById('toast-container');
    const el = document.createElement('div');
    el.className = 'toast ' + (type || 'info');

    const icons = {
        ok:   '<svg class="toast-icon" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polyline points="20 6 9 17 4 12"/></svg>',
        err:  '<svg class="toast-icon" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/></svg>',
        info: '<svg class="toast-icon" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>',
    };
    el.innerHTML = (icons[type] || icons.info) + esc(msg);
    container.appendChild(el);

    // auto-dismiss after 3.5 s
    setTimeout(() => {
        el.classList.add('removing');
        el.addEventListener('animationend', () => el.remove(), { once: true });
    }, 3500);
}

/* ── API helpers ──────────────────────────────────────────────── */
async function api(path, opts) {
    const r = await fetch('/api/' + path, {
        headers: { 'Content-Type': 'application/json' },
        ...opts
    });
    let data;
    try { data = await r.json(); } catch { data = {}; }
    if (!r.ok) throw new Error(data.message || r.statusText || 'Request failed');
    return data;
}
const apiGet    = path       => api(path);
const apiPost   = (path, b) => api(path, { method:'POST',   body:JSON.stringify(b) });
const apiPut    = (path, b) => api(path, { method:'PUT',    body:JSON.stringify(b) });
const apiDelete = path       => api(path, { method:'DELETE' });

/* wrap with error toast */
async function apiSafe(fn) {
    try { return await fn(); }
    catch (e) { toast(e.message, 'err'); return null; }
}

/* ── Tab Switching ────────────────────────────────────────────── */
let activeTab = null;
let wifiRefreshTimer = null;

document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.addEventListener('click', () => switchTab(btn.dataset.tab));
});

function switchTab(name) {
    if (activeTab === name) return;
    activeTab = name;

    document.querySelectorAll('.tab-btn').forEach(b => {
        const active = b.dataset.tab === name;
        b.classList.toggle('active', active);
        b.setAttribute('aria-selected', active);
    });
    document.querySelectorAll('.tab-pane').forEach(p => {
        p.classList.toggle('active', p.id === 'tab-' + name);
    });

    clearInterval(wifiRefreshTimer);
    wifiRefreshTimer = null;

    const loaders = {
        wifi:     () => { loadWifiStatus(); loadSavedNetworks();
                          wifiRefreshTimer = setInterval(loadWifiStatus, 5000); },
        ethernet: loadEthConfig,
        sdr:      loadSdrConfig,
        services: loadServices,
        notify:   loadNotifyConfig,
        system:   loadSystemInfo,
    };
    if (loaders[name]) loaders[name]();
}

/* ══════════════════════════════════════════════════════════════
   WiFi Tab
   ══════════════════════════════════════════════════════════════ */

/* Signal strength → bar count */
function rssiToBars(rssi) {
    if (rssi >= -55) return 4;
    if (rssi >= -67) return 3;
    if (rssi >= -78) return 2;
    if (rssi >= -89) return 1;
    return 0;
}

function rssiColor(rssi) {
    if (rssi >= -67) return 'green';
    if (rssi >= -80) return 'yellow';
    return 'red';
}

/* Large signal SVG in status card */
function renderStatusSignal(rssi, connected) {
    const bars = connected ? rssiToBars(rssi) : 0;
    const color = rssiColor(rssi);
    ['sig-b1','sig-b2','sig-b3','sig-b4'].forEach((id, i) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.className = 'sig-bar';
        if (i < bars) el.classList.add('active-' + color);
    });
    const rssiEl = document.getElementById('wifi-rssi-val');
    if (rssiEl) rssiEl.textContent = connected ? rssi + ' dBm' : '— dBm';
}

/* 4-bar mini SVG for network list items */
function signalBarsHtml(rssi) {
    const bars = rssiToBars(rssi);
    const color = rssiColor(rssi);
    return [1,2,3,4].map(i => {
        const lit = i <= bars ? ` lit-${color}` : '';
        return `<span class="ns-bar${lit}"></span>`;
    }).join('');
}

async function loadWifiStatus() {
    const d = await apiSafe(() => apiGet('wifi/status'));
    if (!d) return;

    /* Header dot */
    const dot = document.getElementById('hdr-conn-dot');
    const lbl = document.getElementById('hdr-conn-label');
    if (dot && lbl) {
        dot.className = 'conn-dot ' + (d.connected ? 'connected' : 'disconnected');
        lbl.textContent = d.connected ? (d.ssid || 'Connected') : 'Disconnected';
    }

    /* Status pill */
    const pill = document.getElementById('wifi-state-pill');
    if (pill) {
        const pillDot  = pill.querySelector('.pill-dot');
        const pillText = pill.querySelector('.pill-text');
        if (pillDot)  pillDot.className  = 'pill-dot ' + (d.connected ? 'on' : 'off');
        if (pillText) pillText.textContent = d.state || (d.connected ? 'Connected' : 'Disconnected');
    }

    /* Signal bars */
    renderStatusSignal(d.rssi || -100, d.connected);

    /* Details */
    const set = (id, val) => { const e = document.getElementById(id); if (e) e.textContent = val || '—'; };
    set('wifi-ssid', d.ssid);
    set('wifi-ip',   d.ip);
    set('wifi-mode', d.mode);
}

async function scanWifi() {
    const btn = document.getElementById('wifi-scan-btn');
    if (btn) { btn.disabled = true; }

    const dot = document.getElementById('hdr-conn-dot');
    if (dot) dot.className = 'conn-dot scanning';

    const list = document.getElementById('wifi-list');
    if (list) list.innerHTML = '<div class="empty-state">Scanning…</div>';

    try {
        const d = await apiGet('wifi/scan');
        renderNetworkList(d.networks || []);
    } catch (e) {
        toast(e.message, 'err');
        if (list) list.innerHTML = '<div class="empty-state">Scan failed</div>';
    } finally {
        if (btn) btn.disabled = false;
        loadWifiStatus(); // refresh dot
    }
}

function renderNetworkList(networks) {
    const list = document.getElementById('wifi-list');
    if (!list) return;
    if (!networks.length) {
        list.innerHTML = '<div class="empty-state">No networks found</div>';
        return;
    }
    list.innerHTML = networks
        .sort((a,b) => b.rssi - a.rssi)
        .map(n => {
            const lockIcon = n.auth && n.auth !== 'open' && n.auth !== 'OPEN'
                ? `<span class="badge badge-lock" title="${esc(n.auth)}">
                     <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 10 0v4"/></svg>
                   </span>`
                : '';
            const savedBadge = n.saved
                ? '<span class="badge badge-saved">Saved</span>'
                : '';
            const connectBtn = !n.saved
                ? `<button class="net-connect-btn" onclick="openConnectModal(${JSON.stringify(esc(n.ssid))},${n.auth && n.auth!=='open'})">Connect</button>`
                : '';
            return `<div class="net-item ${n.saved ? 'is-saved' : ''}" ${!n.saved ? `onclick="openConnectModal('${esc(n.ssid).replace(/'/g,"\\'")}',${n.auth && n.auth!=='open'})"` : ''}>
                <div class="net-signal">${signalBarsHtml(n.rssi)}</div>
                <span class="net-rssi">${n.rssi} dBm</span>
                <span class="net-ssid">${esc(n.ssid)}</span>
                <div class="net-badges">${lockIcon}${savedBadge}${connectBtn}</div>
            </div>`;
        }).join('');
}

async function loadSavedNetworks() {
    const d = await apiSafe(() => apiGet('wifi/networks'));
    const el = document.getElementById('saved-networks');
    if (!el) return;
    const nets = d ? (d.networks || []) : [];
    if (!nets.length) {
        el.innerHTML = '<div class="empty-state">No saved networks</div>';
        return;
    }
    el.innerHTML = nets.map(n => `
        <div class="saved-item">
            <span class="saved-ssid">${esc(n.ssid)}</span>
            <div class="saved-actions">
                <button class="icon-btn icon-btn-ghost" title="Connect" onclick="openConnectModal('${esc(n.ssid).replace(/'/g,"\\'")}',false)">
                    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polygon points="5 3 19 12 5 21 5 3"/></svg>
                </button>
                <button class="icon-btn icon-btn-danger" title="Forget" onclick="deleteWifi('${esc(n.ssid).replace(/'/g,"\\'")}')">
                    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polyline points="3 6 5 6 21 6"/><path d="M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6"/><path d="M10 11v6M14 11v6"/><path d="M9 6V4a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v2"/></svg>
                </button>
            </div>
        </div>`).join('');
}

async function deleteWifi(ssid) {
    if (!confirm('Forget network "' + ssid + '"?')) return;
    const ok = await apiSafe(() => apiDelete('wifi/networks?ssid=' + encodeURIComponent(ssid)));
    if (ok !== null) { toast('Network forgotten', 'ok'); loadSavedNetworks(); }
}

/* ── Connect modal ────────────────────────────────────────────── */
let _modalSSID = '';
let _modalNeedsPW = false;

function openConnectModal(ssid, needsPassword) {
    _modalSSID   = ssid;
    _modalNeedsPW = needsPassword;
    const title = document.getElementById('pw-modal-title');
    const pwIn  = document.getElementById('modal-password');
    if (title) title.textContent = 'Connect to ' + ssid;
    if (pwIn) { pwIn.value = ''; pwIn.placeholder = needsPassword ? 'Password required' : 'Leave blank if open'; }
    document.getElementById('pw-modal').classList.add('open');
    setTimeout(() => { if (pwIn) pwIn.focus(); }, 50);
}

function closeModal(id) {
    document.getElementById(id).classList.remove('open');
}

async function confirmModal() {
    const pwIn  = document.getElementById('modal-password');
    const btn   = document.getElementById('modal-confirm-btn');
    const pass  = pwIn ? pwIn.value : '';
    setButtonLoading(btn, true);
    try {
        const d = await apiPost('wifi/connect', { ssid: _modalSSID, password: pass });
        toast(d.message || 'Connected', 'ok');
        closeModal('pw-modal');
        setTimeout(loadWifiStatus, 1500);
        setTimeout(loadSavedNetworks, 2000);
    } catch (e) {
        toast(e.message, 'err');
    } finally {
        setButtonLoading(btn, false);
    }
}

/* close modal on backdrop click */
document.getElementById('pw-modal').addEventListener('click', function(e) {
    if (e.target === this) closeModal('pw-modal');
});
document.addEventListener('keydown', e => {
    if (e.key === 'Escape') closeModal('pw-modal');
});

/* ── Button loading state helper ──────────────────────────────── */
function setButtonLoading(btn, loading) {
    if (!btn) return;
    const text    = btn.querySelector('.btn-text');
    const spinner = btn.querySelector('.btn-spinner');
    btn.disabled  = loading;
    if (text)    text.style.display    = loading ? 'none' : '';
    if (spinner) spinner.hidden        = !loading;
}

/* inline connect card (legacy — kept for keyboard accessibility) */
function closeConnectCard() {
    const c = document.getElementById('wifi-connect-card');
    if (c) c.style.display = 'none';
}
async function submitConnect() {
    const ssid = document.getElementById('connect-ssid').value.trim();
    const pass = document.getElementById('connect-pass').value;
    const btn  = document.getElementById('connect-submit-btn');
    const res  = document.getElementById('connect-result');
    if (!ssid) { toast('Enter SSID', 'err'); return; }
    setButtonLoading(btn, true);
    if (res) { res.className = 'connect-result'; res.textContent = ''; }
    try {
        const d = await apiPost('wifi/connect', { ssid, password: pass });
        if (res) { res.className = 'connect-result ok visible'; res.textContent = d.message || 'Connected'; }
        toast(d.message || 'Connected', 'ok');
        setTimeout(loadWifiStatus, 1500);
    } catch (e) {
        if (res) { res.className = 'connect-result err visible'; res.textContent = e.message; }
    } finally {
        setButtonLoading(btn, false);
    }
}

/* ══════════════════════════════════════════════════════════════
   Services Tab
   ══════════════════════════════════════════════════════════════ */

const SERVICE_META = {
    rtl_tcp:    { label:'RTL-TCP',    desc:'Raw IQ streaming via TCP'               },
    rtl_udp:    { label:'RTL-UDP',    desc:'Raw IQ streaming via UDP'               },
    websdr:     { label:'WebSDR',     desc:'Browser-based SDR receiver'             },
    spyserver:  { label:'SpyServer',  desc:'SDRSharp SpyServer protocol'            },
    soapysdr:   { label:'SoapySDR',   desc:'Universal SDR API server'               },
    rtl_433:    { label:'RTL-433',    desc:'ISM-band device decoder'                },
    rtl_power:  { label:'RTL-Power',  desc:'Wide-band power spectral density sweep' },
    fm_player:  { label:'FM Player',  desc:'WFM demodulator audio stream'           },
};

/* debounced per-service save (500ms) */
const _svcDebounce = {};

function debouncedSaveSvc(name) {
    if (!_svcDebounce[name]) {
        _svcDebounce[name] = debounce(() => saveSvcNow(name), 500);
    }
    _svcDebounce[name]();
}

async function saveSvcNow(name) {
    const card = document.querySelector(`.svc-card[data-svc="${CSS.escape(name)}"]`);
    if (!card) return;
    const cfg = collectSvcParams(card, name);
    const ok = await apiSafe(() => apiPut('services/' + name, cfg));
    if (ok !== null) toast(name + ' saved', 'ok');
}

function collectSvcParams(card, name) {
    const cfg = {};
    /* enable toggle */
    const tog = card.querySelector('.svc-toggle');
    if (tog) cfg.enable = tog.checked;
    /* other params */
    card.querySelectorAll('[data-param]').forEach(el => {
        const key = el.dataset.param;
        if (el.type === 'checkbox')     cfg[key] = el.checked;
        else if (el.type === 'number')  cfg[key] = Number(el.value);
        else if (el.type === 'range')   cfg[key] = Number(el.value);
        else                            cfg[key] = el.value;
    });
    return cfg;
}

async function loadServices() {
    const d = await apiSafe(() => apiGet('services'));
    if (!d) return;
    const el = document.getElementById('services-list');
    if (!el) return;
    el.innerHTML = '';

    for (const [name, cfg] of Object.entries(d)) {
        const meta  = SERVICE_META[name] || { label: name, desc: '' };
        const on    = !!cfg.enable;
        const card  = document.createElement('div');
        card.className = 'svc-card' + (on ? ' svc-enabled' : '');
        card.dataset.svc = name;

        const portTag = cfg.port
            ? `<span class="svc-port-tag">:${cfg.port}</span>`
            : '';

        card.innerHTML = `
            <div class="svc-header" onclick="toggleSvcCard(this.closest('.svc-card'))">
                <span class="svc-run-dot"></span>
                <div class="svc-name-wrap">
                    <div class="svc-name">${esc(meta.label)}</div>
                    <div class="svc-desc">${esc(meta.desc)}</div>
                </div>
                ${portTag}
                <label class="toggle-pill" for="svc-tog-${esc(name)}" onclick="event.stopPropagation()" aria-label="Enable ${esc(meta.label)}">
                    <input type="checkbox" id="svc-tog-${esc(name)}" class="svc-toggle" ${on ? 'checked' : ''}
                           onchange="onSvcToggle('${esc(name)}',this)">
                    <span class="toggle-pill-track"></span>
                </label>
                <svg class="svc-expand-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="6 9 12 15 18 9"/></svg>
            </div>
            <div class="svc-body">
                <div class="svc-params">${buildSvcParams(name, cfg)}</div>
            </div>`;

        el.appendChild(card);
        /* wire up instant-apply */
        card.querySelectorAll('[data-param]').forEach(input => {
            input.addEventListener('input', () => debouncedSaveSvc(name));
            input.addEventListener('change', () => debouncedSaveSvc(name));
        });
    }
}

function toggleSvcCard(card) {
    card.classList.toggle('open');
}

async function onSvcToggle(name, checkbox) {
    const card = document.querySelector(`.svc-card[data-svc="${CSS.escape(name)}"]`);
    const on   = checkbox.checked;
    const ok   = await apiSafe(() => apiPut('services/' + name, { enable: on }));
    if (ok !== null) {
        if (card) card.classList.toggle('svc-enabled', on);
        toast((on ? 'Started' : 'Stopped') + ' ' + name, 'ok');
    } else {
        checkbox.checked = !on; // revert
    }
}

/* Build inline param controls based on value type */
function buildSvcParams(name, cfg) {
    const skip = new Set(['enable']);
    let html = '';
    for (const [k, v] of Object.entries(cfg)) {
        if (skip.has(k)) continue;
        const label = k.replace(/_/g, ' ');
        if (typeof v === 'boolean') {
            html += `<div class="form-field toggle-field">
                <label class="field-label" for="svc-${esc(name)}-${esc(k)}">${esc(label)}</label>
                <label class="toggle" for="svc-${esc(name)}-${esc(k)}">
                    <input type="checkbox" id="svc-${esc(name)}-${esc(k)}" data-param="${esc(k)}" ${v ? 'checked' : ''}>
                    <span class="toggle-track"></span>
                </label>
            </div>`;
        } else if (k === 'port' || k.endsWith('_port')) {
            html += `<div class="form-field">
                <label class="field-label" for="svc-${esc(name)}-${esc(k)}">${esc(label)}</label>
                <input class="field-input mono" type="number" id="svc-${esc(name)}-${esc(k)}"
                       data-param="${esc(k)}" value="${esc(v)}" min="1" max="65535">
            </div>`;
        } else if (typeof v === 'number') {
            /* use range if it's likely a small bounded value, else number */
            const isSmall = v <= 100 && v >= 0;
            if (isSmall) {
                html += `<div class="form-field">
                    <label class="field-label" for="svc-${esc(name)}-${esc(k)}">${esc(label)}</label>
                    <div class="slider-header">
                        <input class="range-slider" type="range" id="svc-${esc(name)}-${esc(k)}"
                               data-param="${esc(k)}" min="0" max="100" value="${esc(v)}"
                               oninput="this.nextElementSibling.textContent=this.value">
                        <span class="slider-val mono">${v}</span>
                    </div>
                </div>`;
            } else {
                html += `<div class="form-field">
                    <label class="field-label" for="svc-${esc(name)}-${esc(k)}">${esc(label)}</label>
                    <input class="field-input mono" type="number" id="svc-${esc(name)}-${esc(k)}"
                           data-param="${esc(k)}" value="${esc(v)}">
                </div>`;
            }
        } else {
            html += `<div class="form-field">
                <label class="field-label" for="svc-${esc(name)}-${esc(k)}">${esc(label)}</label>
                <input class="field-input" type="text" id="svc-${esc(name)}-${esc(k)}"
                       data-param="${esc(k)}" value="${esc(v)}">
            </div>`;
        }
    }
    return html || '<div class="empty-state" style="padding:12px 0">No configurable parameters</div>';
}

/* ══════════════════════════════════════════════════════════════
   SDR Tab
   ══════════════════════════════════════════════════════════════ */

let _freqUnit = 'MHz';        /* current display unit */
let _freqHz   = 100000000;    /* canonical Hz value   */
let _sdrData  = null;

const _sdrSaveDebouncedFns = {};
function debouncedSaveSdr() {
    if (!_sdrSaveDebouncedFns.main) {
        _sdrSaveDebouncedFns.main = debounce(saveSdrConfig, 500);
    }
    _sdrSaveDebouncedFns.main();
}

function setFreqUnit(unit) {
    _freqUnit = unit;
    document.querySelectorAll('.unit-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.unit === unit);
    });
    renderFreqInput();
}

function renderFreqInput() {
    const inp = document.getElementById('sdr-freq-raw');
    if (!inp) return;
    inp.value = _freqUnit === 'MHz'
        ? (_freqHz / 1e6).toFixed(3)
        : (_freqHz / 1e3).toFixed(0);
}

function readFreqInput() {
    const inp = document.getElementById('sdr-freq-raw');
    if (!inp) return;
    const val = parseFloat(inp.value) || 0;
    _freqHz = Math.round(_freqUnit === 'MHz' ? val * 1e6 : val * 1e3);
}

function updateGainDisplay(raw) {
    const db = (raw / 10).toFixed(1);
    const el = document.getElementById('sdr-gain-display');
    if (el) el.textContent = db + ' dB';
}

function toggleGainMode() {
    const auto = document.getElementById('sdr-gain-auto');
    const wrap = document.getElementById('gain-manual-wrap');
    const lbl  = document.getElementById('gain-mode-label');
    const isAuto = auto ? auto.checked : false;
    if (wrap) wrap.style.opacity = isAuto ? '0.35' : '1';
    if (wrap) wrap.style.pointerEvents = isAuto ? 'none' : '';
    if (lbl)  lbl.textContent = isAuto ? 'Auto' : 'Manual';
}

async function loadSdrConfig() {
    const d = await apiSafe(() => apiGet('sdr/config'));
    if (!d) return;
    _sdrData = d;

    /* Frequency */
    if (d.center_freq != null) {
        _freqHz = d.center_freq;
        renderFreqInput();
    }

    /* Sample rate */
    const sr = document.getElementById('sdr-sample_rate');
    if (sr && d.sample_rate != null) sr.value = d.sample_rate;

    /* Gain */
    const gainAuto = document.getElementById('sdr-gain-auto');
    if (gainAuto) gainAuto.checked = (d.gain_mode === 'auto');
    const gainSlider = document.getElementById('sdr-tuner_gain_tenth_db');
    if (gainSlider && d.tuner_gain_tenth_db != null) {
        gainSlider.value = d.tuner_gain_tenth_db;
        updateGainDisplay(d.tuner_gain_tenth_db);
    }
    toggleGainMode();

    /* Checkboxes and other fields */
    const boolFields = ['rtl_agc','tuner_agc','offset_tuning','dc_offset_correction',
                        'iq_imbalance_correction','invert_iq','bias_tee'];
    boolFields.forEach(k => {
        const el = document.getElementById('sdr-' + k);
        if (el && d[k] != null) el.checked = d[k];
    });

    const numFields = ['ppm_correction','offset_freq_hz','max_total_users'];
    numFields.forEach(k => {
        const el = document.getElementById('sdr-' + k);
        if (el && d[k] != null) el.value = d[k];
    });

    const txtFields = ['hostname','direct_sampling'];
    txtFields.forEach(k => {
        const el = document.getElementById('sdr-' + k);
        if (el && d[k] != null) el.value = d[k];
    });

    /* Wire instant-apply after first load */
    document.querySelectorAll('#tab-sdr input, #tab-sdr select').forEach(el => {
        if (el._sdrWired) return;
        el._sdrWired = true;
        el.addEventListener('input', debouncedSaveSdr);
        el.addEventListener('change', debouncedSaveSdr);
    });
    /* freq input needs special read */
    const freqInp = document.getElementById('sdr-freq-raw');
    if (freqInp && !freqInp._freqWired) {
        freqInp._freqWired = true;
        freqInp.addEventListener('input', () => { readFreqInput(); debouncedSaveSdr(); });
    }
}

async function saveSdrConfig() {
    const btn = document.getElementById('sdr-save-btn');
    if (btn) btn.disabled = true;
    readFreqInput();

    const gainAuto = document.getElementById('sdr-gain-auto');
    const cfg = {
        center_freq:             _freqHz,
        sample_rate:             Number(document.getElementById('sdr-sample_rate')?.value || 0),
        gain_mode:               gainAuto?.checked ? 'auto' : 'manual',
        tuner_gain_tenth_db:     Number(document.getElementById('sdr-tuner_gain_tenth_db')?.value || 0),
        rtl_agc:                 document.getElementById('sdr-rtl_agc')?.checked  ?? false,
        tuner_agc:               document.getElementById('sdr-tuner_agc')?.checked ?? false,
        ppm_correction:          Number(document.getElementById('sdr-ppm_correction')?.value || 0),
        direct_sampling:         document.getElementById('sdr-direct_sampling')?.value || 'off',
        offset_freq_hz:          Number(document.getElementById('sdr-offset_freq_hz')?.value || 0),
        offset_tuning:           document.getElementById('sdr-offset_tuning')?.checked           ?? false,
        dc_offset_correction:    document.getElementById('sdr-dc_offset_correction')?.checked    ?? false,
        iq_imbalance_correction: document.getElementById('sdr-iq_imbalance_correction')?.checked ?? false,
        invert_iq:               document.getElementById('sdr-invert_iq')?.checked               ?? false,
        bias_tee:                document.getElementById('sdr-bias_tee')?.checked                ?? false,
        max_total_users:         Number(document.getElementById('sdr-max_total_users')?.value || 1),
        hostname:                document.getElementById('sdr-hostname')?.value || '',
    };

    const ok = await apiSafe(() => apiPut('sdr/config', cfg));
    if (ok !== null) toast('SDR config saved', 'ok');
    if (btn) btn.disabled = false;
}

/* ══════════════════════════════════════════════════════════════
   Ethernet Tab
   ══════════════════════════════════════════════════════════════ */

async function loadEthConfig() {
    const d = await apiSafe(() => apiGet('eth/config'));
    if (!d) return;

    const bool = ['enable','dhcp','prefer_over_wifi'];
    bool.forEach(k => {
        const el = document.getElementById('eth-' + k);
        if (el && d[k] != null) el.checked = d[k];
    });

    const num = ['phy_addr','mdc_gpio','mdio_gpio'];
    num.forEach(k => {
        const el = document.getElementById('eth-' + k);
        if (el && d[k] != null) el.value = d[k];
    });

    const str = ['phy_type','static_ip','static_mask','static_gw','static_dns'];
    str.forEach(k => {
        const el = document.getElementById('eth-' + k);
        if (el && d[k] != null) el.value = d[k];
    });

    toggleDhcp();
}

function toggleDhcp() {
    const dhcp  = document.getElementById('eth-dhcp');
    const sf    = document.getElementById('eth-static-fields');
    const lbl   = document.getElementById('dhcp-mode-label');
    const isDhcp = dhcp ? dhcp.checked : true;
    if (sf) {
        sf.style.opacity        = isDhcp ? '0.35' : '1';
        sf.style.pointerEvents  = isDhcp ? 'none' : '';
        sf.style.transition     = 'opacity 0.2s ease';
    }
    if (lbl) lbl.textContent = isDhcp ? 'DHCP' : 'Static';
}

async function saveEthConfig() {
    const cfg = {
        enable:           document.getElementById('eth-enable')?.checked         ?? false,
        dhcp:             document.getElementById('eth-dhcp')?.checked           ?? true,
        phy_type:         document.getElementById('eth-phy_type')?.value         || 'LAN8720',
        phy_addr:         Number(document.getElementById('eth-phy_addr')?.value  || -1),
        mdc_gpio:         Number(document.getElementById('eth-mdc_gpio')?.value  || 31),
        mdio_gpio:        Number(document.getElementById('eth-mdio_gpio')?.value || 27),
        prefer_over_wifi: document.getElementById('eth-prefer_over_wifi')?.checked ?? false,
        static_ip:        document.getElementById('eth-static_ip')?.value        || '',
        static_mask:      document.getElementById('eth-static_mask')?.value      || '',
        static_gw:        document.getElementById('eth-static_gw')?.value        || '',
        static_dns:       document.getElementById('eth-static_dns')?.value       || '',
    };
    const ok = await apiSafe(() => apiPut('eth/config', cfg));
    if (ok !== null) toast('Ethernet config saved', 'ok');
}

/* ══════════════════════════════════════════════════════════════
   Notifications Tab
   ══════════════════════════════════════════════════════════════ */

function updateRateLabel(sliderId, labelId) {
    const val = document.getElementById(sliderId)?.value || '60';
    const el  = document.getElementById(labelId);
    if (el) el.textContent = val >= 60 ? Math.round(val/60) + 'm' : val + 's';
}

async function loadNotifyConfig() {
    const d = await apiSafe(() => apiGet('notify/config'));
    if (!d) return;

    const tg = d.telegram || {};
    const set = (id, v) => { const e = document.getElementById(id); if (e && v != null) e.value = v; };
    const chk = (id, v) => { const e = document.getElementById(id); if (e && v != null) e.checked = v; };

    chk('tg-enable',    tg.enable);
    set('tg-bot-token', tg.bot_token);
    set('tg-chat-id',   tg.chat_id);
    if (tg.rate_limit_s != null) {
        set('tg-rate-limit', tg.rate_limit_s);
        updateRateLabel('tg-rate-limit','tg-rate-val');
    }

    const dc = d.discord || {};
    chk('dc-enable',    dc.enable);
    set('dc-webhook',   dc.webhook_url);
    if (dc.rate_limit_s != null) {
        set('dc-rate-limit', dc.rate_limit_s);
        updateRateLabel('dc-rate-limit','dc-rate-val');
    }
}

async function saveNotifyConfig() {
    const cfg = {
        telegram: {
            enable:       document.getElementById('tg-enable')?.checked     ?? false,
            bot_token:    document.getElementById('tg-bot-token')?.value    || '',
            chat_id:      document.getElementById('tg-chat-id')?.value      || '',
            rate_limit_s: Number(document.getElementById('tg-rate-limit')?.value || 60),
        },
        discord: {
            enable:       document.getElementById('dc-enable')?.checked     ?? false,
            webhook_url:  document.getElementById('dc-webhook')?.value      || '',
            rate_limit_s: Number(document.getElementById('dc-rate-limit')?.value || 60),
        },
    };
    const ok = await apiSafe(() => apiPut('notify/config', cfg));
    if (ok !== null) toast('Notification config saved', 'ok');
}

async function testNotify(channel) {
    const btnId    = channel === 'telegram' ? 'tg-test-btn'    : 'dc-test-btn';
    const resultId = channel === 'telegram' ? 'tg-test-result' : 'dc-test-result';
    const btn    = document.getElementById(btnId);
    const result = document.getElementById(resultId);

    setButtonLoading(btn, true);
    if (result) { result.className = 'test-result'; result.textContent = ''; }

    try {
        const d = await apiPost('notify/test', { channel });
        const msg = d.message || 'Message sent';
        toast(msg, 'ok');
        if (result) { result.className = 'test-result ok visible'; result.textContent = msg; }
    } catch (e) {
        if (result) { result.className = 'test-result err visible'; result.textContent = e.message; }
    } finally {
        setButtonLoading(btn, false);
    }
}

/* ══════════════════════════════════════════════════════════════
   System Tab
   ══════════════════════════════════════════════════════════════ */

function fmtUptime(s) {
    const d = Math.floor(s / 86400);
    const h = Math.floor((s % 86400) / 3600);
    const m = Math.floor((s % 3600) / 60);
    if (d > 0) return `${d}d ${h}h ${m}m`;
    if (h > 0) return `${h}h ${m}m`;
    return `${m}m ${s % 60}s`;
}

function setMemBar(barId, pctId, textId, used, total, unitMB) {
    const pct = total > 0 ? Math.round((used / total) * 100) : 0;
    const bar = document.getElementById(barId);
    const pctEl = document.getElementById(pctId);
    const txt = document.getElementById(textId);

    if (bar) {
        bar.style.width = pct + '%';
        bar.classList.remove('warn','critical');
        if (pct >= 90) bar.classList.add('critical');
        else if (pct >= 75) bar.classList.add('warn');
    }
    if (pctEl) pctEl.textContent = pct + '%';
    if (txt) {
        const free = total - used;
        if (unitMB) txt.textContent = (free/1024/1024).toFixed(1) + ' MB free / ' + (total/1024/1024).toFixed(0) + ' MB';
        else        txt.textContent = (free/1024).toFixed(0) + ' KB free (min: ' + txt.dataset.min + ' KB)';
    }
}

async function loadSystemInfo() {
    const d = await apiSafe(() => apiGet('system/info'));
    if (!d) return;

    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set('sys-hostname', d.hostname);
    set('sys-version',  d.version);
    set('sys-cores',    d.cores);
    set('sys-uptime',   fmtUptime(d.uptime_s || 0));

    /* Heap: assume total ≈ free + used, use heap_min as a baseline estimate */
    /* API gives heap_free and heap_min; total heap estimated at 512KB for ESP32-P4 */
    const heapTotal = 512 * 1024;
    const heapUsed  = heapTotal - (d.heap_free || 0);
    const heapTxt   = document.getElementById('sys-heap-text');
    if (heapTxt) heapTxt.dataset.min = ((d.heap_min || 0) / 1024).toFixed(0);
    setMemBar('sys-heap-bar','sys-heap-pct','sys-heap-text', heapUsed, heapTotal, false);

    /* PSRAM: 32 MB on P4 */
    const psramTotal = 32 * 1024 * 1024;
    const psramFree  = d.psram_free || 0;
    const psramUsed  = psramTotal - psramFree;
    setMemBar('sys-psram-bar','sys-psram-pct','sys-psram-text', psramUsed, psramTotal, true);
}

async function rebootDevice() {
    if (!confirm('Reboot the device now?')) return;
    const ok = await apiSafe(() => apiPost('system/reboot', { confirm: true }));
    if (ok !== null) toast('Rebooting… reconnect in ~10 seconds', 'info');
}

function backupConfig() {
    window.location.href = '/api/system/backup';
    toast('Downloading config backup', 'info');
}

async function restoreConfig() {
    const input = document.getElementById('restore-file-input');
    input.onchange = async () => {
        const file = input.files[0];
        if (!file) return;
        try {
            const text = await file.text();
            const json = JSON.parse(text);
            const ok   = await apiSafe(() => apiPost('system/restore', json));
            if (ok !== null) toast('Config restored successfully', 'ok');
        } catch {
            toast('Invalid backup file', 'err');
        }
        input.value = '';
    };
    input.click();
}

async function factoryReset() {
    if (!confirm('FACTORY RESET: This will erase ALL settings including WiFi passwords and API keys.\n\nAre you sure?')) return;
    if (!confirm('Final confirmation: factory reset cannot be undone. Proceed?')) return;
    const ok = await apiSafe(() => apiPost('system/factory-reset', { confirm: true }));
    if (ok !== null) toast('Factory reset initiated. Device rebooting…', 'info');
}

/* ══════════════════════════════════════════════════════════════
   Init
   ══════════════════════════════════════════════════════════════ */
switchTab('wifi');
