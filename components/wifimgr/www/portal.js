'use strict';

/* ── Tab switching ──────────────────────────────────────────── */
document.querySelectorAll('.tabs button').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tabs button').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
        btn.classList.add('active');
        document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
        if (btn.dataset.tab === 'wifi') loadWifiStatus(), scanWifi();
        if (btn.dataset.tab === 'ethernet') loadEthConfig();
        if (btn.dataset.tab === 'sdr') loadSdrConfig();
        if (btn.dataset.tab === 'services') loadServices();
        if (btn.dataset.tab === 'notify') loadNotifyConfig();
        if (btn.dataset.tab === 'system') loadSystemInfo();
    });
});

/* ── Toast ──────────────────────────────────────────────────── */
function toast(msg, type) {
    const el = document.getElementById('toast');
    el.textContent = msg;
    el.className = 'toast show ' + (type || 'ok');
    clearTimeout(el._t);
    el._t = setTimeout(() => el.classList.remove('show'), 3000);
}

/* ── API helpers ────────────────────────────────────────────── */
async function api(path, opts) {
    try {
        const r = await fetch('/api/' + path, {
            headers: { 'Content-Type': 'application/json' },
            ...opts
        });
        const data = await r.json();
        if (!r.ok) throw new Error(data.message || r.statusText);
        return data;
    } catch (e) {
        toast(e.message, 'err');
        throw e;
    }
}
const apiGet = path => api(path);
const apiPost = (path, body) => api(path, { method: 'POST', body: JSON.stringify(body) });
const apiPut = (path, body) => api(path, { method: 'PUT', body: JSON.stringify(body) });
const apiDelete = path => api(path, { method: 'DELETE' });

/* ── WiFi Tab ───────────────────────────────────────────────── */
async function loadWifiStatus() {
    const d = await apiGet('wifi/status');
    const el = document.getElementById('wifi-status');
    el.innerHTML = `
        <span class="status-dot ${d.connected ? 'green' : 'red'}"></span>
        <strong>${d.state}</strong>
        ${d.connected ? ` — ${d.ssid} (${d.ip}, RSSI: ${d.rssi} dBm)` : ''}
        <span style="float:right;color:var(--text2)">${d.mode}</span>`;
}

async function scanWifi() {
    document.getElementById('wifi-scan-btn').disabled = true;
    try {
        const d = await apiGet('wifi/scan');
        const list = document.getElementById('wifi-list');
        list.innerHTML = '';
        d.networks.forEach(n => {
            const pct = Math.min(100, Math.max(0, (n.rssi + 90) * 1.5));
            list.innerHTML += `
                <div class="wifi-item ${n.saved ? 'saved' : ''}">
                    <span class="ssid">${esc(n.ssid)}</span>
                    <span class="meta">${n.auth} ch${n.channel}</span>
                    <span class="rssi-bar"><span class="fill" style="width:${pct}%"></span></span>
                    <span class="meta">${n.rssi}dBm</span>
                    ${n.saved ? '<span class="status-dot green" title="Saved"></span>' :
                      `<button class="btn btn-sm btn-primary" onclick="addWifi('${esc(n.ssid)}')">Add</button>`}
                </div>`;
        });
    } finally {
        document.getElementById('wifi-scan-btn').disabled = false;
    }
}

async function loadSavedNetworks() {
    const d = await apiGet('wifi/networks');
    const el = document.getElementById('saved-networks');
    el.innerHTML = '';
    d.networks.forEach(n => {
        el.innerHTML += `
            <div class="wifi-item saved">
                <span class="ssid">${esc(n.ssid)}</span>
                <button class="btn btn-sm btn-danger" onclick="deleteWifi('${esc(n.ssid)}')">Remove</button>
            </div>`;
    });
}

function addWifi(ssid) {
    const pass = prompt('Password for ' + ssid + ':');
    if (pass === null) return;
    apiPost('wifi/networks', { ssid, password: pass }).then(() => {
        toast('Network saved');
        loadSavedNetworks();
    });
}

function deleteWifi(ssid) {
    if (!confirm('Remove ' + ssid + '?')) return;
    apiDelete('wifi/networks?ssid=' + encodeURIComponent(ssid)).then(() => {
        toast('Network removed');
        loadSavedNetworks();
    });
}

function testConnect() {
    const ssid = document.getElementById('test-ssid').value;
    const pass = document.getElementById('test-pass').value;
    if (!ssid) return toast('Enter SSID', 'err');
    toast('Connecting...');
    apiPost('wifi/connect', { ssid, password: pass }).then(d => toast(d.message));
}

/* ── Ethernet Tab ───────────────────────────────────────────── */
async function loadEthConfig() {
    const d = await apiGet('eth/config');
    for (const [k, v] of Object.entries(d)) {
        const el = document.getElementById('eth-' + k);
        if (!el) continue;
        if (el.type === 'checkbox') el.checked = v;
        else el.value = v;
    }
    toggleDhcp();
}

function toggleDhcp() {
    const dhcp = document.getElementById('eth-dhcp').checked;
    document.querySelectorAll('.eth-static').forEach(el =>
        el.style.display = dhcp ? 'none' : 'block');
}

function saveEthConfig() {
    const cfg = {};
    document.querySelectorAll('#tab-ethernet [id^=eth-]').forEach(el => {
        const key = el.id.replace('eth-', '');
        cfg[key] = el.type === 'checkbox' ? el.checked :
                   el.type === 'number' ? Number(el.value) : el.value;
    });
    apiPut('eth/config', cfg).then(() => toast('Ethernet config saved'));
}

/* ── SDR Tab ────────────────────────────────────────────────── */
async function loadSdrConfig() {
    const d = await apiGet('sdr/config');
    for (const [k, v] of Object.entries(d)) {
        const el = document.getElementById('sdr-' + k);
        if (!el) continue;
        if (el.type === 'checkbox') el.checked = v;
        else el.value = v;
    }
}

function saveSdrConfig() {
    const cfg = {};
    document.querySelectorAll('#tab-sdr [id^=sdr-]').forEach(el => {
        const key = el.id.replace('sdr-', '');
        cfg[key] = el.type === 'checkbox' ? el.checked :
                   el.type === 'number' ? Number(el.value) : el.value;
    });
    apiPut('sdr/config', cfg).then(() => toast('SDR config saved'));
}

/* ── Services Tab ───────────────────────────────────────────── */
async function loadServices() {
    const d = await apiGet('services');
    const el = document.getElementById('services-list');
    el.innerHTML = '';
    for (const [name, cfg] of Object.entries(d)) {
        const on = cfg.enable;
        el.innerHTML += `
            <div class="svc-header" onclick="toggleSvc(this)">
                <span class="name">${esc(name)}</span>
                ${cfg.port ? `<span class="meta">:${cfg.port}</span>` : ''}
                <span class="status ${on ? 'on' : 'off'}">${on ? 'ON' : 'OFF'}</span>
            </div>
            <div class="svc-body" data-svc="${esc(name)}">
                <pre style="font-size:12px;color:var(--text2);max-height:200px;overflow:auto">${JSON.stringify(cfg, null, 2)}</pre>
                <div class="field" style="margin-top:8px">
                    <label>Edit JSON config</label>
                    <textarea rows="6" id="svc-edit-${esc(name)}" style="font-family:monospace;font-size:12px">${JSON.stringify(cfg, null, 2)}</textarea>
                </div>
                <button class="btn btn-primary btn-sm" onclick="saveSvc('${esc(name)}')">Save</button>
            </div>`;
    }
}

function toggleSvc(header) {
    const body = header.nextElementSibling;
    body.classList.toggle('open');
}

function saveSvc(name) {
    try {
        const json = JSON.parse(document.getElementById('svc-edit-' + name).value);
        apiPut('services/' + name, json).then(() => toast(name + ' config saved'));
    } catch (e) {
        toast('Invalid JSON', 'err');
    }
}

/* ── Notifications Tab ──────────────────────────────────────── */
async function loadNotifyConfig() {
    const d = await apiGet('notify/config');
    document.getElementById('tg-enable').checked = d.telegram.enable;
    document.getElementById('tg-bot-token').value = d.telegram.bot_token;
    document.getElementById('tg-chat-id').value = d.telegram.chat_id;
    document.getElementById('tg-rate-limit').value = d.telegram.rate_limit_s;
    document.getElementById('dc-enable').checked = d.discord.enable;
    document.getElementById('dc-webhook').value = d.discord.webhook_url;
    document.getElementById('dc-rate-limit').value = d.discord.rate_limit_s;
}

function saveNotifyConfig() {
    const cfg = {
        telegram: {
            enable: document.getElementById('tg-enable').checked,
            bot_token: document.getElementById('tg-bot-token').value,
            chat_id: document.getElementById('tg-chat-id').value,
            rate_limit_s: Number(document.getElementById('tg-rate-limit').value)
        },
        discord: {
            enable: document.getElementById('dc-enable').checked,
            webhook_url: document.getElementById('dc-webhook').value,
            rate_limit_s: Number(document.getElementById('dc-rate-limit').value)
        }
    };
    apiPut('notify/config', cfg).then(() => toast('Notification config saved'));
}

function testNotify(channel) {
    apiPost('notify/test', { channel }).then(d => toast(d.message));
}

/* ── System Tab ─────────────────────────────────────────────── */
async function loadSystemInfo() {
    const d = await apiGet('system/info');
    const h = Math.floor(d.uptime_s / 3600);
    const m = Math.floor((d.uptime_s % 3600) / 60);
    document.getElementById('sys-uptime').textContent = `${h}h ${m}m`;
    document.getElementById('sys-version').textContent = d.version;
    document.getElementById('sys-hostname').textContent = d.hostname;
    document.getElementById('sys-cores').textContent = d.cores;

    const heapPct = ((1 - d.heap_free / (d.heap_free + 100000)) * 100).toFixed(0);
    document.getElementById('sys-heap-bar').style.width = heapPct + '%';
    document.getElementById('sys-heap-text').textContent =
        (d.heap_free / 1024).toFixed(0) + ' KB free (min: ' + (d.heap_min / 1024).toFixed(0) + ' KB)';

    const psramTotal = 32 * 1024 * 1024;
    const psramPct = ((1 - d.psram_free / psramTotal) * 100).toFixed(0);
    document.getElementById('sys-psram-bar').style.width = psramPct + '%';
    document.getElementById('sys-psram-text').textContent =
        (d.psram_free / 1024 / 1024).toFixed(1) + ' MB free / 32 MB';
}

function rebootDevice() {
    if (!confirm('Reboot the device?')) return;
    apiPost('system/reboot', { confirm: true }).then(() => toast('Rebooting...'));
}

function backupConfig() {
    window.location.href = '/api/system/backup';
}

async function restoreConfig() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = async () => {
        const file = input.files[0];
        if (!file) return;
        try {
            const text = await file.text();
            const json = JSON.parse(text);
            await apiPost('system/restore', json);
            toast('Config restored');
        } catch (e) {
            toast('Invalid backup file', 'err');
        }
    };
    input.click();
}

function factoryReset() {
    if (!confirm('FACTORY RESET: Erase ALL config including WiFi passwords and API keys?')) return;
    if (!confirm('Are you sure? This cannot be undone.')) return;
    apiPost('system/factory-reset', { confirm: true }).then(() => toast('Factory reset. Rebooting...'));
}

/* ── Utilities ──────────────────────────────────────────────── */
function esc(s) {
    return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
}

/* ── Init ───────────────────────────────────────────────────── */
document.querySelector('.tabs button').click();
