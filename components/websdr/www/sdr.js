/* ESP32-P4 WebSDR Client
 * All-in-one: WebSocket, waterfall, spectrum, audio demodulation.
 * No external dependencies.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

(function () {
    'use strict';

    /* ── State ── */
    var ws = null;
    var centerFreq = 100000000;
    var sampleRate = 1024000;
    var fftSize = 1024;
    var currentGain = 0;
    var currentMode = 'WBFM';
    var volume = 0.5;
    var squelch = 0;
    var audioRunning = false;
    var iqSubscribed = false;

    /* ── DOM Elements ── */
    var elStatus = document.getElementById('status');
    var elFreqDisplay = document.getElementById('freq-display');
    var elRateDisplay = document.getElementById('rate-display');
    var elSmeter = document.getElementById('smeter');
    var specCanvas = document.getElementById('spectrum');
    var wfCanvas = document.getElementById('waterfall');
    var elFreqInput = document.getElementById('freq-input');
    var elGainSlider = document.getElementById('gain-slider');
    var elGainVal = document.getElementById('gain-val');
    var elVolSlider = document.getElementById('vol-slider');
    var elVolVal = document.getElementById('vol-val');
    var elSqlSlider = document.getElementById('sql-slider');
    var elSqlVal = document.getElementById('sql-val');
    var elAudioBtn = document.getElementById('audio-btn');

    /* ── Canvas Contexts ── */
    var specCtx = specCanvas.getContext('2d');
    var wfCtx = wfCanvas.getContext('2d');
    var wfImageData = null;

    /* ── Waterfall Color Map (jet-like) ── */
    var colorMap = new Array(256);
    (function buildColorMap() {
        for (var i = 0; i < 256; i++) {
            var t = i / 255;
            var r, g, b;
            if (t < 0.25) {
                r = 0; g = Math.round(t / 0.25 * 255); b = 255;
            } else if (t < 0.5) {
                r = 0; g = 255; b = Math.round((0.5 - t) / 0.25 * 255);
            } else if (t < 0.75) {
                r = Math.round((t - 0.5) / 0.25 * 255); g = 255; b = 0;
            } else {
                r = 255; g = Math.round((1.0 - t) / 0.25 * 255); b = 0;
            }
            colorMap[i] = [r, g, b];
        }
    })();

    /* ── Latest FFT data ── */
    var lastFFT = null;
    var peakHold = null;

    /* ── Audio State ── */
    var audioCtx = null;
    var scriptNode = null;
    var audioQueue = [];
    var prevI = 0, prevQ = 0;

    /* ── Resize Canvases ── */
    function resizeCanvases() {
        var parent = document.getElementById('display');
        var w = parent.clientWidth;

        specCanvas.width = w;
        specCanvas.height = specCanvas.clientHeight;

        wfCanvas.width = w;
        wfCanvas.height = wfCanvas.clientHeight;
        wfImageData = null;
    }
    window.addEventListener('resize', resizeCanvases);

    /* ── Format Frequency ── */
    function formatFreq(hz) {
        if (hz >= 1e9) return (hz / 1e9).toFixed(6) + ' GHz';
        if (hz >= 1e6) return (hz / 1e6).toFixed(3) + ' MHz';
        if (hz >= 1e3) return (hz / 1e3).toFixed(1) + ' kHz';
        return hz + ' Hz';
    }

    function updateDisplay() {
        elFreqDisplay.textContent = formatFreq(centerFreq);
        elRateDisplay.textContent = (sampleRate / 1000).toFixed(0) + ' kSPS';
    }

    /* ── WebSocket ── */
    function wsConnect() {
        var host = location.hostname || '192.168.1.232';
        var port = location.port || '8080';
        var url = 'ws://' + host + ':' + port + '/ws';

        ws = new WebSocket(url);
        ws.binaryType = 'arraybuffer';

        ws.onopen = function () {
            elStatus.textContent = 'Connected';
            elStatus.className = 'connected';
        };

        ws.onclose = function () {
            elStatus.textContent = 'Disconnected';
            elStatus.className = 'disconnected';
            iqSubscribed = false;
            setTimeout(wsConnect, 2000);
        };

        ws.onerror = function () {
            ws.close();
        };

        ws.onmessage = function (evt) {
            if (typeof evt.data === 'string') {
                handleTextMessage(evt.data);
            } else {
                handleBinaryMessage(evt.data);
            }
        };
    }

    function handleTextMessage(text) {
        var msg;
        try { msg = JSON.parse(text); } catch (e) { return; }

        if (msg.type === 'info') {
            centerFreq = msg.freq || centerFreq;
            sampleRate = msg.rate || sampleRate;
            currentGain = msg.gain || 0;
            fftSize = msg.fft_size || 1024;
            elFreqInput.value = (centerFreq / 1e6).toFixed(3);
            peakHold = null;
            updateDisplay();
        } else if (msg.type === 'freq') {
            centerFreq = msg.value;
            elFreqInput.value = (centerFreq / 1e6).toFixed(3);
            peakHold = null;
            updateDisplay();
        } else if (msg.type === 'iq_start') {
            iqSubscribed = true;
        } else if (msg.type === 'iq_stop') {
            iqSubscribed = false;
        }
    }

    function handleBinaryMessage(buf) {
        var data = new Uint8Array(buf);
        if (data.length < 2) return;

        var type = data[0];
        var payload = data.subarray(1);

        if (type === 0x01) {
            /* FFT data */
            lastFFT = payload;
            drawSpectrum(payload);
            drawWaterfall(payload);
            updateSmeter(payload);
        } else if (type === 0x02) {
            /* IQ data for audio */
            if (audioRunning) {
                audioQueue.push(payload.slice());
            }
        }
    }

    /* ── Spectrum Plot ── */
    function drawSpectrum(fft) {
        var w = specCanvas.width;
        var h = specCanvas.height;
        var ctx = specCtx;

        ctx.fillStyle = '#0a0a0a';
        ctx.fillRect(0, 0, w, h);

        var len = fft.length;
        var xScale = w / len;

        /* Grid lines */
        ctx.strokeStyle = '#222';
        ctx.lineWidth = 1;
        for (var db = 0; db <= 100; db += 20) {
            var gy = h - (db / 100) * h;
            ctx.beginPath();
            ctx.moveTo(0, gy);
            ctx.lineTo(w, gy);
            ctx.stroke();
        }

        /* Spectrum line */
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (var i = 0; i < len; i++) {
            var x = i * xScale;
            var y = h - (fft[i] / 255) * h;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        /* Peak hold */
        if (!peakHold || peakHold.length !== len) {
            peakHold = new Uint8Array(len);
        }
        for (var i = 0; i < len; i++) {
            if (fft[i] > peakHold[i]) peakHold[i] = fft[i];
            else if (peakHold[i] > 0) peakHold[i]--;
        }
        ctx.strokeStyle = 'rgba(255,100,100,0.5)';
        ctx.beginPath();
        for (var i = 0; i < len; i++) {
            var x = i * xScale;
            var y = h - (peakHold[i] / 255) * h;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        /* Frequency labels */
        ctx.fillStyle = '#888';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';
        var startFreq = centerFreq - sampleRate / 2;
        for (var p = 0; p <= 4; p++) {
            var fx = (p / 4) * w;
            var fHz = startFreq + (p / 4) * sampleRate;
            ctx.fillText(formatFreq(fHz), fx, h - 2);
        }

        /* dB labels */
        ctx.textAlign = 'left';
        ctx.fillStyle = '#666';
        for (var db = 0; db <= 100; db += 20) {
            var gy = h - (db / 100) * h;
            ctx.fillText(-100 + db + ' dB', 2, gy - 2);
        }
    }

    /* ── Waterfall ── */
    function drawWaterfall(fft) {
        var w = wfCanvas.width;
        var h = wfCanvas.height;
        var ctx = wfCtx;

        if (!wfImageData || wfImageData.width !== w || wfImageData.height !== h) {
            wfImageData = ctx.createImageData(w, h);
            /* Fill black */
            var d = wfImageData.data;
            for (var i = 3; i < d.length; i += 4) d[i] = 255;
        }

        var d = wfImageData.data;
        var stride = w * 4;

        /* Scroll down: copy rows */
        d.copyWithin(stride, 0, (h - 1) * stride);

        /* Draw new line at top */
        var len = fft.length;
        for (var x = 0; x < w; x++) {
            var bin = Math.floor(x * len / w);
            if (bin >= len) bin = len - 1;
            var val = fft[bin];
            var c = colorMap[val];
            var idx = x * 4;
            d[idx] = c[0];
            d[idx + 1] = c[1];
            d[idx + 2] = c[2];
            d[idx + 3] = 255;
        }

        ctx.putImageData(wfImageData, 0, 0);
    }

    /* ── S-Meter ── */
    function updateSmeter(fft) {
        /* Average power across center bins */
        var center = Math.floor(fft.length / 2);
        var span = 20;
        var sum = 0;
        for (var i = center - span; i <= center + span; i++) {
            if (i >= 0 && i < fft.length) sum += fft[i];
        }
        var avg = sum / (2 * span + 1);
        /* Map 0-255 to S0-S9+60 */
        var sVal;
        if (avg < 80) sVal = 'S0';
        else if (avg < 100) sVal = 'S1';
        else if (avg < 115) sVal = 'S3';
        else if (avg < 130) sVal = 'S5';
        else if (avg < 150) sVal = 'S7';
        else if (avg < 170) sVal = 'S9';
        else if (avg < 200) sVal = 'S9+20';
        else if (avg < 230) sVal = 'S9+40';
        else sVal = 'S9+60';
        elSmeter.textContent = sVal;
    }

    /* ── Audio Demodulation ── */
    function getModeParams() {
        switch (currentMode) {
            case 'NFM':  return { bw: 12500, offset: 0 };
            case 'AM':   return { bw: 10000, offset: 0 };
            case 'WBFM': return { bw: 150000, offset: 0 };
            case 'USB':  return { bw: 3000, offset: 1500 };
            case 'LSB':  return { bw: 3000, offset: -1500 };
            default:     return { bw: 25000, offset: 0 };
        }
    }

    function demodFM(iq) {
        /* FM demodulation via arctan differentiation */
        var samples = [];
        for (var n = 0; n < iq.length - 3; n += 2) {
            var i0 = (iq[n] - 127.5) / 127.5;
            var q0 = (iq[n + 1] - 127.5) / 127.5;
            var i1 = prevI, q1 = prevQ;

            /* Phase difference: atan2(q*i_prev - i*q_prev, i*i_prev + q*q_prev) */
            var dot = i0 * i1 + q0 * q1;
            var cross = q0 * i1 - i0 * q1;
            var angle = Math.atan2(cross, dot);
            samples.push(angle / Math.PI);

            prevI = i0;
            prevQ = q0;
        }
        return samples;
    }

    function demodAM(iq) {
        var samples = [];
        var dcAvg = 0;
        for (var n = 0; n < iq.length - 1; n += 2) {
            var i = (iq[n] - 127.5) / 127.5;
            var q = (iq[n + 1] - 127.5) / 127.5;
            var mag = Math.sqrt(i * i + q * q);
            dcAvg = dcAvg * 0.999 + mag * 0.001;
            samples.push(mag - dcAvg);
        }
        return samples;
    }

    function demodSSB(iq, upper) {
        /* Simple SSB: just take I channel (assumes DDC has shifted appropriately) */
        var samples = [];
        for (var n = 0; n < iq.length - 1; n += 2) {
            var i = (iq[n] - 127.5) / 127.5;
            var q = (iq[n + 1] - 127.5) / 127.5;
            /* Phasing method: USB = I+Q, LSB = I-Q (simplified) */
            if (upper) {
                samples.push((i + q) * 0.5);
            } else {
                samples.push((i - q) * 0.5);
            }
        }
        return samples;
    }

    function demodulate(iq) {
        switch (currentMode) {
            case 'NFM':
            case 'WBFM':
                return demodFM(iq);
            case 'AM':
                return demodAM(iq);
            case 'USB':
                return demodSSB(iq, true);
            case 'LSB':
                return demodSSB(iq, false);
            default:
                return demodFM(iq);
        }
    }

    function startAudio() {
        if (audioCtx) return;

        audioCtx = new (window.AudioContext || window.webkitAudioContext)({
            sampleRate: 48000
        });

        /* Use ScriptProcessorNode for broad compatibility */
        scriptNode = audioCtx.createScriptProcessor(4096, 1, 1);
        var gainNode = audioCtx.createGain();
        gainNode.gain.value = volume;

        scriptNode.onaudioprocess = function (e) {
            var output = e.outputBuffer.getChannelData(0);
            var pos = 0;

            while (pos < output.length && audioQueue.length > 0) {
                var iq = audioQueue[0];
                var samples = demodulate(iq);
                audioQueue.shift();

                /* Check squelch */
                var power = 0;
                for (var s = 0; s < samples.length; s++) power += samples[s] * samples[s];
                power = power / (samples.length || 1);

                if (squelch > 0 && power < (squelch / 255) * 0.01) {
                    /* Below squelch - output silence */
                    for (var s = 0; s < samples.length && pos < output.length; s++, pos++) {
                        output[pos] = 0;
                    }
                } else {
                    /* Resample: DDC output rate -> 48000 Hz */
                    for (var s = 0; s < samples.length && pos < output.length; s++, pos++) {
                        output[pos] = samples[s] * 0.8;
                    }
                }
            }

            /* Fill remaining with silence */
            while (pos < output.length) output[pos++] = 0;
        };

        scriptNode.connect(gainNode);
        gainNode.connect(audioCtx.destination);

        /* Store gain node for volume control */
        scriptNode._gainNode = gainNode;

        /* Subscribe to IQ from server */
        var params = getModeParams();
        sendCmd('subscribe_iq', { offset: params.offset, bw: params.bw });

        audioRunning = true;
        elAudioBtn.textContent = 'Stop Audio';
        elAudioBtn.classList.add('active');
    }

    function stopAudio() {
        audioRunning = false;
        iqSubscribed = false;

        if (ws && ws.readyState === WebSocket.OPEN) {
            sendCmd('unsubscribe_iq', {});
        }

        if (scriptNode) {
            scriptNode.disconnect();
            if (scriptNode._gainNode) scriptNode._gainNode.disconnect();
            scriptNode = null;
        }
        if (audioCtx) {
            audioCtx.close();
            audioCtx = null;
        }
        audioQueue = [];
        prevI = 0;
        prevQ = 0;

        elAudioBtn.textContent = 'Start Audio';
        elAudioBtn.classList.remove('active');
    }

    /* ── Commands ── */
    function sendCmd(cmd, params) {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        var msg = { cmd: cmd };
        if (params) {
            for (var k in params) msg[k] = params[k];
        }
        ws.send(JSON.stringify(msg));
    }

    /* ── Canvas Click → Tune ── */
    function onCanvasClick(e) {
        var rect = e.target.getBoundingClientRect();
        var x = e.clientX - rect.left;
        var frac = x / rect.width;
        var newFreq = Math.round(centerFreq - sampleRate / 2 + frac * sampleRate);
        elFreqInput.value = (newFreq / 1e6).toFixed(3);
        /* When audio is running, retune the DDC offset instead of center freq */
        if (audioRunning) {
            var offset = newFreq - centerFreq;
            var params = getModeParams();
            sendCmd('subscribe_iq', { offset: offset, bw: params.bw });
        }
    }

    /* ── Event Handlers ── */
    document.getElementById('freq-set').addEventListener('click', function () {
        var mhz = parseFloat(elFreqInput.value);
        if (isNaN(mhz)) return;
        var hz = Math.round(mhz * 1e6);
        sendCmd('freq', { value: hz });
    });

    elFreqInput.addEventListener('keydown', function (e) {
        if (e.key === 'Enter') document.getElementById('freq-set').click();
    });

    /* Mode buttons */
    var modeButtons = document.querySelectorAll('.mode-btn');
    modeButtons.forEach(function (btn) {
        btn.addEventListener('click', function () {
            modeButtons.forEach(function (b) { b.classList.remove('active'); });
            btn.classList.add('active');
            currentMode = btn.getAttribute('data-mode');
            /* Re-subscribe with new mode params if audio active */
            if (audioRunning) {
                var params = getModeParams();
                sendCmd('subscribe_iq', { offset: params.offset, bw: params.bw });
            }
        });
    });

    elGainSlider.addEventListener('input', function () {
        currentGain = parseInt(this.value);
        elGainVal.textContent = currentGain === 0 ? 'Auto' : (currentGain / 10).toFixed(1) + ' dB';
        sendCmd('gain', { value: currentGain });
    });

    elVolSlider.addEventListener('input', function () {
        volume = parseInt(this.value) / 100;
        elVolVal.textContent = Math.round(volume * 100) + '%';
        if (scriptNode && scriptNode._gainNode) {
            scriptNode._gainNode.gain.value = volume;
        }
    });

    elSqlSlider.addEventListener('input', function () {
        squelch = parseInt(this.value);
        elSqlVal.textContent = squelch;
    });

    elAudioBtn.addEventListener('click', function () {
        if (audioRunning) stopAudio(); else startAudio();
    });

    specCanvas.addEventListener('click', onCanvasClick);
    wfCanvas.addEventListener('click', onCanvasClick);

    /* ── Mouse wheel → frequency fine tune ── */
    specCanvas.addEventListener('wheel', function (e) {
        e.preventDefault();
        var step = sampleRate / 100;
        var delta = e.deltaY > 0 ? -step : step;
        var newFreq = centerFreq + delta;
        sendCmd('freq', { value: Math.round(newFreq) });
    });
    wfCanvas.addEventListener('wheel', function (e) {
        e.preventDefault();
        var step = sampleRate / 100;
        var delta = e.deltaY > 0 ? -step : step;
        var newFreq = centerFreq + delta;
        sendCmd('freq', { value: Math.round(newFreq) });
    });

    /* ── Init ── */
    resizeCanvases();
    updateDisplay();
    wsConnect();

})();
