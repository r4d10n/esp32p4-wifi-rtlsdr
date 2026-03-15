/* ESP32-P4 WebSDR Client
 * All-in-one: WebSocket, waterfall, spectrum, audio demodulation.
 * No external dependencies.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

(function () {
    'use strict';

    /* -- State -- */
    var ws = null;
    var centerFreq = 100000000;
    var sampleRate = 1024000;
    var fftSize = 1024;
    var dbMin = -40, dbMax = 40;
    var currentGain = 0;
    var currentMode = 'WBFM';
    var volume = 0.5;
    var squelch = 0;
    var audioRunning = false;
    var iqSubscribed = false;

    /* Zoom state */
    var zoomLevel = 1;
    var zoomCenter = 0.5; /* fraction 0..1 of FFT bins */

    /* Waterfall speed (ms between draws, derived from slider) */
    var wfSpeed = 50;
    var wfAccum = null; /* latest FFT held for throttled draw */
    var wfLastDraw = 0;

    /* -- DOM Elements -- */
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
    var elRateSelect = document.getElementById('rate-select');
    var elFftSelect = document.getElementById('fft-select');
    var elRangeSlider = document.getElementById('range-slider');
    var elRangeVal = document.getElementById('range-val');
    var elRefSlider = document.getElementById('ref-slider');
    var elRefVal = document.getElementById('ref-val');
    var elZoomIn = document.getElementById('zoom-in');
    var elZoomOut = document.getElementById('zoom-out');
    var elZoomReset = document.getElementById('zoom-reset');
    var elZoomVal = document.getElementById('zoom-val');
    var elWfSpeed = document.getElementById('wf-speed');
    var elWfSpeedVal = document.getElementById('wf-speed-val');

    /* -- Canvas Contexts -- */
    var specCtx = specCanvas.getContext('2d');
    var wfCtx = wfCanvas.getContext('2d');
    var wfImageData = null;

    /* -- Waterfall Color Map (jet-like) -- */
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

    /* -- Latest FFT data -- */
    var lastFFT = null;
    var peakHold = null;

    /* -- Audio State -- */
    var audioCtx = null;
    var workletNode = null;
    var audioGainNode = null;
    var audioQueue = [];
    var prevI = 0, prevQ = 0;
    /* Fallback flag for browsers without AudioWorklet */
    var useScriptProcessor = false;
    var scriptNode = null;

    /* -- Resize Canvases -- */
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

    /* -- Format Frequency -- */
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

    /* -- Zoom helpers -- */
    function getVisibleBinRange(len) {
        var visibleFrac = 1.0 / zoomLevel;
        var halfVisible = visibleFrac / 2;
        var start = zoomCenter - halfVisible;
        var end = zoomCenter + halfVisible;
        if (start < 0) { end -= start; start = 0; }
        if (end > 1) { start -= (end - 1); end = 1; }
        if (start < 0) start = 0;
        var binStart = Math.floor(start * len);
        var binEnd = Math.ceil(end * len);
        if (binEnd > len) binEnd = len;
        return { start: binStart, end: binEnd };
    }

    /* -- WebSocket -- */
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
            fftSize = msg.fft_size || fftSize;
            if (msg.db_min != null) dbMin = msg.db_min;
            if (msg.db_max != null) dbMax = msg.db_max;
            elFreqInput.value = (centerFreq / 1e6).toFixed(3);
            peakHold = null;
            syncControlsFromState();
            updateDisplay();
        } else if (msg.type === 'freq') {
            centerFreq = msg.value;
            elFreqInput.value = (centerFreq / 1e6).toFixed(3);
            peakHold = null;
            updateDisplay();
        } else if (msg.type === 'config') {
            fftSize = msg.fft_size || fftSize;
            sampleRate = msg.sample_rate || sampleRate;
            if (msg.db_min != null) dbMin = msg.db_min;
            if (msg.db_max != null) dbMax = msg.db_max;
            peakHold = null;
            wfImageData = null;
            syncControlsFromState();
            updateDisplay();
        } else if (msg.type === 'iq_start') {
            iqSubscribed = true;
        } else if (msg.type === 'iq_stop') {
            iqSubscribed = false;
        }
    }

    function syncControlsFromState() {
        elRateSelect.value = String(sampleRate);
        elFftSelect.value = String(fftSize);
        var range = dbMax - dbMin;
        elRangeSlider.value = range;
        elRangeVal.textContent = Math.round(range);
        elRefSlider.value = dbMax;
        elRefVal.textContent = Math.round(dbMax);
    }

    function handleBinaryMessage(buf) {
        var data = new Uint8Array(buf);
        if (data.length < 2) return;

        var type = data[0];
        var payload = data.subarray(1);

        if (type === 0x01) {
            lastFFT = payload;
            drawSpectrum(payload);
            /* Throttle waterfall by wfSpeed */
            wfAccum = payload;
            var now = performance.now();
            if (now - wfLastDraw >= wfSpeed) {
                drawWaterfall(wfAccum);
                wfLastDraw = now;
                wfAccum = null;
            }
            updateSmeter(payload);
        } else if (type === 0x02) {
            if (audioRunning) {
                audioQueue.push(payload.slice());
            }
        }
    }

    /* -- Spectrum Plot -- */
    function drawSpectrum(fft) {
        var w = specCanvas.width;
        var h = specCanvas.height;
        var ctx = specCtx;
        var len = fft.length;
        var range = getVisibleBinRange(len);
        var visLen = range.end - range.start;

        ctx.fillStyle = '#0a0a0a';
        ctx.fillRect(0, 0, w, h);

        var xScale = w / visLen;

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
        for (var i = 0; i < visLen; i++) {
            var x = i * xScale;
            var y = h - (fft[range.start + i] / 255) * h;
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
        for (var i = 0; i < visLen; i++) {
            var x = i * xScale;
            var y = h - (peakHold[range.start + i] / 255) * h;
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
            var binFrac = range.start / len + (p / 4) * (visLen / len);
            var fHz = startFreq + binFrac * sampleRate;
            ctx.fillText(formatFreq(fHz), fx, h - 2);
        }

        /* dB labels */
        ctx.textAlign = 'left';
        ctx.fillStyle = '#666';
        var dbRange = dbMax - dbMin;
        for (var db = 0; db <= 100; db += 20) {
            var gy = h - (db / 100) * h;
            var dbLabel = dbMin + (db / 100) * dbRange;
            ctx.fillText(dbLabel.toFixed(0) + ' dB', 2, gy - 2);
        }
    }

    /* -- Waterfall -- */
    function drawWaterfall(fft) {
        var w = wfCanvas.width;
        var h = wfCanvas.height;
        var ctx = wfCtx;
        var len = fft.length;
        var range = getVisibleBinRange(len);
        var visLen = range.end - range.start;

        if (!wfImageData || wfImageData.width !== w || wfImageData.height !== h) {
            wfImageData = ctx.createImageData(w, h);
            var d = wfImageData.data;
            for (var i = 3; i < d.length; i += 4) d[i] = 255;
        }

        var d = wfImageData.data;
        var stride = w * 4;

        /* Scroll down: copy rows */
        d.copyWithin(stride, 0, (h - 1) * stride);

        /* Draw new line at top */
        for (var x = 0; x < w; x++) {
            var bin = range.start + Math.floor(x * visLen / w);
            if (bin >= range.end) bin = range.end - 1;
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

    /* -- S-Meter -- */
    function updateSmeter(fft) {
        var center = Math.floor(fft.length / 2);
        var span = 20;
        var sum = 0;
        for (var i = center - span; i <= center + span; i++) {
            if (i >= 0 && i < fft.length) sum += fft[i];
        }
        var avg = sum / (2 * span + 1);
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

    /* -- Audio Demodulation -- */
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
        var samples = [];
        for (var n = 0; n < iq.length - 3; n += 2) {
            var i0 = (iq[n] - 127.5) / 127.5;
            var q0 = (iq[n + 1] - 127.5) / 127.5;
            var i1 = prevI, q1 = prevQ;
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
        var samples = [];
        for (var n = 0; n < iq.length - 1; n += 2) {
            var i = (iq[n] - 127.5) / 127.5;
            var q = (iq[n + 1] - 127.5) / 127.5;
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

    function processAudioBlock(output) {
        var pos = 0;
        while (pos < output.length && audioQueue.length > 0) {
            var iq = audioQueue[0];
            var samples = demodulate(iq);
            audioQueue.shift();

            var power = 0;
            for (var s = 0; s < samples.length; s++) power += samples[s] * samples[s];
            power = power / (samples.length || 1);

            if (squelch > 0 && power < (squelch / 255) * 0.01) {
                for (var s = 0; s < samples.length && pos < output.length; s++, pos++) {
                    output[pos] = 0;
                }
            } else {
                for (var s = 0; s < samples.length && pos < output.length; s++, pos++) {
                    output[pos] = samples[s] * 0.8;
                }
            }
        }
        while (pos < output.length) output[pos++] = 0;
    }

    function startAudioWorklet() {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)({
            sampleRate: 48000
        });

        audioGainNode = audioCtx.createGain();
        audioGainNode.gain.value = volume;
        audioGainNode.connect(audioCtx.destination);

        /* Try AudioWorklet first, fall back to ScriptProcessor */
        if (audioCtx.audioWorklet) {
            var workletCode =
                'class SDRProcessor extends AudioWorkletProcessor {\n' +
                '  constructor() { super(); this.buf = []; this.port.onmessage = (e) => { this.buf.push(e.data); }; }\n' +
                '  process(inputs, outputs) {\n' +
                '    var out = outputs[0][0]; if (!out) return true;\n' +
                '    var pos = 0;\n' +
                '    while (pos < out.length && this.buf.length > 0) {\n' +
                '      var chunk = this.buf[0]; var take = Math.min(chunk.length - (chunk._off||0), out.length - pos);\n' +
                '      for (var i = 0; i < take; i++) out[pos++] = chunk[(chunk._off||0) + i];\n' +
                '      chunk._off = (chunk._off||0) + take;\n' +
                '      if (chunk._off >= chunk.length) this.buf.shift();\n' +
                '    }\n' +
                '    while (pos < out.length) out[pos++] = 0;\n' +
                '    return true;\n' +
                '  }\n' +
                '}\n' +
                'registerProcessor("sdr-processor", SDRProcessor);\n';
            var blob = new Blob([workletCode], { type: 'application/javascript' });
            var url = URL.createObjectURL(blob);

            audioCtx.audioWorklet.addModule(url).then(function () {
                URL.revokeObjectURL(url);
                workletNode = new AudioWorkletNode(audioCtx, 'sdr-processor');
                workletNode.connect(audioGainNode);
                useScriptProcessor = false;
                startAudioPipeline();
            }).catch(function () {
                URL.revokeObjectURL(url);
                startScriptProcessorFallback();
            });
        } else {
            startScriptProcessorFallback();
        }
    }

    function startScriptProcessorFallback() {
        useScriptProcessor = true;
        scriptNode = audioCtx.createScriptProcessor(4096, 1, 1);
        scriptNode.onaudioprocess = function (e) {
            processAudioBlock(e.outputBuffer.getChannelData(0));
        };
        scriptNode.connect(audioGainNode);
        startAudioPipeline();
    }

    function startAudioPipeline() {
        var params = getModeParams();
        sendCmd('subscribe_iq', { offset: params.offset, bw: params.bw });
        audioRunning = true;
        elAudioBtn.textContent = 'Stop Audio';
        elAudioBtn.classList.add('active');

        /* Feed demodulated audio to worklet via port messages */
        if (!useScriptProcessor && workletNode) {
            feedWorkletLoop();
        }
    }

    function feedWorkletLoop() {
        if (!audioRunning || !workletNode) return;
        while (audioQueue.length > 0) {
            var iq = audioQueue.shift();
            var samples = demodulate(iq);
            var power = 0;
            for (var s = 0; s < samples.length; s++) power += samples[s] * samples[s];
            power = power / (samples.length || 1);
            if (squelch > 0 && power < (squelch / 255) * 0.01) {
                var silence = new Float32Array(samples.length);
                workletNode.port.postMessage(silence);
            } else {
                var arr = new Float32Array(samples.length);
                for (var s = 0; s < samples.length; s++) arr[s] = samples[s] * 0.8;
                workletNode.port.postMessage(arr);
            }
        }
        requestAnimationFrame(feedWorkletLoop);
    }

    function startAudio() {
        if (audioCtx) return;
        startAudioWorklet();
    }

    function stopAudio() {
        audioRunning = false;
        iqSubscribed = false;

        if (ws && ws.readyState === WebSocket.OPEN) {
            sendCmd('unsubscribe_iq', {});
        }

        if (workletNode) {
            workletNode.disconnect();
            workletNode = null;
        }
        if (scriptNode) {
            scriptNode.disconnect();
            scriptNode = null;
        }
        if (audioGainNode) {
            audioGainNode.disconnect();
            audioGainNode = null;
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

    /* -- Commands -- */
    function sendCmd(cmd, params) {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        var msg = { cmd: cmd };
        if (params) {
            for (var k in params) msg[k] = params[k];
        }
        ws.send(JSON.stringify(msg));
    }

    function sendDbRange() {
        var range = parseInt(elRangeSlider.value);
        var refLevel = parseInt(elRefSlider.value);
        sendCmd('db_range', { min: refLevel - range, max: refLevel });
    }

    /* -- Canvas Click -> Tune -- */
    function onCanvasClick(e) {
        var rect = e.target.getBoundingClientRect();
        var x = e.clientX - rect.left;
        var frac = x / rect.width;

        /* Adjust for zoom */
        var len = fftSize;
        var range = getVisibleBinRange(len);
        var visLen = range.end - range.start;
        var binFrac = (range.start + frac * visLen) / len;

        var newFreq = Math.round(centerFreq - sampleRate / 2 + binFrac * sampleRate);
        elFreqInput.value = (newFreq / 1e6).toFixed(3);

        if (audioRunning) {
            var offset = newFreq - centerFreq;
            var params = getModeParams();
            sendCmd('subscribe_iq', { offset: offset, bw: params.bw });
        }
    }

    /* -- Event Handlers -- */
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
            if (audioRunning) {
                var params = getModeParams();
                sendCmd('subscribe_iq', { offset: params.offset, bw: params.bw });
            }
        });
    });

    /* Sample rate */
    elRateSelect.addEventListener('change', function () {
        sendCmd('sample_rate', { value: parseInt(this.value) });
    });

    /* FFT size */
    elFftSelect.addEventListener('change', function () {
        var newSize = parseInt(this.value);
        sendCmd('fft_size', { value: newSize });
        peakHold = null;
        wfImageData = null;
    });

    /* dB range controls */
    elRangeSlider.addEventListener('input', function () {
        elRangeVal.textContent = this.value;
        sendDbRange();
    });

    elRefSlider.addEventListener('input', function () {
        elRefVal.textContent = this.value;
        sendDbRange();
    });

    /* Gain */
    elGainSlider.addEventListener('input', function () {
        currentGain = parseInt(this.value);
        elGainVal.textContent = currentGain === 0 ? 'Auto' : (currentGain / 10).toFixed(1) + ' dB';
        sendCmd('gain', { value: currentGain });
    });

    /* Volume */
    elVolSlider.addEventListener('input', function () {
        volume = parseInt(this.value) / 100;
        elVolVal.textContent = Math.round(volume * 100) + '%';
        if (audioGainNode) {
            audioGainNode.gain.value = volume;
        }
    });

    /* Squelch */
    elSqlSlider.addEventListener('input', function () {
        squelch = parseInt(this.value);
        elSqlVal.textContent = squelch;
    });

    /* Audio toggle */
    elAudioBtn.addEventListener('click', function () {
        if (audioRunning) stopAudio(); else startAudio();
    });

    /* Zoom controls */
    elZoomIn.addEventListener('click', function () {
        if (zoomLevel < 8) {
            zoomLevel *= 2;
            elZoomVal.textContent = zoomLevel + 'x';
            wfImageData = null;
        }
    });

    elZoomOut.addEventListener('click', function () {
        if (zoomLevel > 1) {
            zoomLevel /= 2;
            elZoomVal.textContent = zoomLevel + 'x';
            wfImageData = null;
        }
    });

    elZoomReset.addEventListener('click', function () {
        zoomLevel = 1;
        zoomCenter = 0.5;
        elZoomVal.textContent = '1x';
        wfImageData = null;
    });

    /* Waterfall speed */
    elWfSpeed.addEventListener('input', function () {
        wfSpeed = 110 - parseInt(this.value); /* invert: higher slider = faster = lower ms */
        elWfSpeedVal.textContent = this.value;
    });

    /* Canvas click to tune */
    specCanvas.addEventListener('click', onCanvasClick);
    wfCanvas.addEventListener('click', onCanvasClick);

    /* Mouse wheel on waterfall/spectrum = zoom in/out */
    function onWheelZoom(e) {
        e.preventDefault();
        var rect = e.target.getBoundingClientRect();
        var x = e.clientX - rect.left;
        var frac = x / rect.width;

        /* Set zoom center to mouse position in bin space */
        var len = fftSize;
        var range = getVisibleBinRange(len);
        var visLen = range.end - range.start;
        zoomCenter = (range.start + frac * visLen) / len;

        if (e.deltaY < 0 && zoomLevel < 8) {
            zoomLevel *= 2;
        } else if (e.deltaY > 0 && zoomLevel > 1) {
            zoomLevel /= 2;
        }
        elZoomVal.textContent = zoomLevel + 'x';
        wfImageData = null;
    }
    specCanvas.addEventListener('wheel', onWheelZoom);
    wfCanvas.addEventListener('wheel', onWheelZoom);

    /* -- Init -- */
    resizeCanvases();
    updateDisplay();
    wsConnect();

})();
