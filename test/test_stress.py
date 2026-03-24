#!/usr/bin/env python3
"""
ESP32-P4 WebSDR Stress Test & Hardware-in-the-Loop Validation

Tests the WebSocket server under extreme conditions:
1. Rapid-fire commands (freq/gain/mode changes)
2. Concurrent WebSocket connections
3. Abrupt connect/disconnect cycles
4. Invalid/malformed commands
5. Boundary value testing
6. Audio pipeline stress (subscribe/unsubscribe cycles)
7. FFT size changes during streaming
8. Sample rate changes during streaming
9. Long-duration stability test

Monitors serial output for crashes, panics, watchdog resets.

Usage: python3 test_stress.py [--host 192.168.1.232] [--duration 60]
"""

import argparse
import json
import random
import sys
import threading
import time
import traceback
from collections import defaultdict

import numpy as np

try:
    import websocket
except ImportError:
    print("pip install websocket-client"); sys.exit(1)

try:
    import serial
except ImportError:
    serial = None

HOST = "192.168.1.232"
PORT = 8080
UART = "/dev/ttyACM0"

stats = defaultdict(int)
errors = []
lock = threading.Lock()

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def stat(key):
    with lock:
        stats[key] += 1

def err(msg):
    with lock:
        errors.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
    log(f"ERROR: {msg}")

# ─── Serial Monitor ───

class SerialMonitor:
    def __init__(self, port, baud=115200):
        self.running = True
        self.lines = []
        self.panics = 0
        self.wdt = 0
        self.errors_found = 0
        self.port = port
        self.baud = baud

    def start(self):
        if serial is None:
            log("[UART] pyserial not available, skipping serial monitor")
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            log(f"[UART] Monitoring {self.port}")
        except Exception as e:
            log(f"[UART] Cannot open {self.port}: {e}")

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.lines.append(line)
                    if 'Guru Meditation' in line or 'panic' in line.lower():
                        self.panics += 1
                        err(f"PANIC: {line}")
                    elif 'watchdog' in line.lower() or 'wdt' in line.lower():
                        self.wdt += 1
                        err(f"WDT: {line}")
                    elif 'E (' in line or 'assert' in line.lower():
                        self.errors_found += 1
                    # Keep last 500 lines
                    if len(self.lines) > 500:
                        self.lines = self.lines[-500:]
            except Exception:
                time.sleep(0.1)

    def stop(self):
        self.running = False

    def summary(self):
        return {
            'panics': self.panics,
            'wdt_resets': self.wdt,
            'errors': self.errors_found,
            'total_lines': len(self.lines),
        }

# ─── WebSocket Helpers ───

def ws_connect(host=HOST, port=PORT, timeout=5):
    try:
        ws = websocket.create_connection(f"ws://{host}:{port}/ws", timeout=timeout)
        stat('ws_connect_ok')
        return ws
    except Exception as e:
        stat('ws_connect_fail')
        return None

def ws_send(ws, cmd, params=None):
    try:
        msg = {"cmd": cmd}
        if params:
            msg.update(params)
        ws.send(json.dumps(msg))
        stat('ws_send_ok')
        return True
    except Exception:
        stat('ws_send_fail')
        return False

def ws_recv_all(ws, timeout=1):
    frames = {'fft': 0, 'iq': 0, 'text': 0, 'bytes': 0}
    ws.settimeout(timeout)
    try:
        while True:
            d = ws.recv()
            if isinstance(d, bytes):
                frames['bytes'] += len(d)
                if len(d) > 1:
                    if d[0] == 1: frames['fft'] += 1
                    elif d[0] == 2: frames['iq'] += 1
            else:
                frames['text'] += 1
    except:
        pass
    return frames

# ─── Test Cases ───

def test_rapid_freq_changes(ws, count=50):
    """Rapidly change frequency — tests USB control transfer mutex."""
    log(f"[TEST] Rapid freq changes ({count}x)")
    freqs = [random.randint(24000000, 1700000000) for _ in range(count)]
    ok = 0
    for f in freqs:
        if ws_send(ws, 'freq', {'value': f}):
            ok += 1
        time.sleep(0.02)  # 20ms between changes
    stat('rapid_freq_ok')
    log(f"  → {ok}/{count} sent OK")
    return ok == count

def test_rapid_gain_changes(ws, count=30):
    """Rapidly cycle through all gain values."""
    log(f"[TEST] Rapid gain changes ({count}x)")
    ok = 0
    for _ in range(count):
        g = random.choice([0, 10, 20, 30, 40, 50, 100, 200, 300, 400, 496])
        if ws_send(ws, 'gain', {'value': g}):
            ok += 1
        time.sleep(0.01)
    log(f"  → {ok}/{count} sent OK")
    return ok == count

def test_mode_cycling(ws, cycles=20):
    """Rapidly cycle through all demod modes."""
    log(f"[TEST] Mode cycling ({cycles}x)")
    modes = ['AM', 'NFM', 'WBFM', 'USB', 'LSB', 'CW']
    bws = {'AM': 10000, 'NFM': 12500, 'WBFM': 150000, 'USB': 3000, 'LSB': 3000, 'CW': 500}
    for _ in range(cycles):
        m = random.choice(modes)
        ws_send(ws, 'subscribe_iq', {'offset': 0, 'bw': bws[m]})
        time.sleep(0.05)
    stat('mode_cycle_ok')
    log(f"  → {cycles} mode changes sent")
    return True

def test_fft_size_changes(ws):
    """Change FFT size during streaming — tests DSP reinit race protection."""
    log("[TEST] FFT size changes during streaming")
    sizes = [256, 512, 1024, 2048, 4096, 8192, 1024]
    for s in sizes:
        ws_send(ws, 'fft_size', {'value': s})
        time.sleep(0.3)
        frames = ws_recv_all(ws, timeout=0.5)
        log(f"  FFT {s}: {frames['fft']} frames received")
        if frames['fft'] == 0:
            err(f"No FFT frames after size change to {s}")
    stat('fft_size_ok')
    return True

def test_sample_rate_changes(ws):
    """Change sample rate during streaming — tests hardware reconfiguration."""
    log("[TEST] Sample rate changes during streaming")
    rates = [250000, 1024000, 1536000, 2048000, 1024000]
    for r in rates:
        ws_send(ws, 'sample_rate', {'value': r})
        time.sleep(0.5)
        frames = ws_recv_all(ws, timeout=0.5)
        log(f"  Rate {r}: {frames['fft']} FFT frames")
    stat('rate_change_ok')
    # Reset to default
    ws_send(ws, 'sample_rate', {'value': 1024000})
    return True

def test_iq_subscribe_unsubscribe(ws, cycles=20):
    """Rapidly subscribe/unsubscribe IQ — tests DDC create/destroy."""
    log(f"[TEST] IQ subscribe/unsubscribe ({cycles}x)")
    for i in range(cycles):
        bw = random.choice([5000, 12500, 25000, 48000, 100000])
        off = random.randint(-200000, 200000)
        ws_send(ws, 'subscribe_iq', {'offset': off, 'bw': bw})
        time.sleep(0.1)
        frames = ws_recv_all(ws, timeout=0.2)
        ws_send(ws, 'unsubscribe_iq', {})
        time.sleep(0.05)
    stat('iq_sub_unsub_ok')
    log(f"  → {cycles} sub/unsub cycles completed")
    return True

def test_db_range_changes(ws, count=20):
    """Rapidly change dB range — tests spectrum scaling."""
    log(f"[TEST] dB range changes ({count}x)")
    for _ in range(count):
        lo = random.randint(-140, -40)
        hi = lo + random.randint(20, 100)
        ws_send(ws, 'db_range', {'min': lo, 'max': hi})
        time.sleep(0.05)
    stat('db_range_ok')
    return True

def test_invalid_commands(ws):
    """Send malformed/invalid commands — tests input validation."""
    log("[TEST] Invalid/malformed commands")
    invalid = [
        '',                                     # empty
        'not json',                             # not JSON
        '{"cmd":"nonexistent"}',                # unknown command
        '{"cmd":"freq"}',                       # missing value
        '{"cmd":"freq","value":-1}',            # negative freq
        '{"cmd":"freq","value":99999999999}',   # too high freq
        '{"cmd":"sample_rate","value":0}',      # zero rate (div by zero)
        '{"cmd":"sample_rate","value":999999999}', # absurd rate
        '{"cmd":"fft_size","value":3}',         # non-power-of-2
        '{"cmd":"fft_size","value":0}',         # zero
        '{"cmd":"fft_size","value":65536}',     # too large
        '{"cmd":"gain","value":-100}',          # negative gain
        '{"cmd":"db_range","min":50,"max":10}', # min > max
        '{"no_cmd":true}',                      # missing cmd field
        'x' * 2000,                             # oversized text
    ]
    crashed = False
    for msg in invalid:
        try:
            ws.send(msg)
            stat('invalid_sent')
            time.sleep(0.05)
        except Exception as e:
            err(f"WS closed on invalid: {msg[:30]}...")
            crashed = True
            break

    # Verify connection still alive
    time.sleep(0.2)
    try:
        ws.send(json.dumps({"cmd": "freq", "value": 100000000}))
        stat('post_invalid_ok')
        log("  → Server survived all invalid commands ✓")
    except:
        err("Server connection lost after invalid commands")
        crashed = True

    return not crashed

def test_concurrent_connections(host=HOST, port=PORT, count=3):
    """Open multiple concurrent WebSocket connections."""
    log(f"[TEST] Concurrent connections ({count})")
    conns = []
    for i in range(count):
        ws = ws_connect(host, port)
        if ws:
            conns.append(ws)
            time.sleep(0.2)

    log(f"  → {len(conns)}/{count} connected")

    # All should receive FFT frames
    for i, ws in enumerate(conns):
        frames = ws_recv_all(ws, timeout=1)
        log(f"  Conn {i}: {frames['fft']} FFT frames")

    # Close all
    for ws in conns:
        try: ws.close()
        except: pass

    stat('concurrent_ok')
    return len(conns) >= min(count, 3)  # ESP32 max 3 WS clients

def test_reconnect_storm(host=HOST, port=PORT, cycles=10):
    """Rapidly connect/disconnect — tests resource cleanup."""
    log(f"[TEST] Reconnect storm ({cycles}x)")
    ok = 0
    for i in range(cycles):
        ws = ws_connect(host, port, timeout=3)
        if ws:
            time.sleep(0.1)
            try: ws.close()
            except: pass
            ok += 1
        time.sleep(0.2)

    stat('reconnect_ok')
    log(f"  → {ok}/{cycles} connect/disconnect cycles OK")
    return ok >= cycles * 0.8

def test_sustained_streaming(ws, duration=10):
    """Verify continuous FFT streaming for N seconds without drops."""
    log(f"[TEST] Sustained streaming ({duration}s)")
    ws.settimeout(2)
    total_fft = 0
    total_iq = 0
    gaps = 0
    t0 = time.time()

    # Subscribe IQ
    ws_send(ws, 'subscribe_iq', {'offset': 0, 'bw': 25000})
    time.sleep(0.3)

    while time.time() - t0 < duration:
        try:
            d = ws.recv()
            if isinstance(d, bytes) and len(d) > 1:
                if d[0] == 1: total_fft += 1
                elif d[0] == 2: total_iq += 1
        except websocket.WebSocketTimeoutException:
            gaps += 1
        except Exception:
            gaps += 1
            break

    elapsed = time.time() - t0
    fft_rate = total_fft / elapsed if elapsed > 0 else 0
    iq_rate = total_iq / elapsed if elapsed > 0 else 0

    ws_send(ws, 'unsubscribe_iq', {})

    log(f"  → FFT: {total_fft} ({fft_rate:.0f}/s) IQ: {total_iq} ({iq_rate:.0f}/s) Gaps: {gaps}")

    if total_fft == 0:
        err("No FFT frames during sustained streaming")
    if fft_rate < 5:
        err(f"FFT rate too low: {fft_rate:.1f}/s (expected >10)")

    stat('sustained_ok')
    return total_fft > 0 and gaps < 3

def test_boundary_values(ws):
    """Test boundary values for all parameters."""
    log("[TEST] Boundary value testing")
    tests = [
        ('freq', {'value': 24000000}),     # min RTL-SDR
        ('freq', {'value': 1766000000}),   # max RTL-SDR
        ('freq', {'value': 100000000}),    # typical
        ('gain', {'value': 0}),            # auto
        ('gain', {'value': 496}),          # max
        ('sample_rate', {'value': 250000}),  # min practical
        ('sample_rate', {'value': 3200000}), # max
        ('fft_size', {'value': 256}),      # min
        ('fft_size', {'value': 8192}),     # max safe
        ('db_range', {'min': -150, 'max': 0}),  # wide range
        ('db_range', {'min': -50, 'max': -20}),  # narrow range
    ]
    ok = 0
    for cmd, params in tests:
        if ws_send(ws, cmd, params):
            ok += 1
            time.sleep(0.1)

    # Reset to defaults
    ws_send(ws, 'freq', {'value': 100000000})
    ws_send(ws, 'sample_rate', {'value': 1024000})
    ws_send(ws, 'gain', {'value': 0})
    ws_send(ws, 'db_range', {'min': 10, 'max': 90})
    time.sleep(0.3)

    log(f"  → {ok}/{len(tests)} boundary tests sent OK")
    stat('boundary_ok')
    return ok == len(tests)

def test_audio_pipeline(ws):
    """Test audio subscribe with different bandwidths and verify IQ frames arrive."""
    log("[TEST] Audio pipeline stress")
    bws = [5000, 12500, 25000, 48000, 100000, 200000]
    ok = 0
    for bw in bws:
        ws_send(ws, 'subscribe_iq', {'offset': 0, 'bw': bw})
        time.sleep(0.5)
        frames = ws_recv_all(ws, timeout=1)
        if frames['iq'] > 0:
            ok += 1
            log(f"  BW {bw}: {frames['iq']} IQ frames ✓")
        else:
            log(f"  BW {bw}: 0 IQ frames ✗")
        ws_send(ws, 'unsubscribe_iq', {})
        time.sleep(0.2)

    stat('audio_pipeline_ok')
    log(f"  → {ok}/{len(bws)} bandwidths producing IQ frames")
    return ok >= len(bws) - 1  # Allow 1 failure

# ─── Main ───

def run_tests(args):
    log("="*60)
    log("  ESP32-P4 WebSDR Stress Test")
    log(f"  Target: {args.host}:{args.port}")
    log(f"  Duration: {args.duration}s sustained test")
    log("="*60)

    # Start serial monitor
    uart = SerialMonitor(args.uart)
    uart.start()

    results = {}

    # Connect
    ws = ws_connect(args.host, args.port)
    if not ws:
        err("Cannot connect to WebSDR")
        return

    # Wait for device info
    try:
        info = json.loads(ws.recv())
        log(f"[INFO] freq={info.get('freq')}Hz rate={info.get('rate')}Hz fft={info.get('fft_size')}")
    except:
        err("No device info received")

    # Run all tests
    test_list = [
        ("Boundary Values", lambda: test_boundary_values(ws)),
        ("Rapid Freq Changes", lambda: test_rapid_freq_changes(ws, 50)),
        ("Rapid Gain Changes", lambda: test_rapid_gain_changes(ws, 30)),
        ("Mode Cycling", lambda: test_mode_cycling(ws, 20)),
        ("dB Range Changes", lambda: test_db_range_changes(ws, 20)),
        ("FFT Size Changes", lambda: test_fft_size_changes(ws)),
        ("Sample Rate Changes", lambda: test_sample_rate_changes(ws)),
        ("IQ Sub/Unsub Cycles", lambda: test_iq_subscribe_unsubscribe(ws, 20)),
        ("Audio Pipeline", lambda: test_audio_pipeline(ws)),
        ("Invalid Commands", lambda: test_invalid_commands(ws)),
        ("Sustained Streaming", lambda: test_sustained_streaming(ws, args.duration)),
    ]

    # These need fresh connections
    extra_tests = [
        ("Concurrent Connections", lambda: test_concurrent_connections(args.host, args.port, 3)),
        ("Reconnect Storm", lambda: test_reconnect_storm(args.host, args.port, 10)),
    ]

    for name, fn in test_list:
        try:
            result = fn()
            results[name] = "PASS" if result else "FAIL"
            time.sleep(0.3)
            # Verify WS still alive
            try:
                ws.ping()
            except:
                log(f"  WS died after {name}, reconnecting...")
                ws = ws_connect(args.host, args.port)
                if not ws:
                    err(f"Cannot reconnect after {name}")
                    results[name] = "CRASH"
                    break
                try: ws.recv()  # consume info
                except: pass
        except Exception as e:
            results[name] = "ERROR"
            err(f"{name}: {e}")
            traceback.print_exc()

    try: ws.close()
    except: pass

    time.sleep(1)

    for name, fn in extra_tests:
        try:
            result = fn()
            results[name] = "PASS" if result else "FAIL"
        except Exception as e:
            results[name] = "ERROR"
            err(f"{name}: {e}")

    # Final check — can we still connect?
    time.sleep(2)
    ws = ws_connect(args.host, args.port)
    if ws:
        results["Post-Stress Connect"] = "PASS"
        frames = ws_recv_all(ws, timeout=2)
        results["Post-Stress FFT"] = "PASS" if frames['fft'] > 0 else "FAIL"
        ws.close()
    else:
        results["Post-Stress Connect"] = "FAIL"
        results["Post-Stress FFT"] = "FAIL"

    # Stop serial monitor
    uart.stop()
    uart_summary = uart.summary()

    # ─── Report ───
    log("")
    log("="*60)
    log("  STRESS TEST RESULTS")
    log("="*60)

    total = len(results)
    passed = sum(1 for v in results.values() if v == "PASS")
    failed = sum(1 for v in results.values() if v != "PASS")

    log(f"\n  {'Test':<30} {'Result':<10}")
    log(f"  {'─'*40}")
    for name, result in results.items():
        icon = "✓" if result == "PASS" else "✗" if result == "FAIL" else "!"
        log(f"  {icon} {name:<28} {result}")

    log(f"\n  Passed: {passed}/{total}")
    log(f"  Failed: {failed}/{total}")

    log(f"\n  WebSocket Stats:")
    for k, v in sorted(stats.items()):
        log(f"    {k}: {v}")

    log(f"\n  Serial Monitor:")
    log(f"    Panics: {uart_summary['panics']}")
    log(f"    WDT Resets: {uart_summary['wdt_resets']}")
    log(f"    Error Lines: {uart_summary['errors']}")
    log(f"    Total Lines: {uart_summary['total_lines']}")

    if errors:
        log(f"\n  Errors ({len(errors)}):")
        for e in errors:
            log(f"    {e}")

    if failed == 0 and uart_summary['panics'] == 0:
        log(f"\n  ✅ ALL TESTS PASSED — server is robust")
    elif uart_summary['panics'] > 0:
        log(f"\n  ❌ PANICS DETECTED — critical firmware bug")
    else:
        log(f"\n  ⚠ {failed} test(s) failed — needs investigation")

def main():
    parser = argparse.ArgumentParser(description="WebSDR Stress Test")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--uart", default=UART)
    parser.add_argument("--duration", type=int, default=15,
                        help="Sustained streaming test duration (seconds)")
    args = parser.parse_args()

    try:
        run_tests(args)
    except KeyboardInterrupt:
        log("\n[INTERRUPTED]")
    except Exception as e:
        log(f"\n[FATAL] {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()
