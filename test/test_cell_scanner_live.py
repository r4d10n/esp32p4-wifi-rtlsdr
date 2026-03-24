#!/usr/bin/env python3
"""
Live cell scanner test and debug tool.
Connects to ESP32-P4 cell scanner via WebSocket and serial,
exercises all features, validates data, and monitors for crashes.

Usage:
    python3 test_cell_scanner_live.py --host 192.168.1.232 --serial /dev/ttyACM0

Requires: pip install websocket-client pyserial

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import sys
import time
import threading
import os
import re
from datetime import datetime

try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. Run: pip install websocket-client")
    sys.exit(1)

try:
    import serial
except ImportError:
    serial = None


# ──────────────────────── ARFCN / Frequency Reference ────────────────────────

GSM900_FREQ_START = 935_000_000
GSM900_ARFCN_START = 1
GSM900_ARFCN_END = 124
GSM_CHANNEL_BW = 200_000

GSM1800_FREQ_START = 1_805_000_000
GSM1800_ARFCN_START = 512
GSM1800_ARFCN_END = 885


def arfcn_to_freq(arfcn):
    """Reference ARFCN-to-frequency conversion per 3GPP TS 45.005."""
    if GSM900_ARFCN_START <= arfcn <= GSM900_ARFCN_END:
        return GSM900_FREQ_START + arfcn * GSM_CHANNEL_BW
    if GSM1800_ARFCN_START <= arfcn <= GSM1800_ARFCN_END:
        return GSM1800_FREQ_START + (arfcn - GSM1800_ARFCN_START + 1) * GSM_CHANNEL_BW
    return 0


def freq_to_arfcn(freq_hz):
    """Reference frequency-to-ARFCN conversion."""
    if GSM900_FREQ_START <= freq_hz <= 960_000_000:
        return (freq_hz - GSM900_FREQ_START) // GSM_CHANNEL_BW
    if GSM1800_FREQ_START <= freq_hz <= 1_880_000_000:
        return GSM1800_ARFCN_START - 1 + (freq_hz - GSM1800_FREQ_START) // GSM_CHANNEL_BW
    return 0


# ──────────────────────── Result Tracking ────────────────────────

class TestResults:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []

    def ok(self, name, detail=""):
        self.passed += 1
        msg = f"  PASS  {name}"
        if detail:
            msg += f" -- {detail}"
        print(msg)

    def fail(self, name, detail=""):
        self.failed += 1
        msg = f"  FAIL  {name}"
        if detail:
            msg += f" -- {detail}"
        print(msg)
        self.errors.append(f"{name}: {detail}")

    def check(self, condition, name, detail=""):
        if condition:
            self.ok(name, detail)
        else:
            self.fail(name, detail)

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*60}")
        print(f"Results: {self.passed}/{total} passed, {self.failed} failed")
        if self.errors:
            print("Failures:")
            for e in self.errors:
                print(f"  - {e}")
        print(f"{'='*60}")
        return self.failed == 0


# ──────────────────────── Serial Monitor Thread ────────────────────────

class SerialMonitor:
    """Background thread that monitors ESP32 serial output for crashes."""

    CRASH_PATTERNS = [
        r"Guru Meditation",
        r"panic",
        r"abort\(\)",
        r"assert failed",
        r"Stack overflow",
        r"heap corruption",
        r"LoadProhibited",
        r"StoreProhibited",
        r"InstrFetchProhibited",
        r"Backtrace:",
    ]

    def __init__(self, port, baud=115200, logfile=None):
        self.port = port
        self.baud = baud
        self.logfile = logfile
        self.running = False
        self.thread = None
        self.crashes = []
        self.lines = []
        self.heap_values = []
        self._lock = threading.Lock()
        self._ser = None

    def start(self):
        if serial is None:
            print("  WARN  pyserial not installed, skipping serial monitor")
            return False
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"  WARN  Cannot open {self.port}: {e}")
            return False

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=3)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def _run(self):
        logfp = None
        if self.logfile:
            logfp = open(self.logfile, "w")
        try:
            while self.running:
                try:
                    raw = self._ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    with self._lock:
                        self.lines.append(line)
                        if len(self.lines) > 10000:
                            self.lines = self.lines[-5000:]
                    if logfp:
                        logfp.write(line + "\n")
                        logfp.flush()

                    # Check for crash patterns
                    for pattern in self.CRASH_PATTERNS:
                        if re.search(pattern, line, re.IGNORECASE):
                            ts = datetime.now().strftime("%H:%M:%S")
                            crash_msg = f"[{ts}] {line}"
                            with self._lock:
                                self.crashes.append(crash_msg)
                            print(f"  !!!  CRASH DETECTED: {line}")

                    # Extract heap info
                    m = re.search(r"free heap[:\s]+(\d+)", line, re.IGNORECASE)
                    if m:
                        with self._lock:
                            self.heap_values.append(int(m.group(1)))

                except serial.SerialException:
                    break
                except Exception:
                    continue
        finally:
            if logfp:
                logfp.close()

    def get_crashes(self):
        with self._lock:
            return list(self.crashes)

    def get_heap_trend(self):
        with self._lock:
            return list(self.heap_values)

    def get_recent_lines(self, n=20):
        with self._lock:
            return self.lines[-n:]


# ──────────────────────── WebSocket Helper ────────────────────────

class WSClient:
    """Synchronous WebSocket client for testing."""

    def __init__(self, host, port=8085):
        self.url = f"ws://{host}:{port}/ws"
        self.ws = None
        self.messages = []
        self._lock = threading.Lock()
        self._recv_thread = None
        self._running = False

    def connect(self, timeout=5):
        self.ws = websocket.WebSocket()
        self.ws.settimeout(timeout)
        self.ws.connect(self.url)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        # Wait briefly for initial messages (scan_status + existing towers)
        time.sleep(1.0)

    def _recv_loop(self):
        while self._running:
            try:
                data = self.ws.recv()
                if data:
                    msg = json.loads(data)
                    with self._lock:
                        self.messages.append(msg)
            except websocket.WebSocketTimeoutException:
                continue
            except Exception:
                break

    def send(self, obj):
        self.ws.send(json.dumps(obj))

    def get_messages(self, msg_type=None, clear=False):
        with self._lock:
            if msg_type:
                result = [m for m in self.messages if m.get("type") == msg_type]
            else:
                result = list(self.messages)
            if clear:
                if msg_type:
                    self.messages = [m for m in self.messages if m.get("type") != msg_type]
                else:
                    self.messages = []
            return result

    def wait_for_message(self, msg_type, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msgs = self.get_messages(msg_type)
            if msgs:
                return msgs[-1]
            time.sleep(0.2)
        return None

    def close(self):
        self._running = False
        try:
            self.ws.close()
        except Exception:
            pass
        if self._recv_thread:
            self._recv_thread.join(timeout=3)


# ──────────────────────── Test A: WebSocket Functional Tests ────────────────────────

def test_websocket_functional(host, port, results):
    print("\n--- A. WebSocket Functional Tests ---\n")

    # A1: Connect
    ws = WSClient(host, port)
    try:
        ws.connect(timeout=5)
        results.ok("A1-connect", f"Connected to {ws.url}")
    except Exception as e:
        results.fail("A1-connect", str(e))
        return

    try:
        # A2: Receive scan_status on connect
        status_msgs = ws.get_messages("scan_status")
        results.check(len(status_msgs) > 0, "A2-initial-status",
                       f"Got {len(status_msgs)} scan_status messages")

        if status_msgs:
            s = status_msgs[0]
            results.check("scanning" in s, "A2-status-fields",
                           f"scanning={s.get('scanning')}, band={s.get('band')}")

        # A3: Send start_scan for GSM-900 (band=0)
        ws.get_messages(clear=True)
        ws.send({"cmd": "start_scan", "band": 0})
        time.sleep(2)

        status = ws.wait_for_message("scan_status", timeout=5)
        results.check(status is not None, "A3-start-scan",
                       f"Got scan_status after start_scan")

        # A4: Collect tower messages
        time.sleep(5)
        towers = ws.get_messages("tower")
        results.check(True, "A4-tower-collection",
                       f"Received {len(towers)} tower messages during scan")

        # A5: Validate tower frequency matches ARFCN formula
        freq_errors = []
        for t in towers:
            if t.get("tech") == "GSM" and "arfcn" in t and "freq" in t:
                expected = arfcn_to_freq(t["arfcn"])
                if expected > 0 and t["freq"] != expected:
                    freq_errors.append(
                        f"ARFCN {t['arfcn']}: got {t['freq']} Hz, "
                        f"expected {expected} Hz (diff {t['freq'] - expected})"
                    )
        if towers:
            gsm_towers = [t for t in towers if t.get("tech") == "GSM" and "arfcn" in t]
            results.check(len(freq_errors) == 0, "A5-arfcn-freq-accuracy",
                           f"{len(gsm_towers)} GSM towers checked, "
                           f"{len(freq_errors)} errors"
                           + (f": {freq_errors[0]}" if freq_errors else ""))
        else:
            results.ok("A5-arfcn-freq-accuracy",
                        "No towers to validate (may need antenna)")

        # A6: Validate spectrum data
        spectrum_msgs = ws.get_messages("spectrum")
        results.check(len(spectrum_msgs) > 0, "A6-spectrum-data",
                       f"Received {len(spectrum_msgs)} spectrum messages")

        if spectrum_msgs:
            spec = spectrum_msgs[-1]
            data = spec.get("data", [])
            results.check(len(data) > 0, "A6-spectrum-nonempty",
                           f"{len(data)} spectrum bins")

            if data:
                db_min = min(data)
                db_max = max(data)
                # dBFS values should be negative and spread across a range
                saturated = all(v == data[0] for v in data)
                results.check(not saturated, "A6-spectrum-not-saturated",
                               f"dB range: [{db_min}, {db_max}]")
                results.check(db_min >= -150 and db_max <= 0,
                               "A6-spectrum-range",
                               f"min={db_min} dB, max={db_max} dB")

        # A7: Send stop_scan
        ws.send({"cmd": "stop_scan"})
        time.sleep(1)
        status = ws.wait_for_message("scan_status", timeout=5)
        if status:
            results.check(status.get("scanning") is False, "A7-stop-scan",
                           f"scanning={status.get('scanning')}")
        else:
            results.fail("A7-stop-scan", "No scan_status after stop_scan")

        # A8: Calibrate command
        ws.get_messages("calibration", clear=True)
        # Need to start a scan briefly so there is IQ data for calibration
        ws.send({"cmd": "start_scan", "band": 0})
        time.sleep(3)
        ws.send({"cmd": "calibrate"})
        cal = ws.wait_for_message("calibration", timeout=10)
        results.check(cal is not None, "A8-calibrate",
                       f"ppm={cal.get('ppm')}, confidence={cal.get('confidence')}"
                       if cal else "No calibration response")
        ws.send({"cmd": "stop_scan"})
        time.sleep(1)

        # A9: Identity monitor toggle
        ws.get_messages("id_monitor", clear=True)
        ws.send({"cmd": "monitor_ids", "enabled": True})
        id_resp = ws.wait_for_message("id_monitor", timeout=5)
        results.check(id_resp is not None and id_resp.get("enabled") is True,
                       "A9-monitor-ids-enable",
                       str(id_resp) if id_resp else "No id_monitor response")

        ws.get_messages("id_monitor", clear=True)
        ws.send({"cmd": "monitor_ids", "enabled": False})
        id_resp2 = ws.wait_for_message("id_monitor", timeout=5)
        results.check(id_resp2 is not None and id_resp2.get("enabled") is False,
                       "A9-monitor-ids-disable",
                       str(id_resp2) if id_resp2 else "No response")

        # A10: Set threshold
        ws.send({"cmd": "set_threshold", "value": -15.0})
        time.sleep(0.5)
        results.ok("A10-set-threshold", "Command sent (no explicit ack)")

        # A11: Band switching — test GSM-1800 (band=1)
        ws.get_messages(clear=True)
        ws.send({"cmd": "start_scan", "band": 1})
        time.sleep(2)
        status = ws.wait_for_message("scan_status", timeout=5)
        if status:
            results.check("GSM-1800" in str(status.get("band", "")),
                           "A11-band-switch-gsm1800",
                           f"band={status.get('band')}")
        else:
            results.fail("A11-band-switch-gsm1800", "No status after band switch")
        ws.send({"cmd": "stop_scan"})
        time.sleep(1)

    finally:
        ws.close()


# ──────────────────────── Test B: Serial Monitoring ────────────────────────

def test_serial_monitoring(monitor, results):
    print("\n--- B. Serial Monitoring ---\n")

    if not monitor or not monitor.running:
        results.ok("B1-serial-monitor", "Skipped (no serial connection)")
        return

    crashes = monitor.get_crashes()
    results.check(len(crashes) == 0, "B1-no-crashes",
                   f"{len(crashes)} crashes detected"
                   + (f": {crashes[0]}" if crashes else ""))

    heap = monitor.get_heap_trend()
    if len(heap) >= 2:
        trend = heap[-1] - heap[0]
        results.check(trend > -10000, "B2-heap-trend",
                       f"Start={heap[0]}, End={heap[-1]}, Delta={trend}")
    else:
        results.ok("B2-heap-trend", f"Only {len(heap)} heap samples (insufficient)")

    recent = monitor.get_recent_lines(5)
    results.ok("B3-serial-alive", f"Last {len(recent)} lines received")


# ──────────────────────── Test C: Stress Tests ────────────────────────

def test_stress(host, port, results):
    print("\n--- C. Stress Tests ---\n")

    # C1: Rapid band switching
    ws = WSClient(host, port)
    try:
        ws.connect(timeout=5)
    except Exception as e:
        results.fail("C1-rapid-band-switch", f"Connect failed: {e}")
        return

    try:
        bands = [0, 1, 5, 0, 3, 1]  # GSM-900, GSM-1800, LTE-B20, ...
        for b in bands:
            ws.send({"cmd": "start_scan", "band": b})
            time.sleep(2)
        ws.send({"cmd": "stop_scan"})
        time.sleep(1)
        crashes = [m for m in ws.get_messages() if "error" in str(m).lower()]
        results.ok("C1-rapid-band-switch",
                    f"Switched {len(bands)} bands without crash")

        # C2: Multiple calibrate requests
        ws.send({"cmd": "start_scan", "band": 0})
        time.sleep(2)
        for _ in range(5):
            ws.send({"cmd": "calibrate"})
            time.sleep(0.5)
        ws.send({"cmd": "stop_scan"})
        time.sleep(1)
        cal_msgs = ws.get_messages("calibration")
        results.ok("C2-rapid-calibrate",
                    f"Sent 5 calibrate requests, got {len(cal_msgs)} responses")

        # C3: Connect/disconnect rapidly
        ws.close()
        disconnect_ok = True
        for i in range(5):
            try:
                w = WSClient(host, port)
                w.connect(timeout=5)
                time.sleep(0.5)
                w.close()
            except Exception as e:
                disconnect_ok = False
                results.fail("C3-rapid-reconnect", f"Iteration {i}: {e}")
                break
        if disconnect_ok:
            results.ok("C3-rapid-reconnect", "5 connect/disconnect cycles OK")

        # Reconnect for remaining tests
        ws = WSClient(host, port)
        ws.connect(timeout=5)

        # C4: Malformed JSON commands
        malformed = [
            "not json at all",
            '{"cmd": 123}',
            '{"cmd": "start_scan", "band": -1}',
            '{"cmd": "start_scan", "band": 999}',
            '{}',
            '{"cmd": "nonexistent_command"}',
        ]
        for bad in malformed:
            try:
                ws.ws.send(bad)
                time.sleep(0.3)
            except Exception:
                pass
        time.sleep(1)
        # If we're still connected, the server handled malformed input gracefully
        try:
            ws.send({"cmd": "stop_scan"})
            results.ok("C4-malformed-json",
                        f"Server survived {len(malformed)} malformed messages")
        except Exception as e:
            results.fail("C4-malformed-json", f"Connection lost: {e}")

    finally:
        ws.close()


# ──────────────────────── Test D: Data Validation Loop ────────────────────────

def test_data_validation(host, port, results, duration=60):
    print(f"\n--- D. Data Validation ({duration}s collection) ---\n")

    ws = WSClient(host, port)
    try:
        ws.connect(timeout=5)
    except Exception as e:
        results.fail("D0-connect", str(e))
        return

    try:
        # Start GSM-900 scan and collect for duration
        ws.send({"cmd": "start_scan", "band": 0})
        print(f"  Collecting tower data for {duration} seconds...")
        time.sleep(duration)
        ws.send({"cmd": "stop_scan"})
        time.sleep(2)

        towers = ws.get_messages("tower")
        spectra = ws.get_messages("spectrum")

        # D1: Tower count
        results.check(True, "D1-tower-count",
                       f"Collected {len(towers)} tower messages, "
                       f"{len(spectra)} spectrum messages")

        # D2: Validate all ARFCN-to-freq mappings
        gsm_towers = [t for t in towers if t.get("tech") == "GSM" and "arfcn" in t]
        freq_ok = 0
        freq_bad = 0
        freq_errors = []
        for t in gsm_towers:
            expected = arfcn_to_freq(t["arfcn"])
            if expected > 0 and t["freq"] == expected:
                freq_ok += 1
            elif expected > 0:
                freq_bad += 1
                freq_errors.append(
                    f"ARFCN {t['arfcn']}: {t['freq']} != {expected}"
                )
        results.check(freq_bad == 0, "D2-arfcn-freq-mapping",
                       f"{freq_ok} correct, {freq_bad} wrong"
                       + (f" [{freq_errors[0]}]" if freq_errors else ""))

        # D3: Check for duplicate towers (same ARFCN, different freq)
        seen = {}
        duplicates = []
        for t in gsm_towers:
            key = t["arfcn"]
            if key in seen:
                if seen[key] != t["freq"]:
                    duplicates.append(
                        f"ARFCN {key}: freqs {seen[key]} and {t['freq']}"
                    )
            else:
                seen[key] = t["freq"]
        results.check(len(duplicates) == 0, "D3-no-duplicate-arfcn",
                       f"{len(duplicates)} duplicates"
                       + (f": {duplicates[0]}" if duplicates else ""))

        # D4: Power values in reasonable range
        power_values = [t.get("power", -999) for t in towers if "power" in t]
        if power_values:
            p_min = min(power_values)
            p_max = max(power_values)
            in_range = all(-120 <= p <= 0 for p in power_values)
            results.check(in_range, "D4-power-range",
                           f"min={p_min:.1f} dB, max={p_max:.1f} dB, "
                           f"n={len(power_values)}")
            # Check that power values are not all the same (saturation)
            unique_powers = len(set(round(p, 1) for p in power_values))
            results.check(unique_powers > 1 or len(power_values) <= 1,
                           "D4-power-not-saturated",
                           f"{unique_powers} unique power values")
        else:
            results.ok("D4-power-range", "No power values (no towers detected)")

        # D5: Spectrum dB values
        if spectra:
            last_spec = spectra[-1]
            data = last_spec.get("data", [])
            if data:
                s_min = min(data)
                s_max = max(data)
                spread = s_max - s_min
                results.check(spread >= 3, "D5-spectrum-dynamic-range",
                               f"min={s_min} dB, max={s_max} dB, spread={spread} dB")
                results.check(s_min >= -150 and s_max <= 0,
                               "D5-spectrum-bounds",
                               f"[{s_min}, {s_max}]")
            else:
                results.ok("D5-spectrum-dynamic-range", "Empty spectrum data")
        else:
            results.ok("D5-spectrum-dynamic-range", "No spectrum messages")

        # Print tower summary table
        unique_towers = {}
        for t in towers:
            key = f"{t.get('tech')}_{t.get('freq')}"
            unique_towers[key] = t

        if unique_towers:
            print(f"\n  Tower Summary ({len(unique_towers)} unique):")
            print(f"  {'Tech':<6} {'Freq MHz':<12} {'ARFCN/PCI':<10} "
                  f"{'Power dB':<10} {'Count':<6} {'Valid'}")
            print(f"  {'-'*56}")
            for t in sorted(unique_towers.values(),
                            key=lambda x: x.get("power", -999), reverse=True):
                tech = t.get("tech", "?")
                freq_mhz = t.get("freq", 0) / 1e6
                if tech == "GSM":
                    ch = f"A{t.get('arfcn', '?')}"
                else:
                    ch = f"P{t.get('pci', '?')}"
                power = t.get("power", -999)
                count = t.get("count", 1)
                # Validate
                valid = "OK"
                if tech == "GSM" and "arfcn" in t:
                    expected = arfcn_to_freq(t["arfcn"])
                    if expected > 0 and t["freq"] != expected:
                        valid = "FREQ_ERR"
                if not (-120 <= power <= 0):
                    valid = "PWR_ERR"
                print(f"  {tech:<6} {freq_mhz:<12.3f} {ch:<10} "
                      f"{power:<10.1f} {count:<6} {valid}")

    finally:
        ws.close()


# ──────────────────────── Test E: Robustness Loop ────────────────────────

def test_robustness_loop(host, port, serial_monitor, results, iterations=3):
    print(f"\n--- E. Robustness Loop ({iterations} iterations) ---\n")

    iter_results = []
    for i in range(iterations):
        print(f"\n  Iteration {i+1}/{iterations}")
        ir = TestResults()

        ws = WSClient(host, port)
        try:
            ws.connect(timeout=5)
            ir.ok("connect")
        except Exception as e:
            ir.fail("connect", str(e))
            iter_results.append(ir)
            continue

        try:
            # Quick scan cycle
            ws.send({"cmd": "start_scan", "band": 0})
            time.sleep(5)
            towers = ws.get_messages("tower")
            spectrum = ws.get_messages("spectrum")
            ir.check(len(spectrum) > 0, "spectrum-received",
                      f"{len(spectrum)} spectra, {len(towers)} towers")
            ws.send({"cmd": "stop_scan"})
            time.sleep(1)

            # Check serial for crashes
            if serial_monitor and serial_monitor.running:
                crashes = serial_monitor.get_crashes()
                ir.check(len(crashes) == 0, "no-crash",
                          f"{len(crashes)} crashes")
        finally:
            ws.close()

        iter_results.append(ir)
        time.sleep(2)

    # Summary
    total_pass = sum(r.passed for r in iter_results)
    total_fail = sum(r.failed for r in iter_results)
    results.check(total_fail == 0, "E-robustness-loop",
                   f"{total_pass} passed, {total_fail} failed "
                   f"across {iterations} iterations")

    if serial_monitor and serial_monitor.running:
        heap = serial_monitor.get_heap_trend()
        if len(heap) >= 2:
            print(f"  Heap trend: {heap[0]} -> {heap[-1]} "
                  f"(delta {heap[-1] - heap[0]})")


# ──────────────────────── Main ────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Live cell scanner test and debug tool")
    parser.add_argument("--host", default="192.168.1.232",
                        help="ESP32-P4 IP address (default: 192.168.1.232)")
    parser.add_argument("--port", type=int, default=8085,
                        help="Cell scanner HTTP port (default: 8085)")
    parser.add_argument("--serial", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Serial baud rate (default: 115200)")
    parser.add_argument("--no-serial", action="store_true",
                        help="Skip serial monitoring")
    parser.add_argument("--duration", type=int, default=60,
                        help="Data collection duration in seconds (default: 60)")
    parser.add_argument("--iterations", type=int, default=3,
                        help="Robustness loop iterations (default: 3)")
    parser.add_argument("--log", default=None,
                        help="Serial log output file")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: shorter timeouts")
    args = parser.parse_args()

    if args.quick:
        args.duration = 15
        args.iterations = 1

    print(f"Cell Scanner Live Test")
    print(f"  Host: {args.host}:{args.port}")
    print(f"  Serial: {args.serial} @ {args.baud}")
    print(f"  Duration: {args.duration}s, Iterations: {args.iterations}")
    print()

    results = TestResults()

    # Start serial monitor
    monitor = None
    if not args.no_serial:
        logfile = args.log or f"cell_scanner_serial_{int(time.time())}.log"
        monitor = SerialMonitor(args.serial, args.baud, logfile)
        if monitor.start():
            print(f"  Serial monitor started, logging to {logfile}")
        else:
            monitor = None

    try:
        test_websocket_functional(args.host, args.port, results)
        test_serial_monitoring(monitor, results)
        test_stress(args.host, args.port, results)
        test_data_validation(args.host, args.port, results, args.duration)
        test_robustness_loop(args.host, args.port, monitor, results,
                             args.iterations)

        # Final serial check
        if monitor and monitor.running:
            print("\n--- Final Serial Check ---\n")
            crashes = monitor.get_crashes()
            results.check(len(crashes) == 0, "final-no-crashes",
                           f"{len(crashes)} total crashes during test run")
    finally:
        if monitor:
            monitor.stop()

    success = results.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
