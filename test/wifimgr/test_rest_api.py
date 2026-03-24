#!/usr/bin/env python3
"""
Integration Tests — WiFi Manager REST API

Tests all 22 REST API endpoints against a running ESP32-P4 device.
Usage:
    python test_rest_api.py [--host HOST] [--port PORT]

Default: http://192.168.4.1:80 (AP mode) or http://esp32p4-rtlsdr.local:80
"""

import argparse
import json
import sys
import time
import urllib.request
import urllib.error

# ── Color output ────────────────────────────────────────────
GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
RESET = "\033[0m"

tests_run = 0
tests_passed = 0
tests_failed = 0
tests_skipped = 0


def test(name):
    global tests_run
    tests_run += 1
    print(f"  TEST: {name:<55}", end="", flush=True)


def passed():
    global tests_passed
    tests_passed += 1
    print(f"{GREEN}[PASS]{RESET}")


def failed(msg=""):
    global tests_failed
    tests_failed += 1
    print(f"{RED}[FAIL] {msg}{RESET}")


def skipped(msg=""):
    global tests_skipped
    tests_skipped += 1
    print(f"{YELLOW}[SKIP] {msg}{RESET}")


def api(base_url, path, method="GET", data=None, expect_status=200):
    """Make an API request and return (status_code, json_body)."""
    url = f"{base_url}/api/{path}"
    headers = {"Content-Type": "application/json"}

    body = json.dumps(data).encode() if data else None
    req = urllib.request.Request(url, data=body, headers=headers, method=method)

    try:
        resp = urllib.request.urlopen(req, timeout=15)
        status = resp.status
        body = json.loads(resp.read().decode())
        return status, body
    except urllib.error.HTTPError as e:
        body = json.loads(e.read().decode()) if e.fp else {}
        return e.code, body
    except Exception as e:
        return 0, {"error": str(e)}


# ═══════════════════════════════════════════════════════════════
#  Connectivity Test
# ═══════════════════════════════════════════════════════════════

def test_connectivity(base_url):
    test("Device reachable")
    status, body = api(base_url, "system/info")
    if status == 200:
        passed()
        return True
    else:
        failed(f"HTTP {status}")
        return False


# ═══════════════════════════════════════════════════════════════
#  WiFi Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_wifi_status(base_url):
    test("GET /api/wifi/status")
    status, body = api(base_url, "wifi/status")
    if status != 200:
        return failed(f"HTTP {status}")
    for key in ["connected", "ssid", "ip", "rssi", "mode", "state"]:
        if key not in body:
            return failed(f"Missing key: {key}")
    assert isinstance(body["connected"], bool)
    assert body["state"] in ["init", "scanning", "connecting", "connected", "ap_mode", "apsta_mode"]
    passed()


def test_wifi_scan(base_url):
    test("GET /api/wifi/scan")
    status, body = api(base_url, "wifi/scan")
    if status != 200:
        return failed(f"HTTP {status}")
    if "networks" not in body:
        return failed("Missing 'networks' key")
    assert isinstance(body["networks"], list)
    if len(body["networks"]) > 0:
        net = body["networks"][0]
        for key in ["ssid", "rssi", "channel", "saved", "auth"]:
            assert key in net, f"Missing key: {key}"
    passed()


def test_wifi_networks_get(base_url):
    test("GET /api/wifi/networks")
    status, body = api(base_url, "wifi/networks")
    if status != 200:
        return failed(f"HTTP {status}")
    assert "networks" in body
    assert isinstance(body["networks"], list)
    passed()


def test_wifi_network_add(base_url):
    test("POST /api/wifi/networks (add test network)")
    status, body = api(base_url, "wifi/networks", "POST", {
        "ssid": "__test_network__",
        "password": "test12345678",
        "auth": 3  # WPA2
    })
    if status != 200:
        return failed(f"HTTP {status}: {body.get('message', '')}")
    assert body.get("status") == "ok"
    passed()


def test_wifi_network_delete(base_url):
    test("DELETE /api/wifi/networks (remove test network)")
    status, body = api(base_url, "wifi/networks?ssid=__test_network__", method="DELETE")
    if status != 200:
        return failed(f"HTTP {status}: {body.get('message', '')}")
    assert body.get("status") == "ok"
    passed()


def test_wifi_network_delete_nonexistent(base_url):
    test("DELETE /api/wifi/networks (nonexistent → 404)")
    status, body = api(base_url, "wifi/networks?ssid=__does_not_exist__", method="DELETE")
    if status != 404:
        return failed(f"Expected 404, got {status}")
    passed()


def test_wifi_connect_missing_ssid(base_url):
    test("POST /api/wifi/connect (missing ssid → 400)")
    status, body = api(base_url, "wifi/connect", "POST", {"password": "test"})
    if status != 400:
        return failed(f"Expected 400, got {status}")
    passed()


# ═══════════════════════════════════════════════════════════════
#  Ethernet Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_eth_config_get(base_url):
    test("GET /api/eth/config")
    status, body = api(base_url, "eth/config")
    if status != 200:
        return failed(f"HTTP {status}")
    for key in ["enable", "dhcp", "static_ip", "static_mask", "phy_type"]:
        if key not in body:
            return failed(f"Missing key: {key}")
    passed()


def test_eth_config_put(base_url):
    test("PUT /api/eth/config (partial update)")
    status, body = api(base_url, "eth/config", "PUT", {"dhcp": True})
    if status != 200:
        return failed(f"HTTP {status}: {body.get('message', '')}")
    passed()


# ═══════════════════════════════════════════════════════════════
#  SDR Config Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_sdr_config_get(base_url):
    test("GET /api/sdr/config")
    status, body = api(base_url, "sdr/config")
    if status != 200:
        return failed(f"HTTP {status}")
    for key in ["center_freq", "sample_rate", "gain_mode", "ppm_correction",
                "direct_sampling", "bias_tee", "hostname"]:
        if key not in body:
            return failed(f"Missing key: {key}")
    passed()


def test_sdr_config_put(base_url):
    test("PUT /api/sdr/config (change freq)")
    # Read current
    _, orig = api(base_url, "sdr/config")
    # Change freq
    status, body = api(base_url, "sdr/config", "PUT", {"center_freq": 144390000})
    if status != 200:
        return failed(f"HTTP {status}")
    # Verify changed
    _, after = api(base_url, "sdr/config")
    if after.get("center_freq") != 144390000:
        return failed(f"Freq not updated: {after.get('center_freq')}")
    # Restore original
    api(base_url, "sdr/config", "PUT", {"center_freq": orig.get("center_freq", 100000000)})
    passed()


# ═══════════════════════════════════════════════════════════════
#  Services Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_services_list(base_url):
    test("GET /api/services")
    status, body = api(base_url, "services")
    if status != 200:
        return failed(f"HTTP {status}")
    expected = ["rtl_tcp", "rtl_udp", "websdr", "adsb", "ais", "aprs"]
    for svc in expected:
        if svc not in body:
            return failed(f"Missing service: {svc}")
    passed()


def test_service_get(base_url):
    test("GET /api/services/rtl_tcp")
    status, body = api(base_url, "services/rtl_tcp")
    if status != 200:
        return failed(f"HTTP {status}")
    assert "enable" in body
    assert "port" in body
    passed()


def test_service_put(base_url):
    test("PUT /api/services/rtl_tcp (toggle enable)")
    _, orig = api(base_url, "services/rtl_tcp")
    status, body = api(base_url, "services/rtl_tcp", "PUT", {"max_clients": 2})
    if status != 200:
        return failed(f"HTTP {status}")
    passed()


def test_service_not_found(base_url):
    test("GET /api/services/nonexistent (→ 404)")
    status, _ = api(base_url, "services/nonexistent")
    if status != 404:
        return failed(f"Expected 404, got {status}")
    passed()


# ═══════════════════════════════════════════════════════════════
#  Notification Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_notify_config_get(base_url):
    test("GET /api/notify/config")
    status, body = api(base_url, "notify/config")
    if status != 200:
        return failed(f"HTTP {status}")
    assert "telegram" in body
    assert "discord" in body
    assert "enable" in body["telegram"]
    passed()


def test_notify_config_put(base_url):
    test("PUT /api/notify/config (update rate limit)")
    status, body = api(base_url, "notify/config", "PUT", {
        "telegram": {"rate_limit_s": 120}
    })
    if status != 200:
        return failed(f"HTTP {status}")
    passed()


# ═══════════════════════════════════════════════════════════════
#  System Endpoint Tests
# ═══════════════════════════════════════════════════════════════

def test_system_info(base_url):
    test("GET /api/system/info")
    status, body = api(base_url, "system/info")
    if status != 200:
        return failed(f"HTTP {status}")
    for key in ["uptime_s", "heap_free", "psram_free", "hostname", "version"]:
        if key not in body:
            return failed(f"Missing key: {key}")
    assert body["heap_free"] > 0
    assert body["psram_free"] > 0
    passed()


def test_system_backup(base_url):
    test("GET /api/system/backup")
    status, body = api(base_url, "system/backup")
    if status != 200:
        return failed(f"HTTP {status}")
    assert "sdr" in body or "services" in body
    passed()


def test_system_reboot_no_confirm(base_url):
    test("POST /api/system/reboot (no confirm → 400)")
    status, _ = api(base_url, "system/reboot", "POST", {"confirm": False})
    if status != 400:
        return failed(f"Expected 400, got {status}")
    passed()


def test_factory_reset_no_confirm(base_url):
    test("POST /api/system/factory-reset (no confirm → 400)")
    status, _ = api(base_url, "system/factory-reset", "POST", {"confirm": False})
    if status != 400:
        return failed(f"Expected 400, got {status}")
    passed()


# ═══════════════════════════════════════════════════════════════
#  CORS Test
# ═══════════════════════════════════════════════════════════════

def test_cors_headers(base_url):
    test("CORS headers present on API response")
    url = f"{base_url}/api/system/info"
    req = urllib.request.Request(url, method="GET")
    try:
        resp = urllib.request.urlopen(req, timeout=10)
        cors = resp.getheader("Access-Control-Allow-Origin")
        if cors != "*":
            return failed(f"CORS header: {cors}")
        passed()
    except Exception as e:
        failed(str(e))


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="WiFi Manager REST API Integration Tests")
    parser.add_argument("--host", default="192.168.4.1", help="Device IP or hostname")
    parser.add_argument("--port", type=int, default=80, help="Config portal port")
    args = parser.parse_args()

    base_url = f"http://{args.host}:{args.port}"
    print(f"\n=== WiFi Manager REST API Integration Tests ===")
    print(f"Target: {base_url}\n")

    # Connectivity check
    if not test_connectivity(base_url):
        print(f"\n{RED}Device not reachable at {base_url}{RESET}")
        print("Make sure the ESP32-P4 is running and accessible.")
        print("For AP mode: connect to ESP32P4-SDR-Setup WiFi, then use --host 192.168.4.1")
        sys.exit(1)

    # WiFi
    print("\n[WiFi Endpoints]")
    test_wifi_status(base_url)
    test_wifi_scan(base_url)
    test_wifi_networks_get(base_url)
    test_wifi_network_add(base_url)
    test_wifi_network_delete(base_url)
    test_wifi_network_delete_nonexistent(base_url)
    test_wifi_connect_missing_ssid(base_url)

    # Ethernet
    print("\n[Ethernet Endpoints]")
    test_eth_config_get(base_url)
    test_eth_config_put(base_url)

    # SDR
    print("\n[SDR Config Endpoints]")
    test_sdr_config_get(base_url)
    test_sdr_config_put(base_url)

    # Services
    print("\n[Services Endpoints]")
    test_services_list(base_url)
    test_service_get(base_url)
    test_service_put(base_url)
    test_service_not_found(base_url)

    # Notifications
    print("\n[Notification Endpoints]")
    test_notify_config_get(base_url)
    test_notify_config_put(base_url)

    # System
    print("\n[System Endpoints]")
    test_system_info(base_url)
    test_system_backup(base_url)
    test_system_reboot_no_confirm(base_url)
    test_factory_reset_no_confirm(base_url)

    # CORS
    print("\n[CORS]")
    test_cors_headers(base_url)

    # Summary
    print(f"\n=== Results: {tests_passed}/{tests_run} passed", end="")
    if tests_failed:
        print(f", {RED}{tests_failed} FAILED{RESET}", end="")
    if tests_skipped:
        print(f", {YELLOW}{tests_skipped} skipped{RESET}", end="")
    print(" ===\n")

    sys.exit(1 if tests_failed else 0)


if __name__ == "__main__":
    main()
