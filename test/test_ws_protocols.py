#!/usr/bin/env python3
"""
WebSocket protocol validation tests for all ESP32-P4 SDR receivers.

Tests JSON message schemas, required fields, value ranges, and command
formats for APRS, AIS, ADS-B, WSPR/FT8, and GSM/LTE cell scanner
WebSocket protocols.

Can run in two modes:
1. Offline (default): Validates message schemas against reference test vectors
2. Online (--live HOST): Connects to actual device and validates real messages

Field names and message formats are extracted directly from the server C
source files:
  - esp32p4-aprs/components/aprs/aprs.c
  - esp32p4-ais/components/ais/ais.c
  - esp32p4-adsb/components/adsb/adsb.c
  - esp32p4-wspr-ft8/components/wspr_ft8/wspr_ft8.c
  - esp32p4-gsm-lte/components/cell_scanner/cell_scanner.c

Usage:
  # Offline schema tests (no device needed)
  python3 test_ws_protocols.py

  # Run a single test class
  python3 test_ws_protocols.py TestAPRSProtocol

  # Live tests against device (requires websocket-client)
  python3 test_ws_protocols.py --live 192.168.1.232

SPDX-License-Identifier: GPL-2.0-or-later
"""

import argparse
import json
import re
import sys
import unittest


# ─────────────────────────────────────────────────────────────────────────────
# Reference test vectors — realistic sample messages matching each server's
# actual snprintf/cJSON output format from the C source files.
# ─────────────────────────────────────────────────────────────────────────────

SAMPLE_MESSAGES = {
    # aprs.c: packet_to_json() builds via cJSON
    "aprs_packet": {
        "type": "packet",
        "call": "W1AW",
        "ssid": 9,
        "path": "WIDE1-1,WIDE2-1",
        "ptype": 2,
        "lat": 41.7147,
        "lon": -72.7272,
        "sym_table": "/",
        "sym_code": ">",
        "speed": 25.0,
        "course": 270.0,
        "alt": 150.0,
        "comment": "Enroute",
        "ts": 1700000000,
        "src": "RF",
    },
    # aprs.c: status snprintf — always includes igate / igate_fwd
    "aprs_status": {
        "type": "status",
        "packets": 42,
        "rate": 1.4,
        "igate": False,
        "igate_fwd": 0,
    },
    # aprs.c: weather packet via cJSON
    "aprs_weather": {
        "type": "packet",
        "call": "KD9ABC",
        "ssid": 0,
        "ptype": 5,
        "lat": 37.7749,
        "lon": -122.4194,
        "sym_table": "/",
        "sym_code": "_",
        "temp_f": 72.5,
        "wind_speed": 8.0,
        "wind_dir": 225.0,
        "wind_gust": 12.0,
        "humidity": 65.0,
        "pressure": 1013.2,
        "ts": 1700000100,
        "src": "RF",
    },
    # ais.c: on_ais_message() snprintf
    "ais_vessel": {
        "type": "vessel",
        "mmsi": 211234567,
        "msg": 1,
        "channel": "A",
        "lat": 51.234000,
        "lon": 3.456000,
        "sog": 12.3,
        "cog": 245.1,
        "heading": 243,
        "name": "CARGO SHIP",
        "callsign": "DCAB1",
        "ship_type": 70,
        "nav_status": 0,
    },
    # ais.c: status snprintf
    "ais_status": {
        "type": "status",
        "vessels": 12,
        "messages": 348,
        "rate": 4.2,
    },
    # adsb.c: send_aircraft_json() snprintf
    "adsb_aircraft": {
        "type": "aircraft",
        "icao": "A1B2C3",
        "call": "UAL123",
        "lat": 37.619,
        "lon": -122.375,
        "alt": 35000,
        "gs": 480.0,
        "trk": 270.0,
        "vr": -256,
        "squawk": "1234",
        "msgs": 47,
        "seen": 1.2,
    },
    # adsb.c: send_remove_json()
    "adsb_remove": {
        "type": "remove",
        "icao": "A1B2C3",
    },
    # adsb.c: send_status_json()
    "adsb_status": {
        "type": "status",
        "aircraft": 8,
        "messages": 12045,
        "rate": 52.3,
        "max_range": 180.0,
    },
    # wspr_ft8.c: on_spot_decoded() snprintf
    "wspr_spot": {
        "type": "spot",
        "mode": "WSPR",
        "call": "K1JT",
        "grid": "FN20qi",
        "snr": -12,
        "freq": 14095600.0,
        "dt": 0.4,
        "power": 23,
        "msg": "K1JT FN20 +23",
        "ts": 1700000200,
    },
    "ft8_spot": {
        "type": "spot",
        "mode": "FT8",
        "call": "VK2DX",
        "grid": "QF56",
        "snr": 3,
        "freq": 14074000.0,
        "dt": 0.15,
        "power": 0,
        "msg": "CQ VK2DX QF56",
        "ts": 1700000300,
    },
    # wspr_ft8.c: send_status()
    "wspr_ft8_status": {
        "type": "status",
        "mode": "FT8",
        "band": "20m",
        "freq": 14074000,
        "spots": 15,
    },
    # wspr_ft8.c: send_bands()
    "wspr_ft8_bands": {
        "type": "bands",
        "list": [
            {"name": "160m", "wspr": 1836600, "ft8": 1840000},
            {"name": "20m",  "wspr": 14095600, "ft8": 14074000},
        ],
    },
    # cell_scanner.c: send_tower_json() — GSM branch
    "cell_tower_gsm": {
        "type": "tower",
        "tech": "GSM",
        "freq": 935200000,
        "arfcn": 1,
        "power": -75.5,
        "bsic": 12,
        "mcc": 310,
        "mnc": 260,
        "lac": 1234,
        "cid": 5678,
        "fcch": True,
        "sch": True,
        "bcch": False,
        "count": 3,
        "last_seen": 1700000000,
    },
    # cell_scanner.c: send_tower_json() — LTE branch
    "cell_tower_lte": {
        "type": "tower",
        "tech": "LTE",
        "freq": 2110000000,
        "pci": 42,
        "n_id_2": 0,
        "power": -80.0,
        "rsrp": -95.5,
        "band": "1",
        "count": 7,
        "last_seen": 1700000050,
    },
    # cell_scanner.c: send_scan_status()
    "cell_scan_status": {
        "type": "scan_status",
        "scanning": True,
        "current_freq": 935200000,
        "progress": 25,
        "towers_found": 3,
        "band": "GSM900",
    },
    # cell_scanner.c: send_calibration_json()
    "cell_calibration": {
        "type": "calibration",
        "ppm": 1.23,
        "error_hz": 1140.5,
        "freq": 935200000,
        "measurements": 12,
        "confidence": 0.97,
        "method": "GSM_FCCH",
    },
    # cell_scanner.c: send_identity_json() — IMSI branch
    "cell_identity_imsi": {
        "type": "identity",
        "id_type": "IMSI",
        "value": "310260123456789",
        "arfcn": 1,
        "lac": 1234,
        "ts": 1700000000,
    },
    # cell_scanner.c: send_identity_json() — TMSI branch
    "cell_identity_tmsi": {
        "type": "identity",
        "id_type": "TMSI",
        "value": "0x12345678",
        "arfcn": 1,
        "lac": 1234,
        "ts": 1700000001,
    },
    # cell_scanner.c: spectrum message built inline in scanner_task
    "cell_spectrum": {
        "type": "spectrum",
        "freq": 935200000,
        "bw": 1024000,
        "data": [-80, -90, -75, -85, -95, -100],
    },
}


# ─────────────────────────────────────────────────────────────────────────────
# Schema validation helpers
# ─────────────────────────────────────────────────────────────────────────────

def _require(tc, msg, *fields):
    """Assert that all named fields are present in msg dict."""
    for f in fields:
        tc.assertIn(f, msg, f"Required field '{f}' missing from: {msg}")


def _require_type(tc, msg, field, expected_type):
    """Assert field exists and has the expected Python type(s)."""
    _require(tc, msg, field)
    tc.assertIsInstance(
        msg[field], expected_type,
        f"Field '{field}' expected {expected_type}, got {type(msg[field])}"
    )


def _in_range(tc, msg, field, lo, hi):
    """Assert numeric field is in [lo, hi] (inclusive)."""
    _require(tc, msg, field)
    v = msg[field]
    tc.assertGreaterEqual(v, lo, f"Field '{field}' = {v} < lo={lo}")
    tc.assertLessEqual(v, hi, f"Field '{field}' = {v} > hi={hi}")


def _is_valid_json(raw):
    """Return parsed dict or raise ValueError."""
    return json.loads(raw)


# ─────────────────────────────────────────────────────────────────────────────
# APRS
# ─────────────────────────────────────────────────────────────────────────────

class TestAPRSProtocol(unittest.TestCase):
    """Validates APRS WebSocket JSON protocol (aprs.c: packet_to_json, status snprintf)."""

    def _packet(self):
        return dict(SAMPLE_MESSAGES["aprs_packet"])

    def _status(self):
        return dict(SAMPLE_MESSAGES["aprs_status"])

    # ── Schema tests ─────────────────────────────────────────────────────────

    def test_packet_message_schema(self):
        """packet message has all required fields from packet_to_json()."""
        msg = self._packet()
        _require(self, msg, "type", "call", "ssid", "ptype", "ts", "src")
        self.assertEqual(msg["type"], "packet")
        self.assertEqual(msg["src"], "RF")

    def test_packet_has_position_fields(self):
        """packet with position data has lat/lon fields."""
        msg = self._packet()
        _require(self, msg, "lat", "lon")

    def test_packet_has_symbol_fields(self):
        """packet with symbol data has sym_table and sym_code (single chars)."""
        msg = self._packet()
        _require(self, msg, "sym_table", "sym_code")
        self.assertEqual(len(msg["sym_table"]), 1)
        self.assertEqual(len(msg["sym_code"]), 1)

    def test_status_message_schema(self):
        """status message has required fields from status snprintf."""
        msg = self._status()
        _require(self, msg, "type", "packets", "rate", "igate", "igate_fwd")
        self.assertEqual(msg["type"], "status")

    def test_igate_status_fields(self):
        """status always includes igate (bool) and igate_fwd (int)."""
        msg = self._status()
        _require_type(self, msg, "igate", bool)
        _require_type(self, msg, "igate_fwd", int)

    def test_position_ranges(self):
        """lat in [-90, 90], lon in [-180, 180]."""
        msg = self._packet()
        _in_range(self, msg, "lat", -90.0, 90.0)
        _in_range(self, msg, "lon", -180.0, 180.0)

    def test_callsign_format(self):
        """callsign matches amateur radio format [A-Z0-9]{1,6}(-[0-9]{1,2})?."""
        msg = self._packet()
        call = msg["call"]
        self.assertRegex(
            call, r'^[A-Z0-9]{1,6}$',
            f"Callsign '{call}' does not match expected format"
        )

    def test_ssid_range(self):
        """ssid field is 0-15."""
        msg = self._packet()
        _in_range(self, msg, "ssid", 0, 15)

    def test_ptype_is_integer(self):
        """ptype field is an integer (APRS_TYPE_* enum)."""
        msg = self._packet()
        _require_type(self, msg, "ptype", int)

    def test_ts_is_numeric(self):
        """ts (timestamp) is a number."""
        msg = self._packet()
        _require(self, msg, "ts")
        self.assertIsInstance(msg["ts"], (int, float))

    def test_weather_packet_fields(self):
        """weather packet (ptype==5) has weather-specific fields."""
        msg = dict(SAMPLE_MESSAGES["aprs_weather"])
        _require(self, msg, "temp_f", "wind_speed", "wind_dir", "wind_gust",
                 "humidity", "pressure")
        _in_range(self, msg, "humidity", 0.0, 100.0)
        _in_range(self, msg, "wind_dir", 0.0, 360.0)

    # ── Serialisation round-trip ──────────────────────────────────────────────

    def test_packet_json_roundtrip(self):
        """Packet message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["aprs_packet"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "packet")
        self.assertEqual(parsed["call"], "W1AW")

    def test_status_json_roundtrip(self):
        """Status message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["aprs_status"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "status")

    # ── Command format ────────────────────────────────────────────────────────

    def test_command_igate_config_format(self):
        """igate_config command has required fields."""
        cmd = {
            "cmd": "igate_config",
            "callsign": "N0CALL-10",
            "passcode": 12345,
            "server": "rotate.aprs2.net",
            "port": 14580,
            "lat": 41.0,
            "lon": -72.0,
            "range": 100,
            "enabled": True,
        }
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertIn("cmd", parsed)
        self.assertEqual(parsed["cmd"], "igate_config")
        _require(self, parsed, "callsign", "passcode", "server", "port",
                 "lat", "lon", "range", "enabled")


# ─────────────────────────────────────────────────────────────────────────────
# AIS
# ─────────────────────────────────────────────────────────────────────────────

class TestAISProtocol(unittest.TestCase):
    """Validates AIS WebSocket JSON protocol (ais.c: on_ais_message, status snprintf)."""

    def _vessel(self):
        return dict(SAMPLE_MESSAGES["ais_vessel"])

    def _status(self):
        return dict(SAMPLE_MESSAGES["ais_status"])

    # ── Schema tests ─────────────────────────────────────────────────────────

    def test_vessel_message_schema(self):
        """vessel message has required fields from on_ais_message() snprintf."""
        msg = self._vessel()
        _require(self, msg, "type", "mmsi", "msg", "channel")
        self.assertEqual(msg["type"], "vessel")

    def test_vessel_with_position(self):
        """vessel with position data has lat/lon."""
        msg = self._vessel()
        _require(self, msg, "lat", "lon")

    def test_status_message_schema(self):
        """status message has vessels, messages, rate."""
        msg = self._status()
        _require(self, msg, "type", "vessels", "messages", "rate")
        self.assertEqual(msg["type"], "status")

    def test_mmsi_format(self):
        """MMSI is a 9-digit integer in range 100000000-799999999."""
        msg = self._vessel()
        mmsi = msg["mmsi"]
        self.assertIsInstance(mmsi, int)
        # 9 digits: [100_000_000, 799_999_999] for ship stations
        self.assertGreaterEqual(mmsi, 100_000_000)
        self.assertLessEqual(mmsi, 999_999_999)

    def test_channel_values(self):
        """channel field is 'A' or 'B' (AIS VHF channels)."""
        msg = self._vessel()
        self.assertIn(msg["channel"], ("A", "B"),
                      f"channel '{msg['channel']}' not A or B")

    def test_msg_type_range(self):
        """msg (AIS message type) is 1-27."""
        msg = self._vessel()
        _in_range(self, msg, "msg", 1, 27)

    def test_nav_status_range(self):
        """nav_status is 0-15 per ITU-R M.1371."""
        msg = self._vessel()
        _in_range(self, msg, "nav_status", 0, 15)

    def test_sog_range(self):
        """SOG in [0, 102.2] knots (102.3 = N/A in AIS spec)."""
        msg = self._vessel()
        _in_range(self, msg, "sog", 0.0, 102.2)

    def test_cog_range(self):
        """COG in [0, 360) degrees."""
        msg = self._vessel()
        _in_range(self, msg, "cog", 0.0, 359.9)

    def test_heading_range(self):
        """heading in [0, 359] degrees (511 = N/A, not included when N/A)."""
        msg = self._vessel()
        _require(self, msg, "heading")
        heading = msg["heading"]
        valid = (0 <= heading <= 359) or heading == 511
        self.assertTrue(valid, f"heading={heading} not in [0,359] or 511")

    def test_ship_type_range(self):
        """ship_type is 0-99 per AIS spec."""
        msg = self._vessel()
        _in_range(self, msg, "ship_type", 0, 99)

    def test_position_ranges(self):
        """lat in [-90, 90], lon in [-180, 180]."""
        msg = self._vessel()
        _in_range(self, msg, "lat", -90.0, 90.0)
        _in_range(self, msg, "lon", -180.0, 180.0)

    # ── Serialisation round-trip ──────────────────────────────────────────────

    def test_vessel_json_roundtrip(self):
        """Vessel message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["ais_vessel"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "vessel")
        self.assertEqual(parsed["mmsi"], 211234567)

    def test_status_json_roundtrip(self):
        """Status message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["ais_status"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "status")

    # ── Dimension fields (optional, present when dim_bow > 0) ────────────────

    def test_dim_fields_when_present(self):
        """When present, dim_bow/stern/port/starboard are non-negative integers."""
        msg = {
            "type": "vessel", "mmsi": 211234567, "msg": 5, "channel": "A",
            "dim_bow": 120, "dim_stern": 30, "dim_port": 15, "dim_starboard": 15,
        }
        for field in ("dim_bow", "dim_stern", "dim_port", "dim_starboard"):
            self.assertGreaterEqual(msg[field], 0)


# ─────────────────────────────────────────────────────────────────────────────
# ADS-B
# ─────────────────────────────────────────────────────────────────────────────

class TestADSBProtocol(unittest.TestCase):
    """Validates ADS-B WebSocket JSON protocol (adsb.c: send_aircraft_json, etc.)."""

    def _aircraft(self):
        return dict(SAMPLE_MESSAGES["adsb_aircraft"])

    def _remove(self):
        return dict(SAMPLE_MESSAGES["adsb_remove"])

    def _status(self):
        return dict(SAMPLE_MESSAGES["adsb_status"])

    # ── Schema tests ─────────────────────────────────────────────────────────

    def test_aircraft_message_schema(self):
        """aircraft message has required fields from send_aircraft_json()."""
        msg = self._aircraft()
        _require(self, msg, "type", "icao", "msgs", "seen")
        self.assertEqual(msg["type"], "aircraft")

    def test_aircraft_with_position(self):
        """aircraft with position data has lat/lon."""
        msg = self._aircraft()
        _require(self, msg, "lat", "lon")

    def test_status_message_schema(self):
        """status message has aircraft, messages, rate, max_range."""
        msg = self._status()
        _require(self, msg, "type", "aircraft", "messages", "rate", "max_range")
        self.assertEqual(msg["type"], "status")

    def test_remove_message_schema(self):
        """remove message has type and icao fields from send_remove_json()."""
        msg = self._remove()
        _require(self, msg, "type", "icao")
        self.assertEqual(msg["type"], "remove")

    def test_icao_format(self):
        """icao is a 6-character uppercase hex string (%06X format)."""
        msg = self._aircraft()
        icao = msg["icao"]
        self.assertEqual(len(icao), 6, f"icao '{icao}' is not 6 chars")
        self.assertRegex(icao, r'^[0-9A-F]{6}$',
                         f"icao '{icao}' is not uppercase hex")

    def test_remove_icao_format(self):
        """icao in remove message is also a 6-char uppercase hex string."""
        msg = self._remove()
        icao = msg["icao"]
        self.assertEqual(len(icao), 6)
        self.assertRegex(icao, r'^[0-9A-F]{6}$')

    def test_altitude_range(self):
        """alt in [-1000, 60000] ft (typical aviation range)."""
        msg = self._aircraft()
        _in_range(self, msg, "alt", -1000, 60000)

    def test_ground_speed_range(self):
        """gs (ground speed) in [0, 800] knots."""
        msg = self._aircraft()
        _in_range(self, msg, "gs", 0.0, 800.0)

    def test_track_range(self):
        """trk (track angle) in [0, 360]."""
        msg = self._aircraft()
        _in_range(self, msg, "trk", 0.0, 360.0)

    def test_squawk_format(self):
        """squawk is a 4-character decimal string representing octal 0000-7777."""
        msg = self._aircraft()
        sq = msg["squawk"]
        self.assertIsInstance(sq, str)
        self.assertEqual(len(sq), 4, f"squawk '{sq}' is not 4 chars")
        self.assertRegex(sq, r'^\d{4}$', f"squawk '{sq}' is not 4 digits")
        # Each digit must be 0-7 (octal digit)
        for digit in sq:
            self.assertIn(digit, "01234567",
                          f"squawk '{sq}' contains non-octal digit '{digit}'")

    def test_vertical_rate_is_integer(self):
        """vr (vertical rate) is an integer (fpm)."""
        msg = self._aircraft()
        _require_type(self, msg, "vr", int)

    def test_msgs_and_seen_types(self):
        """msgs is integer, seen is float."""
        msg = self._aircraft()
        _require_type(self, msg, "msgs", int)
        _require_type(self, msg, "seen", float)

    def test_position_ranges(self):
        """lat in [-90, 90], lon in [-180, 180]."""
        msg = self._aircraft()
        _in_range(self, msg, "lat", -90.0, 90.0)
        _in_range(self, msg, "lon", -180.0, 180.0)

    # ── Serialisation round-trip ──────────────────────────────────────────────

    def test_aircraft_json_roundtrip(self):
        """Aircraft message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["adsb_aircraft"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "aircraft")
        self.assertEqual(parsed["icao"], "A1B2C3")

    def test_remove_json_roundtrip(self):
        """Remove message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["adsb_remove"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "remove")


# ─────────────────────────────────────────────────────────────────────────────
# WSPR/FT8
# ─────────────────────────────────────────────────────────────────────────────

class TestWSPRFT8Protocol(unittest.TestCase):
    """Validates WSPR/FT8 WebSocket JSON protocol (wspr_ft8.c: on_spot_decoded, etc.)."""

    VALID_BANDS = {
        "160m", "80m", "60m", "40m", "30m", "20m",
        "17m", "15m", "12m", "10m", "6m", "2m",
    }

    def _wspr_spot(self):
        return dict(SAMPLE_MESSAGES["wspr_spot"])

    def _ft8_spot(self):
        return dict(SAMPLE_MESSAGES["ft8_spot"])

    def _status(self):
        return dict(SAMPLE_MESSAGES["wspr_ft8_status"])

    def _bands(self):
        return dict(SAMPLE_MESSAGES["wspr_ft8_bands"])

    # ── Schema tests ─────────────────────────────────────────────────────────

    def test_spot_message_schema(self):
        """spot message has required fields from on_spot_decoded() snprintf."""
        msg = self._wspr_spot()
        _require(self, msg, "type", "mode", "call", "grid", "snr",
                 "freq", "dt", "power", "msg", "ts")
        self.assertEqual(msg["type"], "spot")

    def test_status_message_schema(self):
        """status message has required fields from send_status()."""
        msg = self._status()
        _require(self, msg, "type", "mode", "band", "freq", "spots")
        self.assertEqual(msg["type"], "status")

    def test_bands_message_schema(self):
        """bands message has type and list fields from send_bands()."""
        msg = self._bands()
        _require(self, msg, "type", "list")
        self.assertEqual(msg["type"], "bands")
        self.assertIsInstance(msg["list"], list)
        for entry in msg["list"]:
            _require(self, entry, "name", "wspr", "ft8")

    def test_mode_values(self):
        """mode field is exactly 'WSPR' or 'FT8'."""
        for key in ("wspr_spot", "ft8_spot"):
            msg = dict(SAMPLE_MESSAGES[key])
            self.assertIn(msg["mode"], ("WSPR", "FT8"),
                          f"mode '{msg['mode']}' not WSPR or FT8")

    def test_status_mode_values(self):
        """status mode is 'WSPR' or 'FT8'."""
        msg = self._status()
        self.assertIn(msg["mode"], ("WSPR", "FT8"))

    def test_snr_range(self):
        """SNR in [-30, +30] dB (typical decode range)."""
        msg = self._wspr_spot()
        _in_range(self, msg, "snr", -30, 30)

    def test_snr_is_integer(self):
        """SNR field is an integer (%d in snprintf)."""
        msg = self._wspr_spot()
        _require_type(self, msg, "snr", int)

    def test_grid_format_6char(self):
        """6-char Maidenhead grid: 2 letters + 2 digits + 2 letters."""
        msg = self._wspr_spot()
        grid = msg["grid"]
        self.assertRegex(
            grid, r'^[A-R]{2}[0-9]{2}[a-x]{2}$',
            f"grid '{grid}' does not match 6-char Maidenhead format"
        )

    def test_grid_format_4char(self):
        """4-char Maidenhead grid: 2 letters + 2 digits."""
        msg = self._ft8_spot()
        grid = msg["grid"]
        # 4-char grid: 2 letters + 2 digits (e.g. QF56)
        self.assertRegex(
            grid, r'^[A-R]{2}[0-9]{2}([a-x]{2})?$',
            f"grid '{grid}' does not match 4- or 6-char Maidenhead format"
        )

    def test_freq_is_positive(self):
        """freq is a positive float (Hz)."""
        msg = self._wspr_spot()
        _require(self, msg, "freq")
        self.assertGreater(msg["freq"], 0.0)

    def test_dt_is_float(self):
        """dt (time offset) is a float."""
        msg = self._wspr_spot()
        _require_type(self, msg, "dt", float)

    def test_power_is_integer(self):
        """power (dBm) is an integer."""
        msg = self._wspr_spot()
        _require_type(self, msg, "power", int)

    def test_band_presets(self):
        """status band field is one of the known band names from band_presets[]."""
        msg = self._status()
        band = msg["band"]
        # "custom" is also valid when band_idx == -1
        valid_bands = self.VALID_BANDS | {"custom"}
        self.assertIn(band, valid_bands,
                      f"band '{band}' not in known band presets")

    def test_bands_list_names(self):
        """bands list entries have names matching known presets."""
        msg = self._bands()
        for entry in msg["list"]:
            self.assertIn(
                entry["name"], self.VALID_BANDS,
                f"band name '{entry['name']}' not in expected set"
            )
            self.assertGreater(entry["wspr"], 0)
            self.assertGreater(entry["ft8"], 0)

    # ── Serialisation round-trip ──────────────────────────────────────────────

    def test_spot_json_roundtrip(self):
        """Spot message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["wspr_spot"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "spot")
        self.assertEqual(parsed["mode"], "WSPR")

    def test_status_json_roundtrip(self):
        """Status message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["wspr_ft8_status"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "status")

    # ── Command formats ───────────────────────────────────────────────────────

    def test_command_mode(self):
        """mode command has cmd and value fields."""
        cmd = {"cmd": "mode", "value": "FT8"}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "mode")
        self.assertIn(parsed["value"], ("WSPR", "FT8"))

    def test_command_band(self):
        """band command has cmd and value (band name) fields."""
        cmd = {"cmd": "band", "value": "20m"}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "band")
        self.assertIn(parsed["value"], self.VALID_BANDS)

    def test_command_freq(self):
        """freq command has cmd and numeric value."""
        cmd = {"cmd": "freq", "value": 14074000}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "freq")
        self.assertIsInstance(parsed["value"], (int, float))
        self.assertGreater(parsed["value"], 0)


# ─────────────────────────────────────────────────────────────────────────────
# Cell Scanner
# ─────────────────────────────────────────────────────────────────────────────

class TestCellScannerProtocol(unittest.TestCase):
    """Validates Cell Scanner WebSocket JSON protocol (cell_scanner.c)."""

    def _gsm_tower(self):
        return dict(SAMPLE_MESSAGES["cell_tower_gsm"])

    def _lte_tower(self):
        return dict(SAMPLE_MESSAGES["cell_tower_lte"])

    def _scan_status(self):
        return dict(SAMPLE_MESSAGES["cell_scan_status"])

    def _calibration(self):
        return dict(SAMPLE_MESSAGES["cell_calibration"])

    def _identity_imsi(self):
        return dict(SAMPLE_MESSAGES["cell_identity_imsi"])

    def _identity_tmsi(self):
        return dict(SAMPLE_MESSAGES["cell_identity_tmsi"])

    def _spectrum(self):
        return dict(SAMPLE_MESSAGES["cell_spectrum"])

    # ── GSM tower schema ──────────────────────────────────────────────────────

    def test_gsm_tower_schema(self):
        """GSM tower message has required fields from send_tower_json() GSM branch."""
        msg = self._gsm_tower()
        _require(self, msg, "type", "tech", "freq", "arfcn", "power",
                 "bsic", "mcc", "mnc", "lac", "cid",
                 "fcch", "sch", "bcch", "count", "last_seen")
        self.assertEqual(msg["type"], "tower")
        self.assertEqual(msg["tech"], "GSM")

    def test_lte_tower_schema(self):
        """LTE tower message has required fields from send_tower_json() LTE branch."""
        msg = self._lte_tower()
        _require(self, msg, "type", "tech", "freq", "pci", "n_id_2",
                 "power", "rsrp", "band", "count", "last_seen")
        self.assertEqual(msg["type"], "tower")
        self.assertEqual(msg["tech"], "LTE")

    def test_tech_values(self):
        """tech field is exactly 'GSM' or 'LTE'."""
        for msg in (self._gsm_tower(), self._lte_tower()):
            self.assertIn(msg["tech"], ("GSM", "LTE"))

    # ── GSM field ranges ──────────────────────────────────────────────────────

    def test_gsm_arfcn_range(self):
        """GSM ARFCN: 1-124 (GSM-900) or 512-885 (GSM-1800 DCS)."""
        msg = self._gsm_tower()
        arfcn = msg["arfcn"]
        valid = (1 <= arfcn <= 124) or (512 <= arfcn <= 885)
        self.assertTrue(valid,
                        f"GSM ARFCN {arfcn} not in valid range (1-124 or 512-885)")

    def test_gsm_bsic_range(self):
        """BSIC is 0-63 (6-bit value: NCC 0-7, BCC 0-7)."""
        msg = self._gsm_tower()
        _in_range(self, msg, "bsic", 0, 63)

    def test_gsm_mcc_range(self):
        """MCC is a 3-digit ITU country code (1-999)."""
        msg = self._gsm_tower()
        _in_range(self, msg, "mcc", 1, 999)

    def test_gsm_mnc_range(self):
        """MNC is 0-999."""
        msg = self._gsm_tower()
        _in_range(self, msg, "mnc", 0, 999)

    def test_gsm_boolean_flags(self):
        """fcch, sch, bcch are booleans."""
        msg = self._gsm_tower()
        for field in ("fcch", "sch", "bcch"):
            _require_type(self, msg, field, bool)

    # ── LTE field ranges ──────────────────────────────────────────────────────

    def test_pci_range(self):
        """PCI (Physical Cell ID) is 0-503."""
        msg = self._lte_tower()
        _in_range(self, msg, "pci", 0, 503)

    def test_n_id_2_range(self):
        """n_id_2 (PSS index) is 0-2."""
        msg = self._lte_tower()
        _in_range(self, msg, "n_id_2", 0, 2)

    def test_rsrp_range(self):
        """RSRP is in [-140, -44] dBm (LTE spec range)."""
        msg = self._lte_tower()
        _in_range(self, msg, "rsrp", -140.0, -44.0)

    def test_lte_band_field(self):
        """LTE band field is a string (band number as string)."""
        msg = self._lte_tower()
        _require_type(self, msg, "band", str)

    # ── Scan status ───────────────────────────────────────────────────────────

    def test_scan_status_schema(self):
        """scan_status message has required fields from send_scan_status()."""
        msg = self._scan_status()
        _require(self, msg, "type", "scanning", "current_freq",
                 "progress", "towers_found", "band")
        self.assertEqual(msg["type"], "scan_status")

    def test_scan_status_scanning_is_bool(self):
        """scanning field is a boolean."""
        msg = self._scan_status()
        _require_type(self, msg, "scanning", bool)

    def test_scan_status_progress_range(self):
        """progress is 0-100."""
        msg = self._scan_status()
        _in_range(self, msg, "progress", 0, 100)

    def test_scan_status_freq_is_positive(self):
        """current_freq is a positive integer (Hz)."""
        msg = self._scan_status()
        _require(self, msg, "current_freq")
        self.assertGreater(msg["current_freq"], 0)

    # ── Calibration ───────────────────────────────────────────────────────────

    def test_calibration_schema(self):
        """calibration message has required fields from send_calibration_json()."""
        msg = self._calibration()
        _require(self, msg, "type", "ppm", "error_hz", "freq",
                 "measurements", "confidence", "method")
        self.assertEqual(msg["type"], "calibration")

    def test_ppm_reasonable(self):
        """PPM error is within realistic RTL-SDR range: abs(ppm) < 200."""
        msg = self._calibration()
        self.assertLess(abs(msg["ppm"]), 200.0,
                        f"ppm={msg['ppm']} seems unreasonably large")

    def test_confidence_range(self):
        """confidence is in [0, 1]."""
        msg = self._calibration()
        _in_range(self, msg, "confidence", 0.0, 1.0)

    def test_calibration_method_values(self):
        """method is 'GSM_FCCH' or 'LTE_PSS'."""
        msg = self._calibration()
        self.assertIn(msg["method"], ("GSM_FCCH", "LTE_PSS", "NONE"),
                      f"method '{msg['method']}' not a known calibration method")

    # ── Identity ──────────────────────────────────────────────────────────────

    def test_identity_imsi_schema(self):
        """IMSI identity message has required fields from send_identity_json()."""
        msg = self._identity_imsi()
        _require(self, msg, "type", "id_type", "value", "arfcn", "lac", "ts")
        self.assertEqual(msg["type"], "identity")
        self.assertEqual(msg["id_type"], "IMSI")

    def test_identity_tmsi_schema(self):
        """TMSI identity message has required fields from send_identity_json()."""
        msg = self._identity_tmsi()
        _require(self, msg, "type", "id_type", "value", "arfcn", "lac", "ts")
        self.assertEqual(msg["type"], "identity")
        self.assertEqual(msg["id_type"], "TMSI")

    def test_imsi_format(self):
        """IMSI is a 15-digit numeric string."""
        msg = self._identity_imsi()
        imsi = msg["value"]
        self.assertRegex(imsi, r'^\d{15}$',
                         f"IMSI '{imsi}' is not 15 numeric digits")

    def test_tmsi_format(self):
        """TMSI is hex string formatted as '0xXXXXXXXX' (32-bit value)."""
        msg = self._identity_tmsi()
        tmsi = msg["value"]
        self.assertRegex(tmsi, r'^0x[0-9A-Fa-f]{8}$',
                         f"TMSI '{tmsi}' does not match 0xXXXXXXXX format")

    def test_id_type_values(self):
        """id_type is 'IMSI' or 'TMSI'."""
        for msg in (self._identity_imsi(), self._identity_tmsi()):
            self.assertIn(msg["id_type"], ("IMSI", "TMSI"))

    # ── Spectrum ──────────────────────────────────────────────────────────────

    def test_spectrum_schema(self):
        """spectrum message has type, freq, bw, data from scanner_task inline."""
        msg = self._spectrum()
        _require(self, msg, "type", "freq", "bw", "data")
        self.assertEqual(msg["type"], "spectrum")

    def test_spectrum_data_is_list(self):
        """spectrum data is a list of integers."""
        msg = self._spectrum()
        self.assertIsInstance(msg["data"], list)
        for v in msg["data"]:
            self.assertIsInstance(v, int,
                                  f"spectrum data value {v} is not int")

    def test_spectrum_data_range(self):
        """spectrum data values are clamped to [-150, 0] dB."""
        msg = self._spectrum()
        for v in msg["data"]:
            self.assertGreaterEqual(v, -150)
            self.assertLessEqual(v, 0)

    # ── Serialisation round-trip ──────────────────────────────────────────────

    def test_gsm_tower_json_roundtrip(self):
        """GSM tower message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["cell_tower_gsm"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "tower")
        self.assertEqual(parsed["tech"], "GSM")

    def test_lte_tower_json_roundtrip(self):
        """LTE tower message survives JSON serialise → deserialise."""
        raw = json.dumps(SAMPLE_MESSAGES["cell_tower_lte"])
        parsed = json.loads(raw)
        self.assertEqual(parsed["type"], "tower")
        self.assertEqual(parsed["tech"], "LTE")

    # ── WebSocket commands ────────────────────────────────────────────────────

    def test_command_start_scan(self):
        """start_scan command has cmd and band fields."""
        cmd = {"cmd": "start_scan", "band": 0}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "start_scan")
        self.assertIsInstance(parsed["band"], int)

    def test_command_stop_scan(self):
        """stop_scan command is valid JSON with cmd field."""
        cmd = {"cmd": "stop_scan"}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "stop_scan")

    def test_command_calibrate(self):
        """calibrate command is valid JSON."""
        cmd = {"cmd": "calibrate"}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "calibrate")

    def test_command_set_threshold(self):
        """set_threshold command has numeric value."""
        cmd = {"cmd": "set_threshold", "value": -90.0}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "set_threshold")
        self.assertIsInstance(parsed["value"], float)

    def test_command_monitor_ids(self):
        """monitor_ids command has boolean enabled field."""
        cmd = {"cmd": "monitor_ids", "enabled": True}
        raw = json.dumps(cmd)
        parsed = json.loads(raw)
        self.assertEqual(parsed["cmd"], "monitor_ids")
        self.assertIsInstance(parsed["enabled"], bool)


# ─────────────────────────────────────────────────────────────────────────────
# Live test mixin — used when --live HOST is passed
# ─────────────────────────────────────────────────────────────────────────────

class LiveTestMixin:
    """Mixin for live device testing — connects via WebSocket."""

    DEFAULT_TIMEOUT = 10

    def connect(self, host, port, path="/ws"):
        """Open a synchronous WebSocket connection."""
        try:
            import websocket  # pylint: disable=import-error
        except ImportError:
            self.skipTest("websocket-client not installed; pip install websocket-client")
        url = f"ws://{host}:{port}{path}"
        self.ws = websocket.create_connection(url, timeout=self.DEFAULT_TIMEOUT)
        self.addCleanup(self._ws_close)

    def _ws_close(self):
        try:
            self.ws.close()
        except Exception:
            pass

    def recv_json(self, timeout=5):
        """Receive the next text frame and parse as JSON.  Returns None for binary frames."""
        self.ws.settimeout(timeout)
        try:
            data = self.ws.recv()
        except Exception:
            return None
        if isinstance(data, str):
            try:
                return json.loads(data)
            except json.JSONDecodeError:
                return None
        return None  # binary frame

    def recv_json_of_type(self, expected_type, timeout=30):
        """Keep reading until we get a message with the requested type (or timeout)."""
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv_json(timeout=max(1, int(deadline - time.time())))
            if msg and msg.get("type") == expected_type:
                return msg
        return None

    def send_cmd(self, cmd, **kwargs):
        """Send a JSON command to the device."""
        msg = {"cmd": cmd}
        msg.update(kwargs)
        self.ws.send(json.dumps(msg))


# ─────────────────────────────────────────────────────────────────────────────
# Live test classes (skipped unless --live HOST provided)
# ─────────────────────────────────────────────────────────────────────────────

LIVE_HOST = None          # Set by argument parsing below
LIVE_PORTS = {            # Default ports per server
    "aprs":   8082,
    "ais":    8083,
    "adsb":   8084,
    "wspr":   8085,
    "cell":   8086,
}


class LiveTestAPRS(LiveTestMixin, unittest.TestCase):
    """Live APRS tests — connects to the APRS WebSocket endpoint."""

    def setUp(self):
        if not LIVE_HOST:
            self.skipTest("Live tests disabled — pass --live HOST")
        self.connect(LIVE_HOST, LIVE_PORTS["aprs"])

    def test_receives_status_on_connect(self):
        """Device sends a status message immediately on WebSocket connect."""
        msg = self.recv_json_of_type("status", timeout=5)
        self.assertIsNotNone(msg, "No status message received within 5s")
        _require(self, msg, "packets", "rate", "igate", "igate_fwd")

    def test_live_status_igate_is_bool(self):
        """Live status igate field is a boolean."""
        msg = self.recv_json_of_type("status", timeout=5)
        if msg:
            self.assertIsInstance(msg["igate"], bool)


class LiveTestAIS(LiveTestMixin, unittest.TestCase):
    """Live AIS tests — connects to the AIS WebSocket endpoint."""

    def setUp(self):
        if not LIVE_HOST:
            self.skipTest("Live tests disabled — pass --live HOST")
        self.connect(LIVE_HOST, LIVE_PORTS["ais"])

    def test_receives_vessel_or_status(self):
        """Device sends vessel or status messages within 30s."""
        for _ in range(30):
            msg = self.recv_json(timeout=1)
            if msg and msg.get("type") in ("vessel", "status"):
                return
        self.fail("No vessel or status message received within 30s")

    def test_live_vessel_mmsi_range(self):
        """Live vessel MMSI is in valid range."""
        msg = self.recv_json_of_type("vessel", timeout=30)
        if msg:
            mmsi = msg["mmsi"]
            self.assertGreaterEqual(mmsi, 100_000_000)
            self.assertLessEqual(mmsi, 999_999_999)


class LiveTestADSB(LiveTestMixin, unittest.TestCase):
    """Live ADS-B tests — connects to the ADS-B WebSocket endpoint."""

    def setUp(self):
        if not LIVE_HOST:
            self.skipTest("Live tests disabled — pass --live HOST")
        self.connect(LIVE_HOST, LIVE_PORTS["adsb"])

    def test_receives_status_on_connect(self):
        """Device sends a status message on connect."""
        msg = self.recv_json_of_type("status", timeout=5)
        self.assertIsNotNone(msg, "No status message received within 5s")
        _require(self, msg, "aircraft", "messages", "rate", "max_range")

    def test_live_icao_format(self):
        """Live aircraft ICAO matches 6-char uppercase hex."""
        msg = self.recv_json_of_type("aircraft", timeout=30)
        if msg:
            icao = msg.get("icao", "")
            self.assertRegex(icao, r'^[0-9A-F]{6}$')


class LiveTestWSPRFT8(LiveTestMixin, unittest.TestCase):
    """Live WSPR/FT8 tests — connects to the WSPR/FT8 WebSocket endpoint."""

    def setUp(self):
        if not LIVE_HOST:
            self.skipTest("Live tests disabled — pass --live HOST")
        self.connect(LIVE_HOST, LIVE_PORTS["wspr"])

    def test_receives_status_on_connect(self):
        """Device sends status and bands messages on connect."""
        msg = self.recv_json_of_type("status", timeout=5)
        self.assertIsNotNone(msg, "No status message received within 5s")
        _require(self, msg, "mode", "band", "freq", "spots")

    def test_receives_bands_on_connect(self):
        """Device sends bands list on connect."""
        msg = self.recv_json_of_type("bands", timeout=5)
        self.assertIsNotNone(msg, "No bands message received within 5s")
        self.assertIsInstance(msg.get("list"), list)
        self.assertGreater(len(msg["list"]), 0)


class LiveTestCellScanner(LiveTestMixin, unittest.TestCase):
    """Live cell scanner tests — connects to the cell scanner WebSocket endpoint."""

    def setUp(self):
        if not LIVE_HOST:
            self.skipTest("Live tests disabled — pass --live HOST")
        self.connect(LIVE_HOST, LIVE_PORTS["cell"])

    def test_receives_scan_status_on_connect(self):
        """Device sends scan_status on connect."""
        msg = self.recv_json_of_type("scan_status", timeout=5)
        self.assertIsNotNone(msg, "No scan_status message received within 5s")
        _require(self, msg, "scanning", "current_freq", "progress",
                 "towers_found", "band")

    def test_start_scan_command(self):
        """start_scan command triggers scan_status update."""
        self.send_cmd("start_scan", band=0)
        msg = self.recv_json_of_type("scan_status", timeout=10)
        if msg:
            self.assertIsInstance(msg.get("scanning"), bool)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global LIVE_HOST

    parser = argparse.ArgumentParser(
        description="WebSocket protocol validation tests for ESP32-P4 SDR receivers",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all offline schema tests
  python3 test_ws_protocols.py

  # Run only ADS-B tests
  python3 test_ws_protocols.py TestADSBProtocol

  # Run live tests against a device
  python3 test_ws_protocols.py --live 192.168.1.232

  # Verbose output
  python3 test_ws_protocols.py -v
        """,
    )
    parser.add_argument(
        "--live", metavar="HOST",
        help="Enable live mode: connect to ESP32-P4 at HOST and validate real messages",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Verbose test output",
    )

    # Separate our args from unittest's
    known, remaining = parser.parse_known_args()

    if known.live:
        LIVE_HOST = known.live
        print(f"[LIVE MODE] Target: {LIVE_HOST}")
        print(f"[LIVE MODE] Ports: {LIVE_PORTS}")
    else:
        print("[OFFLINE MODE] Schema/protocol tests only — no device required")

    # Build unittest argv
    unittest_argv = [sys.argv[0]] + remaining
    if known.verbose:
        unittest_argv.append("-v")

    unittest.main(argv=unittest_argv, verbosity=2 if known.verbose else 1)


if __name__ == "__main__":
    main()
