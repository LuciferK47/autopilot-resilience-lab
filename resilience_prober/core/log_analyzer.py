"""
Log Analyzer — Post‑flight .BIN DataFlash log analysis.

Extracts EKF innovations (XKF3 / NKF3), EKF variances (XKF4 / NKF4),
attitude (ATT), position (POS), mode changes (MODE), errors (ERR),
and events (EV) from a DataFlash .BIN file produced by SITL.

Falls back gracefully if the log is too large or DFReader times out.
"""

import os
import time
from typing import Dict, List, Optional


def _try_import_dfr():
    """Import DFReader; return None on failure."""
    try:
        from pymavlink import mavutil
        return mavutil
    except ImportError:
        return None


class LogAnalyzer:
    """Analyze a post‑flight .BIN log file."""

    # Message types we care about
    WANTED = {
        "ATT":  ["TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch",
                  "DesYaw", "Yaw"],
        "POS":  ["TimeUS", "Lat", "Lng", "Alt", "RelHomeAlt"],
        "MODE": ["TimeUS", "Mode", "ModeNum", "Rsn"],
        "ERR":  ["TimeUS", "Subsys", "ECode"],
        "EV":   ["TimeUS", "Id"],
        "BAT":  ["TimeUS", "Volt", "Curr", "CurrTot"],
        "VIBE": ["TimeUS", "VibeX", "VibeY", "VibeZ", "Clip"],
        # EKF3 messages (preferred)
        "XKF3": ["TimeUS", "IVN", "IVE", "IVD", "IPN", "IPE", "IPD",
                  "IMX", "IMY", "IMZ", "ErSc"],
        "XKF4": ["TimeUS", "SV", "SP", "SH", "SM", "SVT", "errRP",
                  "OFN", "OFE", "FS", "TS", "SS", "GPS", "PI"],
        # EKF2 messages (fallback)
        "NKF3": ["TimeUS", "IVN", "IVE", "IVD", "IPN", "IPE", "IPD",
                  "IMX", "IMY", "IMZ"],
        "NKF4": ["TimeUS", "SV", "SP", "SH", "SM", "SVT", "errRP",
                  "OFN", "OFE", "FS", "TS", "SS", "GPS"],
    }

    def __init__(self, log_path: str, timeout: float = 120):
        self.log_path = log_path
        self.timeout = timeout
        self.data: Dict[str, Dict[str, list]] = {}
        self._parsed = False

    def analyze(self) -> dict:
        """Parse the log and return structured results.

        Returns a dict with keys:
            data       — {msg_type: {field: [values]}}
            metrics    — computed quality / drift metrics
            mode_changes — list of mode transition dicts
            errors     — list of error dicts
            success    — True if parse completed
        """
        if not os.path.exists(self.log_path):
            print(f"[LOG] File not found: {self.log_path}")
            return self._empty_result("file not found")

        size_mb = os.path.getsize(self.log_path) / (1024 * 1024)
        print(f"[LOG] Analyzing {self.log_path} ({size_mb:.1f} MB)...")

        if size_mb > 200:
            print("[LOG] Log too large (>200 MB) — skipping deep analysis")
            return self._empty_result("log too large")

        try:
            self._parse()
        except Exception as e:
            print(f"[LOG] Parse error: {e}")
            return self._empty_result(str(e))

        metrics = self._compute_metrics()
        mode_changes = self._extract_mode_changes()
        errors = self._extract_errors()

        print(f"[LOG] Analysis complete — "
              f"{sum(len(v.get(list(v.keys())[0] if v else '', [])) for v in self.data.values() if v)} "
              f"data points across {len(self.data)} message types")

        return {
            "data":         self.data,
            "metrics":      metrics,
            "mode_changes": mode_changes,
            "errors":       errors,
            "success":      True,
        }

    # ── Parse ────────────────────────────────────────────────────

    def _parse(self):
        mu = _try_import_dfr()
        if mu is None:
            raise ImportError("pymavlink not available")

        mlog = mu.mavlink_connection(self.log_path)
        deadline = time.time() + self.timeout

        for msg_type, fields in self.WANTED.items():
            self.data[msg_type] = {f: [] for f in fields}

        count = 0
        while time.time() < deadline:
            msg = mlog.recv_msg()
            if msg is None:
                break
            mt = msg.get_type()
            if mt in self.data:
                for f in self.WANTED[mt]:
                    try:
                        self.data[mt][f].append(getattr(msg, f))
                    except AttributeError:
                        self.data[mt][f].append(None)
            count += 1

        self._parsed = True
        print(f"[LOG] Parsed {count} messages")

    # ── Metric computation ───────────────────────────────────────

    def _compute_metrics(self) -> dict:
        metrics = {}

        # EKF velocity variance (from XKF4 or NKF4)
        ekf4 = self.data.get("XKF4") or self.data.get("NKF4") or {}
        sv = ekf4.get("SV", [])
        if sv:
            valid = [v for v in sv if v is not None]
            if valid:
                metrics["ekf_vel_var_mean"] = sum(valid) / len(valid)
                metrics["ekf_vel_var_max"]  = max(valid)

        sp = ekf4.get("SP", [])
        if sp:
            valid = [v for v in sp if v is not None]
            if valid:
                metrics["ekf_pos_var_mean"] = sum(valid) / len(valid)
                metrics["ekf_pos_var_max"]  = max(valid)

        # Attitude error (from ATT)
        att = self.data.get("ATT", {})
        if att.get("Roll") and att.get("DesRoll"):
            roll_err = [abs((r or 0) - (d or 0))
                        for r, d in zip(att["Roll"], att["DesRoll"])]
            if roll_err:
                metrics["roll_err_mean"] = sum(roll_err) / len(roll_err)
                metrics["roll_err_max"]  = max(roll_err)

        if att.get("Pitch") and att.get("DesPitch"):
            pitch_err = [abs((p or 0) - (d or 0))
                         for p, d in zip(att["Pitch"], att["DesPitch"])]
            if pitch_err:
                metrics["pitch_err_mean"] = sum(pitch_err) / len(pitch_err)
                metrics["pitch_err_max"]  = max(pitch_err)

        # Vibration (from VIBE)
        vibe = self.data.get("VIBE", {})
        for axis in ("VibeX", "VibeY", "VibeZ"):
            vals = [v for v in vibe.get(axis, []) if v is not None]
            if vals:
                metrics[f"vibe_{axis.lower()}_max"] = max(vals)

        return metrics

    def _extract_mode_changes(self) -> list:
        mode_data = self.data.get("MODE", {})
        times  = mode_data.get("TimeUS", [])
        modes  = mode_data.get("Mode", [])
        nums   = mode_data.get("ModeNum", [])
        changes = []
        for i in range(len(times)):
            changes.append({
                "time_us": times[i],
                "mode":    modes[i] if i < len(modes) else None,
                "mode_num": nums[i] if i < len(nums) else None,
            })
        return changes

    def _extract_errors(self) -> list:
        err_data = self.data.get("ERR", {})
        times   = err_data.get("TimeUS", [])
        subsys  = err_data.get("Subsys", [])
        ecodes  = err_data.get("ECode", [])
        errors = []
        for i in range(len(times)):
            errors.append({
                "time_us": times[i],
                "subsys":  subsys[i] if i < len(subsys) else None,
                "ecode":   ecodes[i] if i < len(ecodes) else None,
            })
        return errors

    # ── Helpers ──────────────────────────────────────────────────

    @staticmethod
    def _empty_result(reason: str) -> dict:
        return {
            "data":         {},
            "metrics":      {},
            "mode_changes": [],
            "errors":       [],
            "success":      False,
            "reason":       reason,
        }
