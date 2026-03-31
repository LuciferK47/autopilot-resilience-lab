"""
EKF Forensics — Deep EKF analysis from DataFlash .BIN logs.

Extracts ArduPilot's own internal health metrics from XKF3/XKF4 log
messages — data that is ONLY available in DataFlash logs, not via
real‑time MAVLink telemetry:

  XKF3 fields:
    • errorScore  — the EKF's own composite error metric
                      (used for lane switching decisions)
    • innovations — IVN, IVE, IVD (velocity), IPN, IPE, IPD (position),
                      IMX, IMY, IMZ (mag), IYAW (yaw)

  XKF4 fields:
    • SV, SP, SH, SM — squared innovation test ratios (sqrt'd)
        value < 1.0 → measurement accepted by EKF
        value > 1.0 → measurement REJECTED — filter is stressed
    • PI — primary core index (detects EKF lane switches)
    • FS — fault status bitmask
    • TS — timeout status bitmask

This gives a "post‑mortem" view into exactly how the EKF reacted
internally to each injected fault — far deeper than the real‑time
EKF_STATUS_REPORT MAVLink message.

Nobody else does this in an automated fault‑injection framework.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class EKFForensicsResult:
    """Structured output from EKF forensics analysis."""

    # Raw time‑series (all lists same length)
    time_us: List[int] = field(default_factory=list)

    # From XKF3
    error_score: List[float] = field(default_factory=list)
    innov_vel_n: List[float] = field(default_factory=list)
    innov_vel_e: List[float] = field(default_factory=list)
    innov_vel_d: List[float] = field(default_factory=list)
    innov_pos_n: List[float] = field(default_factory=list)
    innov_pos_e: List[float] = field(default_factory=list)
    innov_pos_d: List[float] = field(default_factory=list)
    innov_mag_x: List[float] = field(default_factory=list)
    innov_mag_y: List[float] = field(default_factory=list)
    innov_mag_z: List[float] = field(default_factory=list)

    # From XKF4
    test_ratio_vel: List[float] = field(default_factory=list)
    test_ratio_pos: List[float] = field(default_factory=list)
    test_ratio_hgt: List[float] = field(default_factory=list)
    test_ratio_mag: List[float] = field(default_factory=list)
    primary_core: List[int] = field(default_factory=list)
    fault_status: List[int] = field(default_factory=list)
    timeout_status: List[int] = field(default_factory=list)

    # Computed summary metrics
    metrics: Dict[str, float] = field(default_factory=dict)

    # Lane switch events
    lane_switches: List[Dict] = field(default_factory=list)

    # Whether analysis succeeded
    success: bool = False

    def as_timeseries(self) -> dict:
        """Return dict suitable for plotting (time in seconds)."""
        if not self.time_us:
            return {}
        t0 = self.time_us[0]
        t_s = [(t - t0) / 1e6 for t in self.time_us]
        return {
            "time_s":          t_s,
            "error_score":     self.error_score,
            "innov_vel_n":     self.innov_vel_n,
            "innov_vel_e":     self.innov_vel_e,
            "innov_vel_d":     self.innov_vel_d,
            "innov_pos_n":     self.innov_pos_n,
            "innov_pos_e":     self.innov_pos_e,
            "innov_pos_d":     self.innov_pos_d,
            "innov_mag_x":     self.innov_mag_x,
            "innov_mag_y":     self.innov_mag_y,
            "innov_mag_z":     self.innov_mag_z,
            "test_ratio_vel":  self.test_ratio_vel,
            "test_ratio_pos":  self.test_ratio_pos,
            "test_ratio_hgt":  self.test_ratio_hgt,
            "test_ratio_mag":  self.test_ratio_mag,
            "primary_core":    self.primary_core,
        }


class EKFForensics:
    """Extract and analyze deep EKF data from a DataFlash .BIN log."""

    def __init__(self, log_path: str, timeout: float = 90):
        self.log_path = log_path
        self.timeout = timeout

    def analyze(self) -> EKFForensicsResult:
        """Parse the .BIN log and extract EKF forensics data.

        Returns an EKFForensicsResult with time‑series and summary metrics.
        """
        result = EKFForensicsResult()

        if not os.path.exists(self.log_path):
            print(f"[EKF‑FORENSICS] File not found: {self.log_path}")
            return result

        size_mb = os.path.getsize(self.log_path) / (1024 * 1024)
        if size_mb > 200:
            print(f"[EKF‑FORENSICS] Log too large ({size_mb:.0f} MB) — skip")
            return result

        print(f"[EKF‑FORENSICS] Analyzing {os.path.basename(self.log_path)} "
              f"({size_mb:.1f} MB)...")

        try:
            self._parse(result)
        except Exception as e:
            print(f"[EKF‑FORENSICS] Parse error: {e}")
            return result

        self._compute_metrics(result)
        self._detect_lane_switches(result)
        result.success = True

        n = len(result.time_us)
        switches = len(result.lane_switches)
        print(f"[EKF‑FORENSICS] Done — {n} samples, "
              f"{switches} lane switch(es)")
        return result

    # ── Parse ────────────────────────────────────────────────────

    def _parse(self, result: EKFForensicsResult):
        from pymavlink import mavutil

        mlog = mavutil.mavlink_connection(self.log_path)
        deadline = time.time() + self.timeout

        # We maintain separate lists for XKF3 and XKF4 then merge by
        # closest timestamp.  In practice they log at the same rate
        # so we just append in lockstep.  If counts differ, we trim
        # to the shorter.

        xkf3_data = []   # list of tuples (TimeUS, ErSc, IVN..IMZ)
        xkf4_data = []   # list of tuples (TimeUS, SV..PI)

        while time.time() < deadline:
            msg = mlog.recv_msg()
            if msg is None:
                break
            mt = msg.get_type()

            if mt == "XKF3":
                try:
                    c = getattr(msg, "C", 0)
                    if c != 0:
                        continue        # only core 0
                    xkf3_data.append((
                        msg.TimeUS,
                        getattr(msg, "ErSc", 0.0),
                        getattr(msg, "IVN", 0),
                        getattr(msg, "IVE", 0),
                        getattr(msg, "IVD", 0),
                        getattr(msg, "IPN", 0),
                        getattr(msg, "IPE", 0),
                        getattr(msg, "IPD", 0),
                        getattr(msg, "IMX", 0),
                        getattr(msg, "IMY", 0),
                        getattr(msg, "IMZ", 0),
                    ))
                except AttributeError:
                    pass

            elif mt == "XKF4":
                try:
                    c = getattr(msg, "C", 0)
                    if c != 0:
                        continue
                    xkf4_data.append((
                        msg.TimeUS,
                        getattr(msg, "SV", 0),
                        getattr(msg, "SP", 0),
                        getattr(msg, "SH", 0),
                        getattr(msg, "SM", 0),
                        getattr(msg, "PI", 0),
                        getattr(msg, "FS", 0),
                        getattr(msg, "TS", 0),
                    ))
                except AttributeError:
                    pass

        # Merge — use XKF3 timestamps as authoritative
        n3 = len(xkf3_data)
        n4 = len(xkf4_data)
        n = min(n3, n4) if n4 > 0 else n3

        for i in range(n):
            d3 = xkf3_data[i]
            result.time_us.append(d3[0])
            result.error_score.append(float(d3[1]))
            # DFReader already applies 0.01 multiplier for 'c' format
            # fields, so msg.IVN etc. are in actual units (m/s, m, mGauss).
            # Do NOT divide by 100 again.
            result.innov_vel_n.append(float(d3[2]))
            result.innov_vel_e.append(float(d3[3]))
            result.innov_vel_d.append(float(d3[4]))
            result.innov_pos_n.append(float(d3[5]))
            result.innov_pos_e.append(float(d3[6]))
            result.innov_pos_d.append(float(d3[7]))
            result.innov_mag_x.append(float(d3[8]))
            result.innov_mag_y.append(float(d3[9]))
            result.innov_mag_z.append(float(d3[10]))

            if i < n4:
                d4 = xkf4_data[i]
                # SV/SP/SH/SM are 'c' format — already scaled by DFReader
                result.test_ratio_vel.append(float(d4[1]))
                result.test_ratio_pos.append(float(d4[2]))
                result.test_ratio_hgt.append(float(d4[3]))
                result.test_ratio_mag.append(float(d4[4]))
                result.primary_core.append(int(d4[5]))
                result.fault_status.append(int(d4[6]))
                result.timeout_status.append(int(d4[7]))
            else:
                result.test_ratio_vel.append(0.0)
                result.test_ratio_pos.append(0.0)
                result.test_ratio_hgt.append(0.0)
                result.test_ratio_mag.append(0.0)
                result.primary_core.append(0)
                result.fault_status.append(0)
                result.timeout_status.append(0)

        # If we had XKF3 data but no XKF4, fill remaining
        for i in range(n, n3):
            d3 = xkf3_data[i]
            result.time_us.append(d3[0])
            result.error_score.append(float(d3[1]))
            result.innov_vel_n.append(float(d3[2]))
            result.innov_vel_e.append(float(d3[3]))
            result.innov_vel_d.append(float(d3[4]))
            result.innov_pos_n.append(float(d3[5]))
            result.innov_pos_e.append(float(d3[6]))
            result.innov_pos_d.append(float(d3[7]))
            result.innov_mag_x.append(float(d3[8]))
            result.innov_mag_y.append(float(d3[9]))
            result.innov_mag_z.append(float(d3[10]))
            result.test_ratio_vel.append(0.0)
            result.test_ratio_pos.append(0.0)
            result.test_ratio_hgt.append(0.0)
            result.test_ratio_mag.append(0.0)
            result.primary_core.append(0)
            result.fault_status.append(0)
            result.timeout_status.append(0)

        print(f"[EKF‑FORENSICS] Parsed {n3} XKF3, {n4} XKF4 messages")

    # ── Metrics ──────────────────────────────────────────────────

    def _compute_metrics(self, result: EKFForensicsResult):
        m = {}
        if result.error_score:
            m["error_score_mean"] = sum(result.error_score) / len(result.error_score)
            m["error_score_max"]  = max(result.error_score)
            m["error_score_min"]  = min(result.error_score)

        for name, series in [
            ("test_ratio_vel", result.test_ratio_vel),
            ("test_ratio_pos", result.test_ratio_pos),
            ("test_ratio_hgt", result.test_ratio_hgt),
            ("test_ratio_mag", result.test_ratio_mag),
        ]:
            if series:
                m[f"{name}_mean"] = sum(series) / len(series)
                m[f"{name}_max"]  = max(series)
                # Count how many samples exceeded 1.0 (rejections)
                rejections = sum(1 for v in series if v > 1.0)
                m[f"{name}_rejections"] = rejections
                m[f"{name}_rejection_pct"] = (
                    rejections / len(series) * 100 if series else 0
                )

        # Innovation magnitudes
        if result.innov_vel_n:
            vel_mag = [
                (n**2 + e**2 + d**2)**0.5
                for n, e, d in zip(
                    result.innov_vel_n,
                    result.innov_vel_e,
                    result.innov_vel_d,
                )
            ]
            m["innov_vel_mag_max"] = max(vel_mag)
            m["innov_vel_mag_mean"] = sum(vel_mag) / len(vel_mag)

        if result.innov_pos_n:
            pos_mag = [
                (n**2 + e**2 + d**2)**0.5
                for n, e, d in zip(
                    result.innov_pos_n,
                    result.innov_pos_e,
                    result.innov_pos_d,
                )
            ]
            m["innov_pos_mag_max"] = max(pos_mag)
            m["innov_pos_mag_mean"] = sum(pos_mag) / len(pos_mag)

        if result.innov_mag_x:
            mag_mag = [
                (x**2 + y**2 + z**2)**0.5
                for x, y, z in zip(
                    result.innov_mag_x,
                    result.innov_mag_y,
                    result.innov_mag_z,
                )
            ]
            m["innov_mag_mag_max"] = max(mag_mag)
            m["innov_mag_mag_mean"] = sum(mag_mag) / len(mag_mag)

        result.metrics = m

    # ── Lane switch detection ────────────────────────────────────

    def _detect_lane_switches(self, result: EKFForensicsResult):
        """Detect changes in the primary EKF core index."""
        if len(result.primary_core) < 2:
            return

        prev = result.primary_core[0]
        for i in range(1, len(result.primary_core)):
            cur = result.primary_core[i]
            if cur != prev:
                t_us = result.time_us[i] if i < len(result.time_us) else 0
                result.lane_switches.append({
                    "time_us": t_us,
                    "from_core": prev,
                    "to_core": cur,
                    "error_score": (
                        result.error_score[i]
                        if i < len(result.error_score) else None
                    ),
                })
                print(f"[EKF‑FORENSICS] Lane switch at "
                      f"T={t_us/1e6:.2f}s: core {prev} → {cur}")
            prev = cur
