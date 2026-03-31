"""
Health Monitor — Real‑time telemetry health scoring and recording.

Processes MAVLink messages (EKF_STATUS_REPORT, VIBRATION, ATTITUDE,
GLOBAL_POSITION_INT, SYS_STATUS, HEARTBEAT) and computes a continuous
health score anchored to ArduPilot's own FS_EKF_THRESH:

    H(t) = 1 − max(vel/θ, pos/θ, hgt/θ, mag/θ, |vib|/60)

    θ   = FS_EKF_THRESH  (default 0.8)
    60  = EKF vibration clip level in m/s²

  H = 1.0  → perfect health
  H = 0.0  → at failsafe threshold
  H < 0    → beyond threshold (EKF will trigger failsafe)
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


# ── Telemetry snapshot data‑class ────────────────────────────────

@dataclass
class TelemetrySnapshot:
    timestamp: float = 0.0

    # EKF (from EKF_STATUS_REPORT)
    velocity_variance: float = 0.0
    pos_horiz_variance: float = 0.0
    pos_vert_variance: float = 0.0
    compass_variance: float = 0.0
    terrain_alt_variance: float = 0.0
    ekf_flags: int = 0

    # Vibration (from VIBRATION)
    vibe_x: float = 0.0
    vibe_y: float = 0.0
    vibe_z: float = 0.0
    clipping: int = 0
    vibe_magnitude: float = 0.0

    # Attitude (from ATTITUDE — degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0

    # Position (from GLOBAL_POSITION_INT)
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    relative_alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    hdg: float = 0.0

    # System
    mode: str = ""
    mode_num: int = 0
    armed: bool = False
    battery_voltage: float = 0.0
    battery_remaining: int = 100

    # Mission progress
    current_waypoint: int = 0
    total_waypoints: int = 0

    # Position drift from home (m)
    position_drift: float = 0.0

    # Computed
    health_score: float = 1.0


# ── Mode number → name map ──────────────────────────────────────

MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO",     2: "ALT_HOLD",
    3: "AUTO",      4: "GUIDED",   5: "LOITER",
    6: "RTL",       7: "CIRCLE",   9: "LAND",
    11: "DRIFT",    13: "SPORT",   16: "POSHOLD",
    17: "BRAKE",    18: "THROW",   21: "SMART_RTL",
    25: "SYSTEMID", 27: "AUTO_RTL",
}


class HealthMonitor:
    """Accumulates MAVLink messages and maintains a health score timeline."""

    FAILSAFE_MODES = {"RTL", "LAND", "ALT_HOLD", "SMART_RTL", "BRAKE"}
    AUTO_MODES = {"AUTO", "GUIDED", "LOITER", "POSHOLD"}

    def __init__(self, ekf_threshold=0.8, vibe_threshold=60.0):
        if ekf_threshold <= 0:
            raise ValueError("ekf_threshold must be > 0")
        if vibe_threshold <= 0:
            raise ValueError("vibe_threshold must be > 0")

        self.ekf_threshold = ekf_threshold
        self.vibe_threshold = vibe_threshold

        # Current (live) state
        self._cur = TelemetrySnapshot()

        # Recorded time‑series
        self.timeline: List[TelemetrySnapshot] = []
        self.mode_changes: List[Dict] = []
        self.events: List[Dict] = []

        # Control
        self._recording = False
        self._start_time = 0.0
        self._last_record_time = 0.0
        self._record_interval = 0.1        # 10 Hz

        # Tracking
        self._last_mode: Optional[str] = None
        self._home_lat: Optional[float] = None
        self._home_lon: Optional[float] = None
        self._injection_time: Optional[float] = None
        self._has_been_armed = False

        # Waypoint tracking
        self._waypoints_reached: set = set()
        self._total_waypoints: int = 0      # set externally
        self._highest_wp_seq: int = 0

        # Summary stats
        self.min_health_score = 1.0
        self.max_position_drift = 0.0
        self.failsafe_triggered = False
        self.failsafe_mode: Optional[str] = None
        self.failsafe_time: Optional[float] = None

        # Message dispatch table (built once, not per-message)
        self._dispatch = {
            "EKF_STATUS_REPORT": self._on_ekf,
            "VIBRATION":         self._on_vibration,
            "ATTITUDE":          self._on_attitude,
            "GLOBAL_POSITION_INT": self._on_position,
            "SYS_STATUS":        self._on_sys_status,
            "HEARTBEAT":         self._on_heartbeat,
            "STATUSTEXT":        self._on_statustext,
            "MISSION_CURRENT":   self._on_mission_current,
            "MISSION_ITEM_REACHED": self._on_wp_reached,
        }

    def _reset_run_state(self):
        """Reset per-run state so monitor instances can be reused safely."""
        self._cur = TelemetrySnapshot()
        self.timeline.clear()
        self.mode_changes.clear()
        self.events.clear()

        self._last_record_time = 0.0
        self._last_mode = None
        self._home_lat = None
        self._home_lon = None
        self._injection_time = None
        self._has_been_armed = False

        self._waypoints_reached.clear()
        self._highest_wp_seq = 0

        self.min_health_score = 1.0
        self.max_position_drift = 0.0
        self.failsafe_triggered = False
        self.failsafe_mode = None
        self.failsafe_time = None

    # ── Start / stop ─────────────────────────────────────────────

    def start(self):
        self._reset_run_state()
        self._recording = True
        self._start_time = time.time()
        print("[HEALTH] Monitoring started")

    def stop(self):
        self._recording = False
        n = len(self.timeline)
        dur = self.timeline[-1].timestamp if self.timeline else 0
        print(f"[HEALTH] Stopped — {n} samples over {dur:.1f} s")
        print(f"[HEALTH]   min health score : {self.min_health_score:.3f}")
        print(f"[HEALTH]   max position drift: {self.max_position_drift:.1f} m")
        if self.failsafe_triggered:
            print(f"[HEALTH]   failsafe: {self.failsafe_mode} "
                  f"at T+{self.failsafe_time:.1f} s")

    def mark_injection_time(self):
        self._injection_time = time.time() - self._start_time
        self.events.append({"time": self._injection_time,
                            "event": "FAULT_INJECTED"})

    # ── Message dispatch ─────────────────────────────────────────

    def set_total_waypoints(self, n: int):
        """Set the total number of mission waypoints (excluding home/RTL)."""
        self._total_waypoints = n

    def process_message(self, msg):
        """Feed a raw MAVLink message into the monitor."""
        if msg is None:
            return
        t = msg.get_type()
        handler = self._dispatch.get(t)
        if handler:
            handler(msg)

        # Snapshot at 10 Hz
        if self._recording:
            now = time.time() - self._start_time
            if now - self._last_record_time >= self._record_interval:
                self._record(now)
                self._last_record_time = now

    # ── Per‑message handlers ─────────────────────────────────────

    def _on_ekf(self, m):
        self._cur.velocity_variance    = m.velocity_variance
        self._cur.pos_horiz_variance   = m.pos_horiz_variance
        self._cur.pos_vert_variance    = m.pos_vert_variance
        self._cur.compass_variance     = m.compass_variance
        self._cur.terrain_alt_variance = m.terrain_alt_variance
        self._cur.ekf_flags            = m.flags
        self._recompute_score()

    def _on_vibration(self, m):
        self._cur.vibe_x = m.vibration_x
        self._cur.vibe_y = m.vibration_y
        self._cur.vibe_z = m.vibration_z
        self._cur.clipping = m.clipping_0 + m.clipping_1 + m.clipping_2
        self._cur.vibe_magnitude = math.sqrt(
            m.vibration_x**2 + m.vibration_y**2 + m.vibration_z**2
        )
        self._recompute_score()

    def _on_attitude(self, m):
        self._cur.roll  = math.degrees(m.roll)
        self._cur.pitch = math.degrees(m.pitch)
        self._cur.yaw   = math.degrees(m.yaw)
        self._cur.rollspeed  = math.degrees(m.rollspeed)
        self._cur.pitchspeed = math.degrees(m.pitchspeed)
        self._cur.yawspeed   = math.degrees(m.yawspeed)

    def _on_position(self, m):
        self._cur.lat          = m.lat / 1e7
        self._cur.lon          = m.lon / 1e7
        self._cur.alt          = m.alt / 1000.0
        self._cur.relative_alt = m.relative_alt / 1000.0
        self._cur.vx           = m.vx / 100.0
        self._cur.vy           = m.vy / 100.0
        self._cur.vz           = m.vz / 100.0
        self._cur.hdg          = m.hdg / 100.0

        # Store home on first valid lat
        if self._home_lat is None and (self._cur.lat != 0 or self._cur.lon != 0):
            self._home_lat = self._cur.lat
            self._home_lon = self._cur.lon

        # Track drift from home (always, not just post‑injection)
        if self._home_lat is not None and self._recording:
            drift = self._haversine(
                self._home_lat, self._home_lon,
                self._cur.lat, self._cur.lon,
            )
            self._cur.position_drift = drift

            # Max drift only counted after injection
            if self._injection_time is not None:
                elapsed = time.time() - self._start_time
                if elapsed > self._injection_time:
                    self.max_position_drift = max(
                        self.max_position_drift, drift)

    def _on_sys_status(self, m):
        self._cur.battery_voltage   = m.voltage_battery / 1000.0
        self._cur.battery_remaining = m.battery_remaining

    def _on_heartbeat(self, m):
        from pymavlink import mavutil
        # Only process autopilot heartbeats (type 2 = quadrotor, etc.)
        copter_types = {2, 13, 14, 15, 20, 21, 29}
        if m.type not in copter_types:
            return

        mode_num  = m.custom_mode
        mode_name = MODE_NAMES.get(mode_num, f"MODE_{mode_num}")
        armed     = bool(m.base_mode
                         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        self._cur.mode     = mode_name
        self._cur.mode_num = mode_num
        self._cur.armed    = armed

        if armed:
            self._has_been_armed = True

        # Detect mode transitions
        if self._last_mode is not None and mode_name != self._last_mode:
            elapsed = (time.time() - self._start_time) if self._recording else 0
            self.mode_changes.append({
                "time": elapsed,
                "from": self._last_mode,
                "to":   mode_name,
            })
            print(f"[HEALTH] Mode: {self._last_mode} → {mode_name} "
                  f"at T+{elapsed:.1f} s")

            # Failsafe detection
            if (mode_name in self.FAILSAFE_MODES
                    and self._last_mode in self.AUTO_MODES
                    and not self.failsafe_triggered):
                self.failsafe_triggered = True
                self.failsafe_mode = mode_name
                self.failsafe_time = elapsed
                self.events.append({
                    "time": elapsed,
                    "event": f"FAILSAFE_{mode_name}",
                })
                print(f"[HEALTH] *** FAILSAFE: {mode_name} "
                      f"at T+{elapsed:.1f} s ***")

        self._last_mode = mode_name

    def _on_statustext(self, m):
        if not self._recording:
            return
        elapsed = time.time() - self._start_time
        text = m.text if hasattr(m, "text") else str(m)
        self.events.append({"time": elapsed, "event": f"STATUS: {text}"})
        lowered = text.lower()
        if "ekf" in lowered or "failsafe" in lowered or "fail" in lowered:
            print(f"[HEALTH] FC event: {text}")

    def _on_mission_current(self, m):
        """Track current mission item (waypoint progress)."""
        seq = m.seq
        self._cur.current_waypoint = seq
        self._highest_wp_seq = max(self._highest_wp_seq, seq)

    def _on_wp_reached(self, m):
        """Track which waypoints have been reached."""
        seq = m.seq
        self._waypoints_reached.add(seq)
        if self._recording:
            elapsed = time.time() - self._start_time
            self.events.append(
                {"time": elapsed, "event": f"WAYPOINT_REACHED: seq={seq}"}
            )
            print(f"[HEALTH] Waypoint {seq} reached at T+{elapsed:.1f}s")

    # ── Health score ─────────────────────────────────────────────

    def _recompute_score(self):
        ekf = max(
            self._cur.velocity_variance  / self.ekf_threshold,
            self._cur.pos_horiz_variance / self.ekf_threshold,
            self._cur.pos_vert_variance  / self.ekf_threshold,
            self._cur.compass_variance   / self.ekf_threshold,
        )
        vib = self._cur.vibe_magnitude / self.vibe_threshold
        self._cur.health_score = max(0.0, 1.0 - max(ekf, vib))

        if self._recording:
            self.min_health_score = min(self.min_health_score,
                                        self._cur.health_score)

    # ── Recording ────────────────────────────────────────────────

    def _record(self, ts):
        # Shallow copy via dataclass fields
        snap = TelemetrySnapshot(
            timestamp=ts,
            velocity_variance=self._cur.velocity_variance,
            pos_horiz_variance=self._cur.pos_horiz_variance,
            pos_vert_variance=self._cur.pos_vert_variance,
            compass_variance=self._cur.compass_variance,
            terrain_alt_variance=self._cur.terrain_alt_variance,
            ekf_flags=self._cur.ekf_flags,
            vibe_x=self._cur.vibe_x,
            vibe_y=self._cur.vibe_y,
            vibe_z=self._cur.vibe_z,
            clipping=self._cur.clipping,
            vibe_magnitude=self._cur.vibe_magnitude,
            roll=self._cur.roll,
            pitch=self._cur.pitch,
            yaw=self._cur.yaw,
            rollspeed=self._cur.rollspeed,
            pitchspeed=self._cur.pitchspeed,
            yawspeed=self._cur.yawspeed,
            lat=self._cur.lat,
            lon=self._cur.lon,
            alt=self._cur.alt,
            relative_alt=self._cur.relative_alt,
            vx=self._cur.vx,
            vy=self._cur.vy,
            vz=self._cur.vz,
            hdg=self._cur.hdg,
            mode=self._cur.mode,
            mode_num=self._cur.mode_num,
            armed=self._cur.armed,
            battery_voltage=self._cur.battery_voltage,
            battery_remaining=self._cur.battery_remaining,
            current_waypoint=self._cur.current_waypoint,
            total_waypoints=self._total_waypoints,
            position_drift=self._cur.position_drift,
            health_score=self._cur.health_score,
        )
        self.timeline.append(snap)

    # ── Public queries ───────────────────────────────────────────

    def get_current_score(self):
        return self._cur.health_score

    def is_armed(self) -> bool:
        return self._cur.armed

    def get_current_mode(self) -> str:
        return self._cur.mode

    def get_relative_altitude_m(self) -> float:
        return self._cur.relative_alt

    def vehicle_has_disarmed_after_flight(self):
        """True once the vehicle was armed and then disarmed."""
        return self._has_been_armed and not self._cur.armed

    def get_mission_completion_pct(self) -> float:
        """Return 0‑100 mission completion percentage.

        Based on the highest waypoint sequence reached vs total
        mission waypoints (excluding home item 0 and RTL at end).
        """
        if self._total_waypoints <= 0:
            return 100.0      # no mission → consider complete
        # Highest wp seq includes takeoff(1), wps(2..N+1), so
        # progress = (highest_seq - 1) / total_wp_count
        progress = max(0, self._highest_wp_seq - 1)
        return min(100.0, (progress / self._total_waypoints) * 100)

    def get_summary(self) -> dict:
        return {
            "samples": len(self.timeline),
            "duration_s": self.timeline[-1].timestamp if self.timeline else 0,
            "min_health_score": self.min_health_score,
            "max_position_drift_m": self.max_position_drift,
            "failsafe_triggered": self.failsafe_triggered,
            "failsafe_mode": self.failsafe_mode,
            "failsafe_time_s": self.failsafe_time,
            "mode_changes": self.mode_changes,
            "events": self.events,
            "injection_time_s": self._injection_time,
            "mission_completion_pct": self.get_mission_completion_pct(),
            "waypoints_reached": sorted(self._waypoints_reached),
            "highest_waypoint_seq": self._highest_wp_seq,
        }

    def get_timeseries(self) -> dict:
        if not self.timeline:
            return {}
        def _col(attr):
            return [getattr(s, attr) for s in self.timeline]
        return {
            "time":               _col("timestamp"),
            "health_score":       _col("health_score"),
            "velocity_variance":  _col("velocity_variance"),
            "pos_horiz_variance": _col("pos_horiz_variance"),
            "pos_vert_variance":  _col("pos_vert_variance"),
            "compass_variance":   _col("compass_variance"),
            "vibe_magnitude":     _col("vibe_magnitude"),
            "roll":               _col("roll"),
            "pitch":              _col("pitch"),
            "yaw":                _col("yaw"),
            "relative_alt":       _col("relative_alt"),
            "lat":                _col("lat"),
            "lon":                _col("lon"),
            "battery_voltage":    _col("battery_voltage"),
            "mode":               _col("mode"),
            "position_drift":     _col("position_drift"),
            "current_waypoint":   _col("current_waypoint"),
        }

    # ── Helpers ──────────────────────────────────────────────────

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6_371_000
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = (math.sin(dp / 2) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
