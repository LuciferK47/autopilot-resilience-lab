"""
Vehicle Controller — Arm, takeoff, fly waypoint missions, and land via pymavlink.

Supports:
  - Raw MAVLink arming / disarming
  - GUIDED‑mode takeoff
  - Full MAVLink mission upload protocol (MISSION_COUNT → MISSION_ITEM_INT)
  - AUTO mission execution with waypoint tracking
  - Parameter get / set
"""

import time
from pymavlink import mavutil


# ── ArduCopter mode numbers ──────────────────────────────────────
COPTER_MODES = {
    "STABILIZE": 0,  "ACRO": 1,       "ALT_HOLD": 2,
    "AUTO": 3,       "GUIDED": 4,     "LOITER": 5,
    "RTL": 6,        "CIRCLE": 7,     "LAND": 9,
    "DRIFT": 11,     "SPORT": 13,     "POSHOLD": 16,
    "BRAKE": 17,     "THROW": 18,     "SMART_RTL": 21,
    "SYSTEMID": 25,
}
COPTER_MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}


class VehicleController:
    """High‑level ArduCopter control via a single pymavlink connection."""

    def __init__(self, connection):
        self.master = connection
        self.target_system = connection.target_system
        self.target_component = connection.target_component

    # ── Parameters ───────────────────────────────────────────────

    def set_param(self, name, value, timeout=3.0, tolerance=1e-3):
        """Set a parameter and verify it was applied.

        Returns True if the value is confirmed within tolerance.
        """
        name_bytes = name.encode("utf-8") if isinstance(name, str) else name
        self.master.mav.param_set_send(
            self.target_system, self.target_component,
            name_bytes,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        time.sleep(0.1)

        if timeout <= 0:
            return True

        actual = self.get_param(name, timeout=timeout)
        if actual is None:
            print(f"[CTRL] Param set not confirmed: {name}")
            return False
        if abs(actual - float(value)) > tolerance:
            print(f"[CTRL] Param mismatch {name}: expected {value}, got {actual}")
            return False
        return True

    def get_param(self, name, timeout=3.0):
        self.master.mav.param_request_read_send(
            self.target_system, self.target_component,
            name.encode("utf-8") if isinstance(name, str) else name, -1,
        )
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type="PARAM_VALUE",
                                         blocking=True, timeout=1)
            if not msg:
                continue
            param_id = msg.param_id.strip("\x00")
            if isinstance(name, bytes):
                name_cmp = name.decode("utf-8", errors="replace")
            else:
                name_cmp = name
            if param_id == name_cmp:
                return msg.param_value
        return None

    # ── Mode control ─────────────────────────────────────────────

    def set_mode(self, mode_name, timeout=10):
        """Set flight mode; returns True when the mode is confirmed."""
        if mode_name not in COPTER_MODES:
            raise ValueError(f"Unknown mode '{mode_name}'. "
                             f"Valid: {list(COPTER_MODES)}")

        mode_num = COPTER_MODES[mode_name]
        per_try_timeout = max(4, timeout // 3)

        for _ in range(3):
            # pymavlink helper
            self.master.set_mode(mode_num)

            # explicit command path (more reliable across stacks)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_num,
                0,
                0,
                0,
                0,
                0,
            )

            if self.wait_mode(mode_name, per_try_timeout):
                return True

        return False

    def wait_mode(self, mode_name, timeout=10):
        target = COPTER_MODES[mode_name]
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type="HEARTBEAT",
                                        blocking=True, timeout=1)
            if hb and hb.custom_mode == target:
                print(f"[CTRL] Mode confirmed: {mode_name}")
                return True
            # Surface any FC status messages
            st = self.master.recv_match(type="STATUSTEXT", blocking=False)
            if st:
                print(f"[CTRL] FC: {st.text}")
        print(f"[CTRL] Mode change to {mode_name} timed out")
        return False

    def get_current_mode(self):
        hb = self.master.recv_match(type="HEARTBEAT",
                                    blocking=True, timeout=5)
        if hb:
            return COPTER_MODE_NAMES.get(hb.custom_mode,
                                         f"UNKNOWN({hb.custom_mode})")
        return "UNKNOWN"

    # ── Arming / disarming ───────────────────────────────────────

    def arm(self, timeout=30, force=False):
        print("[CTRL] Arming vehicle...")
        if not self.set_mode("GUIDED"):
            print("[CTRL] Failed to enter GUIDED for arming")
            return False

        def _send_arm(force_flag: bool):
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1,                              # arm
                21196 if force_flag else 0,     # force‑arm magic
                0, 0, 0, 0, 0,
            )

        def _wait_armed(wait_s: float) -> bool:
            t0 = time.time()
            while time.time() - t0 < wait_s:
                hb = self.master.recv_match(type="HEARTBEAT",
                                            blocking=True, timeout=1)
                if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("[CTRL] Vehicle armed")
                    return True
                st = self.master.recv_match(type="STATUSTEXT", blocking=False)
                if st:
                    print(f"[CTRL] FC: {st.text}")
            return False

        _send_arm(force)
        if _wait_armed(timeout):
            return True

        if not force:
            print("[CTRL] Arm timed out, retrying with force-arm...")
            _send_arm(True)
            if _wait_armed(12):
                return True

        print("[CTRL] Arm timed out")
        return False

    def disarm(self, timeout=10, force=False):
        print("[CTRL] Disarming...")
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196 if force else 0, 0, 0, 0, 0, 0,
        )
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type="HEARTBEAT",
                                        blocking=True, timeout=1)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[CTRL] Vehicle disarmed")
                return True
        print("[CTRL] Disarm timed out")
        return False

    # ── Takeoff ──────────────────────────────────────────────────

    def takeoff(self, altitude, timeout=60):
        print(f"[CTRL] Taking off to {altitude} m...")
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,
        )
        return self.wait_altitude(altitude, timeout=timeout)

    def wait_altitude(self, target, tolerance=2.0, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT",
                                         blocking=True, timeout=2)
            if msg:
                alt = msg.relative_alt / 1000.0
                if abs(alt - target) < tolerance:
                    print(f"[CTRL] Altitude reached: {alt:.1f} m")
                    return True
            time.sleep(0.1)
        print("[CTRL] Altitude wait timed out")
        return False

    # ── Mission upload (MAVLink mission protocol) ────────────────

    def upload_mission(self, waypoints, altitude=None):
        """Upload a waypoint mission.

        Parameters
        ----------
        waypoints : list[dict]
            Each dict has 'lat' (deg), 'lon' (deg), 'alt' (m relative).
        altitude : float, optional
            Override altitude for takeoff item.

        Returns
        -------
        bool — True if MISSION_ACK received with ACCEPTED.
        """
        print(f"[CTRL] Uploading mission ({len(waypoints)} waypoints + "
              f"takeoff + RTL)...")

        takeoff_alt = altitude or waypoints[0].get("alt", 20)
        items = []

        # 0 — home (placeholder, FC will overwrite)
        items.append(self._mission_item(
            seq=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            lat=waypoints[0]["lat"], lon=waypoints[0]["lon"], alt=0,
            current=1, autocontinue=1,
        ))

        # 1 — takeoff
        items.append(self._mission_item(
            seq=1,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            lat=0, lon=0, alt=takeoff_alt,
            autocontinue=1,
        ))

        # 2..N — waypoints
        for i, wp in enumerate(waypoints):
            items.append(self._mission_item(
                seq=i + 2,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                lat=wp["lat"], lon=wp["lon"],
                alt=wp.get("alt", takeoff_alt),
                autocontinue=1,
            ))

        # N+2 — RTL
        items.append(self._mission_item(
            seq=len(waypoints) + 2,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            lat=0, lon=0, alt=0,
            autocontinue=1,
        ))

        total = len(items)

        # Clear existing mission first for deterministic upload
        try:
            self.master.mav.mission_clear_all_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.master.mav.mission_clear_all_send(
                self.target_system,
                self.target_component,
            )
        time.sleep(0.2)

        # Step 1 — send MISSION_COUNT
        try:
            self.master.mav.mission_count_send(
                self.target_system,
                self.target_component,
                total,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.master.mav.mission_count_send(
                self.target_system,
                self.target_component,
                total,
            )

        # Step 2 — respond to MISSION_REQUEST_INT with items
        sent = set()
        t0 = time.time()
        while len(sent) < total and time.time() - t0 < 30:
            msg = self.master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST",
                      "MISSION_ACK"],
                blocking=True, timeout=5,
            )
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                seq = msg.seq
                if seq < total:
                    item = items[seq]
                    if mtype == "MISSION_REQUEST_INT":
                        self.master.mav.mission_item_int_send(
                            self.target_system,
                            self.target_component,
                            item["seq"],
                            item["frame"],
                            item["cmd"],
                            item["current"],
                            item["autocontinue"],
                            item["p1"],
                            item["p2"],
                            item["p3"],
                            item["p4"],
                            int(item["lat"] * 1e7),
                            int(item["lon"] * 1e7),
                            item["alt"],
                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                        )
                    else:
                        try:
                            self.master.mav.mission_item_send(
                                self.target_system,
                                self.target_component,
                                item["seq"],
                                item["frame"],
                                item["cmd"],
                                item["current"],
                                item["autocontinue"],
                                item["p1"],
                                item["p2"],
                                item["p3"],
                                item["p4"],
                                item["lat"],
                                item["lon"],
                                item["alt"],
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                            )
                        except TypeError:
                            self.master.mav.mission_item_send(
                                self.target_system,
                                self.target_component,
                                item["seq"],
                                item["frame"],
                                item["cmd"],
                                item["current"],
                                item["autocontinue"],
                                item["p1"],
                                item["p2"],
                                item["p3"],
                                item["p4"],
                                item["lat"],
                                item["lon"],
                                item["alt"],
                            )
                    sent.add(seq)
            elif mtype == "MISSION_ACK":
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"[CTRL] Mission accepted ({total} items)")
                    return True
                else:
                    print(f"[CTRL] Mission rejected (err={msg.type})")
                    return False

        print("[CTRL] Mission upload timed out")
        return False

    def _mission_item(self, seq, frame, cmd, lat, lon, alt,
                      current=0, autocontinue=0,
                      p1=0, p2=0, p3=0, p4=0):
        """Return a normalized mission item payload for upload senders."""
        return {
            "seq": seq,
            "frame": frame,
            "cmd": cmd,
            "current": current,
            "autocontinue": autocontinue,
            "p1": float(p1),
            "p2": float(p2),
            "p3": float(p3),
            "p4": float(p4),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt),
        }

    # ── Mission execution helpers ────────────────────────────────

    def start_mission(self):
        """Enter AUTO mode and begin the uploaded mission.

        ArduCopter handles skipping the TAKEOFF item automatically when the
        vehicle is already airborne — no need to manually set mission_current.
        """
        print("[CTRL] Starting AUTO mission...")
        if self.set_mode("AUTO", timeout=15):
            return True

        # Tolerate late mode confirmation if AUTO is already active
        cur = self.get_current_mode()
        return cur == "AUTO"

    def get_position(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT",
                                     blocking=True, timeout=3)
        if msg:
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.alt / 1000.0,
                "relative_alt": msg.relative_alt / 1000.0,
                "vx": msg.vx / 100.0,
                "vy": msg.vy / 100.0,
                "vz": msg.vz / 100.0,
                "hdg": msg.hdg / 100.0,
            }
        return None

    def is_armed(self):
        hb = self.master.recv_match(type="HEARTBEAT",
                                    blocking=True, timeout=2)
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def wait_disarmed(self, timeout=120):
        print("[CTRL] Waiting for disarm...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type="HEARTBEAT",
                                        blocking=True, timeout=2)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[CTRL] Vehicle disarmed — mission complete")
                return True
        print("[CTRL] Disarm wait timed out")
        return False

    def request_message_interval(self, message_id, interval_us):
        """Ask the FC to stream a specific message at *interval_us* µs."""
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0,
        )
