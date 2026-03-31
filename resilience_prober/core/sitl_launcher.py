"""
SITL Launcher — Programmatically launch and manage ArduPilot SITL instances.

Handles:
  - Launching sim_vehicle.py via subprocess
  - Waiting for heartbeat, GPS lock, EKF initialization
  - Configuring LOG_FILE_DSRMROT, SIM_SPEEDUP
  - Clean shutdown with process group kill
"""

import subprocess
import time
import os
import signal
import sys
from pymavlink import mavutil


class SITLLauncher:
    """Launch and manage an ArduPilot SITL instance."""

    DEFAULT_HOME = "-35.363261,149.165230,584,353"

    def __init__(self, ardupilot_path, vehicle="ArduCopter", instance=0,
                 speedup=5, home=None, extra_args=None, wipe=True):
        self.ardupilot_path = os.path.abspath(ardupilot_path)
        self.vehicle = vehicle
        self.instance = instance
        self.speedup = speedup
        self.home = home or self.DEFAULT_HOME
        self.extra_args = extra_args or []
        self.wipe = wipe
        self.process = None
        self.master = None

        self._log_dir = os.path.join(self.ardupilot_path, "logs")
        self._initial_log_num = self._get_last_log_num()

        # Port calculation based on instance
        self.tcp_port = 5760 + (instance * 10)

        # Path to sim_vehicle.py
        self.sim_vehicle_path = os.path.join(
            self.ardupilot_path, "Tools", "autotest", "sim_vehicle.py"
        )

    # ── Log tracking ─────────────────────────────────────────────

    def _get_last_log_num(self):
        """Get the current last log number."""
        if not os.path.isdir(self._log_dir):
            return 0
        lastlog = os.path.join(self._log_dir, "LASTLOG.TXT")
        if os.path.exists(lastlog):
            try:
                return int(open(lastlog).read().strip())
            except (ValueError, OSError):
                pass
        return 0

    # ── Launch & connect ─────────────────────────────────────────

    def launch(self):
        """Launch SITL and return a pymavlink connection.

        Returns:
            mavutil.mavlink_connection — ready to use
        Raises:
            FileNotFoundError  — sim_vehicle.py missing
            RuntimeError       — SITL exited immediately
            TimeoutError       — heartbeat / GPS / EKF never arrived
        """
        if not os.path.exists(self.sim_vehicle_path):
            raise FileNotFoundError(
                f"sim_vehicle.py not found at {self.sim_vehicle_path}"
            )

        print(f"[SITL] Launching {self.vehicle} (instance {self.instance})...")

        cmd = [
            sys.executable, self.sim_vehicle_path,
            "-v", self.vehicle,
            "-I", str(self.instance),
            "--no-mavproxy",
            "--no-rebuild",
            f"--speedup={self.speedup}",
            f"--custom-location={self.home}",
        ]
        if self.wipe:
            cmd.append("-w")
        cmd += self.extra_args

        print(f"[SITL] Command: {' '.join(cmd)}")

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=self.ardupilot_path,
            preexec_fn=os.setsid,
        )

        # Give SITL time to boot (15s needed with -w wipe on slower machines)
        time.sleep(15)

        if self.process.poll() is not None:
            output = self.process.stdout.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"SITL exited immediately. Output:\n{output}")

        # Connect via TCP — retry up to 3 times in case SITL is still starting
        conn_str = f"tcp:127.0.0.1:{self.tcp_port}"
        print(f"[SITL] Connecting to {conn_str}...")
        last_exc = None
        for attempt in range(1, 4):
            try:
                self.master = mavutil.mavlink_connection(
                    conn_str, retries=5, timeout=10)
                break
            except Exception as exc:
                last_exc = exc
                print(f"[SITL] TCP connect attempt {attempt}/3 failed: {exc}")
                time.sleep(5)
        else:
            raise RuntimeError(
                f"Could not connect to SITL after 3 attempts: {last_exc}")

        self._wait_for_ready()
        self._configure_sitl()

        print(f"[SITL] {self.vehicle} ready on port {self.tcp_port}")
        return self.master

    # ── Readiness checks ─────────────────────────────────────────

    def _request_data_streams(self, rate_hz=10):
        """Request all data streams from SITL.

        Without MAVProxy, ArduPilot only sends heartbeats by default.
        We must explicitly request data streams to receive GPS_RAW_INT,
        EKF_STATUS_REPORT, SYS_STATUS, ATTITUDE, etc.
        """
        # MAV_DATA_STREAM_ALL = 0 requests everything
        for stream_id in range(0, 13):
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                rate_hz,
                1,  # start sending
            )
        # Also set SR0_ params for persistent stream rates
        for name, val in [("SR0_RAW_SENS", rate_hz),
                          ("SR0_EXT_STAT", rate_hz),
                          ("SR0_RC_CHAN", rate_hz),
                          ("SR0_RAW_CTRL", rate_hz),
                          ("SR0_POSITION", rate_hz),
                          ("SR0_EXTRA1", rate_hz),
                          ("SR0_EXTRA2", rate_hz),
                          ("SR0_EXTRA3", rate_hz)]:
            self._set_param(name, val)
        time.sleep(0.5)

    def _wait_for_ready(self, timeout=120):
        """Wait for heartbeat → GPS 3‑D fix → EKF convergence."""
        deadline = time.time() + timeout

        # 1. Heartbeat
        print("[SITL] Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=timeout)
        print(f"[SITL] Heartbeat received  (sysid={self.master.target_system})")

        # 2. Request data streams (essential without MAVProxy)
        print("[SITL] Requesting data streams...")
        self._request_data_streams(rate_hz=10)

        # 3. GPS 3‑D fix
        print("[SITL] Waiting for GPS 3‑D fix...")
        while time.time() < deadline:
            msg = self.master.recv_match(type="GPS_RAW_INT",
                                         blocking=True, timeout=5)
            if msg and msg.fix_type >= 3:
                print(f"[SITL] GPS locked  ({msg.satellites_visible} sats, "
                      f"fix_type={msg.fix_type})")
                break
        else:
            raise TimeoutError("GPS lock timed out")

        # 3. EKF convergence (check variances only, flags are unreliable)
        print("[SITL] Waiting for EKF convergence...")
        while time.time() < deadline:
            msg = self.master.recv_match(type="EKF_STATUS_REPORT",
                                         blocking=True, timeout=5)
            if msg and msg.velocity_variance < 1.0 \
                   and msg.pos_horiz_variance < 1.0:
                print(f"[SITL] EKF ready  (vel_var={msg.velocity_variance:.3f}, "
                      f"pos_var={msg.pos_horiz_variance:.3f})")
                break
        else:
            raise TimeoutError("EKF convergence timed out")

        # 4. Pre‑arm (best‑effort)
        print("[SITL] Waiting for pre‑arm checks...")
        prearm_deadline = time.time() + 30
        while time.time() < prearm_deadline:
            msg = self.master.recv_match(type="SYS_STATUS",
                                         blocking=True, timeout=2)
            if msg:
                prearm = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                if msg.onboard_control_sensors_health & prearm:
                    print("[SITL] Pre‑arm checks passed")
                    break
            time.sleep(0.5)
        else:
            print("[SITL] Pre‑arm timeout — continuing anyway")

    # ── Configuration ────────────────────────────────────────────

    def _configure_sitl(self):
        """Apply logging / speedup params for resilience testing."""
        # LOG_BITMASK = 0xFFFF enables all standard log channels,
        # ensuring XKF3/XKF4 (EKF forensics), ATT, POS, MODE, ERR,
        # BAT, VIBE are present in the .BIN file.
        # ARMING_CHECK=0 disables pre-arm checks that block SITL arming.
        log_bitmask = 0xFFFF
        for name, val in [("LOG_FILE_DSRMROT", 1),
                          ("LOG_DISARMED", 0),
                          ("LOG_BITMASK", log_bitmask),
                          ("SIM_SPEEDUP", self.speedup),
                          ("ARMING_CHECK", 0)]:
            self._set_param(name, val)
            time.sleep(0.1)   # give FC time to apply each param
        # Re-request data streams after configuration
        time.sleep(0.5)
        self._request_data_streams(rate_hz=10)
        print(f"[SITL] Configured: LOG_FILE_DSRMROT=1, ARMING_CHECK=0, "
              f"LOG_BITMASK=0x{log_bitmask:X}, SIM_SPEEDUP={self.speedup}")

    def _set_param(self, name, value):
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    # ── Log helpers ──────────────────────────────────────────────

    def get_connection(self):
        return self.master

    def get_latest_log(self):
        """Path to the most recent .BIN log."""
        if not os.path.isdir(self._log_dir):
            return None
        lastlog = os.path.join(self._log_dir, "LASTLOG.TXT")
        if os.path.exists(lastlog):
            try:
                num = int(open(lastlog).read().strip())
                path = os.path.join(self._log_dir, f"{num:08d}.BIN")
                if os.path.exists(path):
                    return path
            except (ValueError, OSError):
                pass
        try:
            bins = sorted(f for f in os.listdir(self._log_dir)
                          if f.endswith(".BIN"))
        except OSError:
            return None
        return os.path.join(self._log_dir, bins[-1]) if bins else None

    def get_new_logs(self):
        """Return list of .BIN log paths created during this session."""
        current_last = self._get_last_log_num()
        logs = []
        for i in range(self._initial_log_num + 1, current_last + 1):
            p = os.path.join(self._log_dir, f"{i:08d}.BIN")
            if os.path.exists(p):
                logs.append(p)
        return logs

    # ── Shutdown ─────────────────────────────────────────────────

    def shutdown(self):
        """Kill SITL and close the connection."""
        print("[SITL] Shutting down...")
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    self.process.wait(timeout=5)
                except (ProcessLookupError, OSError):
                    pass
            self.process = None
        print("[SITL] Shutdown complete")
