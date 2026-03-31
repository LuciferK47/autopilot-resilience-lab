"""
Fault Injector — Inject hardware / environmental faults via SIM_ parameters.

Supports:
  - Immediate injection of any SIM_ parameter
  - Timed injection (after N seconds)
  - Clearing / restoring defaults
  - Full injection log for the report
"""

import time
from pymavlink import mavutil


class FaultInjector:
    """Inject faults into SITL by setting SIM_ parameters at runtime."""

    # ── Fault parameter catalog ──────────────────────────────────
    FAULT_CATALOG = {
        # GPS
        "SIM_GPS1_ENABLE":  "GPS enable (0=off, 1=on)",
        "SIM_GPS1_NUMSATS": "GPS satellite count (0=no lock)",
        "SIM_GPS1_JAM":     "GPS jamming (0=off, 1=on)",
        # Motor
        "SIM_ENGINE_FAIL":  "Engine failure bitmask",
        "SIM_ENGINE_MUL":   "Failed‑engine thrust multiplier (0–1)",
        # Battery
        "SIM_BATT_VOLTAGE": "Simulated battery voltage (V)",
        "SIM_BATT_CAP_AH":  "Simulated battery capacity (Ah)",
        # Wind
        "SIM_WIND_SPD":     "Wind speed (m/s)",
        "SIM_WIND_DIR":     "Wind direction‑from (°)",
        "SIM_WIND_TURB":    "Turbulence amplitude (m/s)",
        "SIM_WIND_DIR_Z":   "Vertical wind angle (°)",
        # Vibration
        "SIM_VIB_MOT_MAX":  "Motor vibration frequency (Hz)",
        "SIM_VIB_MOT_MULT": "Motor vibration amplitude scale",
        # Barometer
        "SIM_BARO_DISABLE": "Disable barometer (0/1)",
        "SIM_BARO_FREEZE":  "Freeze barometer (0/1)",
        "SIM_BARO_GLITCH":  "Baro glitch offset (m)",
        # Compass
        "SIM_MAG1_FAIL":    "Compass 1 failure (0/1)",
        "SIM_MAG2_FAIL":    "Compass 2 failure (0/1)",
        # Accelerometer
        "SIM_ACCEL1_FAIL":  "Accel 1 failure (0/1)",
        # Physical shove
        "SIM_SHOVE_X":      "External force X (m/s²)",
        "SIM_SHOVE_Y":      "External force Y (m/s²)",
        "SIM_SHOVE_Z":      "External force Z (m/s²)",
        "SIM_SHOVE_TIME":   "Shove duration (ms)",
        # Rotational twist
        "SIM_TWIST_X":      "Angular acceleration X (rad/s²)",
        "SIM_TWIST_Y":      "Angular acceleration Y (rad/s²)",
        "SIM_TWIST_Z":      "Angular acceleration Z (rad/s²)",
        "SIM_TWIST_TIME":   "Twist duration (ms)",
        # RC
        "SIM_RC_FAIL":      "RC failure (0=ok, 1=no‑pulse, 2=neutral)",
    }

    # Default (safe) values for clearing faults
    SAFE_DEFAULTS = {
        "SIM_GPS1_ENABLE": 1,   "SIM_GPS1_NUMSATS": 10,
        "SIM_GPS1_JAM": 0,     "SIM_ENGINE_FAIL": 0,
        "SIM_ENGINE_MUL": 1.0, "SIM_BATT_VOLTAGE": 12.6,
        "SIM_WIND_SPD": 0,     "SIM_WIND_TURB": 0,
        "SIM_BARO_DISABLE": 0, "SIM_BARO_FREEZE": 0,
        "SIM_BARO_GLITCH": 0,  "SIM_MAG1_FAIL": 0,
        "SIM_MAG2_FAIL": 0,    "SIM_ACCEL1_FAIL": 0,
        "SIM_SHOVE_X": 0,      "SIM_SHOVE_Y": 0,
        "SIM_SHOVE_Z": 0,      "SIM_SHOVE_TIME": 0,
        "SIM_TWIST_X": 0,      "SIM_TWIST_Y": 0,
        "SIM_TWIST_Z": 0,      "SIM_TWIST_TIME": 0,
        "SIM_RC_FAIL": 0,
    }

    def __init__(self, connection):
        self.master = connection
        self.target_system = connection.target_system
        self.target_component = connection.target_component
        self.injection_log = []

    # ── Public API ───────────────────────────────────────────────

    def inject(self, params: dict, delay_between=0.05):
        """Inject a dict of ``{param_name: value}`` faults.

        Also accepts a list of such dicts (applied sequentially).
        """
        if isinstance(params, list):
            for group in params:
                self.inject(group, delay_between)
            return

        ts = time.time()
        for name, value in params.items():
            self._set_param(name, value)
            self.injection_log.append(
                {"time": ts, "param": name, "value": value, "action": "inject"}
            )
            print(f"[FAULT] Injected: {name} = {value}")
            time.sleep(delay_between)

    def clear_fault(self, name, default=None):
        """Restore a single parameter to its safe default."""
        val = default if default is not None else self.SAFE_DEFAULTS.get(name, 0)
        self._set_param(name, val)
        self.injection_log.append(
            {"time": time.time(), "param": name, "value": val, "action": "clear"}
        )
        print(f"[FAULT] Cleared: {name} → {val}")

    def clear_all(self):
        """Restore every known fault parameter to safe defaults."""
        for name, val in self.SAFE_DEFAULTS.items():
            self._set_param(name, val)
            time.sleep(0.02)
        print("[FAULT] All faults cleared")

    def get_injection_log(self):
        return list(self.injection_log)

    # ── Internal ─────────────────────────────────────────────────

    def _set_param(self, name, value):
        self.master.mav.param_set_send(
            self.target_system, self.target_component,
            name.encode("utf-8") if isinstance(name, str) else name,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
