"""
Scenario Runner — Execute a single resilience scenario end‑to‑end.

Orchestrates:
    SITLLauncher → VehicleController → FaultInjector → HealthMonitor
                                                     → LogAnalyzer

The runner is intentionally single‑threaded: a tight message‑polling loop
feeds every MAVLink message to the HealthMonitor while simultaneously
checking fault‑injection timing and mission‑completion state.
"""

import time
import yaml
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from core.sitl_launcher import SITLLauncher
from core.vehicle_controller import VehicleController
from core.fault_injector import FaultInjector
from core.health_monitor import HealthMonitor
from core.log_analyzer import LogAnalyzer
from core.ekf_forensics import EKFForensics
from core.recovery_analyzer import RecoveryAnalyzer, RecoveryFingerprint
from scenarios.scenario_validation import normalize_and_validate_scenarios


# ── Result container ─────────────────────────────────────────────

@dataclass
class ScenarioResult:
    """Captures everything produced by a single scenario run."""
    scenario: dict                                # original YAML config
    monitor_summary: dict = field(default_factory=dict)
    monitor_timeseries: dict = field(default_factory=dict)
    log_analysis: Optional[dict] = None
    ekf_forensics: Optional[dict] = None          # NEW: EKF forensics data
    recovery: Optional[dict] = None               # NEW: recovery fingerprint
    injection_log: list = field(default_factory=list)
    passed: bool = False
    fail_reasons: list = field(default_factory=list)
    verdict: str = ""

    def evaluate(self, criteria: dict):
        """Compare results against pass_criteria and set passed / fail_reasons."""
        self.fail_reasons = []
        summary = self.monitor_summary
        ts = self.monitor_timeseries

        # 1 — minimum health score
        min_hs = criteria.get("min_health_score")
        if min_hs is not None:
            actual = summary.get("min_health_score", 1.0)
            if actual < min_hs:
                self.fail_reasons.append(
                    f"Health score {actual:.3f} < threshold {min_hs}")

        # 2 — position drift
        max_drift = criteria.get("max_position_drift_m")
        if max_drift is not None:
            actual = summary.get("max_position_drift_m", 0)
            if actual > max_drift:
                self.fail_reasons.append(
                    f"Position drift {actual:.1f}m > limit {max_drift}m")

        # 3 — required failsafe
        req = criteria.get("required_failsafe")
        if req and criteria.get("must_detect_failsafe", False):
            triggered = summary.get("failsafe_triggered", False)
            mode = summary.get("failsafe_mode", "")
            if not triggered:
                self.fail_reasons.append("Expected failsafe was not triggered")
            elif mode not in req:
                self.fail_reasons.append(
                    f"Failsafe '{mode}' not in expected set {req}")

        # 4 — must complete mission (for nominal)
        if criteria.get("must_complete_mission", False):
            if summary.get("failsafe_triggered", False):
                self.fail_reasons.append(
                    "Mission did not complete — failsafe triggered")
            min_pct = criteria.get("min_mission_completion_pct", 95.0)
            actual_pct = summary.get("mission_completion_pct", 100.0)
            if actual_pct < min_pct:
                self.fail_reasons.append(
                    f"Mission completion {actual_pct:.1f}% < {min_pct:.1f}%")

        # 5 — max recovery time
        max_rec = criteria.get("max_recovery_time_s")
        if max_rec is not None:
            fs_time = summary.get("failsafe_time_s")
            if fs_time is not None and fs_time > max_rec:
                self.fail_reasons.append(
                    f"Recovery time {fs_time:.1f}s > limit {max_rec}s")

        # 6 — max altitude at end
        max_end_alt = criteria.get("max_altitude_at_end_m")
        if max_end_alt is not None:
            rel_alt = ts.get("relative_alt", []) if isinstance(ts, dict) else []
            if rel_alt:
                end_alt = rel_alt[-1]
                if end_alt > max_end_alt:
                    self.fail_reasons.append(
                        f"End altitude {end_alt:.1f}m > limit {max_end_alt}m")

        self.passed = len(self.fail_reasons) == 0
        self.verdict = "PASS" if self.passed else "FAIL"


# ── Scenario runner ──────────────────────────────────────────────

class ScenarioRunner:
    """Run one scenario from its YAML configuration dict."""

    def __init__(self, ardupilot_path: str):
        self.ardupilot_path = os.path.abspath(ardupilot_path)

    def run(self, scenario: dict) -> ScenarioResult:
        name = scenario.get("name", "unnamed")
        sid  = scenario.get("id", "unknown")
        print(f"\n{'='*60}")
        print(f"  SCENARIO: {name}  [{sid}]")
        print(f"{'='*60}")

        launcher = None
        try:
            result = self._execute(scenario)
        except Exception as exc:
            print(f"[SCENARIO] FATAL: {exc}")
            result = ScenarioResult(
                scenario=scenario,
                verdict="ERROR",
                fail_reasons=[str(exc)],
            )
        finally:
            pass  # launcher cleanup is inside _execute

        status = "PASS ✓" if result.passed else f"FAIL ✗ ({', '.join(result.fail_reasons)})"
        print(f"\n[SCENARIO] {name} → {status}")
        return result

    # ── Core execution ───────────────────────────────────────────

    def _execute(self, scenario: dict) -> ScenarioResult:
        vehicle  = scenario.get("vehicle", "ArduCopter")
        speedup  = scenario.get("speedup", 5)
        timeout  = scenario.get("timeout_s", 180)
        alt      = scenario.get("altitude", 20)
        wps      = scenario.get("waypoints", [])
        faults   = scenario.get("faults", [])
        fs_params = scenario.get("failsafe_params", {})
        criteria = scenario.get("pass_criteria", {})

        # 1 — Launch SITL
        launcher = SITLLauncher(
            self.ardupilot_path,
            vehicle=vehicle,
            speedup=speedup,
        )
        master = launcher.launch()

        controller = VehicleController(master)
        monitor    = HealthMonitor()
        injector   = FaultInjector(master)

        # Start monitor immediately so telemetry during arm/takeoff is captured.
        # This is critical: monitor MUST be started before arming to correctly
        # track the has_been_armed flag and record the full health timeline.
        monitor.set_total_waypoints(len(wps))
        monitor.start()

        try:
            # 2 — Configure failsafe params
            self._apply_params(controller, fs_params)

            # 3 — Request telemetry streams at 10 Hz
            self._request_streams(controller)

            # 4 — Upload mission
            if not controller.upload_mission(wps, altitude=alt):
                raise RuntimeError("Mission upload failed")

            # 5 — Arm + takeoff
            if not controller.arm():
                raise RuntimeError("Arming failed")
            if not controller.takeoff(alt):
                raise RuntimeError("Takeoff failed")

            # 6 — Start AUTO mission
            if not controller.start_mission():
                raise RuntimeError("Failed to enter AUTO mode")

            # 7 — Main flight loop with monitoring + injection
            self._flight_loop(master, monitor, injector,
                              faults, timeout)

            # 8 — Post‑flight log analysis (best‑effort)
            log_path = launcher.get_latest_log()
            log_result = None
            ekf_result = None
            if log_path:
                try:
                    analyzer = LogAnalyzer(log_path, timeout=60)
                    log_result = analyzer.analyze()
                except Exception as e:
                    print(f"[SCENARIO] Log analysis skipped: {e}")
                try:
                    ekf = EKFForensics(log_path, timeout=60)
                    ekf_data = ekf.analyze()
                    if ekf_data.success:
                        ekf_result = {
                            "timeseries": ekf_data.as_timeseries(),
                            "metrics":    ekf_data.metrics,
                            "lane_switches": ekf_data.lane_switches,
                        }
                except Exception as e:
                    print(f"[SCENARIO] EKF forensics skipped: {e}")

            # 9 — Recovery analysis
            ts_data = monitor.get_timeseries()
            summary = monitor.get_summary()
            recovery_fp = None
            try:
                ra = RecoveryAnalyzer(recovery_threshold=0.5)
                fp = ra.analyze(
                    timeseries=ts_data,
                    injection_time_s=summary.get("injection_time_s"),
                    scenario_id=scenario.get("id", ""),
                    mission_completion_pct=summary.get(
                        "mission_completion_pct", 100),
                )
                recovery_fp = fp.as_dict()
                print(f"[SCENARIO] Resilience Index: "
                      f"{fp.resilience_index:.1f}/100")
            except Exception as e:
                print(f"[SCENARIO] Recovery analysis skipped: {e}")

            # 10 — Assemble result
            result = ScenarioResult(
                scenario=scenario,
                monitor_summary=summary,
                monitor_timeseries=ts_data,
                log_analysis=log_result,
                ekf_forensics=ekf_result,
                recovery=recovery_fp,
                injection_log=injector.get_injection_log(),
            )
            result.evaluate(criteria)
            return result

        except Exception as exc:
            # Always capture whatever partial telemetry we recorded.
            # This prevents the all-zero report when exceptions occur early
            # (e.g. TCP connect failure, arming timeout, mission upload fail).
            print(f"[SCENARIO] Exception during execution: {exc}")
            _summary = monitor.get_summary()
            _ts = monitor.get_timeseries()
            result = ScenarioResult(
                scenario=scenario,
                monitor_summary=_summary,
                monitor_timeseries=_ts,
                injection_log=injector.get_injection_log(),
                verdict="ERROR",
                fail_reasons=[f"Execution error: {exc}"],
            )
            return result

        finally:
            if monitor._recording:
                monitor.stop()
            launcher.shutdown()

    # ── Flight loop ──────────────────────────────────────────────

    def _flight_loop(self, master, monitor, injector,
                     faults, timeout):
        """Single‑threaded message loop: monitor + inject + detect end.

        Note: monitor is already started by _execute() before arming.
        """
        t0 = time.time()
        injected_flags = [False] * len(faults)     # support multi-fault
        has_been_armed = False
        last_print_time = 0.0   # for periodic live status output

        # Pre‑compute injection triggers for ALL faults
        fault_triggers = []
        for f in faults:
            trigger = f.get("trigger", {})
            fault_triggers.append({
                "after_s":  trigger.get("after_s", 30),
                "params":   f.get("params", {}),
                "desc":     f.get("description", "fault"),
            })

        while True:
            # Poll ALL available messages (non‑blocking)
            msg = master.recv_match(blocking=False)
            if msg:
                monitor.process_message(msg)

            elapsed = time.time() - t0

            # Track armed state
            if monitor.is_armed():
                has_been_armed = True

            # ── Inject faults when triggers fire (supports multi-fault) ──
            for idx, ft in enumerate(fault_triggers):
                if (not injected_flags[idx]
                        and elapsed >= ft["after_s"]):
                    print(f"\n[SCENARIO] ─── INJECTING FAULT {idx+1}: "
                          f"{ft['desc']} at T+{elapsed:.1f}s ───")
                    if idx == 0:
                        monitor.mark_injection_time()
                    else:
                        # Log subsequent faults as events too
                        monitor.events.append({
                            "time": elapsed,
                            "event": f"FAULT_INJECTED: {ft['desc']}",
                        })
                    injector.inject(ft["params"])
                    injected_flags[idx] = True

            # ── End conditions ──
            # Vehicle disarmed after flight → mission done or crash landed
            if has_been_armed and not monitor.is_armed():
                print("[SCENARIO] Vehicle disarmed — ending")
                break

            # Timeout
            if elapsed > timeout:
                print("[SCENARIO] Timeout reached — ending")
                break

            # Print live health score every 5 seconds
            if elapsed - last_print_time >= 5.0 and elapsed > 1:
                score = monitor.get_current_score()
                mode = monitor.get_current_mode()
                alt_m = monitor.get_relative_altitude_m()
                print(f"[LIVE] T+{elapsed:5.1f}s | "
                      f"Health={score:.3f} | "
                      f"Mode={mode:10s} | "
                      f"Alt={alt_m:5.1f}m")
                last_print_time = elapsed

            time.sleep(0.01)

        monitor.stop()

    # ── Helpers ──────────────────────────────────────────────────

    @staticmethod
    def _apply_params(controller, params: dict):
        if not params:
            return
        print(f"[SCENARIO] Setting {len(params)} failsafe params...")
        failed = []
        for name, val in params.items():
            ok = controller.set_param(name, val)
            if not ok:
                failed.append(name)
        if failed:
            print(f"[SCENARIO] Param set not confirmed: {', '.join(failed)}")
        time.sleep(0.5)

    @staticmethod
    def _request_streams(controller):
        """Ask the FC to stream telemetry at useful rates."""
        # message_id, interval_us
        streams = [
            (24,  200_000),     # GPS_RAW_INT at 5 Hz
            (193, 100_000),     # EKF_STATUS_REPORT at 10 Hz
            (241, 200_000),     # VIBRATION at 5 Hz
            (30,  100_000),     # ATTITUDE at 10 Hz
            (33,  100_000),     # GLOBAL_POSITION_INT at 10 Hz
            (1,   500_000),     # SYS_STATUS at 2 Hz
            (42,  500_000),     # MISSION_CURRENT at 2 Hz
            (147, 500_000),     # BATTERY_STATUS at 2 Hz
            (253, 1_000_000),   # STATUSTEXT at 1 Hz
        ]
        for mid, interval in streams:
            controller.request_message_interval(mid, interval)
        time.sleep(0.3)


# ── YAML loader helper ──────────────────────────────────────────

def load_scenarios(yaml_path: str) -> list:
    """Load scenario list from a scenarios.yaml file."""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    return normalize_and_validate_scenarios(data)
