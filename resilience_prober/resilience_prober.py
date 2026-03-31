#!/usr/bin/env python3
"""
ArduPilot SITL Resilience Prober
================================
Launch SITL → fly missions → inject faults → monitor response → generate report.

Usage
-----
    # Run a single scenario
    python resilience_prober.py --scenario gps_denial

    # Run all scenarios
    python resilience_prober.py --all

    # Custom ArduPilot path
    python resilience_prober.py --all --ardupilot-path /path/to/ardupilot

    # Clean old logs first
    python resilience_prober.py --all --clean-logs

    # List available scenarios
    python resilience_prober.py --list
"""

import argparse
import os
import sys
import glob
import time
import datetime

# Ensure project root is importable
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from scenarios.scenario_runner import ScenarioRunner, load_scenarios
from scenarios.scenario_validation import ScenarioValidationError
from core.report_generator import ReportGenerator
from core.sitl_launcher import SITLLauncher
from core.vehicle_controller import VehicleController


# ── Defaults ─────────────────────────────────────────────────────

DEFAULT_ARDUPILOT = os.path.normpath(
    os.path.join(PROJECT_ROOT, "..", "ardupilot")
)
DEFAULT_SCENARIOS = os.path.join(PROJECT_ROOT, "config", "scenarios.yaml")
DEFAULT_REPORTS   = os.path.join(PROJECT_ROOT, "reports")


def parse_args():
    p = argparse.ArgumentParser(
        description="ArduPilot SITL Resilience Prober — "
                    "automated fault‑injection testing with visual reports.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--scenario", "-s", type=str,
                      help="Run a single scenario by ID "
                           "(e.g. gps_denial, motor_failure)")
    mode.add_argument("--all", "-a", action="store_true",
                      help="Run every scenario defined in scenarios.yaml")
    mode.add_argument("--list", "-l", action="store_true",
                      help="List available scenarios and exit")
    mode.add_argument("--smoke", action="store_true",
                      help="Run a lightweight SITL smoke test and exit")

    p.add_argument("--ardupilot-path", type=str, default=DEFAULT_ARDUPILOT,
                   help=f"Path to ArduPilot source (default: {DEFAULT_ARDUPILOT})")
    p.add_argument("--scenarios-file", type=str, default=DEFAULT_SCENARIOS,
                   help=f"Path to scenarios.yaml (default: {DEFAULT_SCENARIOS})")
    p.add_argument("--report-dir", type=str, default=DEFAULT_REPORTS,
                   help=f"Output directory for reports (default: {DEFAULT_REPORTS})")
    p.add_argument("--clean-logs", action="store_true",
                   help="Delete old SITL logs before running")
    p.add_argument("--speedup", type=int, default=None,
                   help="Override SIM_SPEEDUP for all scenarios")
    p.add_argument("--no-report", action="store_true",
                   help="Skip HTML report generation")

    return p.parse_args()


# ── Helpers ──────────────────────────────────────────────────────

def list_scenarios(scenarios):
    print(f"\n{'ID':<20} {'Name':<30} {'Faults'}")
    print("─" * 70)
    for s in scenarios:
        faults = ", ".join(
            f.get("description", "?") for f in s.get("faults", [])
        ) or "none"
        print(f"{s['id']:<20} {s['name']:<30} {faults}")
    print()


def clean_logs(ardupilot_path):
    log_dir = os.path.join(ardupilot_path, "logs")
    if not os.path.isdir(log_dir):
        return
    bins = glob.glob(os.path.join(log_dir, "*.BIN"))
    if not bins:
        print("[CLEAN] No .BIN logs to remove")
        return
    total = sum(os.path.getsize(f) for f in bins)
    for f in bins:
        os.remove(f)
    print(f"[CLEAN] Removed {len(bins)} log files "
          f"({total / (1024**3):.1f} GB freed)")


def print_banner():
    print(r"""
    ╔═══════════════════════════════════════════════╗
    ║  ArduPilot SITL Resilience Prober             ║
    ║  Automated fault‑injection testing framework  ║
    ╚═══════════════════════════════════════════════╝
    """)


def print_final_summary(results):
    print(f"\n{'='*60}")
    print("  FINAL RESULTS")
    print(f"{'='*60}")
    passed = sum(1 for r in results if r.passed)
    total  = len(results)
    print(f"  {passed}/{total} scenarios passed\n")

    for r in results:
        s = r.scenario
        summary = r.monitor_summary
        recovery = r.recovery or {}
        tag  = "✓ PASS" if r.passed else "✗ FAIL"
        name = s.get("name", "?")
        hs   = summary.get("min_health_score", 0)
        drift = summary.get("max_position_drift_m", 0)
        ri   = recovery.get("resilience_index", 0)
        mis  = summary.get("mission_completion_pct", 100)
        fs    = summary.get("failsafe_mode") or "—"
        fs_t  = summary.get("failsafe_time_s")
        fs_str = f"T+{fs_t:.1f}s" if fs_t else "—"

        print(f"  {tag:8s} | {name:<25s} | "
              f"RI={ri:3.0f} | Health={hs:.3f} | Drift={drift:5.1f}m | "
              f"Mission={mis:.0f}% | Failsafe={fs} @ {fs_str}")
        if not r.passed:
            for reason in r.fail_reasons:
                print(f"            └─ {reason}")

    print()


def run_smoke(ardupilot_path: str) -> bool:
    """Run a lightweight end-to-end SITL control smoke test."""
    print("\n[SMOKE] Starting SITL smoke test...")
    launcher = SITLLauncher(
        ardupilot_path,
        vehicle="ArduCopter",
        speedup=10,
    )

    try:
        master = launcher.launch()
        controller = VehicleController(master)

        if not controller.arm(timeout=35):
            print("[SMOKE] FAIL: Arming failed")
            return False

        if not controller.takeoff(10, timeout=75):
            print("[SMOKE] FAIL: Takeoff failed")
            return False

        if not controller.set_mode("LAND", timeout=20):
            print("[SMOKE] FAIL: LAND mode change failed")
            return False

        if not controller.wait_disarmed(timeout=150):
            print("[SMOKE] FAIL: Vehicle did not disarm after LAND")
            return False

        print("[SMOKE] PASS: launch → arm → takeoff → land → disarm")
        return True
    finally:
        launcher.shutdown()


# ── Main ─────────────────────────────────────────────────────────

def main():
    args = parse_args()
    print_banner()

    # Validate ardupilot path
    ardupilot = os.path.abspath(args.ardupilot_path)
    sim_vehicle = os.path.join(ardupilot, "Tools", "autotest", "sim_vehicle.py")
    if not os.path.exists(sim_vehicle):
        print(f"ERROR: sim_vehicle.py not found at {sim_vehicle}")
        print(f"       Set --ardupilot-path to your ArduPilot source directory")
        sys.exit(1)

    # Load scenarios
    if not os.path.exists(args.scenarios_file):
        print(f"ERROR: Scenarios file not found: {args.scenarios_file}")
        sys.exit(1)
    try:
        scenarios = load_scenarios(args.scenarios_file)
    except ScenarioValidationError as exc:
        print(f"ERROR: Invalid scenarios file: {exc}")
        sys.exit(1)

    # ── Smoke mode ──
    if args.smoke:
        ok = run_smoke(ardupilot)
        sys.exit(0 if ok else 1)

    # ── List mode ──
    if args.list:
        list_scenarios(scenarios)
        return

    # ── Select scenarios to run ──
    if args.scenario:
        to_run = [s for s in scenarios if s["id"] == args.scenario]
        if not to_run:
            print(f"ERROR: Unknown scenario '{args.scenario}'")
            print("Available:")
            list_scenarios(scenarios)
            sys.exit(1)
    elif args.all:
        to_run = scenarios
    else:
        print("No scenario specified. Use --scenario <id>, --all, or --list")
        list_scenarios(scenarios)
        sys.exit(0)

    # Apply speedup override
    if args.speedup:
        for s in to_run:
            s["speedup"] = args.speedup

    # Clean logs
    if args.clean_logs:
        clean_logs(ardupilot)

    # ── Run scenarios ──
    runner = ScenarioRunner(ardupilot)
    results = []

    print(f"\nRunning {len(to_run)} scenario(s)...\n")
    overall_start = time.time()

    for i, scenario in enumerate(to_run, 1):
        print(f"\n[{i}/{len(to_run)}] Starting: {scenario['name']}")
        result = runner.run(scenario)
        results.append(result)

        # Brief pause between scenarios to let ports free
        if i < len(to_run):
            print("\n[PAUSE] Waiting 8s before next scenario...")
            time.sleep(8)

    elapsed = time.time() - overall_start

    # ── Summary ──
    print_final_summary(results)
    print(f"Total time: {elapsed:.0f}s")

    # ── Generate report ──
    if not args.no_report and results:
        print("\n[REPORT] Generating HTML report...")
        gen = ReportGenerator(output_dir=args.report_dir)
        report_path = gen.generate(results)
        print(f"[REPORT] Report saved: {report_path}")
    elif args.no_report:
        print("[REPORT] Skipped (--no-report)")

    # Exit code: 0 if all passed, 1 if any failed
    sys.exit(0 if all(r.passed for r in results) else 1)


if __name__ == "__main__":
    main()
