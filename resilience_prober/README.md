# ArduPilot SITL Resilience Prober

A focused, end-to-end automated tool that launches SITL, flies real waypoint missions, injects hardware/environmental faults mid-flight, monitors the vehicle's autonomous response in real-time, and generates a visual HTML resilience report — all from a single Python command.

## What Makes This Unique

| Feature | Existing tools | This project |
|---|---|---|
| **EKF Forensics** | Nobody extracts XKF3/XKF4 DataFlash data during fault tests | Parses `errorScore`, innovation test ratios, lane switches directly from ArduPilot's internal EKF logs |
| **Resilience Index** | ArduPilot autotest uses binary pass/fail | Quantitative 0-100 composite score (recovery time, drift, health floor, attitude stability, steady-state) |
| **Recovery Curve Analysis** | No existing tool measures *how fast* recovery happens | Time-to-worst, recovery rate (dH/dt), recovery time, steady-state error |
| **Cascading Multi-Fault** | Single fault injection only | Compound failures (GPS + Wind, etc.) with independent timing |
| **SIM_TWIST Testing** | Nobody tests rotational disturbance | First tool to inject angular acceleration faults |
| **Comparative Radar Chart** | No visual cross-scenario comparison | Spider/radar chart on 6 normalized dimensions |
| **Mission Completion %** | Binary complete/fail | Tracks exact waypoint progress (e.g. "75% — reached WP 3 of 4") |
| **GPS Track Map** | No spatial visualization | Lat/lon scatter plot colored by time with fault injection marker |

**Core differentiation**: Not just "did the vehicle survive?" but *"how well did it survive, how fast did it recover, and how much did it drift?"* — plus a deep post-mortem into the EKF's internal state that's invisible via MAVLink.

## Architecture

```
resilience_prober/
├── resilience_prober.py           # CLI entry point
├── config/
│   └── scenarios.yaml             # All 9 scenario definitions
├── core/
│   ├── sitl_launcher.py           # Launch sim_vehicle.py, wait for ready
│   ├── vehicle_controller.py      # Arm, takeoff, mission upload, land
│   ├── fault_injector.py          # SIM_ parameter injection (incl. TWIST)
│   ├── health_monitor.py          # Real-time EKF/vibration/attitude scoring
│   ├── log_analyzer.py            # Post-flight .BIN DataFlash analysis
│   ├── ekf_forensics.py           # Deep EKF analysis (XKF3/XKF4 parsing)
│   ├── recovery_analyzer.py       # Resilience index + recovery fingerprint
│   └── report_generator.py        # Jinja2 HTML + matplotlib + radar chart
├── scenarios/
│   └── scenario_runner.py         # Generic scenario execution engine
├── reports/                       # Generated HTML reports
├── tests/
│   └── test_health_monitor.py     # Unit tests (health, recovery, EKF, waypoints)
└── requirements.txt
```

## Quick Start

### Prerequisites

- **ArduPilot SITL built** (arducopter binary in `build/sitl/bin/`)
- **Python 3.10+** with pymavlink, matplotlib, jinja2, pyyaml, numpy, scipy

### Setup

```bash
cd resilience_prober
pip install -r requirements.txt
pip install -r requirements-dev.txt
```

### Developer Commands

```bash
cd resilience_prober

# Lint (ruff)
make lint

# Unit tests
make test

# Quick SITL smoke test (launch, arm, takeoff, land, disarm)
make smoke
```

### Usage

```bash
# List available scenarios
python resilience_prober.py --list

# Run a single scenario
python resilience_prober.py --scenario gps_denial

# Run all 9 scenarios
python resilience_prober.py --all

# Clean old logs first (recommended — saves disk space)
python resilience_prober.py --all --clean-logs

# Custom ArduPilot path
python resilience_prober.py --all --ardupilot-path /path/to/ardupilot

# Skip report generation
python resilience_prober.py --scenario nominal --no-report
```

### Run Unit Tests

```bash
cd resilience_prober
python -m pytest tests/ -v
```

CI runs lint + tests automatically on push/PR changes under
`resilience_prober/` via `.github/workflows/resilience_prober_ci.yml`.

## Scenarios

| # | ID | Name | Fault Injection | What It Tests |
|---|---|---|---|---|
| 1 | `nominal` | Nominal Flight | None | Baseline metrics |
| 2 | `gps_denial` | GPS Denial | `SIM_GPS1_ENABLE=0` | EKF failsafe → LAND |
| 3 | `motor_failure` | Motor Failure | `SIM_ENGINE_FAIL=1` | Crash detection → LAND |
| 4 | `battery_failsafe` | Battery Failsafe | `SIM_BATT_VOLTAGE=10.3` | Low-battery → RTL |
| 5 | `wind_shear` | Wind Shear | `SIM_WIND_SPD=15` | Position hold under wind |
| 6 | `physical_shove` | Physical Shove | `SIM_SHOVE_X=50` | Attitude recovery from impulse |
| 7 | `compass_failure` | Compass Failure | `SIM_MAG1_FAIL=1` | EKF compass failover |
| 8 | `cascading_gps_wind` | Cascading GPS + Wind | Wind@T+20s, GPS=0@T+35s | Compound cascading failure |
| 9 | `twist_disturbance` | Rotational Twist | `SIM_TWIST_X/Y/Z` | Angular torque rejection |

## Resilience Index (RI)

A quantitative 0–100 composite score computed per scenario:

```
RI = 0.25 · norm(recovery_time, 60s)
   + 0.25 · norm(1 − worst_score)
   + 0.20 · norm(drift, 100m)
   + 0.10 · norm(attitude_dev, 90°)
   + 0.20 · norm(steady_state_score)
```

- **RI ≥ 75**: Excellent — vehicle recovered quickly with minimal drift
- **RI 50–74**: Moderate — survived but with degraded performance
- **RI 25–49**: Poor — significant drift/attitude loss, slow recovery
- **RI < 25**: Critical — near-crash or no recovery

## EKF Forensics

Deep post-mortem analysis from DataFlash logs (data invisible via MAVLink):

- **errorScore** — ArduPilot's own internal EKF health metric (from XKF3)
- **Innovation test ratios** — per-sensor accept/reject signals (from XKF4: SV, SP, SH, SM)
  - Value < 1.0 → measurement accepted by EKF
  - Value > 1.0 → measurement **rejected** — filter is stressed
- **Lane switches** — EKF core swaps (PI field changes in XKF4)
- **Fault/timeout bitmasks** — FS/TS fields from XKF4

## Health Score Formula

Grounded in ArduPilot's own `FS_EKF_THRESH` parameter (default 0.8):

```
H(t) = 1 − max(vel_var/θ, pos_var/θ, hgt_var/θ, mag_var/θ, |vib|/60)
```

- **H = 1.0** → Perfect health
- **H = 0.0** → At failsafe threshold
- **H < 0** → Beyond threshold (EKF failsafe would trigger)

Data sources (real-time via MAVLink):
- `EKF_STATUS_REPORT` — velocity, position, height, compass variances
- `VIBRATION` — accelerometer vibration magnitude
- `ATTITUDE` — roll/pitch/yaw angles
- `GLOBAL_POSITION_INT` — GPS position for drift calculation and GPS track map
- `HEARTBEAT` — mode transitions and failsafe detection
- `MISSION_CURRENT` / `MISSION_ITEM_REACHED` — waypoint progress tracking

**DataFlash (post-flight .BIN analysis):**
- `XKF3` — errorScore, velocity/position/mag innovations (internal EKF metrics)
- `XKF4` — innovation test ratios (SV/SP/SH/SM), primary core index (PI), fault status (FS)
- `ATT`, `POS`, `MODE`, `ERR` — attitude, position, mode changes, error events

## Report Output

The HTML report includes:
- **Comparative radar chart** — spider chart comparing all scenarios on 6 dimensions
- **Summary table** — pass/fail, Resilience Index, recovery time, mission % across all scenarios
- **Resilience Index display** — large color-coded RI with recovery metrics per scenario
- **EKF forensics section** — errorScore metrics, test ratio rejection %, lane switch count
- **Health score timeline** — continuous plot with fault injection marker
- **EKF variance overlay** — velocity, position, height, compass variances
- **EKF test ratios plot** — DataFlash XKF4 innovation test ratios (>1 = rejected)
- **EKF errorScore plot** — DataFlash XKF3 internal error metric
- **GPS track map** — lat/lon scatter colored by time with fault injection marker
- **Position drift plot** — distance from home over time
- **Attitude plot** — roll and pitch during fault
- **Altitude profile** — height above ground
- **Vibration magnitude** — accelerometer vibration levels
- **Battery voltage** — voltage trace (for battery scenarios)
- **Event log** — color-coded chronological list (faults=red, failsafes=amber, waypoints=green)

## Publish Results on GitHub (Proof of Success)

If you want your repository to clearly show that testing succeeded, publish a
small, reproducible evidence bundle with every major run.

### 1) Run scenarios and capture terminal output

```bash
cd resilience_prober
mkdir -p reports/published logs/published

# Full campaign + saved console output
python resilience_prober.py --all --clean-logs \
  | tee logs/published/run_$(date +%Y%m%d_%H%M%S).txt
```

This saves the final pass/fail + RI summary shown in terminal, which is useful
to reference directly from README.

### 2) Copy report and a small log sample into tracked folders

```bash
cd resilience_prober

# Copy newest HTML report into a tracked folder
latest_report=$(ls -t reports/resilience_report_*.html | head -n 1)
cp "$latest_report" reports/published/

# Optional: copy latest 1-3 BIN logs as evidence (can be large)
ls -t ../ardupilot/logs/*.BIN | head -n 3 | while read -r f; do
  cp "$f" logs/published/
done
```

### 3) Add an evidence section in README

Use this template in your GitHub README:

```markdown
## Latest Validation Run

- Date: YYYY-MM-DD
- Commit: <short_sha>
- Command: `python resilience_prober.py --all --clean-logs`
- Result: X/9 scenarios passed

| Scenario | Result | RI | Recovery (s) | Max Drift (m) |
|---|---:|---:|---:|---:|
| nominal | PASS | 92 | 4.1 | 2.0 |
| gps_denial | PASS | 78 | 10.4 | 8.7 |
| ... | ... | ... | ... | ... |

Artifacts:
- Full HTML report: `resilience_prober/reports/published/<report_file>.html`
- Run log: `resilience_prober/logs/published/<run_log>.txt`
- Sample DataFlash logs: `resilience_prober/logs/published/*.BIN`
```

Tip: Keep only the latest 1-3 runs in the repo, and move older/heavier logs to
GitHub Releases or Git LFS.

## Do You Need Extra Software?

Short answer: **No, not for this tool's core workflow.**

- **Gazebo**: Not required. This project uses ArduPilot SITL via
  `sim_vehicle.py` and MAVLink-driven fault injection.
- **UAV log viewer apps** (Mission Planner/APM Planner/UAV Log Viewer): Not
  required for report generation.

Optional but useful:

- **MAVExplorer** (ArduPilot tools) for manual deep dives into `.BIN` logs.
- **QGroundControl/Mission Planner** for visual flight playback and sanity checks.
- **Git LFS** if you plan to store many large `.BIN` logs in the repository.

## ArduPilot Subsystems Demonstrated

| Area | How It's Used | Key Parameters |
|---|---|---|
| SITL | Programmatic launch via `sim_vehicle.py` | `-v Copter`, `-I 0`, `--no-mavproxy` |
| MAVLink | Raw pymavlink — no DroneKit | `HEARTBEAT`, `COMMAND_LONG`, `PARAM_SET`, `MISSION_ITEM_INT` |
| Flight modes | Auto waypoint missions, mode transition detection | `GUIDED`, `AUTO`, `RTL`, `LAND` |
| EKF | Real-time variance monitoring, failsafe detection | `EKF_STATUS_REPORT`, `FS_EKF_THRESH`, `FS_EKF_ACTION` |
| Failsafes | Trigger and verify every major failsafe | `FS_GCS_ENABLE`, `FS_BATT_ENABLE`, `FS_EKF_ACTION` |
| SIM_ params | Runtime fault injection | `SIM_GPS1_ENABLE`, `SIM_ENGINE_FAIL`, `SIM_WIND_SPD`, `SIM_SHOVE_*`, `SIM_TWIST_*` |
| Mission protocol | Programmatic waypoint upload via MISSION_ITEM_INT | `MISSION_COUNT`, `MISSION_REQUEST_INT`, `MISSION_ACK` |
| DataFlash logs | Post-flight .BIN analysis | `XKF3`, `XKF4`, `ATT`, `POS`, `MODE`, `ERR` |

## Adding Custom Scenarios

Edit `config/scenarios.yaml`:

```yaml
- name: "My Custom Scenario"
  id: my_custom
  vehicle: ArduCopter
  description: "Description here"
  altitude: 20
  speedup: 5
  timeout_s: 180
  waypoints:
    - {lat: -35.3627, lon: 149.1652, alt: 20}
    - {lat: -35.3620, lon: 149.1652, alt: 20}
  failsafe_params:
    FS_EKF_ACTION: 1
  faults:
    - trigger: {type: time, after_s: 25}
      params:
        SIM_YOUR_PARAM: value
      description: "What happens"
  pass_criteria:
    min_health_score: 0.3
    max_position_drift_m: 15.0
    must_detect_failsafe: true
    required_failsafe: ["LAND", "RTL"]

The loader validates scenarios before execution and will fail fast with a clear
error if required fields are missing or malformed (duplicate IDs, invalid
waypoints, invalid fault triggers, or unsupported `pass_criteria` keys).
```

## License

GPLv3 — consistent with ArduPilot.
