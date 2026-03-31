"""
Report Generator — Produce a self‑contained HTML resilience report.

Uses matplotlib to produce time‑series plots (embedded as base64 PNGs)
and Jinja2 to render the final HTML from an inline template.

Generates:
  1. Executive dashboard with resilience index + radar chart
  2. Health‑score timeline (per scenario)
  3. EKF variance overlay
  4. EKF forensics — innovation test ratios + errorScore (from DataFlash)
  5. Recovery curve with annotations
  6. Attitude (roll / pitch)
  7. Altitude profile
  8. GPS track map (lat/lon with fault marker)
  9. Vibration magnitude
  10. Cross‑scenario comparison radar chart
"""

import base64
import io
import math
import os
import datetime
from typing import Dict, List, Optional

import numpy as np
import matplotlib
matplotlib.use("Agg")                       # headless backend
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from jinja2 import Template


# ── HTML template (embedded) ─────────────────────────────────────

HTML_TEMPLATE = Template(r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Resilience Report — {{ generated }}</title>
<style>
  :root{--bg:#0f172a;--card:#1e293b;--accent:#38bdf8;--ok:#22c55e;
        --warn:#f59e0b;--fail:#ef4444;--txt:#e2e8f0;--muted:#94a3b8;
        --gold:#fbbf24;--purple:#a78bfa}
  *{margin:0;padding:0;box-sizing:border-box}
  body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);
       color:var(--txt);padding:2rem}
  h1{font-size:1.8rem;margin-bottom:.3rem}
  h2{font-size:1.3rem;color:var(--accent);margin:2rem 0 .8rem;
     border-bottom:1px solid #334155;padding-bottom:.4rem}
  h3{font-size:1.1rem;margin:1.5rem 0 .5rem}
  .subtitle{color:var(--muted);font-size:.95rem;margin-bottom:1.5rem}
  .card{background:var(--card);border-radius:10px;padding:1.2rem;
        margin-bottom:1.5rem;box-shadow:0 4px 20px rgba(0,0,0,.3)}
  table{width:100%;border-collapse:collapse;font-size:.9rem}
  th{text-align:left;padding:.6rem .8rem;background:#334155;color:var(--accent)}
  td{padding:.6rem .8rem;border-bottom:1px solid #334155}
  .pass{color:var(--ok);font-weight:700}
  .fail{color:var(--fail);font-weight:700}
  .warn{color:var(--warn);font-weight:700}
  .metric{display:inline-block;background:#334155;border-radius:6px;
          padding:.3rem .7rem;margin:.2rem;font-size:.85rem}
  .metric b{color:var(--accent)}
  .plot-grid{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
  .plot-grid img{width:100%;border-radius:8px;background:#0f172a}
  @media(max-width:900px){.plot-grid{grid-template-columns:1fr}}
  .event-log{max-height:300px;overflow-y:auto;font-size:.82rem;
             font-family:monospace;line-height:1.6}
  .event-log .fault{color:var(--fail)}
  .event-log .failsafe{color:var(--warn);font-weight:700}
  .event-log .waypoint{color:var(--ok)}
  .badge{display:inline-block;padding:.15rem .5rem;border-radius:4px;
         font-size:.8rem;font-weight:700}
  .badge-pass{background:rgba(34,197,94,.15);color:var(--ok)}
  .badge-fail{background:rgba(239,68,68,.15);color:var(--fail)}
  .ri-big{font-size:2.8rem;font-weight:800;line-height:1}
  .ri-label{font-size:.85rem;color:var(--muted);margin-top:.2rem}
  .ri-row{display:flex;gap:1.5rem;flex-wrap:wrap;margin:1rem 0}
  .ri-card{background:#334155;border-radius:8px;padding:1rem 1.5rem;
           text-align:center;min-width:110px}
  .forensics-metric{font-size:.78rem;font-family:monospace;
                    color:var(--muted);line-height:1.8}
  .forensics-metric .label{color:var(--accent);font-weight:600}
  .forensics-metric .bad{color:var(--fail)}
  .forensics-metric .good{color:var(--ok)}
</style>
</head>
<body>

<h1>🛡️ ArduPilot SITL Resilience Report</h1>
<p class="subtitle">Generated {{ generated }} · {{ total_scenarios }} scenarios</p>

<!-- ── Radar Chart (all scenarios) ────────────────── -->
{% if radar_chart %}
<div class="card">
<h2>Comparative Radar Chart</h2>
<p style="color:var(--muted);margin-bottom:.8rem;font-size:.9rem">
  Multi‑dimensional resilience comparison across all scenarios.
  Each axis is normalized 0‑1 (outer edge = best).
</p>
<div style="text-align:center">
  <img src="data:image/png;base64,{{ radar_chart }}"
       alt="Radar Chart" style="max-width:650px;border-radius:10px">
</div>
</div>
{% endif %}

<!-- ── Summary Table ─────────────────────────────────── -->
<div class="card">
<h2>Summary</h2>
<table>
<tr>
  <th>Scenario</th><th>Result</th><th>RI</th><th>Min Health</th>
  <th>Drift</th><th>Recovery</th><th>Mission</th>
  <th>Failsafe</th>
</tr>
{% for s in scenarios %}
<tr>
  <td>{{ s.name }}</td>
  <td><span class="badge {{ 'badge-pass' if s.passed else 'badge-fail' }}">
      {{ 'PASS' if s.passed else 'FAIL' }}</span></td>
  <td><b>{{ '%.0f'|format(s.ri) }}</b></td>
  <td>{{ '%.3f'|format(s.min_health) }}</td>
  <td>{{ '%.1f m'|format(s.drift) }}</td>
  <td>{{ s.recovery_str }}</td>
  <td>{{ '%.0f%%'|format(s.mission_pct) }}</td>
  <td>{{ s.failsafe_mode or '—' }}</td>
</tr>
{% endfor %}
</table>
</div>

<!-- ── Per‑Scenario Detail ───────────────────────────── -->
{% for s in scenarios %}
<div class="card">
<h2>{{ s.name }}</h2>
<p style="color:var(--muted);margin-bottom:.8rem">{{ s.description }}</p>

<div class="ri-row">
  <div class="ri-card">
    <div class="ri-big" style="color:{{ s.ri_color }}">{{ '%.0f'|format(s.ri) }}</div>
    <div class="ri-label">Resilience Index</div>
  </div>
  <div class="ri-card">
    <span class="metric"><b>Result:</b>
      <span class="{{ 'pass' if s.passed else 'fail' }}">
        {{ 'SURVIVED' if s.passed else 'FAILED' }}</span></span>
  </div>
  <div class="ri-card">
    <span class="metric"><b>Min Health:</b> {{ '%.3f'|format(s.min_health) }}</span>
  </div>
  <div class="ri-card">
    <span class="metric"><b>Drift:</b> {{ '%.1f m'|format(s.drift) }}</span>
  </div>
  <div class="ri-card">
    <span class="metric"><b>Mission:</b> {{ '%.0f%%'|format(s.mission_pct) }}</span>
  </div>
  {% if s.recovery_time is not none %}
  <div class="ri-card">
    <span class="metric"><b>Recovery:</b> {{ '%.1fs'|format(s.recovery_time) }}</span>
  </div>
  {% endif %}
  {% if s.failsafe_mode %}
  <div class="ri-card">
    <span class="metric"><b>Failsafe:</b>
      <span class="warn">{{ s.failsafe_mode }}</span></span>
  </div>
  {% endif %}
</div>

{% if s.forensics_metrics %}
<h3>EKF Forensics</h3>
<div class="forensics-metric">
{% for key, val in s.forensics_metrics.items() %}
  <span class="label">{{ key }}:</span>
  {% if 'rejection' in key|lower %}
    <span class="{{ 'bad' if val > 5 else 'good' }}">{{ '%.1f'|format(val) }}%</span>
  {% elif 'max' in key|lower and val > 1.0 %}
    <span class="bad">{{ '%.3f'|format(val) }}</span>
  {% else %}
    {{ '%.3f'|format(val) }}
  {% endif %}
  &nbsp;·&nbsp;
{% endfor %}
{% if s.lane_switches > 0 %}
  <span class="label">Lane Switches:</span>
  <span class="warn">{{ s.lane_switches }}</span>
{% endif %}
</div>
{% endif %}

{% if s.plots %}
<h3>Time‑Series Plots</h3>
<div class="plot-grid">
{% for name, b64 in s.plots.items() %}
  <img src="data:image/png;base64,{{ b64 }}" alt="{{ name }}">
{% endfor %}
</div>
{% endif %}

{% if s.events %}
<h3>Event Log</h3>
<div class="event-log">
{% for ev in s.events %}
  <div class="{{ 'fault' if 'FAULT' in ev.event else ('failsafe' if 'FAILSAFE' in ev.event else ('waypoint' if 'WAYPOINT' in ev.event else '')) }}">
    T+{{ '%.1f'|format(ev.time) }}s — {{ ev.event }}
  </div>
{% endfor %}
</div>
{% endif %}
</div>
{% endfor %}

<p style="text-align:center;color:var(--muted);margin-top:2rem;font-size:.8rem">
  ArduPilot SITL Resilience Prober · EKF Forensics · Recovery Analysis
</p>
</body></html>""")


class ReportGenerator:
    """Generate an HTML resilience report from scenario results."""

    def __init__(self, output_dir: str = "reports"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate(self, results: list, filename: str = None) -> str:
        """Generate the HTML report.

        Parameters
        ----------
        results : list[ScenarioResult]
        filename : str, optional — output file name (default: auto)

        Returns
        -------
        str — absolute path to the generated HTML file.
        """
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if filename is None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"resilience_report_{ts}.html"

        # Build radar chart data from recovery fingerprints
        radar_chart = self._make_radar_chart(results)

        scenario_data = []
        for res in results:
            summary = res.monitor_summary
            ts_data = res.monitor_timeseries
            recovery = res.recovery or {}
            ekf_f = res.ekf_forensics or {}
            plots = self._make_plots(ts_data, summary, res.scenario, ekf_f)

            ri = recovery.get("resilience_index", 0)
            rec_time = recovery.get("recovery_time_s")
            if rec_time is not None:
                recovery_str = f"{rec_time:.1f}s"
            else:
                recovery_str = "—"

            forensics_metrics = ekf_f.get("metrics", {}) if ekf_f else {}
            lane_switches = len(ekf_f.get("lane_switches", [])) if ekf_f else 0

            scenario_data.append({
                "name":           res.scenario.get("name", "?"),
                "description":    res.scenario.get("description", ""),
                "passed":         res.passed,
                "ri":             ri,
                "ri_color":       _ri_color(ri),
                "min_health":     summary.get("min_health_score", 0),
                "drift":          summary.get("max_position_drift_m", 0),
                "failsafe_mode":  summary.get("failsafe_mode"),
                "failsafe_time":  summary.get("failsafe_time_s"),
                "mission_pct":    summary.get("mission_completion_pct", 100),
                "recovery_time":  rec_time,
                "recovery_str":   recovery_str,
                "duration":       summary.get("duration_s", 0),
                "events":         summary.get("events", []),
                "forensics_metrics": forensics_metrics,
                "lane_switches":  lane_switches,
                "plots":          plots,
            })

        html = HTML_TEMPLATE.render(
            generated=now,
            total_scenarios=len(results),
            scenarios=scenario_data,
            radar_chart=radar_chart,
        )

        out_path = os.path.join(self.output_dir, filename)
        with open(out_path, "w") as f:
            f.write(html)
        print(f"[REPORT] Written to {out_path}")
        
        # Output as PDF as well
        try:
            import weasyprint
            pdf_path = out_path.replace(".html", ".pdf")
            print(f"[REPORT] Converting to PDF...")
            weasyprint.HTML(string=html).write_pdf(pdf_path)
            print(f"[REPORT] PDF saved: {pdf_path}")
        except Exception as e:
            print(f"[REPORT] PDF generation failed: {e}")

        return os.path.abspath(out_path)

    # ── Radar / Spider chart ─────────────────────────────────────

    def _make_radar_chart(self, results: list) -> Optional[str]:
        """Generate a comparative radar chart across all scenarios."""
        from core.recovery_analyzer import RecoveryAnalyzer, RecoveryFingerprint

        fingerprints = []
        labels = []
        for res in results:
            rec = res.recovery
            if rec is None:
                continue
            fp = RecoveryFingerprint(**{
                k: v for k, v in rec.items()
                if k in RecoveryFingerprint.__dataclass_fields__
            })
            fingerprints.append(fp)
            labels.append(res.scenario.get("id", "?"))

        if len(fingerprints) < 2:
            return None

        dims = RecoveryAnalyzer.compare(fingerprints)
        dim_names = list(dims.keys())
        n_dims = len(dim_names)
        n_scenarios = len(fingerprints)

        # Build angle array
        angles = np.linspace(0, 2 * np.pi, n_dims, endpoint=False).tolist()
        angles += angles[:1]   # close polygon

        fig, ax = plt.subplots(figsize=(7, 7), dpi=120,
                               subplot_kw=dict(polar=True))
        fig.patch.set_facecolor("#0f172a")
        ax.set_facecolor("#0f172a")

        colors = ["#38bdf8", "#f59e0b", "#22c55e", "#ef4444",
                  "#a78bfa", "#fb923c", "#e879f9", "#14b8a6",
                  "#f87171"]

        for i in range(n_scenarios):
            vals = [dims[d][i] for d in dim_names]
            vals += vals[:1]    # close
            c = colors[i % len(colors)]
            ax.plot(angles, vals, linewidth=2, label=labels[i], color=c)
            ax.fill(angles, vals, alpha=0.08, color=c)

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(dim_names, fontsize=9, color="#e2e8f0")
        ax.set_ylim(0, 1)
        ax.set_yticks([0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(["0.25", "0.50", "0.75", "1.00"],
                           fontsize=7, color="#94a3b8")
        ax.tick_params(colors="#94a3b8")
        ax.spines["polar"].set_color("#334155")
        ax.grid(color="#334155", linewidth=0.5)
        ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1),
                  fontsize=8, facecolor="#1e293b", edgecolor="#334155",
                  labelcolor="#e2e8f0")

        return _fig_to_b64(fig)

    # ── Per‑scenario plots ───────────────────────────────────────

    def _make_plots(self, ts: dict, summary: dict,
                    config: dict, ekf_forensics: dict) -> dict:
        if not ts or not ts.get("time"):
            return {}

        t = ts["time"]
        inject_t = summary.get("injection_time_s")
        plots = {}

        plt.style.use("dark_background")

        # 1. Health score
        plots["Health Score"] = self._plot(
            t, [ts["health_score"]],
            labels=["Health Score"],
            title="Health Score",
            ylabel="Score (0–1)",
            inject_time=inject_t,
            ylim=(-0.1, 1.1),
            hlines=[0.0],
        )

        # 2. EKF variances
        plots["EKF Variances"] = self._plot(
            t,
            [ts["velocity_variance"], ts["pos_horiz_variance"],
             ts["pos_vert_variance"], ts["compass_variance"]],
            labels=["Velocity", "Horiz Pos", "Vert Pos", "Compass"],
            title="EKF Variances",
            ylabel="Variance",
            inject_time=inject_t,
            hlines=[0.8],
        )

        # 3. EKF Forensics — Innovation Test Ratios (from DataFlash)
        ekf_ts = ekf_forensics.get("timeseries", {}) if ekf_forensics else {}
        if ekf_ts and ekf_ts.get("time_s"):
            et = ekf_ts["time_s"]
            plots["EKF Test Ratios (DataFlash)"] = self._plot(
                et,
                [ekf_ts.get("test_ratio_vel", []),
                 ekf_ts.get("test_ratio_pos", []),
                 ekf_ts.get("test_ratio_hgt", []),
                 ekf_ts.get("test_ratio_mag", [])],
                labels=["Vel TR", "Pos TR", "Hgt TR", "Mag TR"],
                title="EKF Innovation Test Ratios (from DataFlash XKF4)",
                ylabel="Test Ratio (>1 = rejected)",
                inject_time=inject_t,
                hlines=[1.0],
            )

        # 4. EKF errorScore (from DataFlash)
        if ekf_ts and ekf_ts.get("error_score"):
            plots["EKF errorScore (DataFlash)"] = self._plot(
                ekf_ts["time_s"],
                [ekf_ts["error_score"]],
                labels=["errorScore"],
                title="EKF Internal Error Score (from DataFlash XKF3)",
                ylabel="Error Score (lower = healthier)",
                inject_time=inject_t,
            )

        # 5. Attitude
        plots["Attitude"] = self._plot(
            t, [ts["roll"], ts["pitch"]],
            labels=["Roll (°)", "Pitch (°)"],
            title="Attitude",
            ylabel="Degrees",
            inject_time=inject_t,
        )

        # 6. Altitude
        plots["Altitude"] = self._plot(
            t, [ts["relative_alt"]],
            labels=["Relative Alt (m)"],
            title="Altitude",
            ylabel="Meters",
            inject_time=inject_t,
        )

        # 7. GPS Track Map
        lats = ts.get("lat", [])
        lons = ts.get("lon", [])
        if lats and lons and any(lat != 0 for lat in lats):
            track = self._gps_track_plot(lats, lons, t, inject_t)
            if track:
                plots["GPS Track"] = track

        # 8. Position Drift
        drifts = ts.get("position_drift", [])
        if drifts and any(d > 0 for d in drifts):
            plots["Position Drift"] = self._plot(
                t, [drifts],
                labels=["Drift from home (m)"],
                title="Position Drift",
                ylabel="Meters",
                inject_time=inject_t,
            )

        # 9. Vibration
        plots["Vibration"] = self._plot(
            t, [ts["vibe_magnitude"]],
            labels=["Vibration magnitude (m/s²)"],
            title="Vibration",
            ylabel="m/s²",
            inject_time=inject_t,
        )

        # 10. Battery
        if any(v > 0 for v in ts.get("battery_voltage", [])):
            plots["Battery"] = self._plot(
                t, [ts["battery_voltage"]],
                labels=["Voltage (V)"],
                title="Battery Voltage",
                ylabel="Volts",
                inject_time=inject_t,
            )

        return plots

    # ── GPS Track scatter plot ───────────────────────────────────

    @staticmethod
    def _gps_track_plot(lats, lons, times, inject_time=None):
        """Render lat/lon scatter coloured by time with fault marker."""
        fig, ax = plt.subplots(figsize=(6, 5), dpi=110)
        fig.patch.set_facecolor("#0f172a")
        ax.set_facecolor("#0f172a")

        # Filter zero coordinates
        valid = [(la, lo, t) for la, lo, t in zip(lats, lons, times)
                 if la != 0 and lo != 0]
        if not valid:
            plt.close(fig)
            return None
        vlats, vlons, vtimes = zip(*valid)

        sc = ax.scatter(vlons, vlats, c=vtimes, cmap="plasma",
                        s=8, alpha=0.8, edgecolors="none")
        cb = fig.colorbar(sc, ax=ax, label="Time (s)", pad=0.02)
        cb.ax.tick_params(labelsize=7, colors="#94a3b8")
        cb.set_label("Time (s)", fontsize=8, color="#94a3b8")

        # Mark fault injection point
        if inject_time is not None:
            # Find closest time to injection
            closest_idx = min(range(len(vtimes)),
                              key=lambda i: abs(vtimes[i] - inject_time))
            ax.scatter([vlons[closest_idx]], [vlats[closest_idx]],
                       marker="X", c="#ef4444", s=120, zorder=5,
                       label="Fault injected", edgecolors="white",
                       linewidths=0.5)
            ax.legend(fontsize=7, loc="best", facecolor="#1e293b",
                      edgecolor="#334155", labelcolor="#e2e8f0")

        # Mark start
        ax.scatter([vlons[0]], [vlats[0]], marker="^", c="#22c55e",
                   s=80, zorder=5, edgecolors="white", linewidths=0.5)

        ax.set_title("GPS Track", fontsize=10, pad=6, color="#e2e8f0")
        ax.set_xlabel("Longitude", fontsize=8, color="#94a3b8")
        ax.set_ylabel("Latitude", fontsize=8, color="#94a3b8")
        ax.tick_params(labelsize=7, colors="#94a3b8")
        ax.grid(alpha=0.15)
        fig.tight_layout()

        return _fig_to_b64(fig)

    # ── Generic line plot ────────────────────────────────────────

    @staticmethod
    def _plot(x, ys, labels, title, ylabel,
              inject_time=None, ylim=None, hlines=None):
        """Render a single plot and return a base64 PNG string."""
        fig, ax = plt.subplots(figsize=(6, 3), dpi=110)
        fig.patch.set_facecolor("#0f172a")
        colors = ["#38bdf8", "#f59e0b", "#22c55e", "#ef4444",
                  "#a78bfa", "#fb923c"]
        for i, (y, lab) in enumerate(zip(ys, labels)):
            if y:   # skip empty series
                ax.plot(x[:len(y)], y, linewidth=1.2, label=lab,
                        color=colors[i % len(colors)])

        if inject_time is not None:
            ax.axvline(inject_time, color="#ef4444", linestyle="--",
                       linewidth=1, alpha=0.8, label="Fault injected")

        if hlines:
            for h in hlines:
                ax.axhline(h, color="#94a3b8", linestyle=":",
                           linewidth=0.8, alpha=0.6)

        ax.set_title(title, fontsize=10, pad=6, color="#e2e8f0")
        ax.set_xlabel("Time (s)", fontsize=8, color="#94a3b8")
        ax.set_ylabel(ylabel, fontsize=8, color="#94a3b8")
        ax.tick_params(labelsize=7, colors="#94a3b8")
        ax.legend(fontsize=7, loc="best", facecolor="#1e293b",
                  edgecolor="#334155", labelcolor="#e2e8f0")
        if ylim:
            ax.set_ylim(ylim)
        ax.grid(alpha=0.15)
        fig.tight_layout()

        return _fig_to_b64(fig)


# ── Utility functions ────────────────────────────────────────────

def _fig_to_b64(fig) -> str:
    """Convert a matplotlib figure to base64 PNG string."""
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight",
                facecolor="#0f172a", edgecolor="none")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")


def _ri_color(ri: float) -> str:
    """Return CSS colour for a resilience index value."""
    if ri >= 75:
        return "#22c55e"    # green
    elif ri >= 50:
        return "#f59e0b"    # amber
    elif ri >= 25:
        return "#fb923c"    # orange
    else:
        return "#ef4444"    # red
