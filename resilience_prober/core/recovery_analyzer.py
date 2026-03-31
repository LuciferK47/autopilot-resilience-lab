"""
Recovery Analyzer — Quantitative resilience fingerprinting.

Given a health‑score timeline and a fault‑injection timestamp, computes:

  • pre_fault_score      — average health before the fault
  • worst_score          — minimum health after fault
  • time_to_worst_s      — seconds from injection to minimum health
  • recovery_score       — health score at end of flight (or at recovery)
  • recovery_time_s      — seconds from worst to crossing recovery threshold
  • recovery_rate        — dH/dt during the recovery phase (score/s)
  • steady_state_score   — average health in the last 20 % of flight
  • max_position_drift_m — peak distance from home after injection
  • attitude_deviation   — max |roll| + |pitch| after injection (degrees)
  • resilience_index     — composite 0‑100 score (the "money metric")

The *resilience_index* is:

    RI = w₁·norm(recovery_time) + w₂·norm(worst_score)
       + w₃·norm(drift) + w₄·norm(attitude_dev)
       + w₅·norm(steady_state)

Each component is normalised 0‑1; higher RI = more resilient.

This module does NOT touch MAVLink or SITL — it operates purely on the
timeseries dict produced by HealthMonitor.get_timeseries().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class RecoveryFingerprint:
    """All recovery metrics for one scenario."""
    scenario_id: str = ""

    # Pre‑fault baseline
    pre_fault_score: float = 1.0

    # Impact
    worst_score: float = 1.0
    time_to_worst_s: float = 0.0
    score_drop: float = 0.0           # pre_fault - worst

    # Recovery
    recovery_time_s: Optional[float] = None   # None = never recovered
    recovery_rate: float = 0.0                # dH/dt (score/s)
    recovery_score: float = 1.0               # score at recovery or end

    # Steady state
    steady_state_score: float = 1.0

    # Spatial / attitude
    max_position_drift_m: float = 0.0
    max_attitude_deviation: float = 0.0       # max(|roll|+|pitch|) post‑fault

    # Mission
    mission_completion_pct: float = 100.0

    # Composite
    resilience_index: float = 100.0

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


# ── Weights for resilience index ─────────────────────────────────

_WEIGHTS = {
    "recovery_time":   0.25,
    "worst_score":     0.25,
    "drift":           0.20,
    "attitude":        0.10,
    "steady_state":    0.20,
}

# Reference maxima for normalisation (values beyond these clip to 0)
_REFS = {
    "recovery_time_s":        60.0,    # >60 s → worst
    "worst_score":             0.0,    # 0 → worst (inverted)
    "max_position_drift_m":  100.0,    # >100 m → worst
    "max_attitude_deviation": 90.0,    # >90° → worst
    "steady_state_score":      0.0,    # 0 → worst (inverted)
}


class RecoveryAnalyzer:
    """Compute a RecoveryFingerprint from a health‑score timeline."""

    def __init__(self, recovery_threshold: float = 0.5):
        """
        Parameters
        ----------
        recovery_threshold : float
            Health score above which the vehicle is considered "recovered."
        """
        self.recovery_threshold = recovery_threshold

    def analyze(
        self,
        timeseries: dict,
        injection_time_s: Optional[float],
        scenario_id: str = "",
        mission_completion_pct: float = 100.0,
    ) -> RecoveryFingerprint:
        """Analyse a single scenario's timeseries.

        Parameters
        ----------
        timeseries : dict
            Output of ``HealthMonitor.get_timeseries()``.  Must contain
            at least ``time`` and ``health_score`` lists of equal length.
        injection_time_s : float | None
            Seconds after monitor start when the fault was injected.
            If None (nominal), returns a baseline fingerprint.
        scenario_id : str
            Identifier echoed into the fingerprint.
        mission_completion_pct : float
            0‑100 mission completion percentage.

        Returns
        -------
        RecoveryFingerprint
        """
        fp = RecoveryFingerprint(scenario_id=scenario_id)
        fp.mission_completion_pct = mission_completion_pct

        times = timeseries.get("time", [])
        scores = timeseries.get("health_score", [])
        if not times or not scores or len(times) != len(scores):
            return fp

        # ── Nominal (no injection) → baseline fingerprint ────────
        if injection_time_s is None:
            fp.pre_fault_score = _mean(scores)
            fp.worst_score = min(scores)
            fp.steady_state_score = _mean(scores[-max(1, len(scores)//5):])
            fp.recovery_score = scores[-1]
            fp.resilience_index = 100.0
            return fp

        # ── Split pre‑fault / post‑fault ─────────────────────────
        pre_scores = [s for t, s in zip(times, scores)
                      if t < injection_time_s]
        post_idx = [i for i, t in enumerate(times)
                    if t >= injection_time_s]

        if not post_idx:
            fp.pre_fault_score = _mean(scores)
            fp.worst_score = min(scores)
            return fp

        fp.pre_fault_score = _mean(pre_scores) if pre_scores else 1.0

        # ── Impact metrics ───────────────────────────────────────
        post_scores = [scores[i] for i in post_idx]
        post_times  = [times[i] for i in post_idx]

        fp.worst_score = min(post_scores)
        worst_idx_in_post = post_scores.index(fp.worst_score)
        fp.time_to_worst_s = post_times[worst_idx_in_post] - injection_time_s
        fp.score_drop = fp.pre_fault_score - fp.worst_score

        # ── Recovery metrics ─────────────────────────────────────
        # Recovery = first time health crosses recovery_threshold AFTER worst
        after_worst = post_scores[worst_idx_in_post:]
        after_worst_t = post_times[worst_idx_in_post:]
        recovered = False
        for j, (t, s) in enumerate(zip(after_worst_t, after_worst)):
            if s >= self.recovery_threshold and j > 0:
                fp.recovery_time_s = t - after_worst_t[0]
                fp.recovery_score = s
                recovered = True
                break

        if not recovered:
            fp.recovery_time_s = None
            fp.recovery_score = post_scores[-1]

        # Recovery rate = slope from worst to recovery point
        if fp.recovery_time_s and fp.recovery_time_s > 0:
            fp.recovery_rate = (
                (fp.recovery_score - fp.worst_score) / fp.recovery_time_s
            )

        # ── Steady state ─────────────────────────────────────────
        tail_n = max(1, len(post_scores) // 5)
        fp.steady_state_score = _mean(post_scores[-tail_n:])

        # ── Spatial / attitude ───────────────────────────────────
        # Use position_drift from timeseries if available (more accurate,
        # computed from live home position by HealthMonitor)
        drifts = timeseries.get("position_drift", [])
        if drifts and len(drifts) == len(times):
            post_drifts = [drifts[i] for i in post_idx]
            if post_drifts:
                fp.max_position_drift_m = max(post_drifts)
        else:
            # Fallback: recompute from lat/lon
            lats = timeseries.get("lat", [])
            lons = timeseries.get("lon", [])
            if lats and lons and len(lats) == len(times):
                home_lat = lats[0] if lats[0] != 0 else None
                home_lon = lons[0] if lons[0] != 0 else None
                if home_lat is not None:
                    max_drift = 0.0
                    for i in post_idx:
                        d = _haversine(home_lat, home_lon, lats[i], lons[i])
                        max_drift = max(max_drift, d)
                    fp.max_position_drift_m = max_drift

        # Attitude deviation
        rolls  = timeseries.get("roll", [])
        pitches = timeseries.get("pitch", [])
        if rolls and pitches and len(rolls) == len(times):
            max_att = 0.0
            for i in post_idx:
                att = abs(rolls[i]) + abs(pitches[i])
                max_att = max(max_att, att)
            fp.max_attitude_deviation = max_att

        # ── Composite resilience index ───────────────────────────
        fp.resilience_index = self._compute_ri(fp)
        return fp

    @staticmethod
    def _compute_ri(fp: RecoveryFingerprint) -> float:
        """Compute 0‑100 resilience index from fingerprint components."""
        # Normalise each component to 0‑1 (1 = best)
        rec_t = fp.recovery_time_s if fp.recovery_time_s is not None else 120.0
        n_recovery = max(0.0, 1.0 - rec_t / _REFS["recovery_time_s"])
        n_worst    = max(0.0, fp.worst_score)      # already 0‑1
        n_drift    = max(0.0, 1.0 - fp.max_position_drift_m
                         / _REFS["max_position_drift_m"])
        n_att      = max(0.0, 1.0 - fp.max_attitude_deviation
                         / _REFS["max_attitude_deviation"])
        n_steady   = max(0.0, fp.steady_state_score)

        ri = (
            _WEIGHTS["recovery_time"] * n_recovery
            + _WEIGHTS["worst_score"]  * n_worst
            + _WEIGHTS["drift"]        * n_drift
            + _WEIGHTS["attitude"]     * n_att
            + _WEIGHTS["steady_state"] * n_steady
        )
        return round(min(100.0, max(0.0, ri * 100.0)), 1)

    # ── Multi‑scenario comparison ────────────────────────────────

    @staticmethod
    def compare(fingerprints: List[RecoveryFingerprint]) -> dict:
        """Return a dict suitable for the radar chart.

        Keys are dimension names; values are lists (one per scenario)
        normalised 0‑1 (1 = best).
        """
        dims = {
            "Health Floor":     [],
            "Recovery Speed":   [],
            "Position Hold":    [],
            "Attitude Stability": [],
            "Steady State":     [],
            "Mission Completion": [],
        }
        for fp in fingerprints:
            dims["Health Floor"].append(max(0, fp.worst_score))
            rec_t = fp.recovery_time_s if fp.recovery_time_s is not None else 120
            dims["Recovery Speed"].append(
                max(0, 1 - rec_t / _REFS["recovery_time_s"]))
            dims["Position Hold"].append(
                max(0, 1 - fp.max_position_drift_m
                    / _REFS["max_position_drift_m"]))
            dims["Attitude Stability"].append(
                max(0, 1 - fp.max_attitude_deviation
                    / _REFS["max_attitude_deviation"]))
            dims["Steady State"].append(max(0, fp.steady_state_score))
            dims["Mission Completion"].append(fp.mission_completion_pct / 100.0)
        return dims


# ── Utility ──────────────────────────────────────────────────────

def _mean(vals: list) -> float:
    if not vals:
        return 0.0
    return sum(vals) / len(vals)

def _haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
