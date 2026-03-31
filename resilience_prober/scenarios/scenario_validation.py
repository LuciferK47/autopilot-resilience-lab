"""Scenario config validation and normalization utilities."""

from __future__ import annotations

import copy
from typing import Any, Dict, List


_ALLOWED_PASS_CRITERIA = {
    "min_health_score",
    "max_position_drift_m",
    "required_failsafe",
    "must_detect_failsafe",
    "must_complete_mission",
    "min_mission_completion_pct",
    "max_recovery_time_s",
    "max_altitude_at_end_m",
}


class ScenarioValidationError(ValueError):
    """Raised when scenarios.yaml has invalid structure or values."""


def _as_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _validate_waypoint(wp: Dict[str, Any], sid: str, idx: int):
    for key in ("lat", "lon", "alt"):
        if key not in wp:
            raise ScenarioValidationError(
                f"Scenario '{sid}' waypoint[{idx}] missing '{key}'"
            )
        if not _as_number(wp[key]):
            raise ScenarioValidationError(
                f"Scenario '{sid}' waypoint[{idx}].{key} must be numeric"
            )


def _validate_fault(fault: Dict[str, Any], sid: str, idx: int):
    trigger = fault.get("trigger", {})
    if not isinstance(trigger, dict):
        raise ScenarioValidationError(
            f"Scenario '{sid}' fault[{idx}] trigger must be a mapping"
        )

    trigger_type = trigger.get("type", "time")
    if trigger_type != "time":
        raise ScenarioValidationError(
            f"Scenario '{sid}' fault[{idx}] trigger.type must be 'time'"
        )

    after_s = trigger.get("after_s", 30)
    if not _as_number(after_s) or after_s < 0:
        raise ScenarioValidationError(
            f"Scenario '{sid}' fault[{idx}] trigger.after_s must be >= 0"
        )

    params = fault.get("params", {})
    if not isinstance(params, dict) or not params:
        raise ScenarioValidationError(
            f"Scenario '{sid}' fault[{idx}] params must be a non-empty mapping"
        )


def _validate_criteria(criteria: Dict[str, Any], sid: str):
    unknown = sorted(set(criteria) - _ALLOWED_PASS_CRITERIA)
    if unknown:
        raise ScenarioValidationError(
            f"Scenario '{sid}' has unknown pass_criteria key(s): {', '.join(unknown)}"
        )

    req = criteria.get("required_failsafe")
    if req is not None:
        if not isinstance(req, list) or not req or not all(isinstance(x, str) for x in req):
            raise ScenarioValidationError(
                f"Scenario '{sid}' required_failsafe must be a non-empty string list"
            )


def normalize_and_validate_scenarios(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Apply defaults and validate scenario definitions."""
    if not isinstance(data, dict):
        raise ScenarioValidationError("Top-level YAML must be a mapping")

    defaults = data.get("defaults", {})
    if defaults is None:
        defaults = {}
    if not isinstance(defaults, dict):
        raise ScenarioValidationError("'defaults' must be a mapping")

    raw = data.get("scenarios", [])
    if not isinstance(raw, list):
        raise ScenarioValidationError("'scenarios' must be a list")
    if not raw:
        raise ScenarioValidationError("No scenarios defined")

    normalized: List[Dict[str, Any]] = []
    seen_ids = set()

    for idx, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ScenarioValidationError(f"Scenario index {idx} must be a mapping")

        merged = copy.deepcopy(defaults)
        merged.update(item)

        sid = merged.get("id")
        name = merged.get("name")
        if not isinstance(sid, str) or not sid.strip():
            raise ScenarioValidationError(f"Scenario index {idx} missing valid 'id'")
        if not isinstance(name, str) or not name.strip():
            raise ScenarioValidationError(f"Scenario '{sid}' missing valid 'name'")
        if sid in seen_ids:
            raise ScenarioValidationError(f"Duplicate scenario id: '{sid}'")
        seen_ids.add(sid)

        for key in ("altitude", "speedup", "timeout_s"):
            val = merged.get(key)
            if not _as_number(val) or val <= 0:
                raise ScenarioValidationError(
                    f"Scenario '{sid}' field '{key}' must be > 0"
                )

        waypoints = merged.get("waypoints", [])
        if not isinstance(waypoints, list) or not waypoints:
            raise ScenarioValidationError(
                f"Scenario '{sid}' must define at least one waypoint"
            )
        for wp_idx, wp in enumerate(waypoints):
            if not isinstance(wp, dict):
                raise ScenarioValidationError(
                    f"Scenario '{sid}' waypoint[{wp_idx}] must be a mapping"
                )
            _validate_waypoint(wp, sid, wp_idx)

        failsafe_params = merged.get("failsafe_params", {})
        if failsafe_params is None:
            failsafe_params = {}
        if not isinstance(failsafe_params, dict):
            raise ScenarioValidationError(
                f"Scenario '{sid}' failsafe_params must be a mapping"
            )

        faults = merged.get("faults", [])
        if faults is None:
            faults = []
        if not isinstance(faults, list):
            raise ScenarioValidationError(f"Scenario '{sid}' faults must be a list")
        for fault_idx, fault in enumerate(faults):
            if not isinstance(fault, dict):
                raise ScenarioValidationError(
                    f"Scenario '{sid}' fault[{fault_idx}] must be a mapping"
                )
            _validate_fault(fault, sid, fault_idx)

        pass_criteria = merged.get("pass_criteria", {})
        if pass_criteria is None:
            pass_criteria = {}
        if not isinstance(pass_criteria, dict):
            raise ScenarioValidationError(
                f"Scenario '{sid}' pass_criteria must be a mapping"
            )
        _validate_criteria(pass_criteria, sid)

        merged["failsafe_params"] = failsafe_params
        merged["faults"] = faults
        merged["pass_criteria"] = pass_criteria

        normalized.append(merged)

    return normalized
