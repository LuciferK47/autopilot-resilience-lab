import tempfile
import textwrap
import unittest

from scenarios.scenario_runner import ScenarioResult, load_scenarios
from scenarios.scenario_validation import (
    ScenarioValidationError,
    normalize_and_validate_scenarios,
)


class TestScenarioValidation(unittest.TestCase):
    def test_defaults_are_applied(self):
        data = {
            "defaults": {
                "vehicle": "ArduCopter",
                "altitude": 20,
                "speedup": 5,
                "timeout_s": 180,
                "waypoints": [{"lat": 1.0, "lon": 2.0, "alt": 10.0}],
            },
            "scenarios": [
                {
                    "id": "s1",
                    "name": "Scenario 1",
                    "pass_criteria": {},
                    "faults": [],
                    "failsafe_params": {},
                }
            ],
        }
        scenarios = normalize_and_validate_scenarios(data)
        self.assertEqual(len(scenarios), 1)
        self.assertEqual(scenarios[0]["vehicle"], "ArduCopter")
        self.assertEqual(scenarios[0]["altitude"], 20)
        self.assertEqual(scenarios[0]["waypoints"][0]["lat"], 1.0)

    def test_duplicate_id_rejected(self):
        data = {
            "defaults": {
                "altitude": 20,
                "speedup": 5,
                "timeout_s": 180,
                "waypoints": [{"lat": 1.0, "lon": 2.0, "alt": 10.0}],
            },
            "scenarios": [
                {"id": "dup", "name": "A"},
                {"id": "dup", "name": "B"},
            ],
        }
        with self.assertRaises(ScenarioValidationError):
            normalize_and_validate_scenarios(data)

    def test_invalid_fault_rejected(self):
        data = {
            "defaults": {
                "altitude": 20,
                "speedup": 5,
                "timeout_s": 180,
                "waypoints": [{"lat": 1.0, "lon": 2.0, "alt": 10.0}],
            },
            "scenarios": [
                {
                    "id": "bad_fault",
                    "name": "Bad Fault",
                    "faults": [{"trigger": {"type": "time", "after_s": -1}, "params": {"X": 1}}],
                }
            ],
        }
        with self.assertRaises(ScenarioValidationError):
            normalize_and_validate_scenarios(data)

    def test_load_scenarios_from_yaml_file(self):
        content = textwrap.dedent(
            """
            defaults:
              altitude: 20
              speedup: 5
              timeout_s: 180
              waypoints:
                - {lat: 1.0, lon: 2.0, alt: 10.0}
            scenarios:
              - id: demo
                name: Demo
                faults: []
                failsafe_params: {}
                pass_criteria: {}
            """
        )
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(content)
            path = f.name

        loaded = load_scenarios(path)
        self.assertEqual(len(loaded), 1)
        self.assertEqual(loaded[0]["id"], "demo")


class TestScenarioEvaluation(unittest.TestCase):
    def test_max_altitude_at_end_is_enforced(self):
        result = ScenarioResult(
            scenario={"id": "motor_failure", "name": "Motor Failure"},
            monitor_summary={},
            monitor_timeseries={"relative_alt": [20.0, 5.2]},
        )
        result.evaluate({"max_altitude_at_end_m": 2.0})
        self.assertFalse(result.passed)
        self.assertTrue(any("End altitude" in r for r in result.fail_reasons))


if __name__ == "__main__":
    unittest.main()
