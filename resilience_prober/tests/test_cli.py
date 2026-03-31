import types
import unittest
from unittest.mock import patch

import resilience_prober as cli


class TestCliSmokeMode(unittest.TestCase):
    def _args(self, smoke_result=True):
        return types.SimpleNamespace(
            scenario=None,
            all=False,
            list=False,
            smoke=True,
            ardupilot_path="/tmp/ardupilot",
            scenarios_file="/tmp/scenarios.yaml",
            report_dir="/tmp/reports",
            clean_logs=False,
            speedup=None,
            no_report=True,
            _smoke_result=smoke_result,
        )

    @patch("resilience_prober.load_scenarios", return_value=[])
    @patch("resilience_prober.os.path.exists", return_value=True)
    def test_smoke_success_exit_code_zero(self, *_):
        args = self._args(smoke_result=True)
        with patch("resilience_prober.parse_args", return_value=args), \
             patch("resilience_prober.run_smoke", return_value=True):
            with self.assertRaises(SystemExit) as cm:
                cli.main()
        self.assertEqual(cm.exception.code, 0)

    @patch("resilience_prober.load_scenarios", return_value=[])
    @patch("resilience_prober.os.path.exists", return_value=True)
    def test_smoke_failure_exit_code_one(self, *_):
        args = self._args(smoke_result=False)
        with patch("resilience_prober.parse_args", return_value=args), \
             patch("resilience_prober.run_smoke", return_value=False):
            with self.assertRaises(SystemExit) as cm:
                cli.main()
        self.assertEqual(cm.exception.code, 1)


if __name__ == "__main__":
    unittest.main()
