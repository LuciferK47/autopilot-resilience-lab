"""
Unit tests for core.health_monitor + new modules
===================================
Run with:  python -m pytest tests/ -v
"""

import math
import time
import unittest
from unittest.mock import MagicMock
from core.health_monitor import HealthMonitor, TelemetrySnapshot
from core.recovery_analyzer import RecoveryAnalyzer, RecoveryFingerprint
from core.ekf_forensics import EKFForensics, EKFForensicsResult


class FakeMsg:
    """Minimal stand‑in for a MAVLink message."""
    def __init__(self, msg_type, **kwargs):
        self._type = msg_type
        for k, v in kwargs.items():
            setattr(self, k, v)

    def get_type(self):
        return self._type


class TestHealthScore(unittest.TestCase):
    """Verify the composite health‑score formula."""

    def setUp(self):
        self.mon = HealthMonitor(ekf_threshold=0.8, vibe_threshold=60.0)

    def test_perfect_health(self):
        """All variances at zero → score = 1.0."""
        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.0,
                       pos_horiz_variance=0.0,
                       pos_vert_variance=0.0,
                       compass_variance=0.0,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(msg)
        self.assertAlmostEqual(self.mon.get_current_score(), 1.0, places=3)

    def test_at_threshold(self):
        """Velocity variance = threshold → score = 0.0."""
        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.8,
                       pos_horiz_variance=0.0,
                       pos_vert_variance=0.0,
                       compass_variance=0.0,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(msg)
        self.assertAlmostEqual(self.mon.get_current_score(), 0.0, places=3)

    def test_beyond_threshold(self):
        """Variance > threshold → score clamped to 0."""
        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=1.6,
                       pos_horiz_variance=0.0,
                       pos_vert_variance=0.0,
                       compass_variance=0.0,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(msg)
        self.assertAlmostEqual(self.mon.get_current_score(), 0.0, places=3)

    def test_half_health(self):
        """Velocity variance at half threshold → score ≈ 0.5."""
        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.4,
                       pos_horiz_variance=0.0,
                       pos_vert_variance=0.0,
                       compass_variance=0.0,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(msg)
        self.assertAlmostEqual(self.mon.get_current_score(), 0.5, places=3)

    def test_vibration_dominates(self):
        """High vibration with low EKF variance → vibration drives score."""
        ekf = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.0,
                       pos_horiz_variance=0.0,
                       pos_vert_variance=0.0,
                       compass_variance=0.0,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(ekf)

        vib = FakeMsg("VIBRATION",
                       vibration_x=30.0, vibration_y=30.0, vibration_z=30.0,
                       clipping_0=0, clipping_1=0, clipping_2=0)
        self.mon.process_message(vib)

        mag = math.sqrt(30**2 + 30**2 + 30**2)
        expected = 1.0 - mag / 60.0
        self.assertAlmostEqual(self.mon.get_current_score(), expected,
                               places=2)

    def test_worst_component_wins(self):
        """The worst single component drives the score down."""
        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.1,
                       pos_horiz_variance=0.6,
                       pos_vert_variance=0.1,
                       compass_variance=0.1,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        self.mon.process_message(msg)
        expected = 1.0 - (0.6 / 0.8)
        self.assertAlmostEqual(self.mon.get_current_score(), expected,
                               places=3)


class TestModeDetection(unittest.TestCase):
    """Verify failsafe mode‑change detection."""

    def setUp(self):
        self.mon = HealthMonitor()
        self.mon.start()

    def _heartbeat(self, mode_num, armed=True):
        from pymavlink import mavutil
        base = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        return FakeMsg("HEARTBEAT",
                       custom_mode=mode_num,
                       base_mode=base,
                       type=2)  # quadrotor

    def test_auto_to_land_triggers_failsafe(self):
        """AUTO → LAND should trigger failsafe detection."""
        self.mon.process_message(self._heartbeat(3))   # AUTO
        self.mon.process_message(self._heartbeat(9))   # LAND
        self.assertTrue(self.mon.failsafe_triggered)
        self.assertEqual(self.mon.failsafe_mode, "LAND")

    def test_auto_to_rtl_triggers_failsafe(self):
        self.mon.process_message(self._heartbeat(3))   # AUTO
        self.mon.process_message(self._heartbeat(6))   # RTL
        self.assertTrue(self.mon.failsafe_triggered)
        self.assertEqual(self.mon.failsafe_mode, "RTL")

    def test_guided_to_auto_no_failsafe(self):
        """GUIDED → AUTO should NOT trigger failsafe."""
        self.mon.process_message(self._heartbeat(4))   # GUIDED
        self.mon.process_message(self._heartbeat(3))   # AUTO
        self.assertFalse(self.mon.failsafe_triggered)

    def test_disarm_detection(self):
        """Armed → Disarmed transition tracked."""
        self.mon.process_message(self._heartbeat(3, armed=True))
        self.assertTrue(self.mon._has_been_armed)
        self.mon.process_message(self._heartbeat(3, armed=False))
        self.assertTrue(self.mon.vehicle_has_disarmed_after_flight())


class TestAttitudeProcessing(unittest.TestCase):

    def test_attitude_degrees(self):
        """ATTITUDE message radians are converted to degrees."""
        mon = HealthMonitor()
        msg = FakeMsg("ATTITUDE",
                       roll=math.radians(15),
                       pitch=math.radians(-10),
                       yaw=math.radians(90),
                       rollspeed=0, pitchspeed=0, yawspeed=0)
        mon.process_message(msg)
        self.assertAlmostEqual(mon._cur.roll, 15.0, places=1)
        self.assertAlmostEqual(mon._cur.pitch, -10.0, places=1)
        self.assertAlmostEqual(mon._cur.yaw, 90.0, places=1)


class TestPositionDrift(unittest.TestCase):

    def test_haversine(self):
        """Haversine returns ~111 km for 1 degree latitude."""
        d = HealthMonitor._haversine(0, 0, 1, 0)
        self.assertAlmostEqual(d, 111_195, delta=200)

    def test_zero_distance(self):
        d = HealthMonitor._haversine(-35.36, 149.16, -35.36, 149.16)
        self.assertAlmostEqual(d, 0.0, places=1)


class TestTimeseries(unittest.TestCase):

    def test_recording(self):
        """Starting + processing → timeline grows."""
        mon = HealthMonitor()
        mon._record_interval = 0        # record every call
        mon.start()

        msg = FakeMsg("EKF_STATUS_REPORT",
                       velocity_variance=0.1,
                       pos_horiz_variance=0.1,
                       pos_vert_variance=0.1,
                       compass_variance=0.1,
                       terrain_alt_variance=0.0,
                       flags=0x01)
        mon.process_message(msg)
        time.sleep(0.01)
        mon.process_message(msg)

        mon.stop()
        ts = mon.get_timeseries()
        self.assertGreater(len(ts.get("time", [])), 0)
        self.assertGreater(len(ts.get("health_score", [])), 0)


class TestWaypointTracking(unittest.TestCase):
    """Verify waypoint progress tracking."""

    def setUp(self):
        self.mon = HealthMonitor()
        self.mon.start()
        self.mon.set_total_waypoints(4)

    def test_set_total_waypoints(self):
        self.assertEqual(self.mon._total_waypoints, 4)

    def test_mission_current(self):
        """MISSION_CURRENT updates current waypoint."""
        msg = FakeMsg("MISSION_CURRENT", seq=2)
        self.mon.process_message(msg)
        self.assertEqual(self.mon._cur.current_waypoint, 2)

    def test_mission_item_reached(self):
        """MISSION_ITEM_REACHED adds to waypoints_reached."""
        self.mon._recording = True
        self.mon._start_time = time.time()
        msg = FakeMsg("MISSION_ITEM_REACHED", seq=1)
        self.mon.process_message(msg)
        msg2 = FakeMsg("MISSION_ITEM_REACHED", seq=2)
        self.mon.process_message(msg2)
        self.assertIn(1, self.mon._waypoints_reached)
        self.assertIn(2, self.mon._waypoints_reached)

    def test_completion_pct(self):
        """Mission completion percentage computed correctly."""
        self.mon._recording = True
        self.mon._start_time = time.time()
        # Simulate MISSION_CURRENT advancing — this sets _highest_wp_seq
        self.mon.process_message(FakeMsg("MISSION_CURRENT", seq=1))
        self.mon.process_message(FakeMsg("MISSION_CURRENT", seq=2))
        self.mon.process_message(FakeMsg("MISSION_CURRENT", seq=3))
        self.mon.process_message(FakeMsg("MISSION_ITEM_REACHED", seq=1))
        self.mon.process_message(FakeMsg("MISSION_ITEM_REACHED", seq=2))
        self.mon.process_message(FakeMsg("MISSION_ITEM_REACHED", seq=3))
        # highest_wp_seq = 3, progress = 3-1 = 2, pct = 2/4 * 100 = 50
        pct = self.mon.get_mission_completion_pct()
        self.assertAlmostEqual(pct, 50.0, places=0)

    def test_summary_includes_mission(self):
        """get_summary() contains mission fields."""
        self.mon.stop()
        s = self.mon.get_summary()
        self.assertIn("mission_completion_pct", s)
        self.assertIn("waypoints_reached", s)


class TestRecoveryAnalyzer(unittest.TestCase):
    """Unit tests for the RecoveryAnalyzer."""

    def test_analyze_perfect_flight(self):
        """Constant health=1.0 → RI should be very high."""
        ts = {
            "time": [float(i) for i in range(60)],
            "health_score": [1.0] * 60,
            "roll": [0.0] * 60,
            "pitch": [0.0] * 60,
            "position_drift": [0.0] * 60,
        }
        ra = RecoveryAnalyzer()
        fp = ra.analyze(ts, injection_time_s=10.0,
                        scenario_id="test")
        self.assertGreater(fp.resilience_index, 80)
        self.assertEqual(fp.worst_score, 1.0)
        self.assertAlmostEqual(fp.score_drop, 0.0, places=3)

    def test_analyze_crash(self):
        """Health drops to 0 and stays → RI should be very low."""
        ts = {
            "time": [float(i) for i in range(60)],
            "health_score": [1.0] * 10 + [0.0] * 50,
            "roll": [0.0] * 10 + [45.0] * 50,
            "pitch": [0.0] * 10 + [30.0] * 50,
            "position_drift": [0.0] * 10 + [50.0] * 50,
        }
        ra = RecoveryAnalyzer()
        fp = ra.analyze(ts, injection_time_s=10.0,
                        scenario_id="crash_test")
        self.assertLess(fp.resilience_index, 30)
        self.assertEqual(fp.worst_score, 0.0)
        self.assertIsNone(fp.recovery_time_s)

    def test_analyze_recovery(self):
        """Health drops then recovers → RI moderate, recovery_time_s set."""
        scores = [1.0] * 10 + [0.2] * 5 + [0.8] * 45
        ts = {
            "time": [float(i) for i in range(60)],
            "health_score": scores,
            "roll": [0.0] * 60,
            "pitch": [0.0] * 60,
            "position_drift": [5.0] * 60,
        }
        ra = RecoveryAnalyzer()
        fp = ra.analyze(ts, injection_time_s=10.0,
                        scenario_id="recovery_test")
        self.assertIsNotNone(fp.recovery_time_s)
        self.assertAlmostEqual(fp.worst_score, 0.2, places=3)
        self.assertGreater(fp.resilience_index, 30)

    def test_compare_returns_dict(self):
        """RecoveryAnalyzer.compare() returns correct structure."""
        fp1 = RecoveryFingerprint(scenario_id="a", resilience_index=80,
                                   worst_score=0.5, recovery_time_s=5,
                                   max_position_drift_m=3, steady_state_score=0.9)
        fp2 = RecoveryFingerprint(scenario_id="b", resilience_index=40,
                                   worst_score=0.1, recovery_time_s=20,
                                   max_position_drift_m=30, steady_state_score=0.4)
        dims = RecoveryAnalyzer.compare([fp1, fp2])
        self.assertIn("Health Floor", dims)
        self.assertIn("Recovery Speed", dims)
        self.assertEqual(len(dims["Health Floor"]), 2)


class TestEKFForensicsImport(unittest.TestCase):
    """Verify EKFForensics can be instantiated."""

    def test_result_dataclass(self):
        r = EKFForensicsResult()
        self.assertEqual(r.time_us, [])
        self.assertEqual(r.error_score, [])
        self.assertEqual(r.metrics, {})

    def test_forensics_init(self):
        """EKFForensics can be created without a real log."""
        ef = EKFForensics("/nonexistent.BIN")
        self.assertIsNotNone(ef)


if __name__ == "__main__":
    unittest.main()
