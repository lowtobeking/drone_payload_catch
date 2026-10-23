"""Contract tests for explicitly unverified receive-time ground output."""
import copy
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.mocap_odometry import OdometryBuilder, live_issues, load_config

class ReceiveGroundTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.transform = load_config(ROOT / "config/mocap_a.receive_ground.yaml")
        self.b = OdometryBuilder(self.cfg, self.transform)
        self.wall, self.mono = 1_800_000_000_000_000, 1_000_000
    def sample(self, dt=0, **changes):
        d = dict(schema=1, tracker="planeA", sensor=0, frame="world",
                 server_stamp_us=self.wall + 43_200_000_000 + dt,
                 receive_wall_us=self.wall + dt - 1000,
                 receive_monotonic_us=self.mono + dt - 1000,
                 position=[1., 2., 3. + dt * 1e-10], quaternion_xyzw=[0., 0., 1., 0.])
        d.update(changes)
        return d
    def accept(self, dt=0, **changes):
        return self.b.accept(self.sample(dt, **changes), self.wall+dt, self.mono+dt)
    def test_explicit_opt_in_and_unchanged_calibration_truth(self):
        self.assertTrue(live_issues(self.cfg))
        self.assertEqual(live_issues(self.cfg, ground_receive_time=True), [])
        for key in ("mounting_confirmed", "lever_arm_measured", "server_timing_verified", "tracking_loss_verified"):
            self.assertIs(self.cfg[key], False)
        self.cfg["ground_receive_time_acknowledged"] = False
        self.assertTrue(live_issues(self.cfg, ground_receive_time=True))
    def test_native_timestamp_not_ros_callback_timestamp(self):
        out = self.accept()
        self.assertFalse(self.b.sample_time_verified)
        self.assertEqual(out["timestamp_sample"], self.wall-1000)
    def test_cannot_override_strict_mode(self):
        cfg,_ = load_config(ROOT / "config/mocap_a.bridge.yaml")
        self.assertTrue(live_issues(cfg, ground_receive_time=True))
    def test_stale_native_queue_refused(self):
        with self.assertRaisesRegex(ValueError,"queued"):
            self.accept(receive_wall_us=self.wall-60000, receive_monotonic_us=self.mono-60000)
    def test_duplicate_cannot_keep_stream_alive(self):
        self.accept()
        with self.assertRaisesRegex(ValueError,"duplicate"):
            self.accept(100000, server_stamp_us=self.sample()["server_stamp_us"])
        with self.assertRaisesRegex(ValueError,"dropout"):
            self.b.tick(self.wall+151000,self.mono+151000)
    def test_backlog_or_relative_drift_latches(self):
        self.accept()
        # Absolute server time is deliberately ignored; its progression is still checked.
        with self.assertRaisesRegex(ValueError,"relative"):
            self.accept(10000,server_stamp_us=self.sample()["server_stamp_us"]+100000)
        with self.assertRaisesRegex(ValueError,"restart"):
            self.accept(20000)
    def test_exact_pose_freeze_with_advancing_stamps_latches(self):
        for dt in range(0,500001,50000):
            self.accept(dt,position=[1.,2.,3.])
        with self.assertRaisesRegex(ValueError,"repeated pose"):
            self.accept(550000,position=[1.,2.,3.])
    def test_fresh_jittering_static_pose_allowed(self):
        for dt in range(0,1000000,10000):
            self.accept(dt)
        self.assertFalse(self.b.fault)
    def test_source_clock_regression_latches(self):
        self.accept()
        with self.assertRaisesRegex(ValueError,"regressed"):
            self.accept(10000, server_stamp_us=self.sample()["server_stamp_us"]-1)

if __name__=="__main__":
    unittest.main()
