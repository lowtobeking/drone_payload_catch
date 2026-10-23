#!/usr/bin/env python3
"""Coordinate/time/fail-closed contracts for external-vision commissioning."""
import copy
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.mocap_odometry import GroundGate, OdometryBuilder, live_issues, load_config


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.transform = load_config(ROOT / "config/mocap_a.bridge.yaml")
        self.cfg["server_timing_verified"] = True
        self.cfg["source_to_local_offset_us"] = -43_200_000_000
        self.cfg["source_offset_uncertainty_us"] = 1000
        self.wall, self.mono = 1_800_000_000_000_000, 1_000_000
        self.builder = OdometryBuilder(self.cfg, self.transform)

    def sample(self, dt=0, **changes):
        data = dict(schema=1, tracker="planeA", sensor=0, frame="world",
                    server_stamp_us=self.wall + dt + 43_200_000_000 - 5000,
                    receive_wall_us=self.wall + dt - 1000,
                    receive_monotonic_us=self.mono + dt - 1000,
                    position=[1., 2., 3.], quaternion_xyzw=[0., 0., 1., 0.])
        data.update(changes)
        return data

    def accept(self, dt=0, **changes):
        return self.builder.accept(self.sample(dt, **changes), self.wall + dt, self.mono + dt)

    def test_frame_quaternion_order_and_unknown_velocity(self):
        out = self.accept()
        self.assertEqual(out["pose_frame"], 2)
        self.assertEqual(out["position"], [2., 1., -3.])
        np.testing.assert_allclose(Rotation.from_quat(
            [*out["q"][1:], out["q"][0]]).as_matrix(), np.eye(3), atol=1e-12)
        self.assertAlmostEqual(abs(out["q"][0]), 1.)
        self.assertEqual(out["velocity_frame"], 0)
        for name in ("velocity", "angular_velocity", "velocity_variance"):
            self.assertTrue(all(math.isnan(x) for x in out[name]))
        self.assertEqual(out["quality"], 0)
        self.assertEqual(out["timestamp"] - out["timestamp_sample"], 5000)

    def test_no_double_application_of_dds_offset(self):
        # A DDS estimated_offset is ~minus Unix epoch; application must not add it.
        out = self.accept()
        self.assertEqual(out["timestamp"], self.wall)
        self.assertEqual(out["timestamp_sample"], self.wall - 5000)

    def test_default_config_cannot_send(self):
        cfg, _ = load_config(ROOT / "config/mocap_a.bridge.yaml")
        self.assertGreaterEqual(len(live_issues(cfg)), 6)
        self.assertIsNone(cfg["source_to_local_offset_us"])

    def test_verified_explicit_config_has_no_configuration_issues(self):
        for key in ("enabled", "mounting_confirmed", "lever_arm_measured", "tracking_loss_verified"):
            self.cfg[key] = True
        self.assertEqual(live_issues(self.cfg), [])

    def test_diagnostic_receive_stamp_is_not_marked_verified(self):
        cfg, transform = load_config(ROOT / "config/mocap_a.bridge.yaml")
        builder = OdometryBuilder(cfg, transform)
        out = builder.accept(self.sample(), self.wall, self.mono)
        self.assertFalse(builder.sample_time_verified)
        self.assertEqual(out["timestamp_sample"], self.wall - 1000)
        self.assertTrue(live_issues(cfg))

    def test_twelve_hour_clock_discrepancy_rejected_without_valid_mapping(self):
        self.cfg["source_to_local_offset_us"] = 0
        builder = OdometryBuilder(self.cfg, self.transform)
        with self.assertRaisesRegex(ValueError, "future"):
            builder.accept(self.sample(), self.wall, self.mono)

    def test_stale_source_even_if_received_now(self):
        with self.assertRaisesRegex(ValueError, "stale"):
            self.accept(server_stamp_us=self.wall + 43_200_000_000 - 200000)

    def test_offset_uncertainty_counts_toward_age(self):
        with self.assertRaisesRegex(ValueError, "stale"):
            self.accept(server_stamp_us=self.wall + 43_200_000_000 - 149500)

    def test_native_queue_age_rejected(self):
        with self.assertRaisesRegex(ValueError, "queued"):
            self.accept(receive_monotonic_us=self.mono - 60000,
                        receive_wall_us=self.wall - 60000)

    def test_invalid_sample_does_not_extend_source_lifetime(self):
        self.accept()
        with self.assertRaises(ValueError):
            self.accept(100000, position=[math.nan, 0, 0])
        with self.assertRaisesRegex(ValueError, "dropout"):
            self.builder.tick(self.wall + 151000, self.mono + 151000)

    def test_duplicate_does_not_refresh_lifetime(self):
        self.accept()
        stamp = self.sample()["server_stamp_us"]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.accept(100000, server_stamp_us=stamp)
        with self.assertRaisesRegex(ValueError, "dropout"):
            self.builder.tick(self.wall + 151000, self.mono + 151000)

    def test_source_reversal_latches(self):
        self.accept()
        with self.assertRaisesRegex(ValueError, "regressed"):
            self.accept(10000, server_stamp_us=self.sample()["server_stamp_us"] - 1)
        with self.assertRaisesRegex(ValueError, "regressed"):
            self.accept(20000)

    def test_dropout_does_not_resume_on_new_data(self):
        self.accept()
        with self.assertRaisesRegex(ValueError, "dropout"):
            self.accept(200000)
        with self.assertRaisesRegex(ValueError, "restart"):
            self.accept(210000)

    def test_source_clock_forward_gap_latches(self):
        self.accept()
        with self.assertRaisesRegex(ValueError, "timestamp gap"):
            self.accept(20000, server_stamp_us=self.sample()["server_stamp_us"] + 200000)

    def test_local_clock_step_latches(self):
        self.accept()
        with self.assertRaisesRegex(ValueError, "clock stepped"):
            self.builder.tick(self.wall + 1000000, self.mono + 10000)
        with self.assertRaisesRegex(ValueError, "clock stepped"):
            self.builder.tick(self.wall + 1010000, self.mono + 20000)

    def test_wrong_source_and_bad_quaternion_rejected(self):
        for changes in ({"tracker": "planeB"}, {"sensor": 1}, {"frame": "other"},
                        {"server_stamp_us": True}, {"quaternion_xyzw": [0, 0, 0, 0]}):
            builder = OdometryBuilder(self.cfg, self.transform)
            with self.assertRaises((ValueError, TypeError)):
                builder.accept(self.sample(**changes), self.wall, self.mono)

    def test_rate_limited_samples_still_checked_for_failure(self):
        self.accept()
        self.assertIsNone(self.accept(8000))
        with self.assertRaises(ValueError):
            self.accept(16000, position=[math.nan, 0, 0])
        self.assertIsNotNone(self.accept(24000))

    def test_config_does_not_accept_typo_boolean_or_other_output(self):
        original, _ = load_config(ROOT / "config/mocap_a.bridge.yaml")
        for key, val in (("enabled", "false"), ("expected_system_id", 2),
                         ("max_sample_age_s", True), ("reset_counter", 256),
                         ("scope", "flight"), ("output_topic", "/fmu/in/vehicle_command")):
            cfg = copy.deepcopy(original)
            cfg[key] = val
            with tempfile.TemporaryDirectory(dir=ROOT / "report/mocap_fusion_20261023") as tmp:
                path = Path(tmp) / "bad.yaml"
                path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_config(path)


class GroundTests(unittest.TestCase):
    def setUp(self):
        self.cfg, _ = load_config(ROOT / "config/mocap_a.bridge.yaml")
        self.gate = GroundGate(self.cfg)
        self.wall, self.mono = 1_800_000_000_000_000, 1_000_000

    def ready(self):
        g = self.gate
        g.record_status(SimpleNamespace(system_id=1, component_id=1, arming_state=1), self.mono)
        g.record_land(SimpleNamespace(landed=True), self.mono)
        for i in range(3):
            g.record_sync(SimpleNamespace(source_protocol=2, remote_timestamp=self.wall - 300 + i,
                round_trip_time=500, estimated_offset=-1_700_000_000_000_000), self.wall, self.mono)

    def test_no_output_without_all_ground_and_sync_evidence(self):
        self.assertTrue(self.gate.check(self.mono))
        self.ready()
        self.assertEqual(self.gate.check(self.mono), [])

    def test_arming_or_stale_evidence_latches(self):
        for scenario in ("arm", "stale", "air"):
            self.gate = GroundGate(self.cfg)
            self.ready()
            self.assertEqual(self.gate.check(self.mono), [])
            if scenario == "arm":
                self.gate.record_status(SimpleNamespace(system_id=1, component_id=1, arming_state=2),
                                        self.mono)
                t = self.mono
            elif scenario == "air":
                self.gate.record_land(SimpleNamespace(landed=False), self.mono)
                t = self.mono
            else:
                t = self.mono + 3_000_000
            self.assertTrue(self.gate.check(t))
            self.ready()
            self.assertTrue(self.gate.check(self.mono))
            self.assertTrue(self.gate.fault)

    def test_wrong_identity_no_ready(self):
        self.ready()
        self.gate.record_status(SimpleNamespace(system_id=2, component_id=1, arming_state=1),
                                self.mono)
        self.assertTrue(self.gate.check(self.mono))

    def test_replayed_sync_does_not_refresh_health(self):
        self.ready()
        self.assertFalse(self.gate.check(self.mono))
        previous = self.gate.sync
        old = self.gate.last_sync_remote
        for _ in range(3):
            self.gate.record_sync(SimpleNamespace(source_protocol=2, remote_timestamp=old,
                round_trip_time=500, estimated_offset=-1_700_000_000_000_000),
                self.wall + 1_000_000, self.mono + 1_000_000)
        self.assertEqual(self.gate.sync, previous)
        self.assertTrue(self.gate.check(self.mono + 3_000_000))

    def test_bad_sync_rtt_or_clock_jump_cannot_pass(self):
        for kwargs in ({"source_protocol": 1}, {"round_trip_time": 30000},
                       {"remote_timestamp": self.wall + 100},
                       {"estimated_offset": -1000}):
            self.gate = GroundGate(self.cfg)
            self.ready()
            self.assertFalse(self.gate.check(self.mono))
            vals = dict(source_protocol=2, remote_timestamp=self.wall + 10, round_trip_time=500,
                        estimated_offset=-1_700_000_000_000_000)
            vals.update(kwargs)
            self.gate.record_sync(SimpleNamespace(**vals), self.wall + 20, self.mono + 20)
            self.assertTrue(self.gate.check(self.mono))


if __name__ == "__main__":
    unittest.main()
