"""A-only flight bridge contract tests; no ROS or aircraft access."""
from pathlib import Path
from types import SimpleNamespace
import unittest

from payload_catch.mocap_odometry import GroundGate, OdometryBuilder, live_issues, load_config

ROOT = Path(__file__).resolve().parents[1]


class FlightBridgeTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.transform = load_config(ROOT / "config/mocap_a.solo_flight.yaml")

    def test_explicit_receive_time_flight_policy(self):
        self.assertEqual(self.cfg["scope"], "a_solo_flight")
        self.assertEqual(self.cfg["transform"]["body_origin_in_marker_m"], [0.0, 0.0, -0.16])
        self.assertFalse(self.cfg["mounting_confirmed"])
        self.assertFalse(self.cfg["lever_arm_measured"])
        self.assertFalse(self.cfg["server_timing_verified"])
        self.assertFalse(self.cfg["tracking_loss_verified"])
        self.assertEqual(live_issues(self.cfg, flight_receive_time=True), [])
        self.assertTrue(live_issues(self.cfg))
        self.assertTrue(live_issues(self.cfg, ground_receive_time=True))
        self.cfg["enabled"] = False
        self.assertIn("enabled must explicitly be true", live_issues(self.cfg, flight_receive_time=True))

    def test_receive_time_flight_acknowledgement_required(self):
        self.cfg["flight_receive_time_acknowledged"] = False
        self.assertTrue(live_issues(self.cfg, flight_receive_time=True))

    def test_arm_transition_preserves_fresh_bridge(self):
        gate = GroundGate(self.cfg)
        wall, mono = 1_800_000_000_000_000, 1_000_000
        status = SimpleNamespace(system_id=1, component_id=1, arming_state=1)
        gate.record_status(status, mono)
        gate.record_land(SimpleNamespace(landed=True), mono)
        for i in range(3):
            gate.record_sync(SimpleNamespace(source_protocol=2,
                remote_timestamp=wall + i * 1000 - 100,
                round_trip_time=1000, estimated_offset=1000),
                wall + i * 1000, mono + i * 1000)
        self.assertEqual(gate.check(mono + 2000), [])
        status.arming_state = 2
        gate.record_status(status, mono + 3000)
        gate.record_land(SimpleNamespace(landed=False), mono + 3000)
        self.assertEqual(gate.check(mono + 3000), [])
        self.assertIn("A vehicle status missing/stale", gate.check(mono + 1_600_000))

    def test_cannot_start_when_already_armed(self):
        gate = GroundGate(self.cfg)
        status = SimpleNamespace(system_id=1, component_id=1, arming_state=2)
        gate.record_status(status, 1_000_000)
        gate.record_land(SimpleNamespace(landed=False), 1_000_000)
        self.assertIn("flight bridge must start disarmed", gate.check(1_000_000))

    def test_receive_stamp_used_despite_source_jump(self):
        builder = OdometryBuilder(self.cfg, self.transform)
        wall, mono = 1_800_000_000_000_000, 1_000_000
        for dt, source in ((0, wall + 43_200_000_000),
                           (30_000, wall - 43_200_000_000)):
            data = dict(schema=1, tracker="planeA", sensor=0, frame="world",
                server_stamp_us=source, receive_wall_us=wall + dt - 1000,
                receive_monotonic_us=mono + dt - 1000,
                position=[1.0 + dt / 1e6, 2.0, 3.0],
                quaternion_xyzw=[0.0, 0.0, 1.0, 0.0])
            out = builder.accept(data, wall + dt, mono + dt)
        self.assertFalse(builder.fault)
        self.assertFalse(builder.sample_time_verified)
        self.assertEqual(out["timestamp_sample"], wall + 29000)


if __name__ == "__main__":
    unittest.main()
