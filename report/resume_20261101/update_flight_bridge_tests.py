"""Align offline tests with the saved explicit receive-time flight policy."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
path=ROOT/"tools/test_mocap_flight_bridge.py"
value=path.read_text(encoding="utf-8")
start=value.index("    def test_disabled_until_confirmations(self):")
end=value.index("    def test_arm_transition_preserves_fresh_bridge(self):",start)
new='''    def test_explicit_receive_time_flight_policy(self):
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

'''
value=value[:start]+new+value[end:]
ast.parse(value)
path.write_text(value,encoding="utf-8",newline="\n")
print("Flight-policy tests aligned; production code and flight parameters unchanged")
