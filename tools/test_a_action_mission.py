import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.a_action_geometry import load_action_geometry
from payload_catch.a_action_mission import AActionMission


class AActionMissionTests(unittest.TestCase):
    def setUp(self):
        self.geometry = load_action_geometry(ROOT/'config/a_action_solo.yaml')
        self.origin = self.geometry.expected_px4_start_ned_m
        self.mission = AActionMission(self.geometry)
        self.mission.start(self.origin)

    def test_mocap_x_negative_maps_to_px4_east_negative(self):
        self.assertAlmostEqual(self.origin[1], 2.576)
        self.assertAlmostEqual(self.geometry.target_ned_m(self.origin)[1], .576)

    def test_takeoff_translate_settle_then_one_open(self):
        zero = (0., 0., 0.)
        command, action = self.mission.tick(0., self.origin, zero, flight_healthy=True)
        self.assertEqual(self.mission.phase, 'TAKEOFF')
        self.assertEqual(command[2], -.3)
        takeoff = (self.origin[0], self.origin[1], self.origin[2]-1.)
        self.mission.tick(1., takeoff, zero, flight_healthy=True)
        self.assertEqual(self.mission.phase, 'TRANSLATE')
        end = self.geometry.target_ned_m(self.origin)
        self.mission.tick(2., end, zero, flight_healthy=True)
        self.assertEqual(self.mission.phase, 'SETTLE')
        self.assertEqual(self.mission.tick(3.9, end, zero, flight_healthy=True)[1], None)
        self.assertEqual(self.mission.tick(4., end, zero, flight_healthy=True)[1], 'open')
        self.assertEqual(self.mission.tick(4.1, end, zero, flight_healthy=True)[1], None)
        self.assertEqual(self.mission.action_complete(True), 'land')

    def test_corridor_or_health_loss_requests_land_without_pwm(self):
        command, action = self.mission.tick(0., (self.origin[0]+.5, self.origin[1], self.origin[2]),
                                            (0., 0., 0.), flight_healthy=True)
        self.assertIsNone(command)
        self.assertEqual(action, 'land')
        self.assertFalse(self.mission.action_sent)

    def test_takeoff_deadline_requests_land(self):
        self.mission.tick(0., self.origin, (0., 0., 0.), flight_healthy=True)
        command, action = self.mission.tick(25.1, self.origin, (0., 0., 0.), flight_healthy=True)
        self.assertIsNone(command)
        self.assertEqual(action, 'land')
        self.assertFalse(self.mission.action_sent)

    def test_pwm_position_loss_requests_land(self):
        zero = (0., 0., 0.)
        takeoff = (self.origin[0], self.origin[1], self.origin[2]-1.)
        end = self.geometry.target_ned_m(self.origin)
        self.mission.tick(0., self.origin, zero, flight_healthy=True)
        self.mission.tick(1., takeoff, zero, flight_healthy=True)
        self.mission.tick(2., end, zero, flight_healthy=True)
        self.assertEqual(self.mission.tick(4., end, zero, flight_healthy=True)[1], 'open')
        self.assertEqual(self.mission.tick(4.1, end, (0.3, 0., 0.), flight_healthy=True)[1], 'land')


if __name__ == '__main__':
    unittest.main()
