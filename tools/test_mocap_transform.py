#!/usr/bin/env python3
"""Frame invariants, bad input and local timestamp gate checks."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_catch.mocap_transform import StampGate, Transform, load_config

ROOT = Path(__file__).resolve().parents[1]


class TransformTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.t = load_config(ROOT / "config/mocap_a.preview.yaml")

    def test_frame_basis_and_handedness(self):
        base, _ = self.t.convert([0, 0, 0], [0, 0, 0, 1])
        for raw, expected in (([1, 0, 0], [0, 1, 0]),
                              ([0, 1, 0], [1, 0, 0]),
                              ([0, 0, 1], [0, 0, -1])):
            p, _ = self.t.convert(raw, [0, 0, 0, 1])
            np.testing.assert_allclose(p - base, expected)
        self.assertAlmostEqual(np.linalg.det(self.t.world_to_local), 1.)
        self.assertAlmostEqual(np.linalg.det(self.t.body_to_marker), 1.)

    def test_aircraft_level_forward_at_raw_plus_y(self):
        _, q = self.t.convert([0, 0, 0], Rotation.from_euler("z", 180, degrees=True).as_quat())
        # Entire FRD aircraft basis must be aligned with the chosen lab frame.
        np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(), np.eye(3), atol=1e-12)

    def test_left_turn_is_negative_yaw_in_down_frame(self):
        _, q = self.t.convert([0, 0, 0], Rotation.from_euler("z", 270, degrees=True).as_quat())
        np.testing.assert_allclose(Rotation.from_quat(q).apply([1, 0, 0]), [0, -1, 0], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(q).apply([0, 0, 1]), [0, 0, 1], atol=1e-12)

    def test_marker_sign_flip_is_same_orientation(self):
        q = Rotation.from_euler("xyz", [20, -10, 67], degrees=True).as_quat()
        p1, q1 = self.t.convert([1, 2, 3], q)
        p2, q2 = self.t.convert([1, 2, 3], -q)
        np.testing.assert_allclose(p1, p2)
        np.testing.assert_allclose(Rotation.from_quat(q1).as_matrix(), Rotation.from_quat(q2).as_matrix())

    def test_lever_arm_rotates_with_marker(self):
        t = Transform(np.eye(3), np.eye(3), [1, 2, 3], [1, 0, 0])
        p, _ = t.convert([1, 2, 3], Rotation.from_euler("z", 90, degrees=True).as_quat())
        np.testing.assert_allclose(p, [0, 1, 0], atol=1e-12)

    def test_reject_reflection_and_nonrotation(self):
        for matrix in (np.diag([1, 1, -1]), np.eye(3) * 2):
            with self.assertRaises(ValueError):
                Transform(matrix, np.eye(3), [0, 0, 0], [0, 0, 0])

    def test_bad_pose_is_rejected(self):
        for position, q in (([math.nan, 0, 0], [0, 0, 0, 1]),
                            ([0, 0, 0], [0, 0, 0, 0]),
                            ([0, 0, 0], [0, math.inf, 0, 1]),
                            ([0, 0], [0, 0, 0, 1])):
            with self.assertRaises(ValueError):
                self.t.convert(position, q)

    def test_source_origin_not_rebased(self):
        p, _ = self.t.convert([1, 2, 3], [0, 0, 0, 1])
        np.testing.assert_allclose(p, [2, 1, -3])
        self.assertFalse(self.cfg["mounting_confirmed"])
        self.assertFalse(self.cfg["lever_arm_measured"])
        self.assertEqual(self.cfg["mode"], "preview")


class StampTests(unittest.TestCase):
    def test_duplicate_reverse_stale_future_and_recovery(self):
        gate = StampGate(.15)
        gate.check(1_000_000_000, 1_010_000_000)
        for stamp, now in ((1_000_000_000, 1_020_000_000),
                           (999_000_000, 1_030_000_000),
                           (1_001_000_000, 1_200_000_000),
                           (1_250_000_000, 1_210_000_000)):
            with self.assertRaises(ValueError):
                gate.check(stamp, now)
        gate.check(1_220_000_000, 1_230_000_000)

    def test_local_clock_backwards_latches(self):
        gate = StampGate(.15)
        gate.check(1_000_000_000, 1_010_000_000)
        with self.assertRaises(ValueError):
            gate.check(990_000_000, 1_000_000_000)
        with self.assertRaises(ValueError):
            gate.check(1_020_000_000, 1_030_000_000)


if __name__ == "__main__":
    unittest.main()
