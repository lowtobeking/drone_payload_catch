#!/usr/bin/env python3
"""Fail-closed health tests, no ROS or aircraft connection."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_catch.preflight_health import Snapshot, evaluate, MAX_AGE, EST_BAD, FAILSAFE_FATAL


def healthy():
    return Snapshot(n_lpos=150, xy_valid_frac=1, z_valid_frac=1, v_valid_frac=1,
        heading_good_frac=1, max_eph=.15, max_epv=.20, n_failsafe=50,
        failsafe={k: False for k, _ in FAILSAFE_FATAL}, n_est=50,
        est={**{k: False for k in EST_BAD}, 'cs_tilt_align': True, 'cs_yaw_align': True},
        n_gps=50, gps_max_eph=.9, gps_max_epv=1.2, gps_min_sats=25,
        n_batt=50, batt_connected=True, batt_warning_max=0, batt_remaining_min=.8,
        n_status=50, checks_pass=True, n_att=150, att_valid=True,
        n_rc=50,rc_valid=True,ages={k: .1 for k in MAX_AGE})


def passed(s, **kw):
    return not any(r['status'] == 'fail' for r in evaluate(s, **kw))


class HealthTests(unittest.TestCase):
    def test_healthy_and_exact_40_percent(self):
        self.assertTrue(passed(healthy(), require_gps=True))
        s = healthy(); s.batt_remaining_min = .40
        self.assertTrue(passed(s))

    def test_battery_threshold_and_unknowns(self):
        for remaining in (.3999, .39, .10, .07, -1, math.nan, math.inf, 1.01):
            with self.subTest(remaining=remaining):
                s = healthy(); s.batt_remaining_min = remaining
                self.assertFalse(passed(s))
        for attr, val in [('n_batt', 0), ('batt_connected', False),
                          ('batt_warning_max', 1), ('batt_warning_max', 2)]:
            s = healthy(); setattr(s, attr, val)
            self.assertFalse(passed(s))

    def test_each_critical_stream_missing_one_frame_or_stale(self):
        for key, limit in MAX_AGE.items():
            for count in (0, 1):
                s = healthy(); setattr(s, 'n_' + key, count)
                self.assertFalse(passed(s, require_gps=True,require_rc=True), (key, count))
            s = healthy(); s.ages[key] = limit + .01
            self.assertFalse(passed(s, require_gps=True,require_rc=True), key)

    def test_px4_checks_heading_and_attitude_required(self):
        for attr, value in [('checks_pass', False), ('att_valid', False),
                            ('heading_good_frac', 0), ('v_valid_frac', .99),
                            ('dead_reckoning_frac', .01), ('invalid_data', ('lpos.vx',))]:
            s = healthy(); setattr(s, attr, value)
            self.assertFalse(passed(s), attr)

    def test_fault_flags_and_numeric_estimates(self):
        for flag, _ in FAILSAFE_FATAL:
            s = healthy(); s.failsafe[flag] = True
            self.assertFalse(passed(s), flag)
        for flag in EST_BAD:
            s = healthy(); s.est[flag] = True
            self.assertFalse(passed(s), flag)
        for name in ('max_eph', 'max_epv', 'gps_max_eph', 'gps_max_epv'):
            s = healthy(); setattr(s, name, math.nan)
            self.assertFalse(passed(s, require_gps=True), name)

    def test_offboard_signal_expected_before_authorization(self):
        s = healthy(); s.failsafe['offboard_control_signal_lost'] = True
        self.assertTrue(passed(s))

    def test_gps_only_required_outdoors(self):
        s = healthy(); s.n_gps = 0; s.ages['gps'] = math.inf
        self.assertTrue(passed(s, require_gps=False))
        self.assertFalse(passed(s, require_gps=True))


if __name__ == '__main__':
    unittest.main()
