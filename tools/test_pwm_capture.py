#!/usr/bin/env python3
"""Validate edge timing analysis and reject misleading/invalid captures; no hardware."""
from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.pwm_capture import analyze_edges

def stream(width_ns=1500000,period_ns=20000000,count=30):
    rows=[]
    for i in range(count):
        for stamp,edge in ((1_000_000_000+i*period_ns,1),(1_000_000_000+i*period_ns+width_ns,0)):
            seconds,nano=divmod(stamp,1_000_000_000);rows.append(f'{seconds} {nano} {edge}')
    return '\n'.join(rows)

class CaptureTests(unittest.TestCase):
    def test_50hz_1500us_including_second_rollover(self):
        r=analyze_edges(stream(count=70))
        self.assertTrue(r['timing_samples_usable'])
        self.assertEqual(r['high_time_us']['median'],1500)
        self.assertEqual(r['period_us']['median'],20000)
        self.assertEqual(r['frequency_hz'],50)
    def test_333hz_is_distinguished_from_50hz(self):
        r=analyze_edges(stream(period_ns=3003003,width_ns=1533000))
        self.assertAlmostEqual(r['frequency_hz'],333,places=3)
        self.assertEqual(r['high_time_us']['median'],1533)
    def test_inverted_signal_is_not_misreported_as_correct_pulse(self):
        r=analyze_edges(stream(width_ns=18500000))
        self.assertEqual(r['high_time_us']['median'],18500)
    def test_missing_edge_marks_capture_unreliable(self):
        rows=stream().splitlines();del rows[11]
        self.assertFalse(analyze_edges('\n'.join(rows))['timing_samples_usable'])
    def test_partial_boundary_edges_are_allowed(self):
        rows=stream().splitlines()[1:-1]
        self.assertTrue(analyze_edges('\n'.join(rows))['timing_samples_usable'])
    def test_empty_or_short_capture_does_not_imply_good_signal(self):
        for value in ('',stream(count=2)):
            self.assertFalse(analyze_edges(value)['timing_samples_usable'])
    def test_invalid_or_reordered_records_rejected(self):
        for value in ('bad','1 1000000000 1','1 0 2','-1 0 1','1 10 1\n1 9 0','1 10 1\n1 10 0'):
            with self.assertRaises(ValueError):analyze_edges(value)

if __name__=='__main__':unittest.main()
