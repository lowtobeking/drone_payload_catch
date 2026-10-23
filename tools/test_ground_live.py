from pathlib import Path
import importlib.util
import sys
import unittest

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ground_live',root/'tools/run_a_ground_live.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class GroundTelemetryTests(unittest.TestCase):
    def ready(self,status=None,land=None,last_status=9.5,last_land=9.5,now=10):
        return module.ground_is_fresh([(1,1,100),(1,1,101)] if status is None else status,
            [True,True] if land is None else land,last_status,last_land,now)

    def test_ground_disarmed_fresh_stream(self):
        self.assertTrue(self.ready())

    def test_stopped_stream_future_time_or_missing_is_rejected(self):
        for stamp in (None,0,7.9,10.1,float('nan'),float('inf')):
            self.assertFalse(self.ready(last_status=stamp))
            self.assertFalse(self.ready(last_land=stamp))

    def test_disarm_and_identity_must_hold_for_all_samples(self):
        for status in ([(1,2,100),(1,1,101)],[(2,1,100),(2,1,101)],[],[(1,1,100)]):
            self.assertFalse(self.ready(status=status))

    def test_duplicate_invalid_source_time_and_airborne_rejected(self):
        for status in ([(1,1,100),(1,1,100)],[(1,1,101),(1,1,100)],[(1,1,0),(1,1,1)]):
            self.assertFalse(self.ready(status=status))
        for land in ([],[True,False],[False]):self.assertFalse(self.ready(land=land))

if __name__=='__main__':unittest.main()