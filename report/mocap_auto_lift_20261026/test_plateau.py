import unittest
from lift_plateau import stable_phase
class PlateauTests(unittest.TestCase):
    def test_raise_negative_ned_z(self):
        self.assertTrue(stable_phase([-.7]*160,-.3,"elevated"))
        self.assertFalse(stable_phase([.1]*160,-.3,"elevated"))
    def test_return_requires_actual_baseline(self):
        self.assertFalse(stable_phase([-.7]*160,-.3,"returned"))
        self.assertTrue(stable_phase([-.301]*160,-.3,"returned"))
    def test_motion_and_missing_nonfinite_rejected(self):
        for values in ([-.7]*50,[-.7,-.67]*80,[float("nan")]*160):
            self.assertFalse(stable_phase(values,-.3,"elevated"))
    def test_static_baseline_never_counts_as_lift(self):
        self.assertFalse(stable_phase([-.3]*160,-.3,"elevated"))
    def test_limits_and_phase(self):
        self.assertFalse(stable_phase([-2.]*160,-.3,"elevated"))
        with self.assertRaises(ValueError):stable_phase([-.7]*160,-.3,"unknown")
if __name__=="__main__":unittest.main()
