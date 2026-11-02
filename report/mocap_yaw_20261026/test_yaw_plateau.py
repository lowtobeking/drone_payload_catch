import unittest
from yaw_plateau import stable_left,wrap_degrees
class Tests(unittest.TestCase):
    def test_left_and_right(self):
        self.assertTrue(stable_left([-89]*160,1))
        self.assertFalse(stable_left([91]*160,1))
    def test_wrap_across_boundary(self):
        self.assertTrue(stable_left([100]*160,-170))
        self.assertAlmostEqual(wrap_degrees(270),-90)
    def test_motion_and_invalid(self):
        for values in ([-90]*20,[float("nan")]*160,[-90,-85]*80):
            self.assertFalse(stable_left(values,0))
    def test_small_or_excess_rotation(self):
        self.assertFalse(stable_left([-5]*160,0))
        self.assertFalse(stable_left([-150]*160,0))
if __name__=="__main__":unittest.main()
