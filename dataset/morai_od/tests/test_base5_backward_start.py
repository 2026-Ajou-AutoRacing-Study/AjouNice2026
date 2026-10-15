import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prepare_base5_motion_appearance_probe import backward_start


class BackwardStartTest(unittest.TestCase):
    def test_natural_speed_reaches_target_after_budget(self):
        t=dict(x=20,y=30,z=4);p=backward_start(t,(.6,.8),2.2,5)
        self.assertAlmostEqual(p['x']+2.2*5*.6,t['x'])
        self.assertAlmostEqual(p['y']+2.2*5*.8,t['y'])
        self.assertEqual(t,dict(x=20,y=30,z=4))

    def test_stationary_stays_at_target(self):
        t=dict(x=20,y=30,z=4)
        self.assertEqual(backward_start(t,(1,0),0,5),t)

    def test_invalid_direction_rejected(self):
        with self.assertRaises(ValueError):backward_start(dict(x=0,y=0,z=0),(2,0),1,5)


if __name__=='__main__':unittest.main()
