import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import vertical_motion_review


class VerticalMotionTest(unittest.TestCase):
    actor=dict(unique_id=7,motion='walking',source_list='objectList')

    def samples(self,zs):return [dict(uid=7,t=1.+i*.1,z=z) for i,z in enumerate(zs)]

    def test_jump_rejected(self):
        r=vertical_motion_review(self.actor,self.samples([28.,28.,28.65]),1.,28.)
        self.assertIn('motion_vertical_jump',r['review_reasons'])

    def test_constant_floating_also_rejected(self):
        r=vertical_motion_review(self.actor,self.samples([28.65]*3),1.,28.)
        self.assertIn('motion_vertical_reference_mismatch',r['review_reasons'])
        self.assertNotIn('motion_vertical_jump',r['review_reasons'])

    def test_matched_height_passes(self):
        self.assertFalse(vertical_motion_review(self.actor,self.samples([28.,28.001,28.]),1.,28.)['review_reasons'])

    def test_missing_reference_not_certified(self):
        self.assertIn('motion_vertical_ground_reference_unverified',
                      vertical_motion_review(self.actor,self.samples([28.]*3),1.)['review_reasons'])

    def test_gradual_slope_is_not_jump(self):
        a=dict(self.actor,source_list='pedestrianList')
        self.assertFalse(vertical_motion_review(a,self.samples([28.,28.01,28.02]),1.)['review_reasons'])

    def test_static_and_previous_scene_excluded(self):
        self.assertFalse(vertical_motion_review(dict(self.actor,motion='static'),[],1.)['required'])
        r=vertical_motion_review(self.actor,[dict(uid=7,t=0.,z=40.)]+self.samples([28.]*3),1.,28.)
        self.assertFalse(r['review_reasons'])


if __name__=='__main__':unittest.main()
