import unittest
from pathlib import Path

from capture_traffic_light_natural_campaign import absolute_output, native_brackets_stable


class NaturalCampaignTest(unittest.TestCase):
    def row(self, key='head', color=1, success=True):
        return {'requested_id':key,'readback_success':success,'observations':[{'tl_color':color}]}

    def test_absolute_relative_path_regression(self):
        root=absolute_output('artifacts/a')
        self.assertTrue(root.is_absolute())
        self.assertEqual((root/'owned_probe/a.jpg').relative_to(root),Path('owned_probe/a.jpg'))

    def test_state_changes_invalid_empty_or_different_head_not_stable(self):
        a=[self.row()]
        self.assertTrue(native_brackets_stable(a,a))
        self.assertFalse(native_brackets_stable([],[]))
        self.assertFalse(native_brackets_stable(a,[]))
        self.assertFalse(native_brackets_stable(a,[self.row(color=4)]))
        self.assertFalse(native_brackets_stable(a,[self.row(key='other')]))
        self.assertFalse(native_brackets_stable(a,[self.row(success=False)]))

    def test_same_program_heads_retained_separately(self):
        a=[self.row('near',48),self.row('far',48)]
        self.assertTrue(native_brackets_stable(a,a))
        self.assertFalse(native_brackets_stable(a,list(reversed(a))))


if __name__=='__main__':
    unittest.main()
