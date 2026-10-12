import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from report_base5_parked_npc_support import assess


class ParkedNpcSupportTests(unittest.TestCase):
    def actor(self, confidence):
        return dict(source_list='vehicleList', motion='static', name='bus', unique_id=8500,
                    estimated_size=[12, 3, 3.5], map_ground_initializer=dict(footprint=dict(
                        all_supported=True, samples=[dict(usable=True, confidence=c,
                                                        nearest_road_distance_m=0.1)
                                                     for c in confidence])))

    def test_available_map_height_does_not_certify_road_contact(self):
        actor = self.actor(['road_strip'] + ['roadside_estimate'] * 4)
        value = assess(actor)
        self.assertTrue(value['map_height_available_all'])
        self.assertTrue(value['screening_risk'])
        self.assertEqual(value['roadside_or_unsupported_corners'], 4)
        self.assertFalse(value['contact_verified'])

    def test_supported_footprint_is_not_a_live_contact_certificate(self):
        value = assess(self.actor(['road_strip'] * 5))
        self.assertFalse(value['screening_risk'])
        self.assertFalse(value['contact_verified'])

    def test_missing_samples_and_nonparked_assets(self):
        actor = self.actor([])
        self.assertEqual(assess(actor)['reason'], 'footprint_unknown')
        actor['source_list'] = 'objectList'
        self.assertIsNone(assess(actor))
        actor['source_list'] = 'vehicleList'; actor['motion'] = 'moving'
        self.assertIsNone(assess(actor))


if __name__ == '__main__':
    unittest.main()
