"""Pure known-color/unknown-shape evidence extraction; no inference or ROS."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from traffic_light_unreadable_evidence import unreadable_color_evidence

R, G = ('red', 'circle', None), ('green', 'circle', None)


class UnreadableEvidenceTest(unittest.TestCase):
    def evidence(self, bits, raw=(), confidence=0.):
        return unreadable_color_evidence(raw, {'bits': bits}, confidence)

    def test_confident_positive_bits_keep_only_their_actual_color(self):
        cases = ((dict(red=1, yellow=0, green=1, left_arrow=0), ['red', 'green']),
                 (dict(red=0, yellow=1, green=0, left_arrow=None), ['yellow']),
                 (dict(red=0, yellow=0, green=1, left_arrow=None), ['green']),
                 (dict(red=None, yellow=0, green=0, left_arrow=1), ['green']))
        for bits, colors in cases:
            with self.subTest(bits=bits):
                evidence = self.evidence(bits)
                self.assertEqual([row['color'] for row in evidence], colors)
                self.assertTrue(all(row['shape'] == row['status'] == 'unknown' for row in evidence))
                self.assertTrue(all(row['confidence'] == 0. for row in evidence))

    def test_no_lamps_and_uncertain_bits_do_not_invent_positive_colors(self):
        for bits in (dict(red=0, yellow=0, green=0, left_arrow=0),
                     dict(red=0, yellow=0, green=None, left_arrow=0),
                     dict(red=None, yellow=None, green=None, left_arrow=None)):
            self.assertEqual(self.evidence(bits), [])

    def test_raw_contradiction_colors_are_preserved_with_original_confidence(self):
        evidence = unreadable_color_evidence([R, G], {}, .72)
        self.assertEqual([row['color'] for row in evidence], ['red', 'green'])
        self.assertEqual([row['confidence'] for row in evidence], [.72, .72])
        self.assertTrue(all(row['sources'] == ['raw_observed_tuple'] for row in evidence))

    def test_green_circle_and_arrow_evidence_do_not_duplicate_wire_color(self):
        evidence = self.evidence(dict(red=0, yellow=0, green=1, left_arrow=1), raw=[G], confidence=.7)
        self.assertEqual(len(evidence), 1)
        self.assertEqual(evidence[0]['color'], 'green')
        self.assertEqual(evidence[0]['sources'],
                         ['raw_observed_tuple', 'classifier_bit:green', 'classifier_bit:left_arrow'])

    def test_malformed_bits_and_tuples_cannot_supply_colors(self):
        for bits in (None, {}, dict(red=True, yellow=0, green=0, left_arrow=0),
                     dict(red='1', yellow=0, green=0, left_arrow=0),
                     dict(red=1., yellow=0, green=0, left_arrow=0)):
            self.assertEqual(self.evidence(bits), [])
        self.assertEqual(unreadable_color_evidence([None, ['green'], ['off', 'circle', None]], {}, .9), [])
        self.assertEqual(unreadable_color_evidence(None, None, .9), [])

    def test_invalid_confidence_is_not_replaced_by_invented_certainty(self):
        for confidence in (None, True, float('nan'), float('inf'), -.1, 1.1):
            self.assertEqual(unreadable_color_evidence([R], {}, confidence), [])


if __name__ == '__main__':
    unittest.main()
