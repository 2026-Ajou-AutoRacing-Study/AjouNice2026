"""CPU-only bounds and immutable-reference guards for the large scale deck."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_scale_qualification as prep


class QualificationPreparationTests(unittest.TestCase):
    def test_initial39_and_followup9_are_separate_unapproved_decks(self):
        self.assertEqual(prep.selected_count(prep.plan()), 39)
        self.assertEqual(prep.selected_count(prep.plan('barrier-followup')), 9)
        initial = prep.plan()
        self.assertEqual(initial[1][4], ['sample_2_3_2'])
        self.assertTrue(all(row[3] == [0.] for row in initial[-3:]))
        self.assertTrue(all(row[3] == [90.] for row in prep.plan('barrier-followup')))

    def test_unknown_phase_fails(self):
        with self.assertRaises(ValueError):
            prep.plan('bulk')

    def test_output_cannot_append_to_preserved_source_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            with self.assertRaisesRegex(ValueError,'preserved source'):
                prep.fresh_output(root/'not_created',[root])
            self.assertFalse((root/'not_created').exists())

    def test_reference_merge_pins_sources_and_rejects_conflicts_or_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = dict(bbox_size_m=[1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., .5])
            first, second = root/'first.json', root/'second.json'
            first.write_text(json.dumps(dict(assets=dict(CargoBox=record))))
            second.write_text(json.dumps(dict(assets=dict(WoodBox=record))))
            value, pins = prep.merge_references([first, second], {'CargoBox', 'WoodBox'})
            self.assertEqual(set(value['assets']), {'CargoBox', 'WoodBox'})
            self.assertEqual(len(pins), 2)
            self.assertFalse(value['production_approved'])
            with self.assertRaisesRegex(ValueError, 'Missing native unit'):
                prep.merge_references([first], {'WoodBox'})
            record['bbox_size_m'] = [2., 2., 2.]
            second.write_text(json.dumps(dict(assets=dict(CargoBox=record))))
            with self.assertRaisesRegex(ValueError, 'Conflicting'):
                prep.merge_references([first, second], {'CargoBox'})

    def test_existing_destination_is_never_resumed_or_rewritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'source.json'
            source.write_text('{}')
            output = root/'exists'
            output.mkdir()
            with self.assertRaises(FileExistsError):
                prep.prepare(source, output, [])

    def test_larger_inventory_adopts_only_explicit_requested_whitelist_subset(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'inventory.json'
            record=dict(bbox_size_m=[1.,1.,1.],pivot_to_bbox_center_local_m=[0.,0.,.5])
            path.write_text(json.dumps(dict(assets=dict(CargoBox=record,UnrelatedPerson=dict(unqualified=True)))))
            reference,pins=prep.merge_references([path],{'CargoBox'})
            self.assertEqual(set(reference['assets']),{'CargoBox'})
            self.assertEqual(len(pins),1)

    def test_reference_sources_do_not_alias_the_callers_expanding_input_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'units.json'
            record=dict(bbox_size_m=[1.,1.,1.],pivot_to_bbox_center_local_m=[0.,0.,.5])
            path.write_text(json.dumps(dict(assets=dict(CargoBox=record))))
            reference,pins=prep.merge_references([path],{'CargoBox'})
            expected=dict(pins)
            self.assertIsNot(reference['sources'],pins)
            pins['/later/source/manifest.json']='unrelated-campaign-input'
            self.assertEqual(reference['sources'],expected)
            reference['sources']['/reference-only']='not-an-input-pin'
            self.assertNotIn('/reference-only',pins)


if __name__ == '__main__':
    unittest.main()
