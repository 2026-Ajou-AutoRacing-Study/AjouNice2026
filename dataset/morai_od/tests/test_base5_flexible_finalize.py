import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import finalize_base5_flexible_selection as final


class FlexibleFinalizeTests(unittest.TestCase):
    def fixture(self,root):
        original=root/'original.json'
        source_meta=dict(spec=dict(family='obstacle',mission=True,distance_bounds=[2,5],role='left'),
                         split='val',runtime_actors=[dict(data_id=1,motion='static')])
        final.flex.ac.fa.atomic_json(original,source_meta)
        raw=root/'raw';raw.mkdir()
        actual=dict(source_meta,scene_group_id='old',spec=dict(source_meta['spec'],distance_bounds=[15,30],role='front'),
                    runtime_actors=[dict(data_id=2,motion='static')])
        final.flex.ac.fa.atomic_json(raw/'metadata.json',actual)
        final.flex.ac.fa.atomic_json(raw/'scenario.json',dict(objectList=[dict(DataID=2)]))
        final.flex.ac.fa.atomic_json(raw/'audit.json',dict(review_reasons=[]))
        entry=dict(id='old',metadata=str(original))
        source=dict(config=dict(target_scenes=1),scenes=[entry])
        row=dict(id='old_fx01',source_cell_id='old',capture_root=str(raw),status='accepted_candidate',
                 scenario_sha256=final.flex.ac.fa.sha256(raw/'scenario.json'),flexible_completion=dict(policy='explicit'))
        return source,row,raw,original

    def test_adopted_plan_points_to_native_actual_not_original_failed_spec(self):
        with tempfile.TemporaryDirectory() as d:
            source,row,raw,original=self.fixture(Path(d));before=original.read_bytes()
            deck=final.adopted_deck(source,[row]);entry=deck['scenes'][0]
            self.assertEqual(entry['id'],'old')
            self.assertEqual(entry['scenario'],str(raw/'scenario.json'))
            self.assertEqual(entry['distance_bounds'],[15,30])
            self.assertEqual(entry['split'],'val')
            self.assertEqual(original.read_bytes(),before)

    def test_duplicate_and_unknown_slots_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            source,row,_,_=self.fixture(Path(d))
            with self.assertRaisesRegex(ValueError,'duplicate'):final.adopted_deck(source,[row,row])
            row['source_cell_id']='unknown'
            with self.assertRaisesRegex(ValueError,'Unknown'):final.adopted_deck(source,[row])

    def test_review_calibration_and_raw_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            source,row,raw,_=self.fixture(Path(d))
            final.flex.ac.fa.atomic_json(raw/'audit.json',dict(review_reasons=['missing']))
            with self.assertRaisesRegex(ValueError,'Review'):final.adopted_deck(source,[row])
            final.flex.ac.fa.atomic_json(raw/'audit.json',dict(review_reasons=[]))
            final.flex.ac.fa.atomic_json(raw/'scenario.json',dict(tampered=True))
            with self.assertRaisesRegex(ValueError,'checksum'):final.adopted_deck(source,[row])

    def test_cross_split_and_group_changes_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            source,row,raw,_=self.fixture(Path(d));p=raw/'metadata.json';meta=json.loads(p.read_text())
            meta['split']='train';final.flex.ac.fa.atomic_json(p,meta)
            with self.assertRaisesRegex(ValueError,'split'):final.adopted_deck(source,[row])
            meta['split']='val';meta['scene_group_id']='other';final.flex.ac.fa.atomic_json(p,meta)
            with self.assertRaisesRegex(ValueError,'group'):final.adopted_deck(source,[row])

    def test_existing_empty_background_without_distance_bin_is_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            source,row,raw,original=self.fixture(Path(d))
            meta=json.loads(original.read_text())
            meta.update(spec=dict(family='background',mission=False,role='rear'),runtime_actors=[])
            final.flex.ac.fa.atomic_json(original,meta)
            final.flex.ac.fa.atomic_json(raw/'metadata.json',dict(meta,scene_group_id='old'))
            entry=final.adopted_deck(source,[row])['scenes'][0]
            self.assertEqual(entry['distance_bounds'],[])
            self.assertEqual(entry['family'],'background')


if __name__=='__main__':unittest.main()
