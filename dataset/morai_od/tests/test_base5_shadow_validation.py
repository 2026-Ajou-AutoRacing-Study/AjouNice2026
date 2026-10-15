import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json,sha256
from report_base5_shadow_validation import verified_entries


class ShadowSelectionTests(unittest.TestCase):
    def fixture(self,root,replacement=False,unsafe=False):
        source=root/'source';source.mkdir();child=root/'capture';capture=child/'captures'/'cell';capture.mkdir(parents=True)
        scenario={'test':1};meta=dict(spec=dict(index=1),split='train',runtime_actors=[dict(data_id=7,unique_id=8500)])
        atomic_json(source/'scene.json',scenario);atomic_json(source/'metadata.json',meta)
        original=dict(id='cell',scenario=str(source/'scene.json'),metadata=str(source/'metadata.json'),
                      scenario_sha256=sha256(source/'scene.json'))
        if replacement:
            scenario={'test':2};meta=copy.deepcopy(meta)
            meta['replacement']=dict(source_cell_id='cell',source_scenario_sha256=original['scenario_sha256'])
        atomic_json(capture/'scenario.json',scenario);atomic_json(capture/'metadata.json',meta)
        atomic_json(capture/'audit.json',dict(review_reasons=[],capture_safety=dict(collision=dict(no_collision_verified=True))))
        atomic_json(child/'shutdown_safety.json',dict(park_verified=not unsafe))
        atomic_json(child/'session_contract.json',dict(runner_options=dict(shared_ros_batch=True)))
        row=dict(id='cell_replace' if replacement else 'cell',source_cell_id='cell',status='accepted_candidate',
            scenario_sha256=sha256(capture/'scenario.json'),capture_root=str(capture),between_scene_park=dict(park_verified=True))
        ledger=root/'ledger.jsonl';ledger.write_text(json.dumps(row)+'\n')
        return dict(scenes=[original]),ledger,capture,child

    def test_exact_selection_and_same_cell_replacement(self):
        for replacement in (False,True):
            with tempfile.TemporaryDirectory() as d:
                deck,ledger,_,_=self.fixture(Path(d),replacement)
                self.assertEqual(list(verified_entries(deck,[ledger])),['cell'])

    def test_unsafe_shutdown_and_duplicate_selection_refused(self):
        with tempfile.TemporaryDirectory() as d:
            deck,ledger,_,child=self.fixture(Path(d),unsafe=True)
            with self.assertRaisesRegex(ValueError,'Unsafe'):verified_entries(deck,[ledger])
            atomic_json(child/'shutdown_safety.json',dict(park_verified=True))
            with self.assertRaisesRegex(ValueError,'duplicate'):verified_entries(deck,[ledger,ledger])

    def test_unknown_payload_change_refused(self):
        with tempfile.TemporaryDirectory() as d:
            deck,ledger,capture,_=self.fixture(Path(d),replacement=True)
            meta=json.loads((capture/'metadata.json').read_text());meta.pop('replacement')
            atomic_json(capture/'metadata.json',meta)
            with self.assertRaisesRegex(ValueError,'witnessed'):verified_entries(deck,[ledger])

    def test_replacement_cannot_change_primary_or_quota(self):
        with tempfile.TemporaryDirectory() as d:
            deck,ledger,capture,_=self.fixture(Path(d),replacement=True)
            meta=json.loads((capture/'metadata.json').read_text());meta['runtime_actors'][0]['data_id']=8
            atomic_json(capture/'metadata.json',meta)
            with self.assertRaisesRegex(ValueError,'identity'):verified_entries(deck,[ledger])
            meta['spec']['index']=2;atomic_json(capture/'metadata.json',meta)
            with self.assertRaisesRegex(ValueError,'quota'):verified_entries(deck,[ledger])

    def test_moving_replacement_cannot_skip_independent_proof(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as d:
            deck,ledger,capture,_=self.fixture(Path(d),replacement=True)
            meta=json.loads((capture/'metadata.json').read_text())
            meta['runtime_actors'][0]['motion_path']={'policy':'natural-target-v1'}
            atomic_json(capture/'metadata.json',meta)
            with patch('report_base5_shadow_validation.validate_motion_ground',
                       side_effect=ValueError('independent proof required')):
                with self.assertRaisesRegex(ValueError,'independent proof'):
                    verified_entries(deck,[ledger])


if __name__=='__main__':unittest.main()
