import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_readiness import shutdown_evidence
from base5_capture_safety import atomic_json


class ReadinessBatchTest(unittest.TestCase):
    def test_batch_requires_actual_child_and_boundary_park(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);child=root/'child';child.mkdir()
            atomic_json(root/'batch_contract.json',{})
            atomic_json(root/'status.json',dict(stage='complete_with_deficit'))
            self.assertFalse(shutdown_evidence(root)['park_verified'])
            atomic_json(child/'session_contract.json',{})
            atomic_json(child/'shutdown_safety.json',dict(park_verified=True))
            (child/'ledger.jsonl').write_text('{"id":"a","between_scene_park":{"park_verified":true}}\n')
            self.assertTrue(shutdown_evidence(root)['park_verified'])
            (child/'ledger.jsonl').write_text('{"id":"a","between_scene_park":{"park_verified":false}}\n')
            self.assertFalse(shutdown_evidence(root)['park_verified'])
            atomic_json(root/'status.json',dict(stage='halted'))
            self.assertFalse(shutdown_evidence(root)['park_verified'])

    def test_single_session_and_missing_contract(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with self.assertRaisesRegex(ValueError,'contract'):shutdown_evidence(root)
            atomic_json(root/'shutdown_safety.json',dict(park_verified=True))
            self.assertTrue(shutdown_evidence(root)['park_verified'])
