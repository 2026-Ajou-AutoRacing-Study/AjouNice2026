import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_roundabout_validation as report


class RemainingPilotTests(unittest.TestCase):
    def fixture(self, root, safe=True):
        child = root/'child';capture = child/'captures'/'a';capture.mkdir(parents=True)
        (capture/'metadata.json').write_text(json.dumps(dict(spec={}, runtime_actors=[])))
        (capture/'audit.json').write_text(json.dumps(dict(review_reasons=[],
            capture_safety=dict(collision=dict(no_collision_verified=True)))))
        (child/'shutdown_safety.json').write_text(json.dumps(dict(park_verified=safe)))
        entry = dict(id='a', status='accepted_candidate', scenario_sha256='same',
                     capture_root=str(capture), between_scene_park=dict(park_verified=True))
        ledger = root/'selected.jsonl';ledger.write_text(json.dumps(entry)+'\n')
        deck = dict(scenes=[dict(id='a', scenario_sha256='same'),dict(id='b', scenario_sha256='next')])
        return deck,ledger

    def test_remaining_keeps_predetermined_ids_and_payloads(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);deck,ledger=self.fixture(root);output=root/'remaining.json'
            result=report.remaining_pilot(deck,[ledger],output)
            self.assertEqual(result,dict(safe_prior_cells=1,remaining_cells=1))
            self.assertEqual(json.loads(output.read_text())['scenes'],deck['scenes'][1:])
            with self.assertRaises(FileExistsError):report.remaining_pilot(deck,[ledger],output)

    def test_unsafe_shutdown_or_different_payload_cannot_be_reused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);deck,ledger=self.fixture(root,safe=False)
            with self.assertRaisesRegex(ValueError,'Unsafe child'):
                report.remaining_pilot(deck,[ledger],root/'remaining.json')
            deck['scenes'][0]['scenario_sha256']='different'
            with self.assertRaisesRegex(ValueError,'exact predetermined'):
                report.remaining_pilot(deck,[ledger],root/'remaining.json')

    def test_combination_refuses_incomplete_or_duplicate_cells(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);deck,ledger=self.fixture(root)
            with self.assertRaisesRegex(ValueError,'incomplete'):
                report.combined_ledger(deck,[ledger],root/'combined.jsonl')
            with self.assertRaisesRegex(ValueError,'Duplicate'):
                report.combined_ledger(deck,[ledger,ledger],root/'combined.jsonl')
            deck['scenes']=deck['scenes'][:1]
            self.assertEqual(report.combined_ledger(deck,[ledger],root/'combined.jsonl')['selected_cells'],1)
