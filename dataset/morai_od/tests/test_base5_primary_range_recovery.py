from argparse import Namespace
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import validate_base5_primary_range_recovery as probe


class RangeProbeTests(unittest.TestCase):
    def test_fifth_probe_is_rejected_before_reading_or_capturing(self):
        with self.assertRaisesRegex(ValueError, 'fifth'):
            probe.verify_prepared(Path('/probe'), {'attempts': [{}] * 5, 'contract': {'strata4': []}}, {'scenes': []}, {})

    def test_prepared_path_cannot_be_reassigned(self):
        row = {'source_id': 'a'}
        attempt = dict(id='a', source_id='a', stratum=row, ordinal=1,
                       moving=False, capture='/elsewhere', manifest='/elsewhere/manifest.json')
        with self.assertRaisesRegex(ValueError, 'path changed'):
            probe.verify_prepared(Path('/probe'), dict(attempts=[attempt], contract={'strata4': [row]}), {'scenes': []}, {})

    def test_missing_final_park_never_reaches_native_verifier(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(probe.completion, 'verify') as verify:
            with self.assertRaisesRegex(ValueError, 'final Park'):
                probe.verify_attempt({'capture': directory}, {}, {}, {})
            verify.assert_not_called()

    def test_terminal_resume_is_readonly_and_never_starts_a_child(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / 'probe'
            output.mkdir()
            expected = dict(source_manifest_sha256='manifest', registration_sha256='registration', strata4=[])
            attempts = [dict(id=str(i), capture=str(root / str(i)), result={'status': 'accepted_candidate'}) for i in range(4)]
            state = dict(contract=expected, stage='complete', attempts=attempts)
            state['terminal_inventory'] = probe.snapshot(state)
            probe.atomic_json(output / 'state.json', state)
            before = (output / 'state.json').read_bytes()
            args = Namespace(output=output, manifest=root / 'manifest.json',
                             registration=root / 'registration.json', resume=True,
                             run=True, time_budget_sec=100)
            with patch.object(probe, 'source_inputs', return_value=({}, {}, {str(i): {} for i in range(4)}, [])), \
                    patch.object(probe, 'contract', return_value=expected), \
                    patch.object(probe, 'verify_prepared'), \
                    patch.object(probe.completion.spatial.gen, 'context', return_value={}), \
                    patch.object(probe.completion, 'pool_map', return_value={}), \
                    patch.object(probe, 'verify_attempt', side_effect=lambda a, *args: a['result']), \
                    patch.object(probe.completion, 'run_child') as child:
                self.assertEqual(probe.execute(args), 0)
                child.assert_not_called()
            self.assertEqual((output / 'state.json').read_bytes(), before)
            result = json.loads((output / 'proof.json').read_text())
            self.assertTrue(result['passed'])
            self.assertEqual(result['additional_completed_resume_raw'], 0)
            self.assertFalse(result['actual_parent_deficit'])

    def test_native_proof_requires_four_approvals_and_terminal_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state = dict(stage='complete', contract=dict(source_manifest_sha256='manifest',
                registration_sha256='registration', strata4=[]), attempts=[dict(
                    capture=str(root / str(i)), result={'status': 'accepted_candidate'}) for i in range(4)])
            probe.atomic_json(root / 'state.json', state)
            self.assertFalse(probe.proof(root, state)['passed'])
            self.assertTrue(probe.proof(root, state, 0)['passed'])
            state['attempts'][0]['result']['status'] = 'review'
            self.assertFalse(probe.proof(root, state, 0)['passed'])


if __name__ == '__main__':
    unittest.main()
