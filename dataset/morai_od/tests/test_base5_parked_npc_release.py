import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json
import finalize_base5_parked_npc_release as release


class NpcReleaseTests(unittest.TestCase):
    def test_completion_verifier_receives_deck_and_source_with_correct_roles(self):
        attempt = dict(result={'status': 'accepted_candidate'})
        original, deck, pools = {'spec': 'original'}, {'config': 'deck'}, {'a': 'vehicle'}
        with patch.object(release.completion, 'verify', return_value=attempt['result']) as verifier:
            release.reverify_completion_attempt(attempt, original, deck, pools)
            verifier.assert_called_once_with(attempt, deck=deck, source=original, pools=pools)
        with patch.object(release.completion, 'verify', return_value={'status': 'review'}):
            with self.assertRaises(ValueError): release.reverify_completion_attempt(attempt, original, deck, pools)

    def test_every_repaired_slot_needs_its_final_payload_and_actual_stationary_proof(self):
        with tempfile.TemporaryDirectory() as temporary:
            manifest = Path(temporary) / 'manifest.json'
            atomic_json(manifest, dict(scenes=[dict(id='a', scenario_sha256='source')]))
            registration = dict(all_changed_slots=['a'])
            row = dict(source_cell_id='a', scenario_sha256='source',
                       audit=dict(stationary_vehicle_runtime=dict(actors=[dict(verified=True)])))
            self.assertEqual(release.verify_repaired_cohort(registration, [row], manifest), 1)
            for rows in ([], [dict(row, scenario_sha256='changed')],
                         [dict(row, audit=dict(stationary_vehicle_runtime=dict(actors=[dict(verified=False)])))]):
                with self.assertRaises(ValueError): release.verify_repaired_cohort(registration, rows, manifest)


if __name__ == '__main__': unittest.main()
