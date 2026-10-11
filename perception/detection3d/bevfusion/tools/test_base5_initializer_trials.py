"""Paired comparison refuses incomplete epochs, changed inputs and class counts."""
import json
from pathlib import Path
import tempfile
import unittest

from summarize_base5_initializer_trials import compare


class TrialComparisonTest(unittest.TestCase):
    def fixture(self, root, initializer, score):
        root.mkdir()
        (root/'run_manifest.json').write_text(json.dumps(dict(contract_sha256=initializer,
            identity=dict(initial_checkpoint_sha256=initializer, seed=261010,
                          iterations_per_epoch=1064, epochs=24))))
        (root/'initializer_trial_status.json').write_text(json.dumps(dict(state='finished', epoch=1, iteration=1064)))
        target = root/'20261011_120000/vis_data'
        target.mkdir(parents=True)
        row = {'step': 1, 'val/center/mAP': score}
        for name, count in zip(('vehicle', 'pedestrian', 'obstacle'), (341, 687, 674)):
            row['val/center/' + name + '_GT'] = float(count)
            row['val/center/' + name + '_mean_AP'] = score
        (target/'scalars.json').write_text(json.dumps(row) + '\n')

    def test_only_initializer_difference_and_percentage_points(self):
        with tempfile.TemporaryDirectory() as directory:
            a, b = Path(directory)/'a', Path(directory)/'b'
            self.fixture(a, 'baseline', .3)
            self.fixture(b, 'candidate', .8)
            result = compare(a, b)
            self.assertEqual(result['only_identity_difference'], 'initial_checkpoint_sha256')
            self.assertAlmostEqual(result['delta_percentage_points']['val/center/mAP'], 50.)

    def test_other_identity_and_partial_budget_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            a, b = Path(directory)/'a', Path(directory)/'b'
            self.fixture(a, 'baseline', .3)
            self.fixture(b, 'candidate', .8)
            path = b/'run_manifest.json'
            manifest = json.loads(path.read_text())
            manifest['identity']['seed'] += 1
            path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'beyond initialization'):
                compare(a, b)
            manifest['identity']['seed'] -= 1
            path.write_text(json.dumps(manifest))
            path = b/'initializer_trial_status.json'
            status = json.loads(path.read_text())
            status['iteration'] -= 1
            path.write_text(json.dumps(status))
            with self.assertRaisesRegex(ValueError, 'full training budget'):
                compare(a, b)


if __name__ == '__main__':
    unittest.main()
