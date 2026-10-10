import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import torch

spec = importlib.util.spec_from_file_location('backup', Path(__file__).with_name('backup_base5_checkpoints.py'))
backup = importlib.util.module_from_spec(spec); spec.loader.exec_module(backup)


class BackupTest(unittest.TestCase):
    def fixture(self, root):
        run = root/'run1'; run.mkdir()
        (run/'run_manifest.json').write_text(json.dumps(dict(contract_sha256='c1')))
        (run/'resolved_train.py').write_text('model={}\n')
        checkpoint = dict(state_dict={'w': torch.ones(2)}, optimizer={'loss_scaler': {}},
            param_schedulers=[], message_hub={}, base5_rng={}, meta=dict(epoch=1, iter=4),
            base5_contract_sha256='c1')
        torch.save(checkpoint, run/'epoch_1.pth')
        (run/'epoch_2.pth.partial-dead').write_bytes(b'partial')
        return run

    def test_complete_idempotent_and_partial_ignored(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); run = self.fixture(root); dest = root/'external'
            state = backup.backup_once(run, str(dest), {})
            self.assertEqual(state['errors'], [])
            self.assertEqual(len(state['uploaded']), 1)
            latest = json.loads((dest/'run1/LATEST.json').read_text())
            self.assertEqual(latest['iteration'], 4)
            checkpoint = Path(latest['location'])/'epoch_1.pth'
            self.assertEqual(backup.sha256(checkpoint), latest['sha256'])
            self.assertTrue((Path(latest['location'])/'COMPLETE.json').exists())
            self.assertTrue((run/'epoch_1.pth').exists())
            with patch.object(backup, 'copy_verified', side_effect=AssertionError('No repeated upload')):
                self.assertEqual(backup.backup_once(run, str(dest), state)['errors'], [])

    def test_failure_has_no_latest_pointer_and_retry(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); run=self.fixture(root); dest=root/'external'
            with patch.object(backup, 'copy_verified', side_effect=RuntimeError('upload failed')):
                state=backup.backup_once(run,str(dest),{})
            self.assertTrue(state['errors']); self.assertFalse((dest/'run1/LATEST.json').exists())
            self.assertFalse(list(run.glob('.backup_stage_*')))
            self.assertEqual(backup.backup_once(run,str(dest),state)['errors'], [])


if __name__ == '__main__': unittest.main()
