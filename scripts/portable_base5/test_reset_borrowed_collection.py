"""Disposable CPU fixtures only; never deletes a borrowed-host collection."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import reset_borrowed_collection as resetter


class ResetTests(unittest.TestCase):
    def fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        home = Path(temporary.name)
        workspace = home / 'ajounice_base5_capture'
        root = workspace / 'collections/sunny13_5k'
        root.mkdir(parents=True)
        (workspace / 'work/ros-session-v2').mkdir(parents=True)
        (workspace / 'work/ros-session-v2/capture.lock').touch()
        (root / '.fresh.lock').touch()
        (root / 'status.json').write_text(json.dumps(dict(stage='halted', requested=5000,
            accepted_candidate=63, finished=63, error='Single-cell recovery halted; no next load')))
        (root / 'native_fixture.bin').write_bytes(b'original native fixture')
        (workspace / 'collections/smoke_v1').mkdir()
        (workspace / 'collections/smoke_v1/proof').write_bytes(b'preserve')
        (home / 'MoraiLauncher_Lin').mkdir()
        (home / 'MoraiLauncher_Lin/protected').write_bytes(b'preserve')
        return home, workspace, root

    def test_plan_does_not_write_or_delete(self):
        home, workspace, root = self.fixture()
        before = {str(p):p.read_bytes() for p in workspace.rglob('*') if p.is_file()}
        with patch.object(Path, 'home', return_value=home):
            result = resetter.reset(workspace)
        self.assertFalse(result['executed'])
        self.assertEqual(result['accepted_deleted'], 0)
        self.assertEqual(before, {str(p):p.read_bytes() for p in workspace.rglob('*') if p.is_file()})
        self.assertTrue(root.exists())

    def test_explicit_exact_reset_preserves_neighbors_and_services(self):
        home, workspace, root = self.fixture()
        digest = resetter.sha(root / 'status.json')
        with patch.object(Path, 'home', return_value=home):
            result = resetter.reset(workspace, digest, True)
        self.assertFalse(root.exists())
        self.assertTrue((workspace / 'collections/smoke_v1/proof').exists())
        self.assertTrue((home / 'MoraiLauncher_Lin/protected').exists())
        self.assertTrue((workspace / 'work/ros-session-v2/capture.lock').exists())
        self.assertEqual(result['process_signals_sent'], 0)
        self.assertTrue(Path(result['report']).is_file())

    def test_wrong_sha_or_changed_count_is_never_deleted(self):
        home, workspace, root = self.fixture()
        with patch.object(Path, 'home', return_value=home):
            with self.assertRaises(ValueError):
                resetter.reset(workspace, '0'*64, True)
            status = root / 'status.json'
            data = json.loads(status.read_text());data['finished'] = 64
            status.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                resetter.reset(workspace, resetter.sha(status), True)
        self.assertTrue(root.exists())

    def test_symlink_inside_raw_is_rejected(self):
        home, workspace, root = self.fixture()
        (root / 'external').symlink_to(home / 'MoraiLauncher_Lin', target_is_directory=True)
        with patch.object(Path, 'home', return_value=home), self.assertRaises(ValueError):
            resetter.reset(workspace, resetter.sha(root / 'status.json'), True)
        self.assertTrue(root.exists())

    def test_work_symlink_is_rejected_without_writing_outside_scope(self):
        home, workspace, root = self.fixture()
        work = workspace / 'work'
        work.rename(workspace / 'saved_work')
        work.symlink_to(workspace / 'saved_work', target_is_directory=True)
        with patch.object(Path, 'home', return_value=home), self.assertRaises(ValueError):
            resetter.reset(workspace, resetter.sha(root / 'status.json'), True)
        self.assertTrue(root.exists())


if __name__ == '__main__':
    unittest.main()
