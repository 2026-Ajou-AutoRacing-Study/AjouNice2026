import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('context_archive', Path(__file__).with_name('check_context_archive.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ArchiveTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.git('init', '-q')
        (self.root/'original.md').write_bytes(b'original evidence\n')
        self.git('add', 'original.md')
        self.git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                 '-c', 'commit.gpgsign=false', 'commit', '-qm', 'fixture')
        commit = self.git('rev-parse', 'HEAD').decode().strip()
        (self.root/'legacy.md').write_bytes(b'original evidence\n')
        self.row = dict(domain='test', original_path='original.md', archive_path='legacy.md',
                        sha256=hashlib.sha256(b'original evidence\n').hexdigest(),
                        operation='move', source_commit=commit)

    def git(self, *args):
        return subprocess.check_output(['git', *args], cwd=self.root, stderr=subprocess.DEVNULL)

    def run_audit(self, rows=None):
        (self.root/'manifest.json').write_text(json.dumps(dict(schema='context_archive_v1', records=rows or [self.row])))
        return module.audit(self.root, 'manifest.json')

    def test_preserved(self):
        result = self.run_audit()
        self.assertEqual([], result['errors'])
        self.assertEqual(1, result['verified_git'])

    def test_tamper(self):
        (self.root/'legacy.md').write_bytes(b'edited history')
        self.assertTrue(any('SHA mismatch' in x for x in self.run_audit()['errors']))

    def test_missing_archive(self):
        (self.root/'legacy.md').unlink()
        self.assertTrue(self.run_audit()['errors'])

    def test_path_escape(self):
        self.row['archive_path'] = '../outside'
        self.assertTrue(any('outside root' in x for x in self.run_audit()['errors']))

    def test_duplicate(self):
        self.assertTrue(any('duplicate' in x for x in self.run_audit([self.row, self.row])['errors']))

    def test_invalid_git(self):
        self.row['source_commit'] = '0' * 40
        self.assertTrue(any('Missing Git' in x for x in self.run_audit()['errors']))

    def test_invalid_manifest(self):
        (self.root/'manifest.json').write_text('{')
        self.assertTrue(module.audit(self.root, 'manifest.json')['errors'])


if __name__ == '__main__':
    unittest.main()
