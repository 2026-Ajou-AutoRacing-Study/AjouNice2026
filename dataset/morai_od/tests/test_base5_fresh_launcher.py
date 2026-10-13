import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from argparse import Namespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import launch_base5_fresh as launch


class LauncherTests(unittest.TestCase):
    def fixture(self,root):
        manifest=root/'manifest.json';manifest.write_text('{}')
        suite=root/'suite.json';suite.write_text(json.dumps(dict(environments=[dict(
            environment='foggy13',manifest=str(manifest),manifest_sha256=launch.sha256(manifest),
            capture_output=str(root/'capture'),weather='foggy',hour=13)])))
        code=root/'runtime.py';code.write_text('frozen')
        release=root/'release.json';release.write_text(json.dumps(dict(collector_qualified=True,suite=str(suite),suite_sha256=launch.sha256(suite),
            runtime_hashes={str(code):launch.sha256(code)},recommended_chunk_size=16,
            recommended_setup_profile='static-ready',recommended_runtime_overlap=False)))
        return release,code,manifest
    def test_mutated_runtime_cannot_start(self):
        with tempfile.TemporaryDirectory() as d:
            release,code,_=self.fixture(Path(d));code.write_text('modified')
            with self.assertRaises(ValueError):launch.preflight(release,'foggy13','foggy13')
    def test_mutated_manifest_cannot_start(self):
        with tempfile.TemporaryDirectory() as d:
            release,_,manifest=self.fixture(Path(d));manifest.write_text('changed')
            with self.assertRaises(ValueError):launch.preflight(release,'foggy13','foggy13')
    def test_mutated_suite_cannot_redirect_output(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,_,_=self.fixture(root)
            suite=root/'suite.json';doc=json.loads(suite.read_text());doc['environments'][0]['capture_output']='elsewhere';suite.write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError,'suite changed'):launch.preflight(release,'foggy13','foggy13')
    def test_confirmation_must_match(self):
        with tempfile.TemporaryDirectory() as d:
            release,_,_=self.fixture(Path(d))
            with self.assertRaises(ValueError):launch.preflight(release,'foggy13','sunny13')
    def test_missing_ui_confirmation_never_launches(self):
        with tempfile.TemporaryDirectory() as d:
            release,_,_=self.fixture(Path(d));a=Namespace(release=release,environment='foggy13',confirmed_environment=None,prepare_only=False,complete=False)
            with patch.object(launch.fresh,'run') as capture:
                with self.assertRaises(ValueError):launch.run(a)
                capture.assert_not_called()
    def test_unqualified_completion_never_starts_bulk(self):
        with tempfile.TemporaryDirectory() as d:
            release,_,_=self.fixture(Path(d));a=Namespace(release=release,environment='foggy13',confirmed_environment='foggy13',prepare_only=False,complete=True)
            with patch.object(launch.fresh,'run') as capture:
                with self.assertRaises(ValueError):launch.run(a)
                capture.assert_not_called()
    def test_prepare_only_does_not_write_ui_confirmation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,_,_=self.fixture(root)
            a=Namespace(release=release,environment='foggy13',confirmed_environment=None,prepare_only=True,stop_after_chunks=0,complete=True)
            with patch.object(launch.fresh,'run',return_value=0) as capture,patch.object(launch.completion,'execute') as finish:
                self.assertEqual(launch.run(a),0);self.assertTrue(capture.call_args.args[0].prepare_only)
                self.assertFalse((root/'capture/operator_environment_confirmation.json').exists());finish.assert_not_called()


if __name__=='__main__':unittest.main()
