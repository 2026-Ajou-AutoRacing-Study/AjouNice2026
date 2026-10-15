import json
from pathlib import Path
import sys
import tempfile
import unittest
from argparse import Namespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import launch_base5_obstacle_support as launch
from base5_capture_safety import atomic_json,sha256


class LaunchTests(unittest.TestCase):
    def fixture(self,root):
        manifest=root/'manifest.json'
        atomic_json(manifest,dict(count=5000,config=dict(obstacle_support_policy={'policy':'test'})))
        runtime=root/'runtime.py';runtime.write_text('example')
        release=root/'release.json'
        atomic_json(release,dict(schema='base5_obstacle_support_release_v1',qualified_environments=['sunny13'],
            completion_live_qualified=True,bulk_ready=True,
            runtime_hashes={str(runtime):sha256(runtime)},operational_storage_budget_gib=35,
            environments={'sunny13':dict(manifest=str(manifest),manifest_sha256=sha256(manifest),capture_output=str(root/'output'))}))
        return release,runtime

    def test_default_is_prepare_only_and_cannot_capture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,_=self.fixture(root)
            a=Namespace(release=release,environment='sunny13',output=None,run=False,
                        confirmed_environment=None,complete=False,stop_after_chunks=0)
            with patch.object(launch.fresh,'run',return_value=0) as fresh,patch.object(launch.completion,'execute') as finish:
                self.assertEqual(launch.run(a),0)
                self.assertTrue(fresh.call_args[0][0].prepare_only);finish.assert_not_called()

    def test_unqualified_weather_and_runtime_change_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,runtime=self.fixture(root)
            with self.assertRaises(ValueError):launch.preflight(release,'foggy13')
            runtime.write_text('changed')
            with self.assertRaises(ValueError):launch.preflight(release,'sunny13')

    def test_run_requires_manual_ui_confirmation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,_=self.fixture(root)
            a=Namespace(release=release,environment='sunny13',output=None,run=True,
                        confirmed_environment='foggy13',complete=False,stop_after_chunks=0)
            with patch.object(launch.fresh,'run') as fresh:
                with self.assertRaises(ValueError):launch.run(a)
                fresh.assert_not_called()

    def test_pending_live_probe_blocks_bulk_even_with_ui_confirmation(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release,_=self.fixture(root)
            doc=json.loads(release.read_text())
            doc.update(bulk_ready=False,bulk_blockers=['Long parked NPC probe pending'])
            atomic_json(release,doc)
            a=Namespace(release=release,environment='sunny13',output=None,run=True,
                        confirmed_environment='sunny13',complete=True,stop_after_chunks=0)
            with patch.object(launch.fresh,'run') as fresh,patch.object(launch.completion,'execute') as finish:
                with self.assertRaisesRegex(ValueError,'Long parked NPC probe pending'):launch.run(a)
                fresh.assert_not_called();finish.assert_not_called()
            self.assertFalse((root/'output').exists())


if __name__=='__main__':unittest.main()
