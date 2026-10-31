"""Warm repeats remain bounded single-parent probes, not cold/load evidence."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_redbarrel_warm_probe as prep
import test_base5_scale_probe_preparation as fixtures


class RedWarmPreparationTests(unittest.TestCase):
    def test_single18_preserves_scenario_bytes_across_three_repeats(self):
        harness=fixtures.ScaleProbePreparationTest(methodName='runTest')
        harness.setUp()
        self.addCleanup(harness.doCleanups)
        source,context=harness.fixture()
        source=harness.write(harness.root/'registered'/'manifest.json',json.loads(source.read_text()))
        for mocked in harness.mocks(context):
            mocked.start()
            self.addCleanup(mocked.stop)
        reference=harness.write(harness.root/'references'/'unit.json',dict(assets=dict(RedBarrel=dict(
            bbox_size_m=[.57,.57,.92],pivot_to_bbox_center_local_m=[0.,0.,.46]))))
        path=prep.prepare(source,harness.root/'warm',reference)
        value=json.loads(path.read_text())
        self.assertEqual(value['count'],18)
        self.assertEqual(len({row['scenario_sha256'] for row in value['scenes']}),6)
        self.assertEqual(len({row['id'] for row in value['scenes']}),18)
        self.assertFalse(value['redbarrel_warm_registration']['mixed_load_tested'])
        self.assertFalse(value['redbarrel_warm_registration']['process_restarted'])
        metas=[json.loads(Path(row['metadata']).read_text()) for row in value['scenes']]
        self.assertTrue(all(not meta['redbarrel_warm_probe']['independent_geometry_samples'] for meta in metas))

    def test_unbounded_repeats_rejected_before_filesystem_access(self):
        for repeats in (0,4,True):
            with self.subTest(repeats=repeats),self.assertRaises(ValueError):
                prep.prepare('/missing','/unused','/missing-unit',repeats=repeats)


if __name__=='__main__':
    unittest.main()
