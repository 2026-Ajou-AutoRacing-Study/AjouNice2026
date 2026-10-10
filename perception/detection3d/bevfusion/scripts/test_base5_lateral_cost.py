import json
from pathlib import Path
import tempfile
import unittest

from compare_base5_lateral_cost import compare


class LateralCostTest(unittest.TestCase):
    def fixture(self, folder):
        samples = [dict(id=str(i), sample_group='representative' if i < 200 else 'stress')
                   for i in range(300)]
        for index, lateral in enumerate((30, 40, 40, 30), 1):
            case = dict(valid=True, shape='512x1056', mode='separated',
                        stage_instrumented=False, parameter_sha256='same-weights',
                        semantic_roi=[-100., -lateral, -3., 100., lateral, 2.],
                        voxel=[.075, .075, .2], depth=[1., 100., .5],
                        camera_bev=[-100.8, 100.8, .3],
                        camera_bev_ybound=[-32.4, 32.4, .3] if lateral == 30 else [-40.8, 40.8, .3],
                        sparse_shape=[864 if lateral == 30 else 1088, 2688, 41],
                        view_nx=[672, 216 if lateral == 30 else 272, 1],
                        steady_peak_allocated_mib=900, stages={},
                        per_sample=[dict(id=item['id'], prepare_ms=10, model_ms=lateral,
                                         service_ms=10+lateral, retained_points=100+lateral)
                                    for item in samples])
            report = dict(codec='jpeg90', lateral_range=lateral,
                          source=dict(source_sha256='same-source', samples=samples), cases=[case])
            (folder / ('r%d_y%d.json' % (index, lateral))).write_text(json.dumps(report))

    def test_distinct_scenes_and_repeated_measurements_are_separate(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.fixture(folder)
            report = compare(folder)
            self.assertEqual(report['distinct_scenes'], 300)
            self.assertEqual(report['measured_inferences'], 1200)
            group = report['summary']['40']['groups']['representative']
            self.assertEqual(group['unique_scenes'], 200)
            self.assertEqual(group['service_ms']['n'], 400)
            self.assertEqual(group['service_ms']['mean'], 50)

    def test_different_parameters_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.fixture(folder)
            path = folder / 'r2_y40.json'
            report = json.loads(path.read_text())
            report['cases'][0]['parameter_sha256'] = 'different-weights'
            path.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                compare(folder)

    def test_changed_sampling_interval_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.fixture(folder)
            path = folder / 'r2_y40.json'
            report = json.loads(path.read_text())
            report['cases'][0]['voxel'] = [.1, .1, .2]
            path.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                compare(folder)

    def test_old_geometry_in_widened_model_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.fixture(folder)
            path = folder / 'r2_y40.json'
            report = json.loads(path.read_text())
            report['cases'][0]['view_nx'] = [672, 216, 1]
            path.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                compare(folder)


if __name__ == '__main__':
    unittest.main()
