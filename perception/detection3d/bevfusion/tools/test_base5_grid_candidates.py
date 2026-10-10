"""CPU metric-grid contract tests, no model construction or CUDA required."""
import unittest
import numpy as np
from mmengine.config import Config
from pathlib import Path
from base5_grid_candidates import SPATIAL_CASES, candidate_config, geometry, validate_geometry


class GridCandidatesTest(unittest.TestCase):
    def setUp(self):
        self.base = Config.fromfile(str(Path(__file__).resolve().parents[1]/'configs/bevfusion_base5_dense_512_native_jpeg90.py'))

    def test_all_shapes_and_semantic_roi_unchanged(self):
        expected = {'current075':[108,336], 'common050':[168,504],
                    'common075':[112,336], 'common100':[84,252]}
        for case in SPATIAL_CASES:
            cfg = candidate_config(self.base, case)
            self.assertEqual(geometry(case)['fusion_shape_yx'], expected[case])
            self.assertEqual(list(cfg.point_cloud_range), list(self.base.point_cloud_range))
            self.assertEqual(list(cfg.model.view_transform.image_size), [512,1056])
            self.assertEqual(cfg.model.bbox_head.num_classes, 4)
            self.assertEqual(cfg.model.bbox_head.train_cfg.min_radius, self.base.model.bbox_head.train_cfg.min_radius)
        self.assertEqual(list(self.base.model.view_transform.ybound), [-32.4,32.4,.3])

    def test_nominal_camera_lidar_and_head_cell_indices_match(self):
        xy = np.random.RandomState(261011).uniform([-99.999,-29.999], [99.999,29.999], (20000,2))
        for case in SPATIAL_CASES:
            g = geometry(case); rel = xy-np.array(g['origin'])
            camera = np.floor(rel/(4*g['voxel'][0])).astype(int)//2
            sparse = np.floor(rel/g['voxel'][0]).astype(int)//8
            head = np.floor(rel/g['fusion_pitch_m']).astype(int)
            np.testing.assert_array_equal(camera, head)
            np.testing.assert_array_equal(sparse, head)

    def test_depth_bins_and_wrong_origins_rejected(self):
        for step,n in ((.25,396),(.5,198),(1.,99)):
            cfg = candidate_config(self.base,'current075',step)
            self.assertEqual(len(np.arange(*cfg.model.view_transform.dbound)),n)
        cfg = candidate_config(self.base,'common050')
        cfg.model.bbox_head.bbox_coder.pc_range[1] += .1
        with self.assertRaises(AssertionError): validate_geometry(cfg)
        with self.assertRaises(ValueError):candidate_config(self.base,'unknown')
        with self.assertRaises(ValueError):candidate_config(self.base,'current075',.3)
        cfg=candidate_config(self.base,'current075',.25,pool_cache_mib=512)
        self.assertEqual(cfg.model.view_transform.pool_cache_limit_bytes,512*2**20)
        with self.assertRaises(ValueError):candidate_config(self.base,'current075',.25,pool_cache_mib=999)


if __name__=='__main__':unittest.main()
