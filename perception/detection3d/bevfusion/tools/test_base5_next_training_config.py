"""CPU-only next-training config checks; no model, registry or CUDA loading."""

from fractions import Fraction
import json
from pathlib import Path
import sys
import unittest

from mmengine.config import Config


ROOT = Path(__file__).resolve().parents[4]
CONFIGS = ROOT / 'perception/detection3d/bevfusion/configs'
NEXT_CONFIG = CONFIGS / 'bevfusion_base5_fine_512_depth25_next_training.py'
SEMANTIC_ROI = [-100., -30., -3., 100., 30., 2.]
CLASSES = ('vehicle', 'pedestrian', 'obstacle')


def load_config(name):
    # Preparation must not import project registries or compiled extensions.
    return Config.fromfile(str(CONFIGS / name), import_custom_modules=False)


class NextTrainingConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(NEXT_CONFIG.name)
        cls.e24_training = load_config(
            'bevfusion_base5_fine_512_depth25_pedestrian_merged.py')
        cls.production = load_config(
            'bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py')

    def test_fresh_eager_policy_does_not_rewrite_completed_e24(self):
        cfg = self.config
        self.assertEqual(cfg.required_next_training_policy,
                         'base5_eager_nearest_v1')
        self.assertIs(cfg.model.compile_camera, False)
        self.assertIsNone(cfg.load_from)
        self.assertTrue(self.e24_training.model.compile_camera)
        self.assertIs(self.production.model.compile_camera, False)
        self.assertNotIn('required_next_training_policy', self.e24_training)
        self.assertNotIn('required_next_training_policy', self.production)

    def test_nearest_is_the_shared_training_and_validation_model(self):
        model = self.config.model
        self.assertEqual(model.type, 'MoraiTrainingRuntimeBEVFusion')
        self.assertEqual(model.view_transform.type,
                         'MoraiDepthRasterSeparatedDepthLSSTransform')
        self.assertEqual(model.view_transform.depth_raster_policy,
                         'nearest_positive')
        self.assertEqual(model.view_transform.depth_raster_backend, 'sorted')
        self.assertEqual(model, self.production.model)
        self.assertIn(
            'perception.detection3d.bevfusion.plugin.morai_depth_raster',
            self.config.custom_imports.imports)
        # There is one model/view transform, not a separate legacy-hint model
        # hidden in either loader or evaluator.
        for split in ('train', 'val', 'test'):
            self.assertNotIn('model', self.config[split + '_dataloader'])

    def test_three_classes_preserve_animal_provenance_policy(self):
        cfg = self.config
        self.assertEqual(tuple(cfg.classes), CLASSES)
        self.assertEqual(tuple(cfg.class_names), CLASSES)
        self.assertEqual(cfg.model.bbox_head.num_classes, len(CLASSES))
        self.assertEqual(cfg.required_training_label_policy,
                         'animal_to_pedestrian_v1')
        for split in ('train', 'val', 'test'):
            dataset = cfg[split + '_dataloader'].dataset
            self.assertEqual(dataset.type, 'MoraiBase5PedestrianDataset')
            self.assertEqual(tuple(dataset.metainfo.classes), CLASSES)
            for item in dataset.pipeline:
                if item.type == 'ObjectNameFilter':
                    self.assertEqual(tuple(item.classes), CLASSES)
        self.assertEqual(cfg.val_evaluator.type, 'MoraiBase5MergedCenterMetric')

    def test_native_jpeg_upright_and_uint8_pipeline_contracts(self):
        cfg = self.config
        self.assertEqual(cfg.required_camera_input_policy, 'native_jpeg90')
        self.assertEqual(cfg.required_box_geometry_policy,
                         'native_rpy_fixed_yaw_enclosure_v1')
        for split in ('train', 'val', 'test'):
            pipeline = cfg[split + '_dataloader'].dataset.pipeline
            kinds = [item.type for item in pipeline]
            self.assertNotIn('MoraiOnlineImageDomainAug3D', kinds)
            self.assertNotIn('MoraiUseNativeJPEGCache', kinds)
            images = next(item for item in pipeline
                          if item.type == 'BEVLoadMultiViewImageFromFiles')
            self.assertFalse(images.to_float32)
            self.assertEqual(images.num_views, 4)
            augment = next(item for item in pipeline
                           if item.type == 'MoraiImageAug3D')
            self.assertEqual(augment.output_dtype, 'uint8')
            self.assertEqual(list(augment.final_dim), [512, 1056])
            self.assertEqual(list(augment.resize_lim), [.825, .825])
            self.assertFalse(augment.rand_flip)
            points_index = kinds.index('LoadPointsFromFile')
            self.assertEqual(kinds[points_index + 1], 'MoraiFilterInvalidPoints')
            points = pipeline[points_index]
            self.assertEqual(points.coord_type, 'LIDAR')
            self.assertEqual(points.load_dim, 4)
            self.assertEqual(points.use_dim, 4)
            self.assertEqual(pipeline,
                             self.e24_training[split + '_dataloader'].dataset.pipeline)

    def test_single_frame_seven_dimensional_boxes_without_velocity(self):
        head = self.config.model.bbox_head
        self.assertNotIn('vel', head.common_heads)
        self.assertEqual(set(head.common_heads), {'center', 'height', 'dim', 'rot'})
        self.assertEqual(head.bbox_coder.code_size, 8)  # sin/cos yaw -> 7D box
        for split in ('train', 'val', 'test'):
            for item in self.config[split + '_dataloader'].dataset.pipeline:
                self.assertNotIn('Sweep', item.type)
                self.assertNotIn('Temporal', item.type)
        self.assertFalse(self.config.get('compile', False))

    def test_metric_xy_axes_and_camera_lidar_head_alignment(self):
        cfg = self.config
        voxel = cfg.model.data_preprocessor.voxelize_cfg
        camera = cfg.model.view_transform
        head = cfg.model.bbox_head
        self.assertEqual(list(cfg.point_cloud_range), SEMANTIC_ROI)
        self.assertEqual(list(head.bbox_coder.post_center_range), SEMANTIC_ROI)
        self.assertEqual(list(voxel.voxel_size), [.05, .05, .2])
        self.assertEqual(list(voxel.point_cloud_range),
                         [-100.8, -33.6, -5., 100.8, 33.6, 3.])
        self.assertEqual(cfg.model.pts_middle_encoder.type,
                         'MoraiRectangularSparseEncoder')
        self.assertEqual(list(cfg.model.pts_middle_encoder.sparse_shape),
                         [1344, 4032, 41])
        self.assertEqual(head.type, 'MoraiRectangularTransFusionHead')
        self.assertEqual(list(head.train_cfg.grid_size), [4032, 1344, 41])
        self.assertEqual(camera.downsample, 2)
        self.assertEqual(list(camera.image_size), [512, 1056])
        self.assertEqual(list(camera.feature_size), [64, 132])
        fusion_shape_yx = []
        for axis, bound in enumerate((camera.xbound, camera.ybound)):
            lo, hi, camera_pitch = (Fraction(str(value)) for value in bound)
            lidar_pitch = Fraction(str(voxel.voxel_size[axis]))
            self.assertEqual(camera_pitch, 4 * lidar_pitch)
            self.assertEqual(lo, Fraction(str(voxel.point_cloud_range[axis])))
            self.assertEqual(hi, Fraction(str(voxel.point_cloud_range[axis + 3])))
            bins = (hi - lo) / lidar_pitch
            self.assertEqual(bins.denominator, 1)
            self.assertEqual(int(bins) % 16, 0)
            for role in (head.train_cfg, head.test_cfg, head.bbox_coder):
                self.assertEqual(role.out_size_factor, 8)
                self.assertEqual(Fraction(str(role.voxel_size[axis])), lidar_pitch)
            for role in (head.test_cfg, head.bbox_coder):
                self.assertEqual(Fraction(str(role.pc_range[axis])), lo)
            self.assertEqual(Fraction(str(head.train_cfg.point_cloud_range[axis])), lo)
            self.assertEqual(head.train_cfg.grid_size[axis], int(bins))
            self.assertEqual(head.test_cfg.grid_size[axis], int(bins))
            self.assertEqual(camera_pitch * camera.downsample,
                             lidar_pitch * head.train_cfg.out_size_factor)
            fusion_shape_yx.insert(0, int(bins) // head.train_cfg.out_size_factor)
        self.assertEqual(fusion_shape_yx, [168, 504])

    def test_depth_bins_and_training_optimizations_remain_explicit(self):
        cfg = self.config
        self.assertEqual(list(cfg.model.view_transform.dbound), [1., 100., .25])
        self.assertEqual(Fraction('99') / Fraction('.25'), 396)
        self.assertEqual(cfg.model.view_transform.pool_backward_mode, 'warp')
        self.assertTrue(cfg.model.view_transform.pool_compact_indices)
        self.assertEqual(cfg.model.view_transform.pool_geometry_mode,
                         'batched_checked')
        self.assertTrue(cfg.optim_wrapper.optimizer.fused)
        self.assertFalse(cfg.optim_wrapper.optimizer.foreach)
        self.assertEqual(cfg.optim_wrapper, self.e24_training.optim_wrapper)

    def test_e24_release_source_hashes_are_still_valid(self):
        # This reads only source files. It does not download/install weights or
        # access credentials, ROS, Docker, CUDA or the network.
        sys.path.insert(0, str(ROOT / 'scripts/runtime'))
        try:
            import base5_production_bundle as bundle
            descriptor = json.loads(bundle.DESCRIPTOR.read_text())
            bundle.validate_descriptor(descriptor, root=ROOT)
            required = set(bundle.required_sources(ROOT))
        finally:
            sys.path.pop(0)
        self.assertNotIn(NEXT_CONFIG.relative_to(ROOT).as_posix(), required)
        self.assertNotIn(Path(__file__).resolve().relative_to(ROOT).as_posix(), required)


if __name__ == '__main__':
    unittest.main()
