"""CPU-only server origin declaration; no model loading or sockets."""

import ast
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from app.inference_server import InferenceServer

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('origin_healthcheck',
                                             ROOT/'docker/bevfusion/scripts/healthcheck.py')
healthcheck = importlib.util.module_from_spec(spec)
spec.loader.exec_module(healthcheck)


class ServerOriginTests(unittest.TestCase):
    def test_constructor_declares_origin_without_model_or_socket_work(self):
        args = SimpleNamespace(sensor_config='sensor', camera_order=['front'],
            intrinsic_source='horizontal_fov', projection_only=False, amp=True,
            checkpoint='weights', config='config', device='cuda:0',
            preprocess_workers=4, warmup_iterations=0, temporal_feature_cache=False,
            temporal_maximum_age_sec=.5, base5_runtime=True, parallel_camera=True,
            point_intensity_source='ros255')
        model = SimpleNamespace(box_center_origin='lidar_bottom_v1')
        factory = Mock(return_value=model)
        with patch.dict(sys.modules, {'app.bevfusion_model': SimpleNamespace(BevFusionModel=factory)}), \
                patch('app.inference_server.load_sensor4_calibration', return_value={}), \
                patch('app.inference_server.file_sha256', side_effect=lambda path: 'sha-'+path):
            server = InferenceServer(args)
        self.assertIs(server.model, model)
        self.assertEqual(server.runtime_identity['box_center_origin'], model.box_center_origin)
        self.assertEqual(server.runtime_identity['checkpoint_sha256'], 'sha-weights')
        self.assertEqual(factory.call_args[1]['point_intensity_source'], 'ros255')

    def test_actual_request_declares_raw_bottom_without_converting_boxes(self):
        server = InferenceServer.__new__(InferenceServer)
        server.calibration = dict(config_sha256='sensor', cameras=[dict(frame_id='front')],
                                  intrinsic_source='horizontal_fov', lidar_frame='lidar1')
        detections = [dict(center=[2., 3., -1.], size=[4., 2., 1.5], yaw=.2, score=.9,
                           class_name='vehicle', label=0)]
        server.model = SimpleNamespace(
            preprocessor=SimpleNamespace(final_dim=(512, 1056), point_intensity_source='ros255'),
            infer=lambda *args, **kwargs: (deepcopy(detections), 2., 3.),
            last_query_memory_diagnostics={}, last_runtime_timings={}, box_center_origin='lidar_bottom_v1')
        metadata = dict(sensor_config_sha256='sensor', camera_order=['front'],
                        intrinsic_source='horizontal_fov', point_count=1, point_dim=4,
                        point_intensity_source='ros255', image_encoding='compressed')
        with patch('app.inference_server.send_message') as send:
            server._handle_request(None, metadata, {'points': np.zeros((1, 4), '<f4').tobytes()})
        response = send.call_args[0][2]
        self.assertEqual(response['box_center_origin'], 'lidar_bottom_v1')
        self.assertEqual(response['detections'], detections)
        self.assertEqual(metadata.get('box_center_origin'), None)

    def test_production_health_accepts_legacy_and_rejects_changed_raw_origin(self):
        args = SimpleNamespace(expected_base5='true', expected_checkpoint='weights',
                               expected_config='config', expected_sensor=None)
        metadata = dict(model_loaded=True, runtime_identity=dict(checkpoint_sha256='weights',
            config_sha256='config', base5_runtime=True, amp=True, compile_parts=[], parallel_camera=True),
            point_intensity_source='ros255', image_final_dim=[512, 1056],
            classes=['vehicle', 'pedestrian', 'obstacle'])
        healthcheck.validate_production_identity(metadata, args)
        metadata['runtime_identity']['box_center_origin'] = 'lidar_bottom_v1'
        healthcheck.validate_production_identity(metadata, args)
        for value in ('gravity_center_v1', 'unknown', None):
            metadata['runtime_identity']['box_center_origin'] = value
            with self.assertRaisesRegex(ValueError, 'box-origin'):
                healthcheck.validate_production_identity(metadata, args)

    def test_model_still_exports_tensor_bottom_not_gravity(self):
        source = ROOT/'docker/bevfusion/app/bevfusion_model.py'
        tree = ast.parse(source.read_text())
        body = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BevFusionModel')
        infer = next(n for n in body.body if isinstance(n, ast.FunctionDef) and n.name == 'infer')
        attributes = [n.attr for n in ast.walk(infer) if isinstance(n, ast.Attribute)]
        self.assertIn('tensor', attributes)
        self.assertNotIn('gravity_center', attributes)
        self.assertIn("BOX_CENTER_ORIGIN = 'lidar_bottom_v1'", source.read_text())


if __name__ == '__main__':
    unittest.main()
