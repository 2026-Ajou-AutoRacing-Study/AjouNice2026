import copy
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from bevfusion_production_profile import profile,digest,ROOT,resolve_checkpoint
from prepare_bevfusion_production import prepare


class ProductionTests(unittest.TestCase):
    def test_legacy_env_only_selects_e25_and_explicit_override_is_preserved(self):
        env={'BEVFUSION_INFERENCE_CHECKPOINT':'legacy-e25.pth'}
        self.assertNotEqual(resolve_checkpoint('e24',env),'legacy-e25.pth')
        self.assertEqual(resolve_checkpoint('e25',env),'legacy-e25.pth')
        env['BEVFUSION_CHECKPOINT']='explicit.pth'
        self.assertEqual(resolve_checkpoint('e24',env),'explicit.pth')
    def test_e24_default_and_explicit_rollback_have_distinct_contracts(self):
        a,b=profile(),profile('e25')
        self.assertEqual(a['model'],'e24')
        self.assertEqual(a['thresholds'],[.4,.1,.1])
        self.assertEqual(b['thresholds'],[.4,.15,.1])
        self.assertTrue(a['base5'] and a['parallel'])
        self.assertFalse(b['base5'] or b['parallel'])
        self.assertEqual(a['compile_parts'],[])
        self.assertNotEqual(a['checkpoint_path'],b['checkpoint_path'])

    def test_profile_files_exist_and_e24_uses_canonical_topics(self):
        for name in ('e24','e25'):
            p=profile(name)
            for k in ('model_config','bridge_config','sensor'):self.assertTrue((ROOT/p[k]).is_file())
        launch=ET.parse(ROOT/'base_ws/src/perception/bevfusion_ros/launch/base5_production.launch').getroot()
        args={r.attrib['name']:r.attrib.get('default') for r in launch.findall('arg')}
        self.assertEqual(args['output_objects'],'/perception/object_recognition/detection/objects')
        self.assertEqual(args['box_center_output_origin'],'gravity_center_v1')
        self.assertTrue(profile()['model_config'].endswith('nearest_depth_runtime.py'))
        inc={r.attrib['name']:r.attrib['value'] for r in launch.find('include').findall('arg')}
        self.assertEqual(inc['pedestrian_score_threshold'],'0.10')

    def test_local_install_checks_hash_and_idempotently_preserves_file(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);src=root/'source';src.write_bytes(b'trusted');target=root/'ckpts/E24.pth'
            p=dict(checkpoint_path=str(target),sha256=digest(src),checkpoint=target.name)
            a=SimpleNamespace(model='e24',source_checkpoint=src,check_only=False)
            with patch('prepare_bevfusion_production.profile',return_value=p):
                prepare(a);self.assertEqual(target.read_bytes(),src.read_bytes())
                before=target.stat().st_mtime_ns;prepare(a)
                self.assertEqual(before,target.stat().st_mtime_ns)
                target.write_bytes(b'user-owned')
                with self.assertRaises(ValueError):prepare(a)
                self.assertEqual(target.read_bytes(),b'user-owned')

    def test_wrong_source_is_not_installed(self):
        with tempfile.TemporaryDirectory() as t:
            src=Path(t)/'source';src.write_bytes(b'wrong');target=Path(t)/'weights/E24.pth'
            p=dict(checkpoint_path=str(target),sha256='wrong-hash',checkpoint=target.name)
            a=SimpleNamespace(model='e24',source_checkpoint=src,check_only=False)
            with patch('prepare_bevfusion_production.profile',return_value=p):
                with self.assertRaises(ValueError):prepare(a)
            self.assertFalse(target.exists())

    def test_readiness_rejects_wrong_weights_options_intensity_or_classes(self):
        spec=importlib.util.spec_from_file_location('production_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        a=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',expected_sensor='sensor')
        m=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',base5_runtime=True,
            amp=True,parallel_camera=True,compile_parts=[]),calibration=dict(config_sha256='sensor'),
            point_intensity_source='ros255',image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        module.validate_production_identity(m,a)
        for key,value in [('point_intensity_source','native'),('classes',['vehicle','obstacle','pedestrian']),
                          ('image_final_dim',[320,896]),('model_loaded',False)]:
            bad=copy.deepcopy(m);bad[key]=value
            with self.assertRaises(ValueError):module.validate_production_identity(bad,a)
        for key,value in [('checkpoint_sha256','E25'),('parallel_camera',False),('compile_parts',['camera']),
                          ('config_sha256','stale'),('base5_runtime',False),('amp',False)]:
            bad=copy.deepcopy(m);bad['runtime_identity'][key]=value
            with self.assertRaises(ValueError):module.validate_production_identity(bad,a)

    def test_explicit_corrected_runtime_rejects_legacy_missing_tags(self):
        spec=importlib.util.spec_from_file_location('strict_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',require_box_origin_tag=True,expected_depth_raster_policy='nearest_positive')
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',base5_runtime=True,
            amp=True,parallel_camera=True,compile_parts=[],box_center_origin='lidar_bottom_v1',depth_raster_policy='nearest_positive'),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',image_final_dim=[512,1056],
            classes=['vehicle','pedestrian','obstacle'])
        module.validate_production_identity(metadata,args)
        for key in ('box_center_origin','depth_raster_policy'):
            bad=copy.deepcopy(metadata);del bad['runtime_identity'][key]
            with self.assertRaises(ValueError):module.validate_production_identity(bad,args)


if __name__=='__main__':unittest.main()
