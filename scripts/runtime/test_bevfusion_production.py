import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from bevfusion_production_profile import profile,digest,ROOT,resolve_checkpoint
from prepare_bevfusion_production import prepare

sys.path.insert(0,str(ROOT/'docker/bevfusion'))


class ProductionTests(unittest.TestCase):
    def test_prepared_camera_reuse_checks_engine_or_truthful_startup_fallback(self):
        spec=importlib.util.spec_from_file_location('engine_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_camera_engine_sha256='engine')
        execution=dict(camera_tensorrt=dict(enabled=True,loaded=True,healthy=True,engine_sha256='engine'),
                       camera_tensorrt_startup=dict(fallback=False))
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[],execution_optimization=execution),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',image_final_dim=[512,1056],
            classes=['vehicle','pedestrian','obstacle'])
        module.validate_production_identity(metadata,args)
        for key,value in [('engine_sha256','other'),('loaded',False),('healthy',False)]:
            bad=copy.deepcopy(metadata);bad['runtime_identity']['execution_optimization']['camera_tensorrt'][key]=value
            with self.assertRaises(ValueError):module.validate_production_identity(bad,args)
        execution['camera_tensorrt']=dict(enabled=False)
        with self.assertRaises(ValueError):module.validate_production_identity(metadata,args)
        execution['camera_tensorrt_startup']['fallback']=True
        module.validate_production_identity(metadata,args)

    def test_legacy_env_only_selects_e25_and_explicit_override_is_preserved(self):
        env={'BEVFUSION_INFERENCE_CHECKPOINT':'legacy-e25.pth'}
        self.assertNotEqual(resolve_checkpoint('e24',env),'legacy-e25.pth')
        self.assertNotEqual(resolve_checkpoint('e12',env),'legacy-e25.pth')
        self.assertEqual(resolve_checkpoint('e25',env),'legacy-e25.pth')
        env['BEVFUSION_CHECKPOINT']='explicit.pth'
        self.assertEqual(resolve_checkpoint('e24',env),'explicit.pth')
        self.assertEqual(resolve_checkpoint('e12',env),'explicit.pth')
    def test_e12_default_and_explicit_rollbacks_have_distinct_contracts(self):
        a,b=profile(),profile('e25')
        self.assertEqual(a['model'],'e12')
        self.assertEqual(a['thresholds'],[.4,.1,.1])
        self.assertEqual(b['thresholds'],[.4,.15,.1])
        self.assertTrue(a['base5'] and a['parallel'])
        self.assertFalse(b['base5'] or b['parallel'])
        self.assertEqual(a['compile_parts'],[])
        self.assertNotEqual(a['checkpoint_path'],b['checkpoint_path'])
        self.assertEqual(a['sha256'],'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8')
        self.assertTrue(a['camera_tensorrt'] and a['head_attention_sdpa'] and a['head_cache_bev_pos'])
        self.assertFalse(a['pool_fused_layout'])
        old=profile('e24')
        self.assertNotEqual(a['checkpoint_path'],old['checkpoint_path'])
        self.assertEqual(old['sha256'],'84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746')
        for candidate in (old,b):
            self.assertFalse(candidate['camera_tensorrt'] or candidate['head_attention_sdpa'] or candidate['head_cache_bev_pos'])

    def test_profile_files_exist_and_e24_uses_canonical_topics(self):
        for name in ('e12','e24','e25'):
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

    def test_e12_prepare_without_source_never_downloads_old_e24_weights(self):
        with tempfile.TemporaryDirectory() as t:
            target=Path(t)/'weights/base5_e12_inference.pth'
            p=dict(profile('e12'),checkpoint_path=str(target))
            args=SimpleNamespace(model='e12',source_checkpoint=None,check_only=False)
            with patch('prepare_bevfusion_production.profile',return_value=p), \
                    patch('prepare_bevfusion_production.subprocess.run') as network:
                with self.assertRaisesRegex(ValueError,'install_base5_production.sh'):
                    prepare(args)
                network.assert_not_called()
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

    def test_readiness_checks_intrinsic_source_and_camera_order(self):
        spec=importlib.util.spec_from_file_location('calibration_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        order=['camera_front','camera_front_left','camera_front_right','camera_back']
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_intrinsic_source='horizontal_fov',expected_camera_order=order)
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[]),
            calibration=dict(config_sha256='sensor',intrinsic_source='horizontal_fov',
                cameras=[dict(frame_id=name) for name in order]),
            point_intensity_source='ros255',image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        module.validate_production_identity(metadata,args)
        bad=copy.deepcopy(metadata);bad['calibration']['intrinsic_source']='focal_length_pixel'
        with self.assertRaisesRegex(ValueError,'intrinsic-source'):module.validate_production_identity(bad,args)
        bad=copy.deepcopy(metadata);bad['calibration']['cameras'].reverse()
        with self.assertRaisesRegex(ValueError,'camera order'):module.validate_production_identity(bad,args)

    def test_safe_backend_requires_actual_eager_not_a_claimed_fallback_label(self):
        spec=importlib.util.spec_from_file_location('backend_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_camera_backend='eager')
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[]),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',
            image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        # Existing no-engine identities remain valid without any new fields.
        module.validate_production_identity(metadata,args)
        metadata['runtime_identity']['execution_optimization']=dict(
            camera_tensorrt=dict(enabled=False),
            camera_tensorrt_startup=dict(policy='prefer',fallback=True,active_backend='eager'))
        module.validate_production_identity(metadata,args)
        for camera in (dict(enabled=True,loaded=True,healthy=True),
                       dict(enabled=True,loaded=False,healthy=False)):
            bad=copy.deepcopy(metadata)
            bad['runtime_identity']['execution_optimization']['camera_tensorrt']=camera
            with self.assertRaisesRegex(ValueError,'does not use a TensorRT camera'):
                module.validate_production_identity(bad,args)
        # Optional fallback status must never waive the selected weight/config.
        for key in ('checkpoint_sha256','config_sha256'):
            bad=copy.deepcopy(metadata);bad['runtime_identity'][key]='unselected-model'
            with self.assertRaises(ValueError):module.validate_production_identity(bad,args)

    def test_safe_pool_identity_rejects_preexisting_optional_fused_pool(self):
        spec=importlib.util.spec_from_file_location('safe_pool_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_camera_backend='eager',expected_pool_fused_layout='false')
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[]),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',
            image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        # No new optimization fields on a legacy baseline still means disabled.
        module.validate_production_identity(metadata,args)
        metadata['runtime_identity']['execution_optimization']=dict(
            camera_tensorrt=dict(enabled=False),pool_fused_layout=dict(enabled=False))
        module.validate_production_identity(metadata,args)
        bad=copy.deepcopy(metadata)
        bad['runtime_identity']['execution_optimization']['pool_fused_layout']['enabled']=True
        with self.assertRaises(ValueError):module.validate_production_identity(bad,args)

    def test_head_readiness_allows_legacy_baseline_and_rejects_enabled_safe_head(self):
        spec=importlib.util.spec_from_file_location('head_safe_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_head_attention_sdpa='false',expected_head_cache_bev_pos='false')
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[]),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',
            image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        module.validate_production_identity(metadata,args)
        metadata['runtime_identity']['execution_optimization']=dict(head_runtime=dict(
            requested=dict(attention_sdpa=False,cache_bev_pos=False),enabled=False))
        module.validate_production_identity(metadata,args)
        bad=copy.deepcopy(metadata)
        bad['runtime_identity']['execution_optimization']['head_runtime']['enabled']=True
        with self.assertRaisesRegex(ValueError,'does not use an optimized head'):
            module.validate_production_identity(bad,args)
        bad=copy.deepcopy(metadata)
        bad['runtime_identity']['execution_optimization']['head_runtime']['requested']['attention_sdpa']=True
        bad['runtime_identity']['execution_optimization']['head_runtime'].update(
            fallback=True,original_head_preserved=True)
        # A previously declined request already left the original head in
        # place. Safe startup must not reject its historical requested=True.
        module.validate_production_identity(bad,args)

    def test_matching_head_fallback_is_accepted_but_different_requests_are_not(self):
        spec=importlib.util.spec_from_file_location('head_request_health',ROOT/'docker/bevfusion/scripts/healthcheck.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(expected_checkpoint='pinned',expected_base5='true',expected_config='config',
            expected_sensor='sensor',expected_head_attention_sdpa='true',expected_head_cache_bev_pos='true')
        metadata=dict(model_loaded=True,runtime_identity=dict(checkpoint_sha256='pinned',config_sha256='config',
            base5_runtime=True,amp=True,parallel_camera=True,compile_parts=[],execution_optimization=dict(
                head_runtime=dict(requested=dict(attention_sdpa=True,cache_bev_pos=True),enabled=False,
                    fallback=True,reason_code='head_source_incompatible',original_head_preserved=True))),
            calibration=dict(config_sha256='sensor'),point_intensity_source='ros255',
            image_final_dim=[512,1056],classes=['vehicle','pedestrian','obstacle'])
        # Startup decline preserves the exact requested flags but keeps the
        # original head. Requiring active=True here would defeat safe fallback.
        module.validate_production_identity(metadata,args)
        active=copy.deepcopy(metadata)
        active['runtime_identity']['execution_optimization']['head_runtime'].update(enabled=True,fallback=False)
        module.validate_production_identity(active,args)
        for field in ('attention_sdpa','cache_bev_pos'):
            bad=copy.deepcopy(metadata)
            bad['runtime_identity']['execution_optimization']['head_runtime']['requested'][field]=False
            with self.assertRaisesRegex(ValueError,'request differs'):
                module.validate_production_identity(bad,args)
        bad=copy.deepcopy(metadata);bad['runtime_identity']['checkpoint_sha256']='unselected-model'
        with self.assertRaises(ValueError):module.validate_production_identity(bad,args)


if __name__=='__main__':unittest.main()
