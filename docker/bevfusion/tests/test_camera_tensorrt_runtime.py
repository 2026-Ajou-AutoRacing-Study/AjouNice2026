"""CPU-only explicit runtime wiring, readiness and resource-lifetime tests."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'docker/bevfusion'))
from app.camera_tensorrt import validate_camera_tensorrt_options
from app.inference_server import InferenceServer, parse_args
from app import inference_server
from app.protocol import TYPE_ERROR, TYPE_PING, TYPE_REQUEST
from app.acceleration_startup import AccelerationExecutionError


def enabled_options():
    return dict(engine='explicit.engine', metadata='explicit.engine.json',
                base5_runtime=True, use_amp=True, parallel_camera=True,
                warmup_iterations=2, device='cuda:0')


def test_no_engine_preserves_all_previous_runtime_defaults():
    assert not validate_camera_tensorrt_options()
    assert validate_camera_tensorrt_options(**enabled_options())


@pytest.mark.parametrize('key,value', [('engine', None), ('metadata', None),
    ('base5_runtime', False), ('use_amp', False), ('parallel_camera', False),
    ('compile_parts', ['camera']), ('compile_parts', ['bev']), ('fuse_dense_bn', True),
    ('camera_cuda_graph', True), ('eager_camera_numerics', True),
    ('compile_reference_layout', True), ('temporal_feature_cache', True),
    ('projection_only', True), ('fp32_reference', True),
    ('device', 'cpu'), ('device', 'cuda'), ('warmup_iterations', 0)])
def test_incompatible_engine_options_fail_before_gpu_work(key, value):
    options = enabled_options(); options[key] = value
    with pytest.raises(ValueError): validate_camera_tensorrt_options(**options)


def test_cli_environment_is_explicit_and_does_not_select_weights(monkeypatch):
    base = ['--sensor-config', 'sensor', '--config', 'config', '--checkpoint', 'E24.pth']
    monkeypatch.delenv('BEVFUSION_CAMERA_TENSORRT_ENGINE', raising=False)
    monkeypatch.delenv('BEVFUSION_CAMERA_TENSORRT_METADATA', raising=False)
    args = parse_args(base)
    assert args.camera_tensorrt_engine is None and args.camera_tensorrt_metadata is None
    assert not args.pool_fused_layout and args.checkpoint == 'E24.pth'
    monkeypatch.setenv('BEVFUSION_CAMERA_TENSORRT_ENGINE', 'E12.engine')
    monkeypatch.setenv('BEVFUSION_CAMERA_TENSORRT_METADATA', 'E12.engine.json')
    args = parse_args(base)
    assert args.camera_tensorrt_engine == 'E12.engine' and args.checkpoint == 'E24.pth'
    args = parse_args(base+['--camera-tensorrt-engine', 'chosen.engine',
                           '--camera-tensorrt-metadata', 'chosen.json', '--pool-fused-layout'])
    assert args.camera_tensorrt_engine == 'chosen.engine' and args.pool_fused_layout


def test_blank_compose_acceleration_environment_preserves_no_engine_startup(monkeypatch):
    for name in ('BEVFUSION_CAMERA_TENSORRT_ENGINE', 'BEVFUSION_CAMERA_TENSORRT_METADATA',
                 'BEVFUSION_CAMERA_TENSORRT_POLICY'):
        monkeypatch.setenv(name, '')
    args = parse_args(['--sensor-config', 'sensor', '--config', 'config', '--checkpoint', 'E24.pth'])
    assert args.camera_tensorrt_engine is None and args.camera_tensorrt_metadata is None
    assert args.camera_tensorrt_policy is None and args.checkpoint == 'E24.pth'
    model = SimpleNamespace(box_center_origin='lidar_bottom_v1')
    factory = Mock(return_value=model)
    # Complete normal runtime arguments, retaining the actual parsed blank-env
    # options. No filesystem/GPU operation occurs in this wiring test.
    values = vars(server_args(camera_tensorrt_engine=args.camera_tensorrt_engine,
        camera_tensorrt_metadata=args.camera_tensorrt_metadata,
        camera_tensorrt_policy=args.camera_tensorrt_policy))
    with patch.dict(sys.modules, {'app.bevfusion_model': SimpleNamespace(BevFusionModel=factory)}), \
            patch('app.inference_server.load_sensor4_calibration', return_value={}), \
            patch('app.inference_server.file_sha256', return_value='selected-sha'):
        server = InferenceServer(SimpleNamespace(**values))
    assert factory.call_count == 1
    assert 'camera_tensorrt_engine' not in factory.call_args.kwargs
    assert server.camera_startup_report['policy'] == 'off'
    assert not server.camera_startup_report['requested']


def server_args(**kwargs):
    values = dict(sensor_config='sensor', camera_order=['front'], intrinsic_source='horizontal_fov',
        projection_only=False, amp=True, checkpoint='weights', config='config', device='cuda:0',
        preprocess_workers=4, warmup_iterations=2, temporal_feature_cache=False,
        temporal_maximum_age_sec=.3, base5_runtime=True, parallel_camera=True,
        camera_tensorrt_engine='explicit.engine', camera_tensorrt_metadata='explicit.json',
        pool_fused_layout=True)
    values.update(kwargs)
    return SimpleNamespace(**values)


def test_server_passes_only_explicit_optimization_and_reports_live_identity():
    execution = dict(camera_tensorrt=dict(enabled=True, loaded=True, healthy=True,
                                        engine_sha256='engine-sha'),
                     pool_fused_layout=dict(enabled=True), closed=False)
    model = SimpleNamespace(box_center_origin='lidar_bottom_v1', execution_status=lambda: execution)
    factory = Mock(return_value=model)
    with patch.dict(sys.modules, {'app.bevfusion_model': SimpleNamespace(BevFusionModel=factory)}), \
            patch('app.inference_server.load_sensor4_calibration', return_value={}), \
            patch('app.inference_server.file_sha256', side_effect=lambda path: 'sha-'+path):
        server = InferenceServer(server_args())
    supplied = factory.call_args.kwargs
    assert supplied['camera_tensorrt_engine'] == 'explicit.engine'
    assert supplied['camera_tensorrt_metadata'] == 'explicit.json'
    assert supplied['pool_fused_layout'] is True
    assert server.current_runtime_identity()['execution_optimization'] == execution
    assert server.current_runtime_identity()['checkpoint_sha256'] == 'sha-weights'


@pytest.mark.parametrize('loaded,healthy,ready', [(True, True, True), (False, True, False),
                                                (True, False, False)])
def test_engine_health_does_not_claim_ready_before_load_or_after_failure(loaded, healthy, ready):
    server = InferenceServer.__new__(InferenceServer)
    server._closed = False; server.fp32_reference = False
    server.calibration = {}; server.runtime_identity = dict(base5_runtime=True)
    server.model = SimpleNamespace(execution_status=lambda: dict(closed=False,
        camera_tensorrt=dict(enabled=True, loaded=loaded, healthy=healthy)))
    with patch('app.inference_server.recv_message', side_effect=[(TYPE_PING, {}, {}), EOFError()]), \
            patch('app.inference_server.serializable_calibration', return_value={}), \
            patch('app.inference_server.send_message') as send:
        server._serve_client(None)
    assert send.call_args.args[2]['ready'] is ready


def _model_lifecycle_class():
    # Execute the actual small lifecycle methods without importing Torch,
    # MMEngine or CUDA. Their bodies are not duplicated in this test.
    source = ast.parse((ROOT/'docker/bevfusion/app/bevfusion_model.py').read_text())
    body = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == 'BevFusionModel')
    methods = [node for node in body.body if isinstance(node, ast.FunctionDef) and
               node.name in ('close', 'execution_status')]
    module = ast.Module(body=[ast.ClassDef(name='Lifecycle', bases=[], keywords=[], body=methods,
                                         decorator_list=[])], type_ignores=[])
    scope = {}; exec(compile(ast.fix_missing_locations(module), '<actual-model-lifecycle>', 'exec'), scope)
    return scope['Lifecycle']


def test_model_close_restores_all_opt_ins_and_closes_preprocessor_once():
    model = _model_lifecycle_class()(); calls = []
    model._closed = False
    model.inference_optimization = SimpleNamespace(disable=lambda: calls.append('parallel'))
    model.camera_tensorrt = SimpleNamespace(disable=lambda: calls.append('camera'))
    model.pool_runtime_layout = SimpleNamespace(close=lambda: calls.append('pool'))
    model.preprocessor = SimpleNamespace(close=lambda: calls.append('preprocessor'))
    model.close(); model.close()
    assert calls == ['parallel', 'camera', 'pool', 'preprocessor'] and model._closed


def test_model_close_attempts_remaining_resources_after_one_cleanup_error():
    model = _model_lifecycle_class()(); calls = []
    model._closed = False
    def error():
        calls.append('parallel'); raise RuntimeError('cleanup failure')
    model.inference_optimization = SimpleNamespace(disable=error)
    model.camera_tensorrt = SimpleNamespace(disable=lambda: calls.append('camera'))
    model.pool_runtime_layout = SimpleNamespace(close=lambda: calls.append('pool'))
    model.preprocessor = SimpleNamespace(close=lambda: calls.append('preprocessor'))
    with pytest.raises(RuntimeError, match='cleanup failure'): model.close()
    assert calls == ['parallel', 'camera', 'pool', 'preprocessor'] and model._closed


def test_server_close_delegates_once():
    server = InferenceServer.__new__(InferenceServer)
    server._closed = False; close = Mock(); server.model = SimpleNamespace(close=close)
    server.close(); server.close()
    close.assert_called_once_with()


@pytest.mark.parametrize('error', [
    AccelerationExecutionError('camera_execution_failed', 'TRT enqueue failed'),
    RuntimeError('CUDA error: an illegal memory access was encountered'),
])
def test_nonrecoverable_execution_escapes_rpc_catch_and_requests_fresh_restart(error):
    server = InferenceServer.__new__(InferenceServer)
    server._handle_request = Mock(side_effect=error)
    with patch('app.inference_server.recv_message', return_value=(TYPE_REQUEST, {}, {})) as recv, \
            patch('app.inference_server.send_message') as send:
        with pytest.raises(AccelerationExecutionError, match='fresh process'):
            server._serve_client(None)
    assert recv.call_count == 1
    assert send.call_args.args[1] == TYPE_ERROR
    assert send.call_args.args[2]['restart_required'] is True


def test_bad_input_frame_does_not_disable_server_or_trigger_model_fallback():
    server = InferenceServer.__new__(InferenceServer)
    server._handle_request = Mock(side_effect=[ValueError('Bad camera order'), None])
    with patch('app.inference_server.recv_message', side_effect=[
            (TYPE_REQUEST, {}, {}), (TYPE_REQUEST, {}, {}), EOFError()]), \
            patch('app.inference_server.send_message') as send:
        server._serve_client(None)
    assert server._handle_request.call_count == 2
    assert send.call_count == 1 and send.call_args.args[1] == TYPE_ERROR
    assert 'restart_required' not in send.call_args.args[2]


def test_main_sigterm_unwinds_engine_lifecycle_without_real_signals(monkeypatch):
    args = SimpleNamespace(cpu_threads=None, opencv_threads=None, log_level='INFO')
    installed = []
    def install(signum, handler):
        installed.append((signum, handler))
        return 'original-handler'
    def run():
        installed[0][1](inference_server.signal.SIGTERM, None)
    close = Mock()
    monkeypatch.setattr(inference_server, 'parse_args', lambda: args)
    monkeypatch.setattr(inference_server.signal, 'signal', install)
    monkeypatch.setattr(inference_server, 'InferenceServer',
                        lambda _: SimpleNamespace(serve_forever=run, close=close))
    with pytest.raises(SystemExit) as stopped: inference_server.main()
    assert stopped.value.code == 0
    close.assert_called_once_with()
    assert installed[-1][1] == 'original-handler'
