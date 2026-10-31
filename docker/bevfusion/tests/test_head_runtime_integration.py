"""CPU-only head opt-in wiring; real head math/AP is qualified separately."""
import ast
import logging
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'docker/bevfusion'))
from app.inference_server import InferenceServer, parse_args
from app.acceleration_startup import AccelerationExecutionError
from app.protocol import TYPE_REQUEST


def runtime_class():
    # Execute the actual small installation/lifecycle methods without importing
    # Torch/MMEngine or duplicating the production implementation in the test.
    source = ast.parse((ROOT / 'docker/bevfusion/app/bevfusion_model.py').read_text())
    cls = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == 'BevFusionModel')
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef) and
               node.name in ('_configure_head_runtime', 'execution_status', 'close')]
    module = ast.Module(body=[ast.ClassDef(name='Runtime', bases=[], keywords=[],
        body=methods, decorator_list=[])], type_ignores=[])
    scope = dict(LOGGER=logging.getLogger('bevfusion_server'))
    exec(compile(ast.fix_missing_locations(module), '<actual-head-runtime-integration>', 'exec'), scope)
    return scope['Runtime']


def runtime():
    value = runtime_class()()
    value.model = SimpleNamespace(bbox_head=object())
    value.head_runtime_optimization = None
    value.head_runtime_report = None
    value.camera_tensorrt = value.pool_runtime_layout = None
    value.inference_optimization = value.preprocessor = None
    value._closed = False
    return value


class Unavailable(ValueError):
    pass


def helper(installer):
    return SimpleNamespace(HeadOptimizationUnavailableError=Unavailable,
                           install_head_runtime_optimization=installer)


def test_model_constructor_defaults_off_and_no_topk_api():
    source = ast.parse((ROOT / 'docker/bevfusion/app/bevfusion_model.py').read_text())
    cls = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == 'BevFusionModel')
    init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    defaults = dict(zip([arg.arg for arg in init.args.args][-len(init.args.defaults):], init.args.defaults))
    assert ast.literal_eval(defaults['head_attention_sdpa']) is False
    assert ast.literal_eval(defaults['head_cache_bev_pos']) is False
    assert not any('topk' in arg.arg for arg in init.args.args)
    calls = [node for node in ast.walk(init) if isinstance(node, ast.Call) and
             isinstance(node.func, ast.Attribute) and node.func.attr == '_configure_head_runtime']
    assert len(calls) == 1


def test_default_does_not_import_optional_helper_or_touch_the_head():
    value = runtime(); selected = value.model.bbox_head
    with patch.dict(sys.modules, {'app.head_runtime_optimization': None}):
        value._configure_head_runtime(base5_runtime=True)
    assert value.model.bbox_head is selected and value.head_runtime_optimization is None
    report = value.execution_status()['head_runtime']
    assert not report['enabled'] and not report['fallback']
    assert report['reason_code'] == 'not_requested' and not any(report['requested'].values())
    assert report['original_head_preserved'] and not report['checkpoint_changed']


def test_reconfiguration_cannot_misreport_an_active_head_as_original_fallback():
    value = runtime()
    owner = object()
    value.head_runtime_optimization = owner
    value.head_runtime_report = dict(enabled=True)
    for flags in ((False, False), (True, True)):
        with pytest.raises(RuntimeError, match='startup-only'):
            value._configure_head_runtime(*flags, base5_runtime=True)
        assert value.head_runtime_optimization is owner
        assert value.head_runtime_report == dict(enabled=True)


@pytest.mark.parametrize('sdpa,cache', [(True, False), (False, True), (True, True)])
def test_explicit_flags_install_only_selected_features_and_never_topk(sdpa, cache):
    value = runtime(); selected = value.model.bbox_head
    handle = SimpleNamespace(report=dict(attention_sdpa=sdpa, cache_bev_pos=cache, proposal_topk=False),
                             close=Mock())
    installer = Mock(return_value=handle)
    with patch.dict(sys.modules, {'app.head_runtime_optimization': helper(installer)}):
        value._configure_head_runtime(sdpa, cache, True)
    installer.assert_called_once_with(selected, proposal_topk=False, attention_sdpa=sdpa, cache_bev_pos=cache)
    assert value.head_runtime_optimization is handle and value.model.bbox_head is selected
    report = value.execution_status()['head_runtime']
    assert report['enabled'] and not report['fallback'] and report['startup_only']
    assert report['active_flags'] == dict(attention_sdpa=sdpa, cache_bev_pos=cache, proposal_topk=False)
    assert not report['checkpoint_changed'] and not report['config_changed']
    value.close(); value.close()
    handle.close.assert_called_once_with()
    assert not value.execution_status()['head_runtime']['enabled']


def test_only_named_source_guard_failure_keeps_same_original_head(caplog):
    value = runtime(); selected = value.model.bbox_head
    installer = Mock(side_effect=Unavailable('Upstream head AST changed'))
    with patch.dict(sys.modules, {'app.head_runtime_optimization': helper(installer)}):
        value._configure_head_runtime(True, True, True)
    assert value.model.bbox_head is selected and value.head_runtime_optimization is None
    report = value.execution_status()['head_runtime']
    assert report['fallback'] and not report['enabled'] and report['original_head_preserved']
    assert report['reason_code'] == 'head_source_incompatible'
    assert 'AST changed' in report['reason'] and 'SAME original head' in caplog.text
    assert not any(report['active_flags'].values())


def test_missing_helper_or_unsupported_runtime_does_not_block_baseline(caplog):
    for base5, reason in [(True, 'head_helper_unavailable'), (False, 'head_runtime_incompatible')]:
        value = runtime(); selected = value.model.bbox_head
        with patch.dict(sys.modules, {'app.head_runtime_optimization': None}):
            value._configure_head_runtime(True, True, base5)
        assert value.model.bbox_head is selected and value.head_runtime_optimization is None
        assert value.execution_status()['head_runtime']['reason_code'] == reason
    assert 'SAME original head' in caplog.text


@pytest.mark.parametrize('error', [RuntimeError('CUDA error: device-side assert triggered'),
    RuntimeError('Unexpected installer error'), ValueError('Not a named source guard'),
    AccelerationExecutionError('head_execution_failed', 'Already enqueued')])
def test_unknown_model_or_cuda_errors_are_not_reclassified_as_optional(error):
    value = runtime()
    with patch.dict(sys.modules, {'app.head_runtime_optimization': helper(Mock(side_effect=error))}):
        with pytest.raises(type(error)) as failure:
            value._configure_head_runtime(True, True, True)
    assert failure.value is error and value.head_runtime_optimization is None


def test_cli_default_off_env_opt_in_and_no_topk_switch(monkeypatch, caplog):
    base = ['--sensor-config', 'sensor', '--config', 'config', '--checkpoint', 'E24.pth']
    for key in ('BEVFUSION_HEAD_ATTENTION_SDPA', 'BEVFUSION_HEAD_CACHE_BEV_POS'):
        monkeypatch.setenv(key, '')
    args = parse_args(base)
    assert not args.head_attention_sdpa and not args.head_cache_bev_pos
    assert args.checkpoint == 'E24.pth' and not any('topk' in key for key in vars(args))
    monkeypatch.setenv('BEVFUSION_HEAD_ATTENTION_SDPA', 'true')
    monkeypatch.setenv('BEVFUSION_HEAD_CACHE_BEV_POS', 'true')
    args = parse_args(base)
    assert args.head_attention_sdpa and args.head_cache_bev_pos
    monkeypatch.setenv('BEVFUSION_HEAD_ATTENTION_SDPA', 'bad-optional-value')
    assert not parse_args(base).head_attention_sdpa
    assert 'stays OFF' in caplog.text
    assert parse_args(base + ['--head-attention-sdpa']).head_attention_sdpa
    with pytest.raises(SystemExit):
        parse_args(base + ['--head-proposal-topk'])


def server_args():
    return SimpleNamespace(sensor_config='sensor', camera_order=None, intrinsic_source='horizontal_fov',
        projection_only=False, amp=True, checkpoint='selected-E24.pth', config='selected.py', device='cuda:0',
        preprocess_workers=4, warmup_iterations=2, temporal_feature_cache=False,
        temporal_maximum_age_sec=.3, base5_runtime=True, parallel_camera=True,
        head_attention_sdpa=True, head_cache_bev_pos=True)


@pytest.mark.parametrize('safe', [False, True])
def test_server_preserves_selected_checkpoint_and_safe_mode_forces_head_off(monkeypatch, safe):
    monkeypatch.setenv('BEVFUSION_SAFE_MODE', str(safe).lower())
    factory = Mock(return_value=SimpleNamespace(box_center_origin='lidar_bottom_v1'))
    with patch.dict(sys.modules, {'app.bevfusion_model': SimpleNamespace(BevFusionModel=factory)}), \
            patch('app.inference_server.load_sensor4_calibration', return_value={}), \
            patch('app.inference_server.file_sha256', side_effect=lambda path: 'sha-' + path):
        server = InferenceServer(server_args())
    supplied = factory.call_args.kwargs
    assert supplied['head_attention_sdpa'] is not safe and supplied['head_cache_bev_pos'] is not safe
    assert supplied['checkpoint_path'] == 'selected-E24.pth' and supplied['config_path'] == 'selected.py'
    assert server.runtime_identity['checkpoint_sha256'] == 'sha-selected-E24.pth'
    assert not any('topk' in name for name in supplied)


def test_head_cuda_fault_still_escapes_server_without_hot_disable():
    server = InferenceServer.__new__(InferenceServer)
    server._handle_request = Mock(side_effect=RuntimeError('CUDA error: an illegal memory access was encountered'))
    with patch('app.inference_server.recv_message', return_value=(TYPE_REQUEST, {}, {})), \
            patch('app.inference_server.send_message') as send:
        with pytest.raises(AccelerationExecutionError, match='fresh process'):
            server._serve_client(None)
    assert server._handle_request.call_count == 1
    assert send.call_args.args[2]['restart_required'] is True
