"""CPU-only camera-engine contract and differential-analysis regressions.

These tests deliberately do not import a real TensorRT/CUDA runtime, build an
engine or certify a detector. Binding mocks exercise rejection contracts.
"""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'docker/bevfusion'))
spec = importlib.util.spec_from_file_location('camera_trt_qualifier',
    ROOT/'perception/detection3d/bevfusion/scripts/qualify_base5_camera_tensorrt.py')
qualifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(qualifier)
from app import camera_tensorrt as camera_trt


def metadata():
    return dict(schema=camera_trt.SCHEMA, stage='engine',
        contract=dict(input_name='images', input_shape=[4, 3, 512, 1056],
            input_dtype='float32', input_semantics='normalized_flattened_BNCHW',
            output_kind='tuple', outputs=[dict(name='camera_feature_0',
                shape=[4, 256, 64, 132], dtype='float32')],
            checkpoint_sha256='a'*64, config_sha256='b'*64,
            camera_state_sha256='c'*64, source_sha256={'source.py': 'd'*64}),
        onnx=dict(sha256='e'*64), engine=dict(sha256='f'*64,
            gpu_name='NVIDIA GeForce RTX 4090', compute_capability=[8, 9],
            tensorrt_version='10.13.3', precision='fp16', hardware_compatibility='none'))


class BindingEngine:
    def __init__(self):
        self.bindings = dict(images=dict(mode='input', shape=(4, 3, 512, 1056), dtype='fp32',
                                        location='device', format='linear'),
            camera_feature_0=dict(mode='output', shape=(4, 256, 64, 132), dtype='fp32',
                                  location='device', format='linear'))
        self.num_io_tensors = len(self.bindings)

    def get_tensor_name(self, index): return list(self.bindings)[index]
    def get_tensor_mode(self, name): return self.bindings[name]['mode']
    def get_tensor_shape(self, name): return self.bindings[name]['shape']
    def get_tensor_dtype(self, name): return self.bindings[name]['dtype']
    def get_tensor_location(self, name): return self.bindings[name]['location']
    def get_tensor_format(self, name): return self.bindings[name]['format']


TRT_MOCK = SimpleNamespace(TensorIOMode=SimpleNamespace(INPUT='input', OUTPUT='output'),
    float32='fp32', float16='fp16', TensorLocation=SimpleNamespace(DEVICE='device'),
    TensorFormat=SimpleNamespace(LINEAR='linear'))


def engine_files(tmp_path):
    engine = tmp_path/'camera.engine'; engine.write_bytes(b'cpu-only binding fixture')
    value = metadata(); value['engine']['sha256'] = camera_trt.sha256_file(engine)
    path = tmp_path/'camera.engine.json'; path.write_text(json.dumps(value))
    return engine, path, value


def test_metadata_validation_requires_current_shape_and_frozen_identity():
    value = metadata()
    assert camera_trt.validate_camera_metadata(value, {'checkpoint_sha256': 'a'*64}) is value
    with pytest.raises(ValueError, match='checkpoint_sha256'):
        camera_trt.validate_camera_metadata(value, {'checkpoint_sha256': '1'*64})
    for key, bad in [('input_shape', [4, 3, 576, 1056]), ('input_dtype', 'int8'),
                     ('input_semantics', 'raw_bgr'), ('camera_state_sha256', 'not-a-hash')]:
        broken = deepcopy(value); broken['contract'][key] = bad
        with pytest.raises(ValueError): camera_trt.validate_camera_metadata(broken)
    broken = deepcopy(value); broken['engine']['hardware_compatibility'] = 'all-gpus'
    with pytest.raises(ValueError): camera_trt.validate_camera_metadata(broken)


def test_valid_static_device_linear_engine_binding_contract():
    camera_trt.validate_engine_bindings(BindingEngine(), metadata(), TRT_MOCK)


@pytest.mark.parametrize('binding,key,bad', [
    ('images', 'mode', 'output'), ('camera_feature_0', 'mode', 'input'),
    ('images', 'shape', (-1, 3, 512, 1056)),
    ('camera_feature_0', 'shape', (4, 256, 63, 132)),
    ('images', 'dtype', 'fp16'), ('camera_feature_0', 'dtype', 'int8'),
    ('images', 'location', 'host'), ('camera_feature_0', 'format', 'vectorized'),
])
def test_binding_mocks_reject_shape_dtype_location_mode_format(binding, key, bad):
    engine = BindingEngine(); engine.bindings[binding][key] = bad
    with pytest.raises(ValueError): camera_trt.validate_engine_bindings(engine, metadata(), TRT_MOCK)


def test_binding_mocks_reject_unknown_names_and_count():
    engine = BindingEngine(); engine.num_io_tensors = 3
    with pytest.raises(ValueError, match='count'):
        camera_trt.validate_engine_bindings(engine, metadata(), TRT_MOCK)
    engine = BindingEngine(); engine.bindings['wrong'] = engine.bindings.pop('images')
    with pytest.raises(ValueError, match='names'):
        camera_trt.validate_engine_bindings(engine, metadata(), TRT_MOCK)
    with pytest.raises(RuntimeError, match='TensorRT 10'):
        camera_trt.validate_engine_bindings(SimpleNamespace(), metadata(), TRT_MOCK)


def test_adapter_is_lazy_hash_checked_and_no_implicit_fallback(tmp_path, monkeypatch):
    engine, path, value = engine_files(tmp_path)
    def forbidden(*_): pytest.fail('CPU constructor must not load torch/TRT/CUDA')
    monkeypatch.setattr(camera_trt.importlib, 'import_module', forbidden)
    adapter = camera_trt.CameraTensorRTAdapter(engine, path)
    assert adapter._engine is None and adapter._contexts == {}
    adapter.close(); adapter.close()
    engine.write_bytes(b'changed')
    with pytest.raises(ValueError, match='SHA256'):
        camera_trt.CameraTensorRTAdapter(engine, path)


@pytest.mark.parametrize('version,gpu,capability,match', [
    ('11.0.0', 'NVIDIA GeForce RTX 4090', [8, 9], '10.x'),
    ('10.13.2', 'NVIDIA GeForce RTX 4090', [8, 9], 'version'),
    ('10.13.3', 'NVIDIA GeForce RTX 5060 Ti', [12, 0], 'GPU/compute'),
    ('10.13.3', 'NVIDIA GeForce RTX 4090', [8, 6], 'GPU/compute'),
])
def test_runtime_mocks_refuse_different_trt_or_gpu(tmp_path, monkeypatch, version, gpu, capability, match):
    engine, path, _ = engine_files(tmp_path)
    adapter = camera_trt.CameraTensorRTAdapter(engine, path)
    monkeypatch.setattr(camera_trt.importlib, 'import_module', lambda _: SimpleNamespace(__version__=version))
    torch_mock = SimpleNamespace(cuda=SimpleNamespace(get_device_name=lambda _: gpu,
                  get_device_capability=lambda _: capability))
    with pytest.raises((ValueError, RuntimeError), match=match): adapter._load(torch_mock, 'cuda:0')
    assert adapter._engine is None


def test_installer_is_explicit_reversible_and_rejects_wrong_camera_state(tmp_path, monkeypatch):
    engine, path, _ = engine_files(tmp_path)
    class Module:
        def forward(self, value): return value
    model = SimpleNamespace(training=False, img_backbone=Module(), img_neck=Module())
    original = model.img_backbone.forward, model.img_neck.forward
    # Merely constructing or validating an engine never modifies a model.
    camera_trt.CameraTensorRTAdapter(engine, path).close()
    assert original == (model.img_backbone.forward, model.img_neck.forward)
    monkeypatch.setattr(camera_trt, 'camera_state_sha256', lambda *_: 'c'*64)
    installed = camera_trt.install_camera_tensorrt(model, engine, path)
    with pytest.raises(ValueError, match='already installed'):
        camera_trt.install_camera_tensorrt(model, engine, path)
    with pytest.raises(RuntimeError, match='own fused'):
        model.img_neck.forward(object())
    installed.disable(); installed.disable()
    assert original == (model.img_backbone.forward, model.img_neck.forward)
    monkeypatch.setattr(camera_trt, 'camera_state_sha256', lambda *_: '1'*64)
    with pytest.raises(ValueError, match='camera_state_sha256'):
        camera_trt.install_camera_tensorrt(model, engine, path)
    assert original == (model.img_backbone.forward, model.img_neck.forward)
    model.training = True
    with pytest.raises(ValueError, match='eval'):
        camera_trt.install_camera_tensorrt(model, engine, path)


@pytest.mark.parametrize('fault,match', [('grad', 'inference_mode'), ('cpu', 'CUDA tensor'),
    ('device', 'different CUDA'), ('shape', 'shape'), ('dtype', 'dtype'), ('layout', 'contiguous')])
def test_input_mocks_reject_without_cpu_copy_cast_or_engine_load(tmp_path, monkeypatch, fault, match):
    engine, path, _ = engine_files(tmp_path)
    adapter = camera_trt.CameraTensorRTAdapter(engine, path)
    device = SimpleNamespace(type='cuda', index=0)
    image = SimpleNamespace(is_cuda=fault != 'cpu', device=device,
        shape=(4, 3, 512, 1056), dtype='float32', is_contiguous=lambda: fault != 'layout')
    if fault == 'device': image.device = SimpleNamespace(type='cuda', index=1)
    if fault == 'shape': image.shape = (1, 3, 512, 1056)
    if fault == 'dtype': image.dtype = 'float16'
    torch_mock = SimpleNamespace(is_grad_enabled=lambda: fault == 'grad',
        is_tensor=lambda _: True, device=lambda _: device, float32='float32', float16='float16')
    def importer(name):
        assert name == 'torch', 'Invalid input must not load the real TensorRT engine'
        return torch_mock
    monkeypatch.setattr(camera_trt.importlib, 'import_module', importer)
    with pytest.raises((ValueError, RuntimeError), match=match): adapter(image)
    assert adapter._engine is None


def detection(x=1., score=.8, label=0, yaw=0.):
    return dict(center=[x, 0., 0.], size=[1., 1., 1.], yaw=yaw,
                label=label, class_name=qualifier.CLASSES[label], score=score)


def test_distribution_and_exact_activation_quantiles():
    reference = np.zeros((1, 4, 2, 2), dtype=np.float32)
    actual = np.arange(16, dtype=np.float32).reshape(reference.shape)
    row = qualifier.tensor_difference(reference, actual)
    assert row['absolute_error']['count'] == 16
    assert row['absolute_error']['p50'] == pytest.approx(7.5)
    assert row['absolute_error']['p95'] == pytest.approx(14.25)
    assert row['absolute_error']['maximum'] == 15
    assert qualifier.distribution([])['p95'] is None
    with pytest.raises(ValueError, match='Nonfinite'):
        qualifier.distribution([float('nan')])
    with pytest.raises(ValueError, match='shape'):
        qualifier.tensor_difference(reference, actual.reshape(-1))


def test_all_objects_unmatched_and_yaw_wrapping_are_visible():
    reference = [detection(1., yaw=np.pi-.001), detection(4.), detection(7., label=1)]
    candidate = [detection(1.01, yaw=-np.pi+.001), detection(9.), detection(7., label=2)]
    result = qualifier.compare_detections(reference, candidate)
    assert not result['engineering_passed']
    assert result['matched'] == 1
    assert result['classes']['vehicle']['pairs'][0]['yaw_rad'] == pytest.approx(.002)
    assert len(result['classes']['vehicle']['unmatched_reference']) == 1
    assert len(result['classes']['vehicle']['unmatched_candidate']) == 1
    assert len(result['classes']['pedestrian']['unmatched_reference']) == 1
    assert len(result['classes']['obstacle']['unmatched_candidate']) == 1


def test_empty_outputs_are_not_vacuous_success():
    row = qualifier.compare_detections([], [])
    assert not row['engineering_passed'] and not row['nonvacuous']
    assert not qualifier.aggregate_comparisons([row])['engineering_passed']
    positive = qualifier.compare_detections([detection()], [detection()])
    assert qualifier.aggregate_comparisons([row, positive])['engineering_passed']
    added = qualifier.compare_detections([], [detection()])
    assert not qualifier.aggregate_comparisons([positive, added])['engineering_passed']


def test_engineering_limits_are_not_the_matching_gate():
    result = qualifier.compare_detections([detection()], [detection(1.2)])
    assert result['matched'] == 1
    assert not result['engineering_passed']
    assert result['distributions']['center_xy_m']['maximum'] == pytest.approx(.2)
    with pytest.raises(ValueError, match='Finite'):
        qualifier.compare_detections([detection(score=float('nan'))], [])


def test_ros_filter_order_is_floor_top100_then_class_cutoff():
    # High-score vehicles consume the real server's top100 before the class
    # threshold is applied. Do not promote a low-score pedestrian afterwards.
    rows = [detection(float(i)*3, score=.3) for i in range(100)]
    rows.append(detection(1000., score=.2, label=1))
    assert len(qualifier.ros_delivery(rows, final_thresholds=False)) == 100
    assert qualifier.ros_delivery(rows, final_thresholds=True) == []
    rows = [detection(0., .8), detection(.01, .7), detection(.01, .6, label=1), detection(9., .09)]
    kept = qualifier.ros_delivery(rows)
    assert len(kept) == 2
    assert {d['label'] for d in kept} == {0, 1}


def input_manifest(tmp_path, *, source='native'):
    def payload(name, blob):
        path = tmp_path/name; path.write_bytes(blob)
        return dict(path=name, sha256=hashlib.sha256(blob).hexdigest())
    points = np.array([[1., 2., 3., .5]], dtype=np.float32)
    manifest = dict(schema='base5_camera_trt_inputs_v1', image_policy='native_jpeg90',
        point_intensity_source=source, samples=[dict(id='fixed-token', group='Foggy13',
        points=payload('points.bin', points.tobytes()),
        images={'image/'+name: payload(name+'.jpg', b'\xff\xd8test-'+name.encode())
                for name in qualifier.CAMERAS})])
    path = tmp_path/'selection.json'; path.write_text(json.dumps(manifest))
    return path, manifest


def test_input_sha_and_native_intensity_are_explicit(tmp_path):
    path, manifest = input_manifest(tmp_path)
    samples, contract, pins = qualifier.load_inputs(path, 1)
    assert contract['point_intensity_source'] == 'native'
    assert samples[0]['points'][0, 3] == .5
    assert len(pins) == 6
    (tmp_path/'camera_front.jpg').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum'):
        qualifier.load_inputs(path, 1)


def test_manifest_rejects_repeated_identity_and_implicit_png(tmp_path):
    path, manifest = input_manifest(tmp_path)
    manifest['samples'].append(deepcopy(manifest['samples'][0]))
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='distinct'):
        qualifier.load_inputs(path, 1)
    manifest['samples'].pop()
    item = manifest['samples'][0]['images']['image/camera_front']
    payload = b'\x89PNG arbitrary'; (tmp_path/item['path']).write_bytes(payload)
    item['sha256'] = hashlib.sha256(payload).hexdigest()
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='JPEG'):
        qualifier.load_inputs(path, 1)


def test_intensity_is_not_silently_normalized_or_guessed(tmp_path):
    path, manifest = input_manifest(tmp_path)
    values = np.array([[0., 0., 0., 255.]], dtype=np.float32).tobytes()
    (tmp_path/'points.bin').write_bytes(values)
    manifest['samples'][0]['points']['sha256'] = hashlib.sha256(values).hexdigest()
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='intensity'):
        qualifier.load_inputs(path, 1)
    manifest['point_intensity_source'] = 'ros255'
    path.write_text(json.dumps(manifest))
    samples, _, _ = qualifier.load_inputs(path, 1)
    assert samples[0]['points'][0, 3] == 255  # runtime does the sole division


def test_write_new_never_overwrites_old_evidence(tmp_path):
    path = tmp_path/'receipt.json'
    qualifier.write_new(path, dict(complete=False))
    with pytest.raises(FileExistsError):
        qualifier.write_new(path, dict(complete=True))
    assert json.loads(path.read_text()) == dict(complete=False)
