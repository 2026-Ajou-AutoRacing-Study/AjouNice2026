"""CPU-only startup policy tests; no CUDA/TRT/framework import is required."""
import gc
import sys
from pathlib import Path
from types import SimpleNamespace
import weakref

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docker/bevfusion"))
from app.acceleration_startup import (
    AccelerationExecutionError, AccelerationUnavailableError,
    construct_camera_runtime, is_nonrecoverable_cuda_error,
    resolve_camera_startup_policy,
)
from app.camera_tensorrt import CameraTensorRTAdapter


@pytest.fixture(autouse=True)
def no_real_cuda(monkeypatch):
    # Other test modules may have imported Torch. Never touch its CUDA state.
    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(
        cuda=SimpleNamespace(is_initialized=lambda: False)))


@pytest.mark.parametrize("policy,engine,metadata,expected", [
    (None, None, None, "off"), ("", None, None, "off"),
    (None, "missing.engine", "missing.json", "prefer"),
    (None, None, "missing.json", "prefer"),
    ("off", "engine", "metadata", "off"),
    ("require", None, None, "require"),
])
def test_default_policy_only_prefers_when_acceleration_was_configured(policy, engine, metadata, expected):
    assert resolve_camera_startup_policy(policy, engine, metadata) == expected
    with pytest.raises(ValueError, match="off, prefer, or require"):
        resolve_camera_startup_policy("automatic")


def test_off_ignores_missing_engine_and_never_changes_selected_model():
    supplied = []
    kwargs = dict(checkpoint_path="E24.pth", config_path="E24.py", calibration=object(),
                  use_amp=True, parallel_camera=True, pool_fused_layout=False)
    original = dict(kwargs)
    model = object()
    def factory(**values):
        supplied.append(values)
        return model
    result, report = construct_camera_runtime(factory, kwargs, "off", "E12.engine", "absent.json")
    assert result is model and supplied == [original] and kwargs == original
    assert report["policy"] == "off" and report["requested"]
    assert report["active_backend"] == "eager" and not report["fallback"]
    assert report["reason_code"] == "disabled_by_policy"
    assert not report["checkpoint_changed"] and not report["config_changed"]


def test_no_engine_baseline_is_one_unmodified_factory_call():
    calls = []
    def factory(**kwargs):
        calls.append(kwargs)
        return "model"
    result, report = construct_camera_runtime(factory, {"checkpoint_path": "E24.pth"})
    assert result == "model" and calls == [{"checkpoint_path": "E24.pth"}]
    assert report["policy"] == "off" and report["reason_code"] == "not_requested"


@pytest.mark.parametrize("policy", ["prefer", "require"])
def test_successful_acceleration_does_not_construct_eager_twice(policy):
    calls = []
    def factory(**kwargs):
        calls.append(kwargs)
        return "accelerated"
    result, report = construct_camera_runtime(factory, {"checkpoint_path": "E12.pth"},
        policy, "chosen.engine", "chosen.json")
    assert result == "accelerated" and len(calls) == 1
    assert calls[0] == dict(checkpoint_path="E12.pth", camera_tensorrt_engine="chosen.engine",
                            camera_tensorrt_metadata="chosen.json")
    assert report["active_backend"] == "camera_tensorrt" and not report["fallback"]


def test_prefer_rebuilds_same_checkpoint_outside_exception_and_releases_partial_model(caplog):
    calls, abandoned = [], []
    calibration = object()
    original = dict(checkpoint_path="E24.pth", config_path="E24.py", calibration=calibration,
                    use_amp=True, parallel_camera=True, pool_fused_layout=True)
    class PartialModel:
        pass
    def factory(**kwargs):
        calls.append(kwargs)
        if "camera_tensorrt_engine" in kwargs:
            partial = PartialModel()
            abandoned.append(weakref.ref(partial))
            try:
                raise ValueError("mismatched E12 engine metadata")
            except ValueError as error:
                raise AccelerationUnavailableError("engine_metadata_unavailable", str(error)) from error
        assert sys.exc_info() == (None, None, None), "Do not rebuild inside a retained exception handler"
        gc.collect()
        assert abandoned[0]() is None, "Failed CUDA-model constructor is still held by a traceback"
        return "same-model-eager"
    result, report = construct_camera_runtime(factory, original, None, "E12.engine", "E12.json")
    assert result == "same-model-eager" and len(calls) == 2
    assert calls[1] == original and calls[1]["calibration"] is calibration
    assert "camera_tensorrt_engine" not in original
    assert report["policy"] == "prefer" and report["fallback"]
    assert report["reason_code"] == "engine_metadata_unavailable"
    assert report["active_backend"] == "eager" and report["startup_only"]
    assert not report["checkpoint_changed"] and not report["config_changed"]
    assert "SAME selected checkpoint/config" in caplog.text


def test_actual_missing_metadata_is_named_optional_but_direct_adapter_stays_strict(tmp_path):
    engine, metadata = tmp_path / "missing.engine", tmp_path / "missing.json"
    with pytest.raises(AccelerationUnavailableError, match="metadata"):
        CameraTensorRTAdapter(engine, metadata)
    calls = []
    def factory(**kwargs):
        calls.append(kwargs)
        if "camera_tensorrt_engine" in kwargs:
            return CameraTensorRTAdapter(kwargs["camera_tensorrt_engine"], kwargs["camera_tensorrt_metadata"])
        return "eager"
    model, report = construct_camera_runtime(factory, {}, "prefer", str(engine), str(metadata))
    assert model == "eager" and report["fallback"] and len(calls) == 2


def test_require_never_falls_back_and_missing_engine_is_a_configuration_error():
    calls = []
    def factory(**kwargs):
        calls.append(kwargs)
        raise AccelerationUnavailableError("engine_gpu_mismatch", "Different GPU")
    with pytest.raises(AccelerationUnavailableError, match="Different GPU"):
        construct_camera_runtime(factory, {}, "require", "engine", "metadata")
    assert len(calls) == 1
    with pytest.raises(AccelerationUnavailableError, match="explicit engine"):
        construct_camera_runtime(factory, {}, "require")
    assert len(calls) == 1


@pytest.mark.parametrize("error", [
    FileNotFoundError("Selected checkpoint missing"), ValueError("Class order differs"),
    RuntimeError("Strict state_dict load failed"), RuntimeError("CUDA out of memory"),
    TypeError("Invalid essential model config"),
    AccelerationExecutionError("camera_execution_failed", "Already enqueued"),
])
def test_prefer_does_not_swallow_essential_errors_or_post_enqueue_failures(error):
    calls = []
    def factory(**kwargs):
        calls.append(kwargs)
        raise error
    with pytest.raises(type(error), match=str(error).split(";")[0]):
        construct_camera_runtime(factory, {}, "prefer", "engine", "metadata")
    assert len(calls) == 1


def test_cuda_health_is_checked_before_retry_and_unhealthy_context_never_rebuilt(monkeypatch):
    calls, gpu = [], []
    def synchronize():
        gpu.append("synchronize")
        raise RuntimeError("CUDA error: an illegal memory access was encountered")
    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(cuda=SimpleNamespace(
        is_initialized=lambda: True, synchronize=synchronize,
        empty_cache=lambda: gpu.append("empty_cache"))))
    def factory(**kwargs):
        calls.append(kwargs)
        raise AccelerationUnavailableError("engine_deserialization_unavailable", "Engine incompatible")
    with pytest.raises(AccelerationExecutionError, match="fresh process") as failure:
        construct_camera_runtime(factory, {}, "prefer", "engine", "metadata")
    assert failure.value.restart_required
    assert len(calls) == 1 and gpu == ["synchronize"]


def test_healthy_context_drained_and_allocator_released_before_eager_retry(monkeypatch):
    events = []
    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(cuda=SimpleNamespace(
        is_initialized=lambda: True,
        synchronize=lambda: events.append("synchronize"),
        empty_cache=lambda: events.append("empty_cache"))))
    def factory(**kwargs):
        if "camera_tensorrt_engine" in kwargs:
            events.append("optional")
            raise AccelerationUnavailableError("engine_gpu_mismatch", "Different GPU")
        events.append("eager")
        return object()
    construct_camera_runtime(factory, {}, "prefer", "engine", "metadata")
    assert events == ["optional", "synchronize", "empty_cache", "eager"]


@pytest.mark.parametrize("message,expected", [
    ("CUDA error: device-side assert triggered", True),
    ("CUDA error: an illegal memory access was encountered", True),
    ("CUDA out of memory", False), ("Invalid RGB frame shape", False),
])
def test_fatal_cuda_classifier_is_narrow(message, expected):
    assert is_nonrecoverable_cuda_error(RuntimeError(message)) is expected
    assert not is_nonrecoverable_cuda_error(ValueError(message))
