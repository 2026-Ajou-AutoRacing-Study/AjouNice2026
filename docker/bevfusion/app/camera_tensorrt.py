"""Explicit, inference-only TensorRT camera stage; no production default.

TensorRT and torch are imported lazily. The engine replaces only the flattened
camera backbone/neck boundary, not normalization, geometry, depth, or detection.
Engines are trusted local executable artifacts, not portable model weights.
"""
from dataclasses import dataclass
import hashlib
import importlib
import json
from pathlib import Path
import threading

from app.acceleration_startup import (
    AccelerationExecutionError, AccelerationUnavailableError, is_nonrecoverable_cuda_error)


SCHEMA = "base5_camera_tensorrt_v1"
INPUT_NAME = "images"
SUPPORTED_DTYPES = ("float32", "float16")


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha(value, name):
    if (not isinstance(value, str) or len(value) != 64 or
            any(character not in "0123456789abcdef" for character in value)):
        raise ValueError(f"{name} must be a lowercase SHA256")


def _shape(value, name):
    if (not isinstance(value, (tuple, list)) or not value or
            any(type(dimension) is not int or dimension <= 0 for dimension in value)):
        raise ValueError(f"{name} must have fixed positive integer dimensions")
    return tuple(value)


def validate_camera_metadata(metadata, expected_contract=None):
    """Validate an artifact contract without importing GPU libraries.

    expected_contract is an explicit subset of contract fields to compare. It
    should include checkpoint/source identities in a qualification launcher.
    Schema validation is not an accuracy or source-authenticity certificate.
    """
    if not isinstance(metadata, dict) or metadata.get("schema") != SCHEMA:
        raise ValueError("Unknown camera TensorRT metadata schema")
    if metadata.get("stage") not in ("onnx", "engine"):
        raise ValueError("Unknown camera artifact stage")
    contract = metadata.get("contract", {})
    shape = _shape(contract.get("input_shape"), "input_shape")
    if len(shape) != 4 or shape != (4, 3, 512, 1056):
        raise ValueError("Only the audited batch-one four-camera 512x1056 contract is supported")
    if contract.get("input_dtype") not in SUPPORTED_DTYPES:
        raise ValueError("Unsupported camera input dtype")
    if contract.get("input_name") != INPUT_NAME:
        raise ValueError("Unexpected camera input binding name")
    if contract.get("input_semantics") != "normalized_flattened_BNCHW":
        raise ValueError("Engine input must follow existing GPU normalization and camera flattening")
    if contract.get("output_kind") not in ("tensor", "tuple", "list"):
        raise ValueError("Unsupported original neck return type")
    outputs = contract.get("outputs")
    if not isinstance(outputs, list) or not outputs:
        raise ValueError("Camera output bindings are missing")
    if contract["output_kind"] == "tensor" and len(outputs) != 1:
        raise ValueError("Tensor neck output must contain exactly one binding")
    names = {INPUT_NAME}
    for index, output in enumerate(outputs):
        if output.get("name") != f"camera_feature_{index}" or output["name"] in names:
            raise ValueError("Camera output binding order/name differs")
        names.add(output["name"])
        dimensions = _shape(output.get("shape"), output["name"])
        if len(dimensions) != 4 or dimensions[0] != shape[0]:
            raise ValueError("Camera neck output must preserve flattened view count")
        if output.get("dtype") not in SUPPORTED_DTYPES:
            raise ValueError("Unsupported camera output dtype")
    for name in ("checkpoint_sha256", "config_sha256", "camera_state_sha256"):
        _sha(contract.get(name), name)
    sources = contract.get("source_sha256")
    if not isinstance(sources, dict) or not sources:
        raise ValueError("Source identity mapping is missing")
    for path, digest in sources.items():
        if not isinstance(path, str) or not path:
            raise ValueError("Invalid source identity path")
        _sha(digest, "source_sha256")
    _sha(metadata.get("onnx", {}).get("sha256"), "onnx.sha256")
    if metadata["stage"] == "engine":
        engine = metadata.get("engine", {})
        _sha(engine.get("sha256"), "engine.sha256")
        if not engine.get("gpu_name") or not engine.get("tensorrt_version"):
            raise ValueError("Engine GPU/TensorRT build identity is missing")
        capability = engine.get("compute_capability")
        if (not isinstance(capability, list) or len(capability) != 2 or
                any(type(value) is not int or value < 0 for value in capability)):
            raise ValueError("Engine compute capability is missing")
        if engine.get("precision") not in ("fp32", "fp16"):
            raise ValueError("Unsupported engine precision")
        if engine.get("hardware_compatibility") != "none":
            raise ValueError("Portable/hardware-compatible engines are not qualified")
    for key, expected in (expected_contract or {}).items():
        if key not in contract or contract[key] != expected:
            raise ValueError(f"Camera engine contract mismatch: {key}")
    return metadata


def load_camera_metadata(path, expected_contract=None):
    return validate_camera_metadata(json.loads(Path(path).read_text()), expected_contract)


def validate_camera_tensorrt_options(engine=None, metadata=None, *, base5_runtime=False,
                                   use_amp=False, parallel_camera=False, compile_parts=(),
                                   fuse_dense_bn=False, camera_cuda_graph=False,
                                   eager_camera_numerics=False, compile_reference_layout=False,
                                   temporal_feature_cache=False, device="cuda:0",
                                   warmup_iterations=2, projection_only=False,
                                   fp32_reference=False):
    """Fail closed before model allocation; an absent engine changes nothing."""
    requested = bool(engine) or bool(metadata)
    if not requested:
        return False
    if not engine or not metadata:
        raise ValueError("Camera TensorRT requires both explicit engine and metadata paths")
    if projection_only:
        raise ValueError("Projection-only mode does not load a camera TensorRT engine")
    if not base5_runtime or not use_amp or not parallel_camera or fp32_reference:
        raise ValueError("Camera TensorRT requires explicit Base5 eval, AMP and parallel camera; not FP32 reference")
    if (compile_parts or fuse_dense_bn or camera_cuda_graph or eager_camera_numerics or
            compile_reference_layout or temporal_feature_cache):
        raise ValueError("Camera TensorRT cannot combine with compile, Conv/BN folding, CUDA Graph or temporal candidates")
    value = str(device)
    if not value.startswith("cuda:") or not value[5:].isdigit():
        raise ValueError("Camera TensorRT requires an explicit CUDA device index")
    if int(warmup_iterations) < 1:
        raise ValueError("Camera TensorRT requires startup warmup before reporting ready")
    return True


def validate_engine_bindings(engine, metadata, trt):
    """Check the TRT 10 name API and exact static DEVICE/LINEAR bindings."""
    validate_camera_metadata(metadata)
    contract = metadata["contract"]
    expected = [(INPUT_NAME, contract["input_shape"], contract["input_dtype"], True)]
    expected += [(item["name"], item["shape"], item["dtype"], False)
                 for item in contract["outputs"]]
    if not hasattr(engine, "num_io_tensors"):
        raise RuntimeError("TensorRT 10 named tensor API is required")
    if engine.num_io_tensors != len(expected):
        raise ValueError("Engine input/output binding count differs")
    actual_names = {engine.get_tensor_name(index) for index in range(engine.num_io_tensors)}
    if actual_names != {entry[0] for entry in expected}:
        raise ValueError("Engine input/output binding names differ")
    for name, shape, dtype, is_input in expected:
        mode = trt.TensorIOMode.INPUT if is_input else trt.TensorIOMode.OUTPUT
        expected_dtype = trt.float32 if dtype == "float32" else trt.float16
        if engine.get_tensor_mode(name) != mode:
            raise ValueError(f"Engine binding mode differs: {name}")
        if tuple(engine.get_tensor_shape(name)) != tuple(shape):
            raise ValueError(f"Engine binding shape differs: {name}")
        if engine.get_tensor_dtype(name) != expected_dtype:
            raise ValueError(f"Engine binding dtype differs: {name}")
        if engine.get_tensor_location(name) != trt.TensorLocation.DEVICE:
            raise ValueError(f"Host/shape bindings are not supported: {name}")
        if engine.get_tensor_format(name) != trt.TensorFormat.LINEAR:
            raise ValueError(f"Non-contiguous engine binding is unsupported: {name}")


def camera_state_sha256(backbone, neck):
    """Semantic hash including every camera parameter/persistent buffer."""
    torch = importlib.import_module("torch")
    digest = hashlib.sha256()
    for prefix, module in (("img_backbone", backbone), ("img_neck", neck)):
        for name, tensor in sorted(module.state_dict().items()):
            value = tensor.detach().cpu().contiguous()
            header = json.dumps([prefix + "." + name, str(value.dtype), list(value.shape)],
                                separators=(",", ":")).encode()
            digest.update(len(header).to_bytes(8, "little"))
            digest.update(header)
            digest.update(value.reshape(-1).view(torch.uint8).numpy().tobytes())
    return digest.hexdigest()


class CameraTensorRTAdapter:
    """Lazy fixed-shape engine callable with direct torch CUDA bindings.

    No implicit dtype/device copy, default-stream switch, CPU round-trip, output
    reuse, or eager fallback occurs. Each CUDA stream owns its TRT context.
    Output tensors are newly allocated on the caller stream. The caller must
    follow normal PyTorch stream hand-off rules when consuming them elsewhere.
    close() waits for submitted TRT work before releasing runtime/context state.
    """
    def __init__(self, engine_path, metadata_path=None, expected_contract=None,
                 device="cuda:0"):
        self._ever_enqueued = False
        self.engine_path = Path(engine_path)
        self.metadata_path = (Path(metadata_path) if metadata_path else
                              Path(str(engine_path) + ".json"))
        try:
            self.metadata = load_camera_metadata(self.metadata_path, expected_contract)
            self.metadata_sha256 = sha256_file(self.metadata_path)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
            raise AccelerationUnavailableError("engine_metadata_unavailable",
                "Camera engine metadata is missing, invalid or incompatible: " + str(error)) from error
        if self.metadata["stage"] != "engine":
            raise AccelerationUnavailableError("engine_metadata_unavailable", "An engine metadata artifact is required")
        if not self.engine_path.is_file():
            raise AccelerationUnavailableError("engine_file_unavailable", "Camera engine file is missing: " + str(self.engine_path))
        try:
            engine_sha = sha256_file(self.engine_path)
        except OSError as error:
            raise AccelerationUnavailableError("engine_file_unavailable", "Cannot read camera engine: " + str(error)) from error
        if engine_sha != self.metadata["engine"]["sha256"]:
            raise AccelerationUnavailableError("engine_hash_mismatch", "TensorRT engine SHA256 differs from metadata")
        self.device_spec = str(device)
        self._lock = threading.RLock()
        self._contexts = {}
        self._runtime = self._engine = self._logger = self._trt = None
        self._closed = False
        self._poisoned = False

    def _load(self, torch, device):
        if self._engine is not None:
            return
        try:
            trt = importlib.import_module("tensorrt")
        except (ImportError, OSError) as error:
            raise AccelerationUnavailableError("tensorrt_import_unavailable",
                "TensorRT package/native libraries are unavailable: " + str(error)) from error
        version = str(trt.__version__)
        if version.split(".")[0] != "10":
            raise AccelerationUnavailableError("tensorrt_version_mismatch", "This opt-in adapter is qualified for TensorRT 10.x only")
        identity = self.metadata["engine"]
        if version != identity["tensorrt_version"]:
            raise AccelerationUnavailableError("tensorrt_version_mismatch", "TensorRT runtime version differs from engine build")
        if (torch.cuda.get_device_name(device) != identity["gpu_name"] or
                list(torch.cuda.get_device_capability(device)) != identity["compute_capability"]):
            raise AccelerationUnavailableError("engine_gpu_mismatch", "Engine GPU/compute capability differs; rebuild on the target GPU")
        # Keep logger/runtime/engine alive in this order for every context.
        try:
            payload = self.engine_path.read_bytes()
        except OSError as error:
            raise AccelerationUnavailableError("engine_file_unavailable", "Cannot read camera engine: " + str(error)) from error
        if hashlib.sha256(payload).hexdigest() != identity["sha256"]:
            raise AccelerationUnavailableError("engine_hash_mismatch", "TensorRT engine changed before lazy deserialization")
        engine = runtime = None
        try:
            logger = trt.Logger(trt.Logger.WARNING)
            runtime = trt.Runtime(logger)
            engine = runtime.deserialize_cuda_engine(payload)
        except (RuntimeError, ValueError, OSError, AttributeError) as error:
            engine = None
            runtime = None
            if is_nonrecoverable_cuda_error(error):
                raise AccelerationExecutionError("cuda_context_unhealthy", str(error)) from error
            raise AccelerationUnavailableError("engine_deserialization_unavailable",
                "TensorRT engine deserialization failed before enqueue: " + str(error)) from error
        if engine is None:
            runtime = None
            raise AccelerationUnavailableError("engine_deserialization_unavailable", "TensorRT engine deserialization failed")
        try:
            validate_engine_bindings(engine, self.metadata, trt)
        except (ValueError, RuntimeError, AttributeError) as error:
            engine = None
            runtime = None
            raise AccelerationUnavailableError("engine_bindings_incompatible",
                "TensorRT engine bindings are incompatible: " + str(error)) from error
        self._trt, self._logger, self._runtime, self._engine = trt, logger, runtime, engine

    def _unavailable(self, code, message):
        if self._ever_enqueued:
            self._poisoned = True
            raise AccelerationExecutionError(code, message)
        raise AccelerationUnavailableError(code, message)

    def __call__(self, images):
        torch = importlib.import_module("torch")
        if torch.is_grad_enabled():
            raise RuntimeError("TensorRT camera inference requires inference_mode/no_grad")
        if not torch.is_tensor(images) or not images.is_cuda:
            self._unavailable("engine_input_incompatible", "TensorRT camera input must be a CUDA tensor")
        device = torch.device(self.device_spec)
        if device.type != "cuda" or device.index is None:
            self._unavailable("engine_input_incompatible", "An explicit CUDA device index is required")
        contract = self.metadata["contract"]
        if images.device != device:
            self._unavailable("engine_input_incompatible", "TensorRT camera input is on a different CUDA device")
        if tuple(images.shape) != tuple(contract["input_shape"]):
            self._unavailable("engine_input_incompatible", "TensorRT camera input shape differs")
        if images.dtype != getattr(torch, contract["input_dtype"]):
            self._unavailable("engine_input_incompatible", "TensorRT camera input dtype differs; implicit casting is disabled")
        if not images.is_contiguous():
            self._unavailable("engine_input_incompatible", "TensorRT camera input must be contiguous NCHW")
        with self._lock, torch.cuda.device(device):
            if self._closed:
                raise RuntimeError("TensorRT camera adapter is closed")
            if self._poisoned:
                raise AccelerationExecutionError("camera_execution_failed", "TensorRT camera adapter failed")
            self._load(torch, device)
            stream = torch.cuda.current_stream(device)
            key = int(stream.cuda_stream)
            if key not in self._contexts:
                if len(self._contexts) >= 2:
                    self._unavailable("engine_stream_contract_incompatible", "Camera engine supports at most two explicitly reused CUDA streams")
                try:
                    context = self._engine.create_execution_context()
                except RuntimeError as error:
                    self._unavailable("engine_context_unavailable", "TensorRT execution context allocation failed: " + str(error))
                if context is None:
                    self._unavailable("engine_context_unavailable", "TensorRT execution context allocation failed")
                self._contexts[key] = dict(context=context, stream=stream, event=None)
            entry = self._contexts[key]
            context = entry["context"]
            try:
                outputs = [torch.empty(tuple(item["shape"]), dtype=getattr(torch, item["dtype"]),
                                       device=device) for item in contract["outputs"]]
            except getattr(torch, 'OutOfMemoryError', MemoryError) as error:
                self._unavailable("engine_output_memory_unavailable", "TensorRT output buffer allocation failed: " + str(error))
            images.record_stream(stream)
            bindings = [(INPUT_NAME, images)] + [(item["name"], output)
                         for item, output in zip(contract["outputs"], outputs)]
            try:
                for name, tensor in bindings:
                    if not context.set_tensor_address(name, int(tensor.data_ptr())):
                        self._unavailable("engine_bindings_incompatible", f"TensorRT rejected CUDA tensor address: {name}")
                # Once an enqueue is attempted, this process must never switch
                # to an eager model after any accelerator execution failure.
                self._ever_enqueued = True
                if not context.execute_async_v3(stream_handle=key):
                    raise AccelerationExecutionError("camera_enqueue_failed", "TensorRT execute_async_v3 failed")
                event = torch.cuda.Event()
                event.record(stream)
                entry["event"] = event
            except AccelerationUnavailableError:
                raise
            except Exception as error:
                self._poisoned = True
                # An enqueue failure may have submitted some GPU work. Do not
                # free CUDA bindings or destroy a partially active context.
                try:
                    torch.cuda.synchronize(device)
                except Exception:
                    pass
                if isinstance(error, AccelerationExecutionError):
                    raise
                raise AccelerationExecutionError("camera_execution_failed",
                    "TensorRT CUDA execution failed: " + str(error)) from error
            kind = contract["output_kind"]
            return outputs[0] if kind == "tensor" else tuple(outputs) if kind == "tuple" else outputs

    def close(self):
        with self._lock:
            if self._closed:
                return
            # Do not leave a loop-local reference to the last context alive
            # while releasing its parent engine/runtime below.
            events = [entry["event"] for entry in self._contexts.values()
                      if entry["event"] is not None]
            for event in events:
                event.synchronize()
            self._contexts.clear()
            self._engine = None
            self._runtime = None
            self._logger = None
            self._trt = None
            self._closed = True

    def status(self):
        """Serializable health identity; never performs CUDA work or fallback."""
        with self._lock:
            engine = self.metadata["engine"]
            contract = self.metadata["contract"]
            return dict(enabled=not self._closed, loaded=self._engine is not None,
                healthy=not self._closed and not self._poisoned,
                state=("closed" if self._closed else "failed" if self._poisoned else
                       "loaded" if self._engine is not None else "not_loaded"),
                engine_sha256=engine["sha256"], metadata_sha256=self.metadata_sha256,
                checkpoint_sha256=contract["checkpoint_sha256"],
                config_sha256=contract["config_sha256"],
                camera_state_sha256=contract["camera_state_sha256"],
                tensorrt_version=engine["tensorrt_version"], gpu_name=engine["gpu_name"],
                compute_capability=engine["compute_capability"], precision=engine["precision"],
                input_dtype=contract["input_dtype"],
                output_dtypes=[item["dtype"] for item in contract["outputs"]],
                output_cast_policy=contract.get("output_cast_policy", "preserve"),
                cuda_device=self.device_spec, scope="camera backbone+neck only",
                eager_fallback=False, production_promoted=False)

    def __enter__(self):
        if self._closed:
            raise RuntimeError("TensorRT camera adapter is closed")
        return self

    def __exit__(self, *_):
        self.close()


@dataclass(frozen=True)
class _CameraFeatures:
    owner: object
    value: object


class _InstalledCameraTensorRT:
    def __init__(self, model, adapter):
        self.model, self.adapter = model, adapter
        self.backbone_forward, self.neck_forward = model.img_backbone.forward, model.img_neck.forward
        self.enabled = True
        model.img_backbone.forward = self.backbone
        model.img_neck.forward = self.neck

    def backbone(self, images):
        if self.model.training:
            raise RuntimeError("Disable TensorRT camera before training")
        return _CameraFeatures(self, self.adapter(images))

    def neck(self, features):
        if not isinstance(features, _CameraFeatures) or features.owner is not self:
            raise RuntimeError("TensorRT camera neck must receive its own fused backbone/neck result")
        return features.value

    def disable(self):
        if self.enabled:
            self.model.img_backbone.forward = self.backbone_forward
            self.model.img_neck.forward = self.neck_forward
            self.enabled = False
        self.adapter.close()

    close = disable

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disable()


def install_camera_tensorrt(model, engine_path, metadata_path=None,
                           expected_contract=None, device="cuda:0"):
    """Explicit reversible installation; existing state_dict keys are retained.

    Existing ParallelBranches keeps using the caller camera side stream;
    image flattening and neck-return handling stay in extract_img_feat.
    Do not combine with camera torch.compile,
    CUDA Graph, or inference Conv/BN folding. No production flag is changed.
    """
    if model.training or model.img_backbone is None or model.img_neck is None:
        raise ValueError("TensorRT camera installation requires an eval camera model")
    if isinstance(getattr(model.img_backbone.forward, "__self__", None), _InstalledCameraTensorRT):
        raise ValueError("TensorRT camera is already installed")
    for module in (model.img_backbone, model.img_neck):
        if (hasattr(module, "_orig_mod") or
                hasattr(module.forward, "_torchdynamo_orig_callable")):
            raise ValueError("Disable camera/neck torch.compile before TensorRT installation")
    contract = dict(expected_contract or {})
    state_hash = camera_state_sha256(model.img_backbone, model.img_neck)
    if "camera_state_sha256" in contract and contract["camera_state_sha256"] != state_hash:
        raise AccelerationUnavailableError("engine_camera_state_mismatch", "Caller camera identity differs from current model")
    contract["camera_state_sha256"] = state_hash
    adapter = CameraTensorRTAdapter(engine_path, metadata_path, contract, device)
    return _InstalledCameraTensorRT(model, adapter)
