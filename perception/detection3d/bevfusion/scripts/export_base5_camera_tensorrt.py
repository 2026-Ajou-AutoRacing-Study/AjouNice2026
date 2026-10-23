#!/usr/bin/env python3
"""Opt-in Base5 camera-only ONNX export / target-GPU TensorRT 10 build.

No install, download, checkpoint mutation, model resize, or production change.
Default FP32 ONNX bindings + FP16 TRT tactics preserve normalized FP32 inputs.
--onnx-dtype float16 is a separate full-half export experiment, not AMP parity.
Use a new output directory. GPU execution occurs only when this CLI is run.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import re
import shutil
import sys


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", type=Path, required=True)
    result.add_argument("--checkpoint", type=Path, required=True,
                        help="Trusted local E12/full Base5 checkpoint; never a URL")
    result.add_argument("--output-dir", type=Path, required=True)
    result.add_argument("--device", default="cuda:0")
    result.add_argument("--onnx-dtype", choices=("float32", "float16"), default="float32")
    result.add_argument("--output-dtype", choices=("preserve", "float16"), default="preserve",
                        help="Explicit neck-boundary A/B: preserve ONNX computation output dtype or cast only final neck outputs to FP16; input dtype is unchanged")
    result.add_argument("--precision", choices=("fp32", "fp16"), default="fp16")
    result.add_argument("--opset", type=int, default=17)
    result.add_argument("--workspace-mib", type=int, default=1024)
    result.add_argument("--fp32-layer-regex", action="append", default=[],
                        help="Explicit FP16-engine numerical probe: force matching floating-point layers/output to FP32; may be repeated")
    result.add_argument("--fp32-layer-types", nargs="*", default=[],
                        choices=("SOFTMAX", "REDUCE", "NORMALIZATION"),
                        help="Explicit FP16-engine numerical probe; obey precision constraints")
    result.add_argument("--build-engine", action="store_true",
                        help="Build on the actual target GPU after ONNX validation")
    result.add_argument("--build-only", type=Path,
                        help="Existing ONNX metadata JSON; no checkpoint/model load; still requires matching --config/--checkpoint SHA")
    result.add_argument("--dry-run", action="store_true",
                        help="Validate CLI and local paths only; do not import torch/CUDA or create files")
    result.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[4])
    return result


def _app(root):
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "docker/bevfusion"))
    return importlib.import_module("app.camera_tensorrt")


def _write_new(path, payload):
    with path.open("x") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _source_hashes(root, camera, app):
    paths = {Path(__file__).resolve(), Path(inspect.getfile(app)).resolve()}
    for module in camera.modules():
        path = inspect.getsourcefile(type(module))
        if path is not None:
            paths.add(Path(path).resolve())
    result = {}
    for path in sorted(paths):
        try:
            key = str(path.relative_to(root))
        except ValueError:
            key = str(path)
        result[key] = app.sha256_file(path)
    return result


def export(args, app):
    import torch
    import onnx
    from mmengine.config import Config
    from mmengine.utils import import_modules_from_strings
    from mmdet3d.registry import MODELS
    from mmdet3d.utils import register_all_modules
    from app.inference_optimization import validate_base5_runtime

    device = torch.device(args.device)
    if device.type != "cuda" or device.index is None or not torch.cuda.is_available():
        raise ValueError("Export requires an explicit available CUDA device")
    # These audited configs use legacy _base_ inheritance plus deepcopy.
    # Automatic lazy-import detection differs between MMEngine releases.
    cfg = Config.fromfile(str(args.config), lazy_import=False)
    cfg.model.img_backbone.init_cfg = None
    if "compile_camera" in cfg.model:
        cfg.model.compile_camera = False
    validate_base5_runtime(cfg)
    if tuple(cfg.model.view_transform.image_size) != (512, 1056):
        raise ValueError("Only the current 512x1056 camera contract can be exported")
    if cfg.get("temporal_num_sweeps", 1) != 1:
        raise ValueError("Temporal camera export is outside this experiment")
    register_all_modules(init_default_scope=True)
    if cfg.get("custom_imports"):
        import_modules_from_strings(**cfg.custom_imports)
    source_checkpoint_sha = app.sha256_file(args.checkpoint)
    source_config_sha = app.sha256_file(args.config)
    checkpoint = torch.load(str(args.checkpoint), map_location="cpu", weights_only=False)
    classes = checkpoint.get("meta", {}).get("dataset_meta", {}).get("classes")
    if classes is not None and list(classes) != list(cfg.metainfo.classes):
        raise ValueError("Checkpoint/config class identity differs")
    state = checkpoint["state_dict"]

    class Camera(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.img_backbone = MODELS.build(deepcopy(cfg.model.img_backbone))
            self.img_neck = MODELS.build(deepcopy(cfg.model.img_neck))

        def forward(self, images):
            outputs = self.img_neck(self.img_backbone(images))
            if args.output_dtype == "preserve":
                return outputs
            if torch.is_tensor(outputs):
                return outputs.to(dtype=torch.float16)
            if isinstance(outputs, tuple):
                return tuple(value.to(dtype=torch.float16) for value in outputs)
            if isinstance(outputs, list):
                return [value.to(dtype=torch.float16) for value in outputs]
            raise TypeError("Camera neck must return Tensor, tuple, or list")

    camera = Camera().eval()
    camera_state = {name: value for name, value in state.items()
                    if name.startswith(("img_backbone.", "img_neck."))}
    camera.load_state_dict(camera_state, strict=True)
    state_hash = app.camera_state_sha256(camera.img_backbone, camera.img_neck)
    source_hashes = _source_hashes(args.root, camera, app)
    del checkpoint, state, camera_state
    dtype = getattr(torch, args.onnx_dtype)
    camera.to(device=device, dtype=dtype)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    # Fixed synthetic shape establishes wiring only; it does not establish AP
    # or parity with the eager autocast runtime on recorded inputs.
    images = torch.zeros((4, 3, 512, 1056), device=device, dtype=dtype)
    with torch.inference_mode(), torch.autocast("cuda", enabled=False):
        outputs = camera(images)
        kind = "tensor" if torch.is_tensor(outputs) else "tuple" if isinstance(outputs, tuple) else "list" if isinstance(outputs, list) else None
        if kind is None:
            raise TypeError("Camera neck must return Tensor, tuple, or list")
        values = [outputs] if kind == "tensor" else list(outputs)
        if not values or not all(torch.is_tensor(value) for value in values):
            raise TypeError("Camera neck contains unsupported nested/non-tensor outputs")
        specifications = [dict(name=f"camera_feature_{index}", shape=list(value.shape),
                               dtype=str(value.dtype).removeprefix("torch."))
                          for index, value in enumerate(values)]
        destination = args.output_dir / "camera.onnx"
        torch.onnx.export(camera, (images,), str(destination), input_names=[app.INPUT_NAME],
                          output_names=[item["name"] for item in specifications],
                          opset_version=args.opset, dynamo=False,
                          do_constant_folding=True, training=torch.onnx.TrainingMode.EVAL)
    graph = onnx.load(str(destination), load_external_data=True)
    onnx.checker.check_model(graph, full_check=True)
    # Keep the deliverable single-file; no untracked external weight shards.
    if any(value.data_location == onnx.TensorProto.EXTERNAL for value in graph.graph.initializer):
        raise ValueError("External ONNX weight files are not supported by this artifact contract")
    metadata = dict(schema=app.SCHEMA, stage="onnx", created_utc=datetime.now(timezone.utc).isoformat(),
                    contract=dict(input_name=app.INPUT_NAME, input_shape=list(images.shape),
                                  input_dtype=args.onnx_dtype,
                                  input_semantics="normalized_flattened_BNCHW",
                                  output_kind=kind, outputs=specifications,
                                  output_cast_policy=("preserve" if args.output_dtype == "preserve"
                                                      else "float16_after_neck"),
                                  checkpoint_sha256=source_checkpoint_sha,
                                  config_sha256=source_config_sha,
                                  resolved_config_sha256=hashlib.sha256(cfg.pretty_text.encode()).hexdigest(),
                                  camera_state_sha256=state_hash,
                                  source_sha256=source_hashes),
                    onnx=dict(file=destination.name, sha256=app.sha256_file(destination),
                              opset=args.opset),
                    export=dict(torch_version=str(torch.__version__), torch_cuda=torch.version.cuda,
                                gpu_name=torch.cuda.get_device_name(device),
                                compute_capability=list(torch.cuda.get_device_capability(device)),
                                output_dtype=args.output_dtype,
                                precision_note="No autocast during ONNX export; optional final neck FP16 cast does not change input dtype or make internal TRT computation equivalent to eager AMP; parity is unqualified"))
    app.validate_camera_metadata(metadata)
    if (app.sha256_file(args.checkpoint) != source_checkpoint_sha or
            app.sha256_file(args.config) != source_config_sha or
            _source_hashes(args.root, camera, app) != source_hashes):
        raise RuntimeError("Input changed during camera export")
    _write_new(args.output_dir / "camera.onnx.json", metadata)
    return destination, metadata


def build(args, app, onnx_path, metadata):
    import torch
    import tensorrt as trt

    if str(trt.__version__).split(".")[0] != "10":
        raise RuntimeError("This tool targets TensorRT 10.x; no automatic TRT 11/CUDA upgrade")
    device = torch.device(args.device)
    if device.type != "cuda" or device.index is None or not torch.cuda.is_available():
        raise ValueError("Engine build requires an explicit target CUDA device")
    app.validate_camera_metadata(metadata)
    if app.sha256_file(onnx_path) != metadata["onnx"]["sha256"]:
        raise ValueError("ONNX SHA256 differs from metadata")
    logger = trt.Logger(trt.Logger.WARNING)
    builder_sources = {str(Path(__file__).resolve()): app.sha256_file(__file__),
                       str(Path(inspect.getfile(app)).resolve()): app.sha256_file(inspect.getfile(app))}
    with torch.cuda.device(device):
        builder = trt.Builder(logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        onnx_parser = trt.OnnxParser(network, logger)
        if not onnx_parser.parse(onnx_path.read_bytes()):
            errors = [str(onnx_parser.get_error(index)) for index in range(onnx_parser.num_errors)]
            raise RuntimeError("TensorRT ONNX parser rejected camera graph:\n" + "\n".join(errors))
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, args.workspace_mib * 1024 * 1024)
        config.clear_flag(trt.BuilderFlag.TF32)
        if args.precision == "fp16":
            if not builder.platform_has_fast_fp16:
                raise RuntimeError("Target GPU does not report fast FP16")
            config.set_flag(trt.BuilderFlag.FP16)
        elif metadata["contract"]["input_dtype"] == "float16":
            raise ValueError("FP32 comparison requires float32 ONNX bindings, not full-half ONNX")
        precision_layers = []
        if args.fp32_layer_regex or args.fp32_layer_types:
            if args.precision != "fp16":
                raise ValueError("FP32 layer constraints are only a separate FP16-engine numerical probe")
            patterns = [re.compile(pattern) for pattern in args.fp32_layer_regex]
            selected_types = {getattr(trt.LayerType, name) for name in args.fp32_layer_types
                              if hasattr(trt.LayerType, name)}
            if len(selected_types) != len(set(args.fp32_layer_types)):
                raise ValueError("Requested FP32 layer type is unsupported by this TensorRT version")
            for index in range(network.num_layers):
                layer = network.get_layer(index)
                if (layer.type not in selected_types and
                        not any(pattern.search(layer.name) for pattern in patterns)):
                    continue
                floating_outputs = [output for output in range(layer.num_outputs)
                                    if layer.get_output(output).dtype in (trt.float32, trt.float16)]
                if not floating_outputs:
                    continue
                layer.precision = trt.float32
                for output in floating_outputs:
                    layer.set_output_type(output, trt.float32)
                precision_layers.append(layer.name)
            if not precision_layers:
                raise ValueError("FP32 layer constraints matched no floating-point layers")
            config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            raise RuntimeError("TensorRT engine build failed; no fallback or partial engine will be published")
        result = deepcopy(metadata)
        destination = args.output_dir / "camera.engine"
        payload = bytes(serialized)
        result.update(stage="engine", created_utc=datetime.now(timezone.utc).isoformat())
        result["engine"] = dict(file=destination.name, sha256=hashlib.sha256(payload).hexdigest(),
                                tensorrt_version=str(trt.__version__), precision=args.precision,
                                gpu_name=torch.cuda.get_device_name(device),
                                compute_capability=list(torch.cuda.get_device_capability(device)),
                                gpu_total_memory=int(torch.cuda.get_device_properties(device).total_memory),
                                torch_version=str(torch.__version__), torch_cuda=torch.version.cuda,
                                workspace_mib=args.workspace_mib, tf32=False,
                                fp32_layer_regex=args.fp32_layer_regex,
                                fp32_layer_types=args.fp32_layer_types,
                                fp32_constrained_layers=precision_layers,
                                builder_source_sha256=builder_sources,
                                hardware_compatibility="none", accuracy_qualified=False)
        app.validate_camera_metadata(result)
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(payload)
        if engine is None:
            raise RuntimeError("Built engine failed deserialization")
        app.validate_engine_bindings(engine, result, trt)
        if any(app.sha256_file(path) != digest for path, digest in builder_sources.items()):
            raise RuntimeError("Builder source changed during engine build")
        if app.sha256_file(onnx_path) != metadata["onnx"]["sha256"]:
            raise RuntimeError("ONNX input changed during engine build")
        with destination.open("xb") as stream:
            stream.write(payload)
        _write_new(Path(str(destination) + ".json"), result)
        del engine, runtime
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    args.root = args.root.resolve()
    args.config, args.checkpoint = args.config.resolve(), args.checkpoint.resolve()
    if not args.config.is_file() or not args.checkpoint.is_file():
        raise ValueError("Trusted local config/checkpoint files are required")
    if args.workspace_mib <= 0 or args.opset < 17:
        raise ValueError("Positive workspace and ONNX opset >=17 are required")
    if args.build_only and not args.build_engine:
        raise ValueError("--build-only requires explicit --build-engine")
    if args.output_dir.exists():
        raise FileExistsError("Use a new output directory; prior experiments are never overwritten")
    if args.dry_run:
        print(json.dumps(dict(event="camera_export_dry_run", gpu_executed=False,
                              onnx_dtype=args.onnx_dtype, precision=args.precision,
                              output_dtype=args.output_dtype,
                              build_engine=args.build_engine, input_shape=[4, 3, 512, 1056])))
        return
    app = _app(args.root)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    if args.build_only:
        metadata = app.load_camera_metadata(args.build_only)
        if (metadata["contract"]["checkpoint_sha256"] != app.sha256_file(args.checkpoint) or
                metadata["contract"]["config_sha256"] != app.sha256_file(args.config)):
            raise ValueError("Build-only config/checkpoint identity differs")
        onnx_path = args.build_only.parent / metadata["onnx"]["file"]
        if app.sha256_file(onnx_path) != metadata["onnx"]["sha256"]:
            raise ValueError("Build-only ONNX identity differs")
        destination = args.output_dir / "camera.onnx"
        shutil.copyfile(onnx_path, destination)
        onnx_path = destination
        metadata = deepcopy(metadata)
        metadata["onnx"]["file"] = destination.name
        _write_new(args.output_dir / "camera.onnx.json", metadata)
    else:
        onnx_path, metadata = export(args, app)
    if args.build_engine:
        metadata = build(args, app, onnx_path, metadata)
    print(json.dumps(dict(event="camera_artifact_ready", stage=metadata["stage"],
                          output=str(args.output_dir), accuracy_qualified=False)))


if __name__ == "__main__":
    main()
