"""Opt-in, reversible inference-only optimizations; never mutate a checkpoint.

Compile bound callables, not modules, so checkpoint keys stay unchanged. Dense
Conv/BN folding is experimental and must pass prediction parity qualification.
Sparse convolutions and the detection head are deliberately excluded.
"""
import torch
import threading
from torch import nn
from torch.nn.utils.fusion import fuse_conv_bn_eval


BASE5_CLASSES = ["vehicle", "pedestrian", "obstacle", "animal"]
COMPILE_PARTS = {"camera": ("img_backbone",), "neck": ("img_neck",),
                 "bev": ("fusion_layer", "pts_backbone", "pts_neck")}


def validate_base5_runtime(config):
    if list(config.get("metainfo", {}).get("classes", ())) != BASE5_CLASSES:
        raise ValueError("Base5 runtime requires the explicit four-class order")
    if (config.model.type not in ("BEVFusion", "MoraiTrainingRuntimeBEVFusion") or
            config.model.pts_voxel_encoder.num_features != 4 or
            config.model.get("compile_camera", False)):
        raise ValueError("Base5 runtime requires a single-frame four-field BEVFusion")
    # code_size=8 encodes yaw as sin/cos; it decodes to a seven-field box.
    if (config.model.bbox_head.bbox_coder.code_size != 8 or
            "vel" in config.model.bbox_head.common_heads):
        raise ValueError("Base5 runtime requires a 7D head without OD velocity")
    if config.get("required_camera_input_policy") != "native_jpeg90":
        raise ValueError("Base5 runtime requires native-JPEG preprocessing")
    if config.get("required_box_geometry_policy") != "native_rpy_fixed_yaw_enclosure_v1":
        raise ValueError("Base5 runtime requires the current upright box contract")


class InferenceOptimization:
    def __init__(self, model, compile_parts=(), fuse_dense_bn=False,
                 emulate_precision_casts=True, camera_cuda_graph=False,
                 eager_camera_numerics=False, reference_layout=False):
        if model.training:
            raise ValueError("Inference optimization requires model.eval()")
        parts = tuple(dict.fromkeys(compile_parts))
        if set(parts) - COMPILE_PARTS.keys():
            raise ValueError("Unknown compile part")
        if camera_cuda_graph and "camera" in parts:
            raise ValueError("Choose camera compile OR CUDA Graph, not both")
        if eager_camera_numerics and "camera" not in parts:
            raise ValueError("Eager camera numerics requires camera compile")
        if reference_layout and not parts:
            raise ValueError("Reference layout requires a compiled part")
        self.model = model
        self.forwards = []
        self.folded = []
        self.original_train = model.train
        self.enabled = True
        try:
            if eager_camera_numerics:
                # Explicit graph breaks preserve eager normalization/nonlinear
                # reductions rather than silently relaxing output tolerance.
                for module in model.img_backbone.modules():
                    if isinstance(module, (nn.LayerNorm, nn.GELU)):
                        original = module.forward
                        self.forwards.append((module, original))
                        module.forward = torch._dynamo.disable(original)
            if fuse_dense_bn:
                roots = [getattr(model, name) for name in
                         ("img_neck", "fusion_layer", "pts_backbone", "pts_neck")]
                roots += [getattr(model.view_transform, name) for name in
                          ("dtransform", "depthnet", "downsample") if
                          hasattr(model.view_transform, name)]
                visited = set()
                for root in roots:
                    self._fold(root, visited)
            for part in parts:
                for name in COMPILE_PARTS[part]:
                    module = getattr(model, name)
                    original = module.forward
                    options = {"emulate_precision_casts": bool(emulate_precision_casts)}
                    if reference_layout:
                        options.update(layout_optimization=False, force_layout_optimization=False,
                                       shape_padding=False)
                    compiled = torch.compile(original, dynamic=False, options=options)
                    self.forwards.append((module, original))
                    module.forward = compiled
            if camera_cuda_graph:
                module = model.img_backbone
                original = module.forward
                self.forwards.append((module, original))
                module.forward = CameraCudaGraph(module, original)

            def inference_only_train(mode=True):
                if mode:
                    raise RuntimeError("Disable inference optimization before training")
                return self.original_train(False)

            model.train = inference_only_train
            self.report = dict(compile_parts=list(parts), folded_dense_bn=len(self.folded),
                camera_cuda_graph=bool(camera_cuda_graph),
                eager_camera_numerics=bool(eager_camera_numerics),
                reference_layout=bool(reference_layout),
                emulate_precision_casts=bool(emulate_precision_casts),
                scope="eval only; source checkpoint is unchanged")
        except Exception:
            self.disable()
            raise

    def _fold(self, module, visited):
        if id(module) in visited:
            return
        visited.add(id(module))
        # MMCV ConvModule registers normalization as `bn` and has explicit order.
        order = getattr(module, "order", ())
        if (tuple(order) == ("conv", "norm", "act") and
                isinstance(getattr(module, "conv", None), nn.Conv2d) and
                isinstance(getattr(module, "norm", None), nn.BatchNorm2d)):
            self._replace(module, "conv", getattr(module, "norm_name"))
        if isinstance(module, nn.Sequential):
            children = list(module._modules.items())
            for (left_name, left), (right_name, right) in zip(children, children[1:]):
                if isinstance(left, nn.Conv2d) and isinstance(right, nn.BatchNorm2d):
                    self._replace(module, left_name, right_name)
        for child in module.children():
            self._fold(child, visited)

    def _replace(self, parent, conv_name, bn_name):
        conv, bn = getattr(parent, conv_name), getattr(parent, bn_name)
        if not bn.track_running_stats:
            return
        fused = fuse_conv_bn_eval(conv, bn)
        self.folded.append((parent, conv_name, conv, bn_name, bn))
        setattr(parent, conv_name, fused)
        setattr(parent, bn_name, nn.Identity())

    def disable(self):
        if not self.enabled:
            return
        for module, original in reversed(self.forwards):
            module.forward = original
        for parent, conv_name, conv, bn_name, bn in reversed(self.folded):
            setattr(parent, conv_name, conv)
            setattr(parent, bn_name, bn)
        self.model.train = self.original_train
        self.enabled = False


class CameraCudaGraph:
    """Replay unchanged eager kernels for a serial fixed-shape camera service.

    Each stream owns its input/output storage. Outputs are borrowed until the
    next call on that stream, like an inference engine binding. No concurrent
    consumers on one stream, training, gradients, or cross-stream output reuse.
    Calibration, LiDAR and detection decoding stay outside this graph.
    """
    def __init__(self, module, forward):
        self.module = module
        self.forward = forward
        self.entries = {}
        self.lock = threading.Lock()

    def __call__(self, images):
        if self.module.training or torch.is_grad_enabled():
            raise RuntimeError("Camera CUDA Graph is inference-only")
        if not images.is_cuda:
            raise ValueError("Camera CUDA Graph requires CUDA input")
        stream = torch.cuda.current_stream(images.device)
        amp = torch.is_autocast_enabled('cuda')
        dtype = torch.get_autocast_dtype('cuda')
        key = (tuple(images.shape), images.stride(), images.dtype, images.device,
               amp, dtype, stream.cuda_stream,
               tuple(p._version for p in self.module.parameters()))
        with self.lock:
            if key not in self.entries:
                if len(self.entries) >= 2:
                    raise RuntimeError("Camera CUDA Graph supports at most two fixed stream/shape contracts")
                static_input = torch.empty_like(images)
                static_input.copy_(images)
                side_stream = torch.cuda.Stream(device=images.device)
                side_stream.wait_stream(stream)
                # Prevent AMP weight cache references to allocations outside
                # the graph from escaping the surrounding autocast context.
                with torch.cuda.stream(side_stream), torch.autocast('cuda',
                        enabled=amp, dtype=dtype, cache_enabled=False):
                    for _ in range(4): self.forward(static_input)
                stream.wait_stream(side_stream)
                graph = torch.cuda.CUDAGraph()
                with torch.cuda.graph(graph, stream=side_stream), torch.autocast('cuda',
                        enabled=amp, dtype=dtype, cache_enabled=False):
                    outputs = self.forward(static_input)
                stream.wait_stream(side_stream)
                self.entries[key] = (static_input, graph, outputs)
            static_input, graph, outputs = self.entries[key]
            static_input.copy_(images)
            graph.replay()
            return outputs
