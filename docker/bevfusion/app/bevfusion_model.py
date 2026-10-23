"""Load production Base3 or explicit Base5 and run in-memory inference."""

import time

import numpy as np
import torch
from mmengine.config import Config

from mmdet3d.apis import init_model

from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.view_geometry import restore_view_geometry
from app.inference_optimization import (
    InferenceOptimization, validate_base5_runtime,
)


DEFAULT_CLASSES = ["vehicle", "pedestrian", "obstacle"]
BOX_CENTER_ORIGIN = 'lidar_bottom_v1'


def build_warmup_points(point_dim, temporal_num_sweeps, point_count=8192):
    """Create a representative current/causal-sweep runtime point tensor."""
    point_dim = int(point_dim)
    temporal_num_sweeps = int(temporal_num_sweeps)
    point_count = int(point_count)
    if point_dim < 4 or point_count < temporal_num_sweeps:
        raise ValueError("invalid warm-up point shape")
    if temporal_num_sweeps > 1 and point_dim < 5:
        raise ValueError("temporal warm-up requires an age feature")

    generator = np.random.default_rng(260916)
    points = np.zeros((point_count, point_dim), dtype=np.float32)
    points[:, 0] = generator.uniform(-29.5, 29.5, len(points))
    points[:, 1] = generator.uniform(-29.5, 29.5, len(points))
    points[:, 2] = generator.uniform(-2.5, 1.5, len(points))
    points[:, 3] = generator.uniform(0.0, 1.0, len(points))
    if temporal_num_sweeps > 1:
        # Match the online bridge contract: current=0, causal past ages > 0.
        for sweep_index, indices in enumerate(
            np.array_split(np.arange(point_count), temporal_num_sweeps)
        ):
            points[indices, 4] = 0.1 * sweep_index
    return points


class BevFusionModel:
    def __init__(
        self,
        config_path,
        checkpoint_path,
        calibration,
        device="cuda:0",
        use_amp=True,
        preprocess_workers=4,
        warmup_iterations=2,
        temporal_feature_cache=False,
        temporal_maximum_age_sec=0.30,
        base5_runtime=False,
        compile_parts=(),
        fuse_dense_bn=False,
        camera_cuda_graph=False,
        eager_camera_numerics=False,
        rgb_after_crop=False,
        compile_reference_layout=False,
        parallel_camera=False,
        point_intensity_source='native',
        camera_tensorrt_engine=None,
        camera_tensorrt_metadata=None,
        pool_fused_layout=False,
    ):
        from app.camera_tensorrt import (
            load_camera_metadata, sha256_file, validate_camera_tensorrt_options)
        from app.acceleration_startup import AccelerationUnavailableError
        self._closed = False
        self.camera_tensorrt = None
        self.pool_runtime_layout = None
        self.inference_optimization = None
        self.preprocessor = None
        if pool_fused_layout and not base5_runtime:
            raise ValueError("Fused pool layout is an explicit Base5-only eval optimization")
        try:
            requested_trt = validate_camera_tensorrt_options(
                camera_tensorrt_engine, camera_tensorrt_metadata,
                base5_runtime=base5_runtime, use_amp=use_amp, parallel_camera=parallel_camera,
                compile_parts=compile_parts, fuse_dense_bn=fuse_dense_bn,
                camera_cuda_graph=camera_cuda_graph, eager_camera_numerics=eager_camera_numerics,
                compile_reference_layout=compile_reference_layout,
                temporal_feature_cache=temporal_feature_cache, device=device,
                warmup_iterations=warmup_iterations)
        except ValueError as error:
            raise AccelerationUnavailableError("engine_runtime_incompatible", str(error)) from error
        trt_contract = None
        if requested_trt:
            # Match the selected local checkpoint/config before GPU allocation.
            # E12 engine + production E24 checkpoint must fail, never auto-switch.
            trt_contract = dict(checkpoint_sha256=sha256_file(checkpoint_path),
                                config_sha256=sha256_file(config_path))
            try:
                load_camera_metadata(camera_tensorrt_metadata, trt_contract)
            except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
                raise AccelerationUnavailableError("engine_metadata_unavailable",
                    "Camera engine metadata is missing, invalid or incompatible: " + str(error)) from error
        # Runtime configs use legacy _base_ inheritance/deepcopy; do not let
        # MMEngine version-dependent automatic lazy classification change it.
        config = Config.fromfile(config_path, lazy_import=False)
        config.model.img_backbone.init_cfg = None
        bbox_head_type = str(config.model.bbox_head.get("type", ""))
        self.requires_query_memory_context = (
            bbox_head_type == "MoraiQueryMemoryTransFusionHead")
        self.classes = list(config.get("metainfo", {}).get("classes", DEFAULT_CLASSES))
        # Raw exports remain MMDetection3D LiDAR bottom-center coordinates.
        # The optional ROS adapter performs the versioned conversion, not the
        # model, historical payloads, decoder, labels or learned parameters.
        self.box_center_origin = BOX_CENTER_ORIGIN
        if point_intensity_source != 'native' and not base5_runtime:
            raise ValueError('ROS255 normalization is Base5-only; E25 retains raw intensity')
        if not base5_runtime and (self.classes != DEFAULT_CLASSES or
                                  config.get('required_training_label_policy')):
            raise ValueError("runtime class contract differs; use explicit --base5-runtime for Base5")
        if (compile_parts or fuse_dense_bn or camera_cuda_graph or eager_camera_numerics or rgb_after_crop or compile_reference_layout or parallel_camera) and not base5_runtime:
            raise ValueError("Experimental inference optimization is Base5-only")
        if base5_runtime:
            # Training compilation is not an inference opt-in. Loading the
            # learned training config must not compile the camera silently.
            if 'compile_camera' in config.model:
                config.model.compile_camera = False
            validate_base5_runtime(config)
            if temporal_feature_cache:
                raise ValueError("Base5 inference does not enable temporal modeling")
            # Explicit trusted LOCAL checkpoint: strict full-model load, not a
            # camera-only initializer or an unsafe remote URL download.
            from pathlib import Path
            if not Path(checkpoint_path).is_file():
                raise ValueError("Base5 requires a trusted local full checkpoint")
            checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
            saved_classes = checkpoint.get("meta", {}).get("dataset_meta", {}).get("classes")
            if saved_classes is not None and list(saved_classes) != self.classes:
                raise ValueError("Base5 checkpoint class order differs from config")
            self.model = init_model(config, None, device=device)
            self.model.load_state_dict(checkpoint["state_dict"], strict=True)
            del checkpoint
        else:
            self.model = init_model(config, checkpoint_path, device=device)
        restore_view_geometry(self.model.view_transform)
        self.model.eval()
        try:
            if pool_fused_layout:
                from app.pool_runtime_layout import install_fused_pool_layout
                self.pool_runtime_layout = install_fused_pool_layout(self.model.view_transform)
            if requested_trt:
                from app.camera_tensorrt import install_camera_tensorrt
                self.camera_tensorrt = install_camera_tensorrt(
                    self.model, camera_tensorrt_engine, camera_tensorrt_metadata,
                    expected_contract=trt_contract, device=device)
            if compile_parts or fuse_dense_bn or camera_cuda_graph or eager_camera_numerics or compile_reference_layout or parallel_camera:
                self.inference_optimization = InferenceOptimization(
                    self.model, compile_parts, fuse_dense_bn, camera_cuda_graph=camera_cuda_graph,
                    eager_camera_numerics=eager_camera_numerics, reference_layout=compile_reference_layout,
                    parallel_camera=parallel_camera)
        except Exception:
            self.close()
            raise
        # Adapted legacy checkpoints may not carry MMDetection3D 1.x
        # dataset_meta. The explicitly validated runtime config is the
        # authoritative label contract; merged Base5 already learned animals as
        # pedestrian. Never remap a four-class checkpoint at load time.
        self.model.dataset_meta = dict(getattr(self.model, "dataset_meta", None) or {})
        self.model.dataset_meta["classes"] = self.classes
        self.point_dim = int(config.model.pts_voxel_encoder.get(
            "num_features", 4))
        self.temporal_num_sweeps = int(getattr(
            self.model, "temporal_num_sweeps", 1))
        self.temporal_feature_cache = bool(temporal_feature_cache)
        self.temporal_maximum_age_sec = float(temporal_maximum_age_sec)
        if self.temporal_feature_cache:
            enable_cache = getattr(
                self.model, "enable_online_temporal_cache", None)
            if enable_cache is None or self.temporal_num_sweeps <= 1:
                raise ValueError(
                    "temporal feature cache requires a temporal BEV model")
            point_cloud_range = config.get(
                "point_cloud_range",
                (-30.0, -30.0, -3.0, 30.0, 30.0, 2.0),
            )
            enable_cache(point_cloud_range, self.temporal_maximum_age_sec)
        self.use_amp = bool(use_amp)
        self.device_type = torch.device(device).type
        try:
            self.preprocessor = Base3OnlinePreprocessor(
                calibration,
                camera_workers=preprocess_workers,
                point_dim=self.point_dim,
                rgb_after_crop=rgb_after_crop,
                point_intensity_source=(point_intensity_source if base5_runtime else 'native'),
                **preprocessing_options_from_config(config),
            )
        except Exception:
            self.close()
            raise
        self.warmup_iterations = max(0, int(warmup_iterations))
        self.warmed_up = False
        self.last_query_memory_diagnostics = {}
        self.last_runtime_timings = {}
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
        try:
            self._warm_up_before_ready()
        except Exception:
            self.close()
            raise

    def execution_status(self):
        return dict(camera_tensorrt=(self.camera_tensorrt.adapter.status()
                    if self.camera_tensorrt is not None else dict(enabled=False)),
                    pool_fused_layout=(dict(self.pool_runtime_layout.report)
                    if self.pool_runtime_layout is not None else dict(enabled=False)),
                    closed=self._closed)

    def close(self):
        """Drain TRT before releasing its parents and restore opt-in hooks."""
        if getattr(self, '_closed', False):
            return
        first_error = None
        for resource, method in ((getattr(self, 'inference_optimization', None), 'disable'),
                                 (getattr(self, 'camera_tensorrt', None), 'disable'),
                                 (getattr(self, 'pool_runtime_layout', None), 'close'),
                                 (getattr(self, 'preprocessor', None), 'close')):
            if resource is None:
                continue
            try:
                getattr(resource, method)()
            except Exception as error:
                if first_error is None:
                    first_error = error
        self._closed = True
        if first_error is not None:
            raise first_error

    def infer(
        self,
        points,
        image_blobs,
        sample_idx,
        score_threshold,
        max_detections,
        images_preprocessed=False,
        timestamp_sec=None,
        lidar_pose_world=None,
    ):
        if self._closed:
            raise RuntimeError("BEVFusion model is closed")
        preprocess_start = time.perf_counter()
        batch = self.preprocessor.prepare(
            points,
            image_blobs,
            sample_idx,
            images_preprocessed=images_preprocessed,
            timestamp_sec=0.0 if timestamp_sec is None else timestamp_sec,
            lidar_pose_world=lidar_pose_world,
            scene_id="online",
        )

        if self.requires_query_memory_context:
            if timestamp_sec is None or lidar_pose_world is None:
                raise ValueError(
                    "query-memory inference requires timestamp and LiDAR pose")
        preprocess_ms = (time.perf_counter() - preprocess_start) * 1000.0

        if not self.warmed_up:
            self._warm_up(batch)
        self._set_temporal_context(timestamp_sec, lidar_pose_world)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        start = time.perf_counter()
        result = self._forward(batch)[0]
        self.last_query_memory_diagnostics = \
            self._collect_query_memory_diagnostics()
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        inference_ms = (time.perf_counter() - start) * 1000.0
        predictions = result.pred_instances_3d
        postprocess_start = time.perf_counter()
        boxes = predictions.bboxes_3d.tensor.detach().cpu()
        scores = predictions.scores_3d.detach().cpu()
        labels = predictions.labels_3d.detach().cpu()
        keep = torch.nonzero(scores >= float(score_threshold), as_tuple=False).flatten()
        keep = keep[torch.argsort(scores[keep], descending=True)]
        keep = keep[: int(max_detections)]
        detections = []
        for index in keep.tolist():
            values = boxes[index].tolist()
            label = int(labels[index])
            detections.append(
                {
                    "center": values[0:3],
                    "size": values[3:6],
                    "yaw": values[6],
                    "velocity": values[7:9] if len(values) >= 9 else [0.0, 0.0],
                    "label": label,
                    "class_name": self.classes[label]
                    if 0 <= label < len(self.classes)
                    else "class_{}".format(label),
                    "score": float(scores[index]),
                }
            )
        self.last_runtime_timings = dict(
            postprocess_ms=(time.perf_counter() - postprocess_start) * 1000.0)
        return detections, preprocess_ms, inference_ms

    def _collect_query_memory_diagnostics(self):
        """Expose scalar Q1 association diagnostics to the ROS client."""
        head = getattr(self.model, "bbox_head", None)
        memory = getattr(head, "query_memory", None)
        if memory is None:
            return {}
        names = (
            "last_active_query_fraction",
            "last_mean_attention_age_sec",
            "last_mutual_pair_fraction",
            "last_consensus_query_fraction",
            "last_fallback_query_fraction",
            "last_association_distance_mean_m",
            "last_association_distance_p95_m",
            "last_pair_velocity_dispersion_mean_mps",
            "last_pair_velocity_dispersion_p95_mps",
            "last_query_switch_fraction",
            "last_consensus_pair_count_mean",
            "last_single_pair_query_fraction",
            "last_robust_blend_fraction",
            "last_robust_confidence_mean",
        )
        output = {}
        for name in names:
            value = getattr(memory, name, None)
            if value is None:
                continue
            if torch.is_tensor(value):
                if value.numel() != 1:
                    continue
                value = value.detach().float().cpu().item()
            output[name.removeprefix("last_")] = float(value)
        fusion = getattr(head, "query_memory_velocity_fusion_logit", None)
        if fusion is not None:
            output["velocity_fusion_gate"] = float(
                torch.sigmoid(fusion.detach().float()).cpu().item())
        return output

    def _warm_up_before_ready(self):
        """Materialize CUDA kernels before the server advertises readiness.

        Deferring this work to the first live sensor request drops several
        seconds of LiDAR frames on a cold container. The synthetic request has
        the same tensor shapes and a representative voxel count, but carries no
        scene data and is never published.
        """
        if self.warmup_iterations <= 0:
            self.warmed_up = True
            return
        points = build_warmup_points(
            self.point_dim,
            1 if self.temporal_feature_cache else self.temporal_num_sweeps,
        )
        if self.preprocessor.point_intensity_source == 'ros255':
            points[:, 3] *= 255.0
        final_height, final_width = self.preprocessor.final_dim
        empty_image = bytes(final_height * final_width * 3)
        images = {
            "image/{}".format(camera["frame_id"]): empty_image
            for camera in self.preprocessor.calibration["cameras"]
        }
        batch = self.preprocessor.prepare(
            points,
            images,
            sample_idx="startup_warmup",
            images_preprocessed=True,
            timestamp_sec=0.0,
            lidar_pose_world=(0.0, 0.0, 0.0, 0.0),
            scene_id="startup_warmup",
        )
        self._warm_up(batch)

    def _warm_up(self, batch):
        for index in range(self.warmup_iterations):
            if self.temporal_feature_cache:
                self._set_temporal_context(
                    0.1 * index, (0.0, 0.0, 0.0, 0.0))
            self._forward(batch)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        self.warmed_up = True
        if self.temporal_feature_cache:
            self.model.reset_online_temporal_cache()

    def _set_temporal_context(self, timestamp_sec, lidar_pose_world):
        if not self.temporal_feature_cache:
            return
        if timestamp_sec is None or lidar_pose_world is None:
            raise ValueError(
                "cached temporal inference requires timestamp and LiDAR pose")
        self.model.set_online_temporal_context(
            float(timestamp_sec), lidar_pose_world)

    def _forward(self, batch):
        with torch.inference_mode():
            with torch.autocast(
                device_type=self.device_type,
                dtype=torch.float16,
                enabled=self.use_amp and self.device_type == "cuda",
            ):
                return self.model.test_step(batch)
