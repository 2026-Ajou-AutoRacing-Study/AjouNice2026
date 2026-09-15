"""Load Base3 BEVFusion and run in-memory low-latency inference."""

import time

import numpy as np
import torch
from mmengine.config import Config

from mmdet3d.apis import init_model

from app.online_preprocessor import Base3OnlinePreprocessor


DEFAULT_CLASSES = ["vehicle", "pedestrian", "obstacle"]


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
    ):
        config = Config.fromfile(config_path)
        config.model.img_backbone.init_cfg = None
        self.model = init_model(config, checkpoint_path, device=device)
        self.model.eval()
        self.classes = list(config.get("metainfo", {}).get(
            "classes", DEFAULT_CLASSES
        ))
        if self.classes != DEFAULT_CLASSES:
            raise ValueError(
                "runtime requires Base3 classes {}, checkpoint exposes {}".format(
                    DEFAULT_CLASSES, self.classes
                )
            )
        # Adapted legacy checkpoints may not carry MMDetection3D 1.x
        # dataset_meta. The output head is still MORAI 3-class, so the runtime
        # config is the authoritative label contract.
        self.model.dataset_meta = dict(self.model.dataset_meta or {})
        self.model.dataset_meta["classes"] = self.classes
        self.use_amp = bool(use_amp)
        self.device_type = torch.device(device).type
        self.preprocessor = Base3OnlinePreprocessor(
            calibration, camera_workers=preprocess_workers
        )
        self.warmup_iterations = max(0, int(warmup_iterations))
        self.warmed_up = False
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
        self._warm_up_before_ready()

    def infer(
        self,
        points,
        image_blobs,
        sample_idx,
        score_threshold,
        max_detections,
        images_preprocessed=False,
    ):
        preprocess_start = time.perf_counter()
        batch = self.preprocessor.prepare(
            points,
            image_blobs,
            sample_idx,
            images_preprocessed=images_preprocessed,
        )
        preprocess_ms = (time.perf_counter() - preprocess_start) * 1000.0

        if not self.warmed_up:
            self._warm_up(batch)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        start = time.perf_counter()
        result = self._forward(batch)[0]
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        inference_ms = (time.perf_counter() - start) * 1000.0
        predictions = result.pred_instances_3d
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
        return detections, preprocess_ms, inference_ms

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
        generator = np.random.default_rng(260916)
        points = np.empty((8192, 4), dtype=np.float32)
        points[:, 0] = generator.uniform(-29.5, 29.5, len(points))
        points[:, 1] = generator.uniform(-29.5, 29.5, len(points))
        points[:, 2] = generator.uniform(-2.5, 1.5, len(points))
        points[:, 3] = generator.uniform(0.0, 1.0, len(points))
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
        )
        self._warm_up(batch)

    def _warm_up(self, batch):
        for _ in range(self.warmup_iterations):
            self._forward(batch)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        self.warmed_up = True

    def _forward(self, batch):
        with torch.inference_mode():
            with torch.autocast(
                device_type=self.device_type,
                dtype=torch.float16,
                enabled=self.use_amp and self.device_type == "cuda",
            ):
                return self.model.test_step(batch)
