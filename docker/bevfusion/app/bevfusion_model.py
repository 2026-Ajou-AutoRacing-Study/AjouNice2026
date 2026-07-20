"""Load the verified BEVFusion checkpoint once and return JSON-safe 3D detections."""

import time

import torch
from mmengine.config import Config

from mmdet3d.apis import inference_multi_modality_detector, init_model


DEFAULT_CLASSES = [
    "car",
    "truck",
    "construction_vehicle",
    "bus",
    "trailer",
    "barrier",
    "motorcycle",
    "bicycle",
    "pedestrian",
    "traffic_cone",
]


class BevFusionModel:
    def __init__(self, config_path, checkpoint_path, device="cuda:0"):
        config = Config.fromfile(config_path)
        config.model.img_backbone.init_cfg = None
        pipeline = config.test_dataloader.dataset.pipeline
        config.test_dataloader.dataset.pipeline = [
            item for item in pipeline if item["type"] != "LoadPointsFromMultiSweeps"
        ]
        self.model = init_model(config, checkpoint_path, device=device)
        self.classes = list(
            self.model.dataset_meta.get("classes", DEFAULT_CLASSES)
        )

    def infer(self, points_path, image_directory, info_path, score_threshold, max_detections):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        start = time.monotonic()
        result, _data = inference_multi_modality_detector(
            self.model, points_path, image_directory, info_path, cam_type="all"
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        inference_ms = (time.monotonic() - start) * 1000.0
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
        return detections, inference_ms

