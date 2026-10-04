"""Center-distance validation metric shared with MORAI PointPainting."""

from typing import Dict, Sequence

import numpy as np
from mmengine.evaluator import BaseMetric

from mmdet3d.registry import METRICS
from perception.detection3d.common.evaluation.morai_detection import (
    morai_detection_eval,
)


class _Boxes:
    def __init__(self, values):
        import torch

        self.tensor = torch.as_tensor(values, dtype=torch.float32)


@METRICS.register_module()
class MoraiCenterMetric(BaseMetric):
    """Report stable dashboard keys without requiring the nuScenes devkit."""

    default_prefix = None

    def __init__(self, point_cloud_range=None, **kwargs):
        super().__init__(**kwargs)
        self.point_cloud_range = point_cloud_range

    def process(self, data_batch: dict, data_samples: Sequence[dict]) -> None:
        for sample in data_samples:
            pred = sample["pred_instances_3d"]
            ann = sample["eval_ann_info"]
            self.results.append({
                "info": {
                    "gt_boxes": ann["gt_bboxes_3d"].tensor.cpu().numpy(),
                    "gt_names": np.asarray([
                        self.dataset_meta["classes"][int(label)]
                        for label in ann["gt_labels_3d"]
                    ]),
                    "num_lidar_pts": np.asarray(
                        ann.get("num_lidar_pts", np.ones(len(ann["gt_labels_3d"]))),
                        dtype=np.int64,
                    ),
                    "valid_flag": np.ones(len(ann["gt_labels_3d"]), dtype=bool),
                },
                "prediction": {
                    "boxes_3d": _Boxes(pred["bboxes_3d"].tensor.cpu().numpy()),
                    "scores_3d": pred["scores_3d"].cpu(),
                    "labels_3d": pred["labels_3d"].cpu(),
                },
            })

    def compute_metrics(self, results: list) -> Dict[str, float]:
        infos = [row["info"] for row in results]
        predictions = [row["prediction"] for row in results]
        return morai_detection_eval(
            infos,
            predictions,
            list(self.dataset_meta["classes"]),
            use_valid_flag=True,
            point_cloud_range=self.point_cloud_range,
        )
