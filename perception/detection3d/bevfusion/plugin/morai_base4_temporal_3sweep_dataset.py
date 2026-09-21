"""MMDetection3D dataset adapter for MORAI Base4-Temporal-3S."""

import numpy as np

from mmdet3d.datasets import Det3DDataset
from mmdet3d.registry import DATASETS
from mmdet3d.structures import LiDARInstance3DBoxes


@DATASETS.register_module()
class MoraiBase4Temporal3SweepDataset(Det3DDataset):
    """Three-class MORAI dataset with 3-sweep LiDAR and velocity GT.

    Point format:
        [x, y, z, intensity, time_lag_sec]

    Box format:
        [x, y, z, length, width, height, yaw, vx, vy]

    Boxes and velocities are expressed in the current LiDAR frame.
    """

    METAINFO = {
        "classes": ("vehicle", "pedestrian", "obstacle"),
        "palette": ((0, 255, 0), (255, 128, 0), (0, 128, 255)),
    }

    def parse_ann_info(self, info):
        ann_info = super().parse_ann_info(info)

        if ann_info is None:
            ann_info = {
                "gt_bboxes_3d": np.zeros((0, 9), dtype=np.float32),
                "gt_labels_3d": np.zeros((0,), dtype=np.int64),
            }

        boxes = np.asarray(
            ann_info["gt_bboxes_3d"], dtype=np.float32
        ).reshape(-1, 9)

        ann_info["gt_bboxes_3d"] = LiDARInstance3DBoxes(
            boxes,
            box_dim=9,
            origin=(0.5, 0.5, 0.5),
        ).convert_to(self.box_mode_3d)

        return ann_info
