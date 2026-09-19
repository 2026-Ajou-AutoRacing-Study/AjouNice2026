"""MMDetection3D v2 dataset adapter for the MORAI Base3 sensor export."""

import numpy as np

from mmdet3d.datasets import Det3DDataset
from mmdet3d.registry import DATASETS
from mmdet3d.structures import LiDARInstance3DBoxes


@DATASETS.register_module()
class MoraiBase3Dataset(Det3DDataset):
    """Three-class, four-camera plus LiDAR MORAI dataset.

    Base3 boxes are already expressed in the LiDAR frame and use a geometric
    center origin. Sparse boxes are removed by the info builder, before both
    training and evaluation, so the two paths share exactly one GT contract.
    """

    METAINFO = {
        "classes": ("vehicle", "pedestrian", "obstacle"),
        "palette": ((0, 255, 0), (255, 128, 0), (0, 128, 255)),
    }

    def parse_ann_info(self, info):
        ann_info = super().parse_ann_info(info)
        if ann_info is None:
            ann_info = {
                "gt_bboxes_3d": np.zeros((0, 7), dtype=np.float32),
                "gt_labels_3d": np.zeros((0,), dtype=np.int64),
            }
        ann_info["gt_bboxes_3d"] = LiDARInstance3DBoxes(
            ann_info["gt_bboxes_3d"],
            box_dim=7,
            origin=(0.5, 0.5, 0.5),
        ).convert_to(self.box_mode_3d)
        return ann_info
