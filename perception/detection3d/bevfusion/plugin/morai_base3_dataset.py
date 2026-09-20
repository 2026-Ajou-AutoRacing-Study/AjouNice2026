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

    def parse_data_info(self, info):
        """Preserve converter-authored absolute paths for causal sweeps.

        ``Det3DDataset`` assumes NuScenes' ``samples``/``sweeps`` directory
        layout and reduces every nested sweep path to its basename. MORAI
        stores each past frame beside its four images, so that rewrite loses
        the scene/frame directory. Temporarily remove the nested records while
        the generic parser handles the current frame, then restore them.
        """
        lidar_sweeps = info.pop("lidar_sweeps", None)
        parsed = super().parse_data_info(info)
        if lidar_sweeps is not None:
            parsed["lidar_sweeps"] = lidar_sweeps
        return parsed

    def parse_ann_info(self, info):
        box_dim = int(self.metainfo.get("box_dim", 7))
        ann_info = super().parse_ann_info(info)
        if ann_info is None:
            ann_info = {
                "gt_bboxes_3d": np.zeros((0, box_dim), dtype=np.float32),
                "gt_labels_3d": np.zeros((0,), dtype=np.int64),
            }
        elif ann_info["gt_bboxes_3d"].ndim == 2:
            box_dim = int(ann_info["gt_bboxes_3d"].shape[1])
        ann_info["gt_bboxes_3d"] = LiDARInstance3DBoxes(
            ann_info["gt_bboxes_3d"],
            box_dim=box_dim,
            origin=(0.5, 0.5, 0.5),
        ).convert_to(self.box_mode_3d)
        return ann_info
