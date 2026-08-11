"""Fast-BEV dataset adapter that accepts MORAI's four-camera rig."""

import numpy as np

from mmdet.datasets import DATASETS
from mmdet3d.datasets.dataset_wrappers import MultiViewMixin
from mmdet3d.datasets.nuscenes_dataset import NuScenesDataset


def _float32(value):
    return value.astype(np.float32) if value is not None else None


@DATASETS.register_module()
class MoraiMultiViewDataset(MultiViewMixin, NuScenesDataset):
    """NuScenes-info-compatible dataset without the upstream 6-camera assert."""

    def get_data_info(self, index):
        data_info = NuScenesDataset.get_data_info(self, index)
        camera_count = len(data_info["img_filename"])
        if camera_count != 4:
            raise AssertionError("MORAI Sensor_6 requires exactly four cameras")
        result = {
            "sample_idx": data_info["sample_idx"],
            "img_prefix": [None] * camera_count,
            "img_info": [dict(filename=value) for value in data_info["img_filename"]],
            "lidar2img": {
                "extrinsic": [_float32(value) for value in data_info["lidar2img"]],
                "intrinsic": np.eye(4, dtype=np.float32),
                "lidar2img_aug": data_info["lidar2img_aug"],
                "lidar2img_extra": data_info["lidar2img_extra"],
            },
        }
        if "ann_info" in data_info:
            boxes = data_info["ann_info"]["gt_bboxes_3d"]
            labels = data_info["ann_info"]["gt_labels_3d"].copy()
            mask = labels >= 0
            result["ann_info"] = {
                "gt_bboxes_3d": boxes[mask],
                "gt_names": data_info["ann_info"]["gt_names"][mask],
                "gt_labels_3d": labels[mask],
            }
        return result
