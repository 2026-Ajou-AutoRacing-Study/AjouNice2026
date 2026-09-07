"""Merge internal obstacle shape classes into one deployment class."""

import numpy as np
import torch
from mmcv.ops import nms_rotated


DEFAULT_SHAPE_MAPPING = {
    "vehicle": "vehicle",
    "pedestrian": "pedestrian",
    "compact_obstacle": "obstacle",
    "tall_obstacle": "obstacle",
    "long_obstacle": "obstacle",
}
DEFAULT_MERGED_CLASSES = ("vehicle", "pedestrian", "obstacle")


def _result_payload(result):
    if "pts_bbox" in result:
        return result["pts_bbox"], "pts_bbox"
    if "img_bbox" in result:
        return result["img_bbox"], "img_bbox"
    return result, None


def _rebuild_result(original, payload, wrapper):
    if wrapper is None:
        return payload
    rebuilt = dict(original)
    rebuilt[wrapper] = payload
    return rebuilt


def merge_obstacle_shape_schema(data_infos,
                                results,
                                source_classes,
                                class_mapping=None,
                                merged_classes=DEFAULT_MERGED_CLASSES,
                                obstacle_nms_iou_threshold=0.2):
    """Return copied infos/results with shape labels merged and de-duplicated.

    Fast-BEV performs NMS separately per internal class.  Once compact, tall
    and long predictions become one semantic obstacle class, a second rotated
    BEV NMS prevents one physical object from being counted multiple times.
    """
    class_mapping = dict(DEFAULT_SHAPE_MAPPING if class_mapping is None
                         else class_mapping)
    source_classes = tuple(source_classes)
    merged_classes = tuple(merged_classes)
    target_indices = []
    for source_name in source_classes:
        target_name = class_mapping.get(source_name, source_name)
        if target_name not in merged_classes:
            raise ValueError(
                f"Mapped class {target_name!r} is not in {merged_classes}")
        target_indices.append(merged_classes.index(target_name))

    merged_infos = []
    for info in data_infos:
        merged_info = dict(info)
        merged_info["gt_names"] = np.asarray([
            class_mapping.get(str(name), str(name))
            for name in info["gt_names"]
        ])
        merged_infos.append(merged_info)

    merged_results = []
    obstacle_index = merged_classes.index("obstacle")
    for original in results:
        result, wrapper = _result_payload(original)
        boxes_3d = result["boxes_3d"]
        scores = result["scores_3d"]
        labels = result["labels_3d"].to(dtype=torch.long)
        mapping_tensor = labels.new_tensor(target_indices)
        if labels.numel() and int(labels.max()) >= len(mapping_tensor):
            raise ValueError("Prediction label exceeds source class schema")
        mapped_labels = mapping_tensor[labels]

        keep_non_obstacle = torch.nonzero(
            mapped_labels != obstacle_index, as_tuple=False).flatten()
        obstacle_indices = torch.nonzero(
            mapped_labels == obstacle_index, as_tuple=False).flatten()
        if obstacle_indices.numel():
            obstacle_boxes = boxes_3d.tensor[obstacle_indices]
            # MMCV 1.4's rotated NMS CPU kernel supports float32 but not the
            # FP16 tensors emitted by an FP16 evaluation model.
            bev_boxes = obstacle_boxes[:, [0, 1, 3, 4, 6]].float()
            _, relative_keep = nms_rotated(
                bev_boxes,
                scores[obstacle_indices].float(),
                float(obstacle_nms_iou_threshold))
            keep_obstacle = obstacle_indices[relative_keep]
            keep = torch.cat((keep_non_obstacle, keep_obstacle))
        else:
            keep = keep_non_obstacle
        if keep.numel():
            keep = keep[torch.argsort(scores[keep], descending=True)]

        tensor = boxes_3d.tensor[keep]
        rebuilt_boxes = type(boxes_3d)(
            tensor,
            box_dim=tensor.shape[-1] if tensor.ndim == 2 else boxes_3d.box_dim,
            with_yaw=boxes_3d.with_yaw)
        merged_payload = dict(result)
        merged_payload.update({
            "boxes_3d": rebuilt_boxes,
            "scores_3d": scores[keep],
            "labels_3d": mapped_labels[keep],
        })
        merged_results.append(
            _rebuild_result(original, merged_payload, wrapper))
    return merged_infos, merged_results
