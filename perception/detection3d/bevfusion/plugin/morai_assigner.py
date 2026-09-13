"""Small compatibility fixes for the upstream BEVFusion target assigner."""

import torch
from mmdet.models.task_modules import AssignResult
from mmdet3d.registry import TASK_UTILS
from projects.BEVFusion.bevfusion.utils import HungarianAssigner3D


@TASK_UTILS.register_module(name="HungarianAssigner3D", force=True)
class MoraiHungarianAssigner3D(HungarianAssigner3D):
    """Return numeric overlaps for intentional Base3 empty frames.

    The upstream assigner returns ``max_overlaps=None`` when no GT exists,
    while its TransFusion head immediately concatenates that field. Base3
    deliberately retains empty/hard-negative frames, so represent every
    proposal as a zero-IoU background assignment instead.
    """

    def assign(self, bboxes, gt_bboxes, gt_labels, cls_pred, train_cfg):
        if gt_bboxes.size(0) or bboxes.size(0) == 0:
            return super().assign(
                bboxes, gt_bboxes, gt_labels, cls_pred, train_cfg)
        assigned_gt_inds = bboxes.new_zeros((bboxes.size(0),), dtype=torch.long)
        assigned_labels = bboxes.new_full(
            (bboxes.size(0),), -1, dtype=torch.long)
        max_overlaps = bboxes.new_zeros((bboxes.size(0),))
        return AssignResult(
            0, assigned_gt_inds, max_overlaps, labels=assigned_labels)
