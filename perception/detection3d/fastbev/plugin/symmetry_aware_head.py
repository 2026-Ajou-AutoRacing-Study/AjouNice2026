"""FreeAnchor head with class-aware obstacle box supervision."""

import torch
from mmcv.runner import force_fp32
from mmdet.core import multi_apply
from mmdet.models import HEADS
from mmdet3d.core.bbox import bbox_overlaps_nearest_3d
from mmdet3d.models.dense_heads.free_anchor3d_head import FreeAnchor3DHead
from mmdet3d.models.dense_heads.train_mixins import get_direction_target


@HEADS.register_module()
class ObstacleAwareFreeAnchor3DHead(FreeAnchor3DHead):
    """Apply obstacle-specific weights only to matched positive-anchor losses.

    The stock Fast-BEV/FreeAnchor head uses a sine yaw residual, which is
    already pi-periodic, plus a two-bin direction cross entropy that separates
    front from rear.  Axis-symmetric obstacles should retain the former but do
    not need the latter.  Optionally, compact near-square obstacles can also
    omit the continuous yaw channel because their occupied footprint has no
    stable major axis.
    """

    def __init__(self,
                 obstacle_label=2,
                 mask_obstacle_direction=True,
                 mask_near_square_obstacle_yaw=False,
                 near_square_aspect_ratio=1.2,
                 yaw_code_index=6,
                 obstacle_xy_loss_weight=1.0,
                 xy_code_indices=(0, 1),
                 **kwargs):
        super().__init__(**kwargs)
        if near_square_aspect_ratio < 1.0:
            raise ValueError("near_square_aspect_ratio must be >= 1.0")
        self.obstacle_label = int(obstacle_label)
        self.mask_obstacle_direction = bool(mask_obstacle_direction)
        self.mask_near_square_obstacle_yaw = bool(
            mask_near_square_obstacle_yaw)
        self.near_square_aspect_ratio = float(near_square_aspect_ratio)
        self.yaw_code_index = int(yaw_code_index)
        if obstacle_xy_loss_weight <= 0.0:
            raise ValueError("obstacle_xy_loss_weight must be > 0")
        if len(xy_code_indices) != 2:
            raise ValueError("xy_code_indices must contain exactly two indices")
        self.obstacle_xy_loss_weight = float(obstacle_xy_loss_weight)
        self.xy_code_indices = tuple(int(index) for index in xy_code_indices)

    def _obstacle_symmetry_masks(self, gt_labels, gt_boxes):
        """Return per-GT masks for direction and continuous yaw losses."""
        is_obstacle = gt_labels == self.obstacle_label
        direction_keep = torch.ones_like(gt_labels, dtype=gt_boxes.dtype)
        if self.mask_obstacle_direction:
            direction_keep[is_obstacle] = 0.0

        yaw_keep = torch.ones_like(gt_labels, dtype=gt_boxes.dtype)
        if self.mask_near_square_obstacle_yaw and gt_boxes.numel():
            # LiDARInstance3DBoxes tensor order is x, y, z, dx, dy, dz, yaw,
            # followed by optional velocity.  A near-square XY footprint has
            # no reliable major axis from dimensions alone.
            dim_x = gt_boxes[:, 3].abs().clamp_min(1e-6)
            dim_y = gt_boxes[:, 4].abs().clamp_min(1e-6)
            aspect_ratio = torch.maximum(dim_x, dim_y) / torch.minimum(
                dim_x, dim_y)
            near_square_obstacle = is_obstacle & (
                aspect_ratio <= self.near_square_aspect_ratio)
            yaw_keep[near_square_obstacle] = 0.0
        return direction_keep, yaw_keep

    def _apply_obstacle_xy_weights(self, bbox_weights, gt_labels):
        """Scale x/y regression for obstacle GT bags and no other codes."""
        if self.obstacle_xy_loss_weight == 1.0 or not gt_labels.numel():
            return bbox_weights
        obstacle_scale = bbox_weights.new_ones(gt_labels.shape)
        obstacle_scale[gt_labels == self.obstacle_label] = (
            self.obstacle_xy_loss_weight)
        for code_index in self.xy_code_indices:
            bbox_weights[:, :, code_index] *= obstacle_scale.unsqueeze(1)
        return bbox_weights

    @force_fp32(apply_to=('cls_scores', 'bbox_preds', 'dir_cls_preds'))
    def loss(self,
             cls_scores,
             bbox_preds,
             dir_cls_preds,
             gt_bboxes,
             gt_labels,
             input_metas,
             gt_bboxes_ignore=None):
        """Calculate FreeAnchor loss with matched-GT symmetry masks."""
        featmap_sizes = [featmap.size()[-2:] for featmap in cls_scores]
        assert len(featmap_sizes) == self.anchor_generator.num_levels

        anchor_list = self.get_anchors(
            featmap_sizes, input_metas, device=cls_scores[0].device)
        anchors = [torch.cat(anchor) for anchor in anchor_list]

        cls_scores = [
            score.permute(0, 2, 3, 1).reshape(
                score.size(0), -1, self.num_classes)
            for score in cls_scores
        ]
        bbox_preds = [
            pred.permute(0, 2, 3, 1).reshape(
                pred.size(0), -1, self.box_code_size)
            for pred in bbox_preds
        ]
        dir_cls_preds = [
            pred.permute(0, 2, 3, 1).reshape(pred.size(0), -1, 2)
            for pred in dir_cls_preds
        ]
        cls_scores = torch.cat(cls_scores, dim=1)
        bbox_preds = torch.cat(bbox_preds, dim=1)
        dir_cls_preds = torch.cat(dir_cls_preds, dim=1)

        cls_prob = torch.sigmoid(cls_scores)
        box_prob = []
        num_pos = 0
        positive_losses = []
        for (anchors_, gt_labels_, gt_bboxes_, cls_prob_, bbox_preds_,
             dir_cls_preds_) in zip(anchors, gt_labels, gt_bboxes, cls_prob,
                                    bbox_preds, dir_cls_preds):
            gt_bboxes_ = gt_bboxes_.tensor.to(anchors_.device)
            direction_keep, yaw_keep = self._obstacle_symmetry_masks(
                gt_labels_, gt_bboxes_)

            with torch.no_grad():
                pred_boxes = self.bbox_coder.decode(anchors_, bbox_preds_)
                object_box_iou = bbox_overlaps_nearest_3d(
                    gt_bboxes_, pred_boxes)
                t1 = self.bbox_thr
                t2 = object_box_iou.max(
                    dim=1, keepdim=True).values.clamp(min=t1 + 1e-12)
                object_box_prob = ((object_box_iou - t1) /
                                   (t2 - t1)).clamp(min=0, max=1)

                num_obj = gt_labels_.size(0)
                indices = torch.stack([
                    torch.arange(num_obj).type_as(gt_labels_), gt_labels_
                ], dim=0)
                object_cls_box_prob = torch.sparse_coo_tensor(
                    indices, object_box_prob)
                box_cls_prob = torch.sparse.sum(
                    object_cls_box_prob, dim=0).to_dense()
                indices = torch.nonzero(box_cls_prob, as_tuple=False).t_()
                if indices.numel() == 0:
                    image_box_prob = torch.zeros(
                        anchors_.size(0), self.num_classes).type_as(
                            object_box_prob)
                else:
                    nonzero_box_prob = torch.where(
                        gt_labels_.unsqueeze(dim=-1) == indices[0],
                        object_box_prob[:, indices[1]],
                        torch.tensor([0]).type_as(object_box_prob)).max(
                            dim=0).values
                    image_box_prob = torch.sparse_coo_tensor(
                        indices.flip([0]), nonzero_box_prob,
                        size=(anchors_.size(0), self.num_classes)).to_dense()
                box_prob.append(image_box_prob)

            match_quality_matrix = bbox_overlaps_nearest_3d(
                gt_bboxes_, anchors_)
            _, matched = torch.topk(
                match_quality_matrix,
                self.pre_anchor_topk,
                dim=1,
                sorted=False)
            del match_quality_matrix

            matched_cls_prob = torch.gather(
                cls_prob_[matched], 2,
                gt_labels_.view(-1, 1, 1).repeat(
                    1, self.pre_anchor_topk, 1)).squeeze(2)
            matched_anchors = anchors_[matched]
            matched_object_targets = self.bbox_coder.encode(
                matched_anchors,
                gt_bboxes_.unsqueeze(dim=1).expand_as(matched_anchors))

            loss_dir = None
            if self.use_direction_classifier:
                matched_dir_targets = get_direction_target(
                    matched_anchors, matched_object_targets, self.dir_offset,
                    one_hot=False)
                loss_dir = self.loss_dir(
                    dir_cls_preds_[matched].transpose(-2, -1),
                    matched_dir_targets,
                    reduction_override='none')
                loss_dir = loss_dir * direction_keep.unsqueeze(1)

            if self.diff_rad_by_sin:
                pos_preds, matched_object_targets = self.add_sin_difference(
                    bbox_preds_[matched], matched_object_targets)
            else:
                pos_preds = bbox_preds_[matched]

            bbox_weights = matched_anchors.new_ones(matched_anchors.size())
            code_weight = self.train_cfg.get('code_weight', None)
            if code_weight:
                bbox_weights = bbox_weights * bbox_weights.new_tensor(
                    code_weight)
            bbox_weights = self._apply_obstacle_xy_weights(
                bbox_weights, gt_labels_)
            bbox_weights[:, :, self.yaw_code_index] *= yaw_keep.unsqueeze(1)
            loss_bbox = self.loss_bbox(
                pos_preds,
                matched_object_targets,
                bbox_weights,
                reduction_override='none').sum(-1)
            if loss_dir is not None:
                loss_bbox += loss_dir
            matched_box_prob = torch.exp(-loss_bbox)

            num_pos += len(gt_bboxes_)
            positive_losses.append(
                self.positive_bag_loss(matched_cls_prob, matched_box_prob))

        positive_loss = torch.cat(positive_losses).sum() / max(1, num_pos)
        box_prob = torch.stack(box_prob, dim=0)
        negative_loss = self.negative_bag_loss(
            cls_prob, box_prob).sum() / max(
                1, num_pos * self.pre_anchor_topk)
        return {
            'positive_bag_loss': positive_loss * self.pos_loss_weight,
            'negative_bag_loss': negative_loss * self.neg_loss_weight,
        }


@HEADS.register_module()
class SymmetryAwareFreeAnchor3DHead(ObstacleAwareFreeAnchor3DHead):
    """Backward-compatible name for the completed yaw-symmetry ablations."""
