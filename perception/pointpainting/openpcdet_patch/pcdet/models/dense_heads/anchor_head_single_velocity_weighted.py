"""AnchorHeadSingle with an object-quality mask on vx/vy only."""

import torch

from .anchor_head_single import AnchorHeadSingle


class AnchorHeadSingleVelocityWeighted(AnchorHeadSingle):
    """Keep standard 7D geometry loss while weighting the two velocity codes.

    Input GT is ``[x,y,z,dx,dy,dz,yaw,vx,vy,velocity_weight,class_id]``.
    The upstream target assigner receives the ordinary 9D box plus class ID.
    Its encoded positive targets are decoded to recover their exact source GT,
    which lets us attach the per-object quality weight without changing anchor
    matching or any geometry target.
    """

    def _flat_anchors(self, batch_size):
        if isinstance(self.anchors, list):
            if self.use_multihead:
                anchors = torch.cat([
                    anchor.permute(3, 4, 0, 1, 2, 5).contiguous().view(-1, anchor.shape[-1])
                    for anchor in self.anchors
                ], dim=0)
            else:
                anchors = torch.cat(self.anchors, dim=-3)
        else:
            anchors = self.anchors
        return anchors.view(1, -1, anchors.shape[-1]).repeat(batch_size, 1, 1)

    def assign_targets(self, gt_boxes):
        if gt_boxes.shape[-1] != self.box_coder.code_size + 2:
            raise ValueError(
                'velocity-weighted head expects 9D box + quality + class, got %s'
                % (tuple(gt_boxes.shape),)
            )
        velocity_quality = gt_boxes[..., self.box_coder.code_size]
        clean_gt = torch.cat([
            gt_boxes[..., :self.box_coder.code_size], gt_boxes[..., -1:]
        ], dim=-1)
        targets = super().assign_targets(clean_gt)
        labels = targets['box_cls_labels']
        encoded = targets['box_reg_targets']
        anchors = self._flat_anchors(int(gt_boxes.shape[0]))
        decoded = self.box_coder.decode_torch(encoded, anchors)
        anchor_quality = encoded.new_zeros(labels.shape)

        for batch_index in range(int(gt_boxes.shape[0])):
            positive = labels[batch_index] > 0
            valid_gt = clean_gt[batch_index, :, -1] > 0
            if not positive.any() or not valid_gt.any():
                continue
            candidate_boxes = clean_gt[batch_index, valid_gt, :self.box_coder.code_size]
            candidate_classes = clean_gt[batch_index, valid_gt, -1].long()
            candidate_quality = velocity_quality[batch_index, valid_gt]
            positive_boxes = decoded[batch_index, positive]
            positive_classes = labels[batch_index, positive].long()
            distance = torch.cdist(positive_boxes[:, :3], candidate_boxes[:, :3])
            distance = distance + (
                positive_classes[:, None] != candidate_classes[None, :]
            ).to(distance.dtype) * 1.0e6
            matched = distance.argmin(dim=1)
            anchor_quality[batch_index, positive] = candidate_quality[matched]

        targets['velocity_reg_weights'] = anchor_quality.clamp(min=0.0, max=1.0)
        return targets

    def get_box_reg_layer_loss(self):
        box_preds = self.forward_ret_dict['box_preds']
        box_dir_cls_preds = self.forward_ret_dict.get('dir_cls_preds', None)
        box_reg_targets = self.forward_ret_dict['box_reg_targets']
        box_cls_labels = self.forward_ret_dict['box_cls_labels']
        velocity_reg_weights = self.forward_ret_dict['velocity_reg_weights']
        batch_size = int(box_preds.shape[0])

        positives = box_cls_labels > 0
        reg_weights = positives.float()
        pos_normalizer = positives.sum(1, keepdim=True).float()
        reg_weights /= torch.clamp(pos_normalizer, min=1.0)
        anchors = self._flat_anchors(batch_size)
        box_preds = box_preds.view(
            batch_size,
            -1,
            box_preds.shape[-1] // self.num_anchors_per_location
            if not self.use_multihead else box_preds.shape[-1],
        )
        box_preds_sin, reg_targets_sin = self.add_sin_difference(
            box_preds, box_reg_targets
        )
        loc_loss_src = self.reg_loss_func(
            box_preds_sin, reg_targets_sin, weights=reg_weights
        )
        loc_loss_src[..., 7:9] *= velocity_reg_weights.unsqueeze(-1)
        loc_weight = self.model_cfg.LOSS_CONFIG.LOSS_WEIGHTS['loc_weight']
        geometry_loss = loc_loss_src[..., :7].sum() / batch_size * loc_weight
        velocity_loss = loc_loss_src[..., 7:9].sum() / batch_size * loc_weight
        loc_loss = geometry_loss + velocity_loss
        box_loss = loc_loss
        tb_dict = {
            'rpn_loss_loc': loc_loss.item(),
            'rpn_loss_geometry': geometry_loss.item(),
            'rpn_loss_velocity': velocity_loss.item(),
            'velocity_positive_weight_mean': (
                velocity_reg_weights[positives].mean().item() if positives.any() else 0.0
            ),
        }

        if box_dir_cls_preds is not None:
            dir_targets = self.get_direction_target(
                anchors,
                box_reg_targets,
                dir_offset=self.model_cfg.DIR_OFFSET,
                num_bins=self.model_cfg.NUM_DIR_BINS,
            )
            dir_logits = box_dir_cls_preds.view(
                batch_size, -1, self.model_cfg.NUM_DIR_BINS
            )
            weights = positives.type_as(dir_logits)
            weights /= torch.clamp(weights.sum(-1, keepdim=True), min=1.0)
            dir_loss = self.dir_loss_func(dir_logits, dir_targets, weights=weights)
            dir_loss = dir_loss.sum() / batch_size
            dir_loss = dir_loss * self.model_cfg.LOSS_CONFIG.LOSS_WEIGHTS['dir_weight']
            box_loss += dir_loss
            tb_dict['rpn_loss_dir'] = dir_loss.item()

        return box_loss, tb_dict
