import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .anchor_head_template import AnchorHeadTemplate


class AnchorHeadSingle(AnchorHeadTemplate):
    def __init__(self, model_cfg, input_channels, num_class, class_names, grid_size, point_cloud_range,
                 predict_boxes_when_training=True, **kwargs):
        super().__init__(
            model_cfg=model_cfg, num_class=num_class, class_names=class_names, grid_size=grid_size, point_cloud_range=point_cloud_range,
            predict_boxes_when_training=predict_boxes_when_training
        )

        self.num_anchors_per_location = sum(self.num_anchors_per_location)

        self.conv_cls = nn.Conv2d(
            input_channels, self.num_anchors_per_location * self.num_class,
            kernel_size=1
        )
        self.conv_box = nn.Conv2d(
            input_channels, self.num_anchors_per_location * self.box_coder.code_size,
            kernel_size=1
        )

        self.predict_velocity = self.model_cfg.get(
            'PREDICT_VELOCITY', False
        )

        if self.predict_velocity:
            self.conv_vel = nn.Conv2d(
                input_channels,
                self.num_anchors_per_location * 2,
                kernel_size=1
            )
        else:
            self.conv_vel = None

        if self.model_cfg.get('USE_DIRECTION_CLASSIFIER', None) is not None:
            self.conv_dir_cls = nn.Conv2d(
                input_channels,
                self.num_anchors_per_location * self.model_cfg.NUM_DIR_BINS,
                kernel_size=1
            )
        else:
            self.conv_dir_cls = None
        self.init_weights()

    def init_weights(self):
        pi = 0.01
        nn.init.constant_(self.conv_cls.bias, -np.log((1 - pi) / pi))
        nn.init.normal_(self.conv_box.weight, mean=0, std=0.001)

        if self.conv_vel is not None:
            nn.init.normal_(self.conv_vel.weight, mean=0, std=0.001)
            nn.init.constant_(self.conv_vel.bias, 0)

    def get_loss(self):
        rpn_loss, tb_dict = super().get_loss()

        if not self.predict_velocity:
            return rpn_loss, tb_dict

        vel_preds = self.forward_ret_dict['vel_preds']
        vel_targets = self.forward_ret_dict['velocity_targets']
        vel_weights = self.forward_ret_dict['velocity_weights']

        batch_size = int(vel_preds.shape[0])

        vel_preds = vel_preds.view(
            batch_size, -1, 2
        )

        vel_targets = vel_targets.view(
            batch_size, -1, 2
        )

        vel_weights = vel_weights.view(
            batch_size, -1
        ).float()

        normalizer = torch.clamp(
            vel_weights.sum(dim=1, keepdim=True),
            min=1.0
        )

        vel_weights = vel_weights / normalizer

        vel_loss_src = F.smooth_l1_loss(
            vel_preds,
            vel_targets,
            reduction='none'
        ).sum(dim=-1)

        vel_loss = (
            vel_loss_src * vel_weights
        ).sum() / batch_size

        vel_weight = float(
            self.model_cfg.LOSS_CONFIG.LOSS_WEIGHTS.get(
                'vel_weight', 1.0
            )
        )

        vel_loss = vel_loss * vel_weight
        rpn_loss = rpn_loss + vel_loss

        tb_dict['rpn_loss_vel'] = vel_loss.item()
        tb_dict['velocity_valid_anchors'] = (
            self.forward_ret_dict[
                'velocity_weights'
            ].sum().item()
        )
        tb_dict['rpn_loss'] = rpn_loss.item()

        return rpn_loss, tb_dict

    def forward(self, data_dict):
        spatial_features_2d = data_dict['spatial_features_2d']

        cls_preds = self.conv_cls(spatial_features_2d)
        box_preds = self.conv_box(spatial_features_2d)

        vel_preds = (
            self.conv_vel(spatial_features_2d)
            if self.conv_vel is not None
            else None
        )

        cls_preds = cls_preds.permute(0, 2, 3, 1).contiguous()  # [N, H, W, C]
        box_preds = box_preds.permute(0, 2, 3, 1).contiguous()  # [N, H, W, C]

        if vel_preds is not None:
            vel_preds = vel_preds.permute(
                0, 2, 3, 1
            ).contiguous()

        self.forward_ret_dict['cls_preds'] = cls_preds
        self.forward_ret_dict['box_preds'] = box_preds

        if vel_preds is not None:
            self.forward_ret_dict['vel_preds'] = vel_preds

        if self.conv_dir_cls is not None:
            dir_cls_preds = self.conv_dir_cls(spatial_features_2d)
            dir_cls_preds = dir_cls_preds.permute(0, 2, 3, 1).contiguous()
            self.forward_ret_dict['dir_cls_preds'] = dir_cls_preds
        else:
            dir_cls_preds = None

        if self.training:
            if self.predict_velocity:
                gt_full = data_dict['gt_boxes']

                if gt_full.shape[-1] != 11:
                    raise RuntimeError(
                        'Velocity model expects gt_boxes shape '
                        '[B, M, 11] = '
                        '[7D box, vx, vy, valid, class], '
                        f'got {gt_full.shape}'
                    )

                det_gt_boxes = torch.cat(
                    [
                        gt_full[..., :7],
                        gt_full[..., -1:]
                    ],
                    dim=-1
                )

                targets_dict = self.target_assigner.assign_targets(
                    self.anchors,
                    det_gt_boxes,
                    gt_velocities=gt_full[..., 7:9],
                    velocity_valid=gt_full[..., 9]
                )
            else:
                targets_dict = self.assign_targets(
                    gt_boxes=data_dict['gt_boxes']
                )

            self.forward_ret_dict.update(targets_dict)

        if not self.training or self.predict_boxes_when_training:
            batch_cls_preds, batch_box_preds = self.generate_predicted_boxes(
                batch_size=data_dict['batch_size'],
                cls_preds=cls_preds, box_preds=box_preds, dir_cls_preds=dir_cls_preds
            )
            data_dict['batch_cls_preds'] = batch_cls_preds
            data_dict['batch_box_preds'] = batch_box_preds

            if vel_preds is not None:
                batch_vel_preds = vel_preds.view(
                    data_dict['batch_size'],
                    -1,
                    2
                )
                data_dict['batch_vel_preds'] = batch_vel_preds

            data_dict['cls_preds_normalized'] = False

        return data_dict
