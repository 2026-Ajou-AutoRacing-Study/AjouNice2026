"""MORAI TransFusion heads, including the Base4 sparse-memory probe."""

import math

import torch
from torch import nn
from torch.nn import functional as F
from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion.transfusion_head import TransFusionHead

from .morai_query_memory import (
    CausalQueryMemoryBank,
    MoraiQueryMemoryAttention,
)


@MODELS.register_module()
class MoraiTransFusionHead(TransFusionHead):
    """Initialize the dense heatmap prior for training without a full head.

    The upstream implementation initializes decoder weights but leaves the
    dense heatmap classifier near a 0.5 foreground prior. A -2.19 bias is the
    same foreground prior used by MMDetection3D's SeparateHead and keeps a
    from-scratch fallback numerically sane. The production experiment then
    overwrites this initialization with its adapted three-class checkpoint.
    """

    def init_weights(self):
        super().init_weights()
        final_conv = self.heatmap_head[-1]
        if getattr(final_conv, 'bias', None) is not None:
            nn.init.constant_(final_conv.bias, -2.19)


@MODELS.register_module()
class MoraiQueryMemoryTransFusionHead(MoraiTransFusionHead):
    """TransFusion with a detached, causal Vehicle-query memory.

    Q1-P0 deliberately supports batch size one only. This makes scene reset
    and strictly ordered recurrence explicit instead of silently mixing
    samples from different scenes. Past query features are detached; current
    spatial detection remains the optimization anchor.
    """

    def __init__(
        self,
        query_memory=None,
        memory_max_frames=4,
        memory_max_queries_per_frame=64,
        memory_reset_gap_sec=0.30,
        temporal_velocity_aux_weight=0.0,
        initial_velocity_fusion_logit=-2.0,
        **kwargs,
    ):
        super().__init__(**kwargs)
        query_memory = dict(query_memory or {})
        query_memory.setdefault("channels", self.shared_conv.out_channels)
        self.query_memory = MoraiQueryMemoryAttention(**query_memory)
        self.query_memory_bank = CausalQueryMemoryBank(
            max_frames=int(memory_max_frames),
            max_queries_per_frame=int(memory_max_queries_per_frame),
            maximum_age_sec=float(
                query_memory.get("maximum_memory_age_sec", 0.60)),
            reset_gap_sec=float(memory_reset_gap_sec),
            vehicle_label=int(query_memory.get("vehicle_label", 0)),
        )
        self.temporal_velocity_aux_weight = float(
            temporal_velocity_aux_weight)
        if self.temporal_velocity_aux_weight > 0.0:
            channels = int(query_memory["channels"])
            self.query_memory_velocity_head = nn.Sequential(
                nn.Conv1d(channels, channels, kernel_size=1),
                nn.SiLU(),
                nn.Conv1d(channels, 2, kernel_size=1),
            )
            nn.init.zeros_(self.query_memory_velocity_head[-1].weight)
            nn.init.zeros_(self.query_memory_velocity_head[-1].bias)
            self.query_memory_velocity_fusion_logit = nn.Parameter(
                torch.tensor(float(initial_velocity_fusion_logit)))
        else:
            self.query_memory_velocity_head = None
            self.register_parameter(
                "query_memory_velocity_fusion_logit", None)
        self.last_temporal_velocity_pair_fraction = None

    @staticmethod
    def _pose_from_lidar2global(matrix):
        if matrix is None:
            raise KeyError("Q1 requires lidar2global in Pack3DDetInputs meta")
        matrix = torch.as_tensor(matrix, dtype=torch.float32)
        if tuple(matrix.shape) != (4, 4):
            raise ValueError("lidar2global must be 4x4")
        return (
            float(matrix[0, 3]),
            float(matrix[1, 3]),
            float(matrix[2, 3]),
            math.atan2(float(matrix[1, 0]), float(matrix[0, 0])),
        )

    def _grid_to_metric(self, query_pos):
        stride_x = (
            float(self.test_cfg['voxel_size'][0])
            * float(self.test_cfg['out_size_factor'])
        )
        stride_y = (
            float(self.test_cfg['voxel_size'][1])
            * float(self.test_cfg['out_size_factor'])
        )
        metric = query_pos.clone()
        metric[..., 0] = (
            float(self.test_cfg['pc_range'][0]) + metric[..., 0] * stride_x
        )
        metric[..., 1] = (
            float(self.test_cfg['pc_range'][1]) + metric[..., 1] * stride_y
        )
        return metric

    def forward_single(self, inputs, metas):
        if inputs.shape[0] != 1 or len(metas) != 1:
            raise ValueError(
                "MoraiQueryMemoryTransFusionHead requires batch_size=1")
        meta = metas[0]
        for key in ("scene_id", "timestamp", "lidar2global"):
            if key not in meta:
                raise KeyError(f"Q1 input meta is missing {key}")
        scene_id = str(meta["scene_id"])
        timestamp_sec = float(meta["timestamp"])
        lidar_pose_world = self._pose_from_lidar2global(meta["lidar2global"])

        batch_size = inputs.shape[0]
        fusion_feat = self.shared_conv(inputs)
        fusion_feat_flatten = fusion_feat.view(
            batch_size, fusion_feat.shape[1], -1)
        bev_pos = self.bev_pos.repeat(batch_size, 1, 1).to(fusion_feat.device)

        with torch.autocast('cuda', enabled=False):
            dense_heatmap = self.heatmap_head(fusion_feat.float())
        heatmap = dense_heatmap.detach().sigmoid()
        padding = self.nms_kernel_size // 2
        local_max = torch.zeros_like(heatmap)
        local_max_inner = F.max_pool2d(
            heatmap, kernel_size=self.nms_kernel_size, stride=1, padding=0)
        local_max[:, :, padding:(-padding), padding:(-padding)] = \
            local_max_inner
        if self.test_cfg['dataset'] == 'nuScenes':
            local_max[:, 8, ] = F.max_pool2d(
                heatmap[:, 8], kernel_size=1, stride=1, padding=0)
            local_max[:, 9, ] = F.max_pool2d(
                heatmap[:, 9], kernel_size=1, stride=1, padding=0)
        elif self.test_cfg['dataset'] == 'Waymo':
            local_max[:, 1, ] = F.max_pool2d(
                heatmap[:, 1], kernel_size=1, stride=1, padding=0)
            local_max[:, 2, ] = F.max_pool2d(
                heatmap[:, 2], kernel_size=1, stride=1, padding=0)
        heatmap = heatmap * (heatmap == local_max)
        heatmap = heatmap.view(batch_size, heatmap.shape[1], -1)

        top_proposals = heatmap.view(batch_size, -1).argsort(
            dim=-1, descending=True)[..., :self.num_proposals]
        top_proposals_class = top_proposals // heatmap.shape[-1]
        top_proposals_index = top_proposals % heatmap.shape[-1]
        query_feat = fusion_feat_flatten.gather(
            index=top_proposals_index[:, None, :].expand(
                -1, fusion_feat_flatten.shape[1], -1),
            dim=-1,
        )
        self.query_labels = top_proposals_class
        one_hot = F.one_hot(
            top_proposals_class,
            num_classes=self.num_classes).permute(0, 2, 1)
        query_feat = query_feat + self.class_encoding(one_hot.float())
        query_pos = bev_pos.gather(
            index=top_proposals_index[:, None, :].permute(0, 2, 1).expand(
                -1, -1, bev_pos.shape[-1]),
            dim=1,
        )

        memory = self.query_memory_bank.prepare(
            scene_id,
            timestamp_sec,
            lidar_pose_world,
            channels=query_feat.shape[1],
            device=query_feat.device,
            dtype=query_feat.dtype,
        )
        query_feat = self.query_memory(
            query_feat,
            self._grid_to_metric(query_pos),
            top_proposals_class,
            memory["features"],
            memory["centers_xy"],
            memory["scores"],
            memory["labels"],
            memory["ages_sec"],
        )
        temporal_velocity = None
        temporal_velocity_active = None
        if self.query_memory_velocity_head is not None:
            temporal_velocity = self.query_memory_velocity_head(
                self.query_memory.last_motion_context)
            temporal_velocity_active = (
                self.query_memory.last_active_query_mask[:, None, :]
                .to(dtype=temporal_velocity.dtype))
        self._last_temporal_velocity = temporal_velocity
        self._last_temporal_velocity_active = temporal_velocity_active

        ret_dicts = []
        for layer_index in range(self.num_decoder_layers):
            query_feat = self.decoder[layer_index](
                query_feat,
                key=fusion_feat_flatten,
                query_pos=query_pos,
                key_pos=bev_pos,
            )
            result = self.prediction_heads[layer_index](query_feat)
            result['center'] = result['center'] + query_pos.permute(0, 2, 1)
            if temporal_velocity is not None and 'vel' in result:
                fusion = torch.sigmoid(
                    self.query_memory_velocity_fusion_logit)
                result['vel'] = result['vel'] + temporal_velocity_active * (
                    fusion * (temporal_velocity - result['vel']))
            ret_dicts.append(result)
            query_pos = result['center'].detach().clone().permute(0, 2, 1)

        proposal_scores = heatmap.gather(
            index=top_proposals_index[:, None, :].expand(
                -1, self.num_classes, -1),
            dim=-1,
        )
        ret_dicts[0]['query_heatmap_score'] = proposal_scores
        ret_dicts[0]['dense_heatmap'] = dense_heatmap
        self.query_memory_bank.push(
            scene_id,
            timestamp_sec,
            lidar_pose_world,
            query_feat[0],
            self._grid_to_metric(query_pos)[0],
            proposal_scores.gather(
                dim=1, index=top_proposals_class[:, None, :]
            ).squeeze(1)[0],
            top_proposals_class[0],
        )

        if self.auxiliary is False:
            return [ret_dicts[-1]]
        combined = {}
        for key in ret_dicts[0].keys():
            if key not in (
                'dense_heatmap', 'dense_heatmap_old', 'query_heatmap_score'
            ):
                combined[key] = torch.cat(
                    [result[key] for result in ret_dicts], dim=-1)
            else:
                combined[key] = ret_dicts[0][key]
        return [combined]

    def get_targets(self, batch_gt_instances_3d, preds_dict):
        targets = super().get_targets(batch_gt_instances_3d, preds_dict)
        self._last_q1_labels = targets[0]
        self._last_q1_bbox_targets = targets[2]
        return targets

    def loss_by_feat(self, preds_dicts, batch_gt_instances_3d, *args,
                     **kwargs):
        losses = super().loss_by_feat(
            preds_dicts, batch_gt_instances_3d, *args, **kwargs)
        if self.query_memory_velocity_head is None:
            return losses

        count = self.num_proposals
        labels = self._last_q1_labels[..., -count:]
        velocity_targets = self._last_q1_bbox_targets[
            ..., -count:, 8:10]
        active = self._last_temporal_velocity_active[:, 0, :].bool()
        positive_vehicle = (
            (labels == self.query_memory.vehicle_label) & active)
        pair_fraction = positive_vehicle.float().mean()
        self.last_temporal_velocity_pair_fraction = pair_fraction.detach()

        if positive_vehicle.any():
            prediction = self._last_temporal_velocity.permute(0, 2, 1)
            aux_loss = F.smooth_l1_loss(
                prediction[positive_vehicle].float(),
                velocity_targets[positive_vehicle].float(),
                beta=1.0,
                reduction='mean')
        else:
            aux_loss = self._last_temporal_velocity.sum() * 0.0
        losses['loss_query_memory_velocity_aux'] = (
            aux_loss * self.temporal_velocity_aux_weight)
        losses['query_memory_velocity_fusion_gate'] = torch.sigmoid(
            self.query_memory_velocity_fusion_logit).detach()
        losses['query_memory_velocity_pair_fraction'] = pair_fraction.detach()
        return losses
