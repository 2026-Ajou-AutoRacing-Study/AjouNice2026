"""Lightweight sweep-separated temporal BEV fusion for MORAI Base4."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion import BEVFusion


def source_to_target_matrix_from_lidar_poses(
    source_pose_world: Sequence[float],
    target_pose_world: Sequence[float],
    *,
    device=None,
) -> Tensor:
    """Return a planar homogeneous transform from source to target LiDAR."""
    source_x, source_y, source_z, source_yaw = map(float, source_pose_world)
    target_x, target_y, target_z, target_yaw = map(float, target_pose_world)
    delta_yaw = source_yaw - target_yaw
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    dx_world = source_x - target_x
    dy_world = source_y - target_y
    cos_target = math.cos(target_yaw)
    sin_target = math.sin(target_yaw)
    transform = torch.eye(4, device=device, dtype=torch.float32)
    transform[0, 0] = cosine
    transform[0, 1] = -sine
    transform[1, 0] = sine
    transform[1, 1] = cosine
    transform[0, 3] = cos_target * dx_world + sin_target * dy_world
    transform[1, 3] = -sin_target * dx_world + cos_target * dy_world
    transform[2, 3] = source_z - target_z
    return transform


def warp_bev_feature_by_source_to_target(
    feature: Tensor,
    source_to_target: Tensor,
    point_cloud_range: Sequence[float],
) -> Tensor:
    """Warp source-frame BEV into target using ``p_target = T p_source``."""
    if feature.ndim != 4:
        raise ValueError("BEV feature must be BCHW")
    transform = torch.as_tensor(
        source_to_target, device=feature.device, dtype=torch.float32)
    if transform.shape != (4, 4):
        raise ValueError("source_to_target must be 4x4")
    if len(point_cloud_range) < 5:
        raise ValueError("point_cloud_range must contain xyz min/max")
    _, _, height, width = feature.shape
    x_min, y_min = map(float, point_cloud_range[:2])
    x_max, y_max = map(float, point_cloud_range[3:5])
    dtype = feature.dtype
    device = feature.device

    target_x = x_min + (torch.arange(
        width, device=device, dtype=torch.float32) + 0.5
    ) * ((x_max - x_min) / width)
    target_y = y_min + (torch.arange(
        height, device=device, dtype=torch.float32) + 0.5
    ) * ((y_max - y_min) / height)
    grid_y, grid_x = torch.meshgrid(target_y, target_x, indexing="ij")

    inverse = torch.linalg.inv(transform)
    sample_x = (
        inverse[0, 0] * grid_x + inverse[0, 1] * grid_y + inverse[0, 3]
    )
    sample_y = (
        inverse[1, 0] * grid_x + inverse[1, 1] * grid_y + inverse[1, 3]
    )
    normalized_x = 2.0 * (sample_x - x_min) / (x_max - x_min) - 1.0
    normalized_y = 2.0 * (sample_y - y_min) / (y_max - y_min) - 1.0
    grid = torch.stack((normalized_x, normalized_y), dim=-1)
    grid = grid.unsqueeze(0).expand(feature.shape[0], -1, -1, -1)
    return F.grid_sample(
        feature,
        grid.to(dtype=dtype),
        mode="bilinear",
        padding_mode="zeros",
        align_corners=False,
    )


def warp_bev_feature_between_lidar_poses(
    feature: Tensor,
    source_pose_world: Sequence[float],
    target_pose_world: Sequence[float],
    point_cloud_range: Sequence[float],
) -> Tensor:
    transform = source_to_target_matrix_from_lidar_poses(
        source_pose_world, target_pose_world, device=feature.device)
    return warp_bev_feature_by_source_to_target(
        feature, transform, point_cloud_range)


@MODELS.register_module()
class MoraiTemporalBEVGate(nn.Module):
    """Fuse aligned per-sweep BEV features with a spatial softmax gate.

    The gate is initialized almost current-only. This preserves the spatial
    behavior of a warm-started single-frame/T1 checkpoint while still giving
    gradients to both past sweeps from the first optimizer update.
    """

    def __init__(
        self,
        channels: int = 256,
        num_sweeps: int = 3,
        current_logit: float = 4.0,
        past_logit: float = -2.0,
    ) -> None:
        super().__init__()
        if num_sweeps < 2:
            raise ValueError("num_sweeps must include current and past frames")
        self.channels = int(channels)
        self.num_sweeps = int(num_sweeps)
        self.gate = nn.Conv2d(
            self.channels * self.num_sweeps,
            self.num_sweeps,
            kernel_size=1,
            bias=True,
        )
        nn.init.zeros_(self.gate.weight)
        with torch.no_grad():
            self.gate.bias.fill_(float(past_logit))
            self.gate.bias[0] = float(current_logit)
        self.last_mean_weights: Optional[Tensor] = None

    def forward(self, features: List[Tensor]) -> Tensor:
        if len(features) != self.num_sweeps:
            raise ValueError(
                f"expected {self.num_sweeps} sweep features, got {len(features)}"
            )
        reference = features[0]
        expected = (reference.shape[0], self.channels, *reference.shape[-2:])
        for index, feature in enumerate(features):
            if tuple(feature.shape) != expected:
                raise ValueError(
                    f"sweep {index} shape {tuple(feature.shape)} != {expected}"
                )
        stacked = torch.stack(features, dim=1)
        logits = self.gate(torch.cat(features, dim=1))
        weights = torch.softmax(logits, dim=1)
        self.last_mean_weights = weights.detach().mean(dim=(0, 2, 3))
        return (stacked * weights.unsqueeze(2)).sum(dim=1)


@MODELS.register_module()
class MoraiTemporalBEVResidual(nn.Module):
    """Preserve current geometry and encode ordered BEV feature changes.

    A convex average can discard the direction of temporal changes. This
    branch instead consumes ``current``, ``current - t1`` and consecutive
    past differences, then predicts a residual on top of the current BEV.
    The last layer starts at zero so a warm-start is exactly current-only.
    """

    def __init__(
        self,
        channels: int = 256,
        num_sweeps: int = 3,
        hidden_channels: int = 128,
    ) -> None:
        super().__init__()
        if num_sweeps < 2:
            raise ValueError("num_sweeps must include current and past frames")
        self.channels = int(channels)
        self.num_sweeps = int(num_sweeps)
        self.motion_encoder = nn.Sequential(
            nn.Conv2d(
                self.channels * self.num_sweeps,
                int(hidden_channels),
                kernel_size=1,
                bias=False,
            ),
            nn.BatchNorm2d(int(hidden_channels)),
            nn.SiLU(inplace=True),
            nn.Conv2d(
                int(hidden_channels), self.channels, kernel_size=1, bias=True
            ),
        )
        nn.init.zeros_(self.motion_encoder[-1].weight)
        nn.init.zeros_(self.motion_encoder[-1].bias)
        self.last_residual_rms: Optional[Tensor] = None

    def forward(self, features: List[Tensor]) -> Tensor:
        if len(features) != self.num_sweeps:
            raise ValueError(
                f"expected {self.num_sweeps} sweep features, got {len(features)}"
            )
        current = features[0]
        expected = (current.shape[0], self.channels, *current.shape[-2:])
        for index, feature in enumerate(features):
            if tuple(feature.shape) != expected:
                raise ValueError(
                    f"sweep {index} shape {tuple(feature.shape)} != {expected}"
                )
        ordered_changes = [
            features[index - 1] - features[index]
            for index in range(1, self.num_sweeps)
        ]
        residual = self.motion_encoder(
            torch.cat([current, *ordered_changes], dim=1)
        )
        self.last_residual_rms = residual.detach().square().mean().sqrt()
        return current + residual


@MODELS.register_module()
class MoraiTemporalBEVFusion(BEVFusion):
    """BEVFusion variant that encodes causal LiDAR sweeps separately.

    Input points still use ``[x, y, z, intensity, age_sec]``. The loader has
    already transformed past XYZ into the current LiDAR frame. This class
    separates the constant-age groups, applies the shared voxel/sparse encoder
    to each group, then fuses their aligned BEV features before camera fusion.
    """

    def __init__(
        self,
        temporal_fusion: Optional[dict] = None,
        temporal_feature_warp: bool = False,
        temporal_point_cloud_range: Optional[Sequence[float]] = None,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        temporal_fusion = dict(temporal_fusion or {})
        temporal_fusion.setdefault("type", "MoraiTemporalBEVGate")
        self.temporal_fusion = MODELS.build(temporal_fusion)
        self.temporal_num_sweeps = self.temporal_fusion.num_sweeps
        self.temporal_age_tolerance = 1.0e-5
        self.temporal_feature_warp = bool(temporal_feature_warp)
        self.temporal_point_cloud_range = tuple(
            temporal_point_cloud_range or (-30, -30, -3, 30, 30, 2)
        )
        self._temporal_batch_input_metas = None
        self.online_temporal_cache_enabled = False
        self.online_temporal_cache_maximum_age_sec = 0.30
        self.online_temporal_point_cloud_range = None
        self._online_temporal_cache = []
        self._online_temporal_context = None

    def enable_online_temporal_cache(
        self, point_cloud_range: Sequence[float], maximum_age_sec: float = 0.30
    ) -> None:
        """Enable inference-only recurrent reuse of encoded LiDAR BEV."""
        if len(point_cloud_range) < 6:
            raise ValueError("point_cloud_range must have six values")
        self.online_temporal_cache_enabled = True
        self.online_temporal_point_cloud_range = tuple(
            float(value) for value in point_cloud_range
        )
        self.online_temporal_cache_maximum_age_sec = float(maximum_age_sec)
        self.reset_online_temporal_cache()

    def reset_online_temporal_cache(self) -> None:
        self._online_temporal_cache = []
        self._online_temporal_context = None

    def set_online_temporal_context(
        self, timestamp_sec: float, lidar_pose_world: Sequence[float]
    ) -> None:
        if len(lidar_pose_world) != 4:
            raise ValueError("lidar_pose_world must be [x, y, z, yaw]")
        values = (float(timestamp_sec), tuple(map(float, lidar_pose_world)))
        if not math.isfinite(values[0]) or not all(
            math.isfinite(value) for value in values[1]
        ):
            raise ValueError("temporal cache context must be finite")
        self._online_temporal_context = values

    def _split_causal_sweeps(self, points: List[Tensor]) -> List[List[Tensor]]:
        grouped: List[List[Tensor]] = [
            [] for _ in range(self.temporal_num_sweeps)
        ]
        for sample_index, sample in enumerate(points):
            if sample.ndim != 2 or sample.shape[1] < 5:
                raise ValueError(
                    f"sample {sample_index} must be Nx5 temporal points"
                )
            age = sample[:, 4]
            current_mask = age.abs() <= self.temporal_age_tolerance
            positive_age = age[age > self.temporal_age_tolerance]
            unique_age = torch.unique(positive_age).sort().values
            expected_past = self.temporal_num_sweeps - 1
            if not bool(current_mask.any()) or len(unique_age) != expected_past:
                raise ValueError(
                    f"sample {sample_index}: expected current + {expected_past} "
                    f"past ages, got current={bool(current_mask.any())}, "
                    f"past={unique_age.detach().cpu().tolist()}"
                )
            grouped[0].append(sample[current_mask])
            for sweep_index, sweep_age in enumerate(unique_age, start=1):
                mask = torch.isclose(
                    age,
                    sweep_age,
                    atol=self.temporal_age_tolerance,
                    rtol=0.0,
                )
                grouped[sweep_index].append(sample[mask])
        return grouped

    def _encode_aligned_sweep(self, points: List[Tensor]) -> Tensor:
        with torch.autocast("cuda", enabled=False):
            points = [point.float() for point in points]
            feats, coords, _ = self.voxelize(points)
            batch_size = len(points)
        return self.pts_middle_encoder(feats, coords, batch_size)

    @staticmethod
    def _transform_points(points: Tensor, transform: Tensor) -> Tensor:
        output = points.clone()
        homogeneous = torch.cat(
            [points[:, :3].float(), points.new_ones((len(points), 1)).float()],
            dim=1,
        )
        output[:, :3] = (transform.float() @ homogeneous.T).T[:, :3]
        return output

    def extract_feat(self, batch_inputs_dict, batch_input_metas, **kwargs):
        self._temporal_batch_input_metas = batch_input_metas
        try:
            return super().extract_feat(
                batch_inputs_dict, batch_input_metas, **kwargs)
        finally:
            self._temporal_batch_input_metas = None

    def extract_pts_feat(self, batch_inputs_dict: Dict) -> Tensor:
        if self.online_temporal_cache_enabled:
            return self._extract_pts_feat_online_cached(batch_inputs_dict)
        grouped = self._split_causal_sweeps(batch_inputs_dict["points"])
        if self.temporal_feature_warp:
            sweep_features = self._encode_sweeps_for_feature_warp(grouped)
        else:
            sweep_features = [
                self._encode_aligned_sweep(sweep_points)
                for sweep_points in grouped
            ]
        return self.temporal_fusion(sweep_features)

    def _encode_sweeps_for_feature_warp(
        self, grouped: List[List[Tensor]]
    ) -> List[Tensor]:
        metas = self._temporal_batch_input_metas
        if metas is None or len(metas) != len(grouped[0]):
            raise RuntimeError("temporal feature warp metadata is unavailable")
        features = [self._encode_aligned_sweep(grouped[0])]
        for sweep_index in range(1, self.temporal_num_sweeps):
            source_points = []
            transforms = []
            for sample_index, aligned_points in enumerate(grouped[sweep_index]):
                matrices = metas[sample_index].get("temporal_sweep2lidar")
                if matrices is None or len(matrices) != self.temporal_num_sweeps:
                    raise ValueError("temporal_sweep2lidar metadata is incomplete")
                transform = torch.as_tensor(
                    matrices[sweep_index],
                    device=aligned_points.device,
                    dtype=torch.float32,
                )
                augmentation = torch.as_tensor(
                    metas[sample_index].get("lidar_aug_matrix", torch.eye(4)),
                    device=aligned_points.device,
                    dtype=torch.float32,
                )
                augmented_transform = (
                    augmentation @ transform @ torch.linalg.inv(augmentation)
                )
                inverse_transform = torch.linalg.inv(
                    augmented_transform.float()).to(dtype=aligned_points.dtype)
                source_points.append(self._transform_points(
                    aligned_points, inverse_transform))
                transforms.append(augmented_transform)
            encoded = self._encode_aligned_sweep(source_points)
            warped_samples = [
                warp_bev_feature_by_source_to_target(
                    encoded[index:index + 1],
                    transforms[index],
                    self.temporal_point_cloud_range,
                )
                for index in range(len(transforms))
            ]
            features.append(torch.cat(warped_samples, dim=0))
        return features

    def _extract_pts_feat_online_cached(self, batch_inputs_dict: Dict) -> Tensor:
        if self.training:
            raise RuntimeError("online temporal cache is inference-only")
        if self._online_temporal_context is None:
            raise RuntimeError("online temporal cache context was not set")
        points = batch_inputs_dict["points"]
        current_points = []
        for sample_index, sample in enumerate(points):
            if sample.ndim != 2 or sample.shape[1] < 5:
                raise ValueError(
                    f"sample {sample_index} must be Nx5 temporal points"
                )
            current_mask = sample[:, 4].abs() <= self.temporal_age_tolerance
            if not bool(current_mask.any()):
                raise ValueError(f"sample {sample_index} has no current points")
            current_points.append(sample[current_mask])
        current = self._encode_aligned_sweep(current_points)
        timestamp, target_pose = self._online_temporal_context

        valid_cache = []
        for source_timestamp, source_pose, source_feature in reversed(
            self._online_temporal_cache
        ):
            age = timestamp - source_timestamp
            if age <= 1.0e-6:
                self._online_temporal_cache = []
                valid_cache = []
                break
            if age <= self.online_temporal_cache_maximum_age_sec:
                valid_cache.append((source_pose, source_feature))
            if len(valid_cache) >= self.temporal_num_sweeps - 1:
                break

        features = [current]
        for source_pose, source_feature in valid_cache:
            transform = source_to_target_matrix_from_lidar_poses(
                source_pose, target_pose, device=source_feature.device)
            features.append(warp_bev_feature_by_source_to_target(
                source_feature, transform, self.online_temporal_point_cloud_range))
        while len(features) < self.temporal_num_sweeps:
            features.append(current)

        self._online_temporal_cache.append(
            (timestamp, target_pose, current.detach())
        )
        self._online_temporal_cache = self._online_temporal_cache[
            -(self.temporal_num_sweeps - 1):
        ]
        self._online_temporal_context = None
        return self.temporal_fusion(features)
