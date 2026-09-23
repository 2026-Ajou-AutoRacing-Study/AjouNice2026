"""Causal sparse query-memory primitives for the Base4 Q1 experiment.

This module intentionally does not depend on MMDetection3D registries.  It is
the checkpoint-safe P0 building block that will be connected to TransFusion
only after its geometry, masking, reset, and gradient contracts are tested.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import torch
from torch import Tensor, nn


def planar_source_to_target_matrix(
    source_pose_world: Sequence[float],
    target_pose_world: Sequence[float],
    *,
    device=None,
) -> Tensor:
    """Return ``p_target = T_target_source p_source`` for planar LiDAR poses."""
    if len(source_pose_world) != 4 or len(target_pose_world) != 4:
        raise ValueError("poses must be [x, y, z, yaw]")
    source_x, source_y, source_z, source_yaw = map(float, source_pose_world)
    target_x, target_y, target_z, target_yaw = map(float, target_pose_world)
    delta_yaw = source_yaw - target_yaw
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)
    dx_world = source_x - target_x
    dy_world = source_y - target_y
    cos_target = math.cos(target_yaw)
    sin_target = math.sin(target_yaw)
    transform = torch.eye(4, dtype=torch.float32, device=device)
    transform[0, 0] = cosine
    transform[0, 1] = -sine
    transform[1, 0] = sine
    transform[1, 1] = cosine
    transform[0, 3] = cos_target * dx_world + sin_target * dy_world
    transform[1, 3] = -sin_target * dx_world + cos_target * dy_world
    transform[2, 3] = source_z - target_z
    return transform


def transform_query_centers(centers_xy: Tensor, transform: Tensor) -> Tensor:
    """Transform query centers while preserving arbitrary leading dimensions."""
    if centers_xy.shape[-1] != 2:
        raise ValueError("query centers must end in XY")
    transform = torch.as_tensor(
        transform, dtype=torch.float32, device=centers_xy.device)
    if transform.shape != (4, 4):
        raise ValueError("transform must be 4x4")
    flat = centers_xy.reshape(-1, 2).float()
    homogeneous = torch.cat(
        [flat, torch.zeros_like(flat[:, :1]), torch.ones_like(flat[:, :1])],
        dim=-1,
    )
    transformed = (transform @ homogeneous.T).T[:, :2]
    return transformed.reshape_as(centers_xy).to(dtype=centers_xy.dtype)


class MoraiQueryMemoryAttention(nn.Module):
    """One causal cross-attention layer over gated Vehicle query memory.

    Features use ``[B, C, N]`` to match TransFusion.  Memory ages are positive
    seconds into the past.  A near-zero residual gate preserves a warm-started
    spatial detector while allowing gradients through the memory branch.
    """

    def __init__(
        self,
        channels: int = 128,
        num_heads: int = 4,
        vehicle_label: int = 0,
        minimum_memory_score: float = 0.20,
        maximum_memory_age_sec: float = 0.60,
        maximum_association_distance_m: float = 6.0,
        initial_residual_logit: float = -4.0,
        use_motion_value: bool = False,
    ) -> None:
        super().__init__()
        if channels % num_heads != 0:
            raise ValueError("channels must be divisible by num_heads")
        self.channels = int(channels)
        self.num_heads = int(num_heads)
        self.head_channels = self.channels // self.num_heads
        self.vehicle_label = int(vehicle_label)
        self.minimum_memory_score = float(minimum_memory_score)
        self.maximum_memory_age_sec = float(maximum_memory_age_sec)
        self.maximum_association_distance_m = float(
            maximum_association_distance_m)
        self.use_motion_value = bool(use_motion_value)

        self.query_projection = nn.Linear(self.channels, self.channels)
        self.key_projection = nn.Linear(self.channels, self.channels)
        self.value_projection = nn.Linear(self.channels, self.channels)
        self.output_projection = nn.Linear(self.channels, self.channels)
        self.motion_bias = nn.Sequential(
            nn.Linear(3, 32), nn.SiLU(), nn.Linear(32, self.num_heads))
        if self.use_motion_value:
            self.motion_value = nn.Sequential(
                nn.Linear(3, self.channels), nn.SiLU(),
                nn.Linear(self.channels, self.channels))
        else:
            self.motion_value = None
        self.residual_logit = nn.Parameter(torch.tensor(
            float(initial_residual_logit), dtype=torch.float32))
        self.last_active_query_fraction: Optional[Tensor] = None
        self.last_mean_attention_age_sec: Optional[Tensor] = None
        self.last_active_query_mask: Optional[Tensor] = None
        self.last_motion_context: Optional[Tensor] = None
        self.last_direct_velocity: Optional[Tensor] = None

    def forward(
        self,
        current_features: Tensor,
        current_centers_xy: Tensor,
        current_labels: Tensor,
        memory_features: Tensor,
        memory_centers_xy: Tensor,
        memory_scores: Tensor,
        memory_labels: Tensor,
        memory_ages_sec: Tensor,
    ) -> Tensor:
        if current_features.ndim != 3 or memory_features.ndim != 3:
            raise ValueError("query features must be [B,C,N]")
        batch, channels, current_count = current_features.shape
        memory_batch, memory_channels, memory_count = memory_features.shape
        if batch != memory_batch or channels != self.channels or \
                memory_channels != self.channels:
            raise ValueError("query-memory feature shape mismatch")
        if current_centers_xy.shape != (batch, current_count, 2):
            raise ValueError("current center shape mismatch")
        if memory_centers_xy.shape != (batch, memory_count, 2):
            raise ValueError("memory center shape mismatch")
        if memory_count == 0:
            self.last_active_query_fraction = current_features.new_tensor(0.0)
            self.last_mean_attention_age_sec = current_features.new_tensor(0.0)
            self.last_active_query_mask = torch.zeros(
                batch, current_count, dtype=torch.bool,
                device=current_features.device)
            self.last_motion_context = torch.zeros_like(current_features)
            self.last_direct_velocity = current_features.new_zeros(
                batch, 2, current_count)
            return current_features

        current = current_features.transpose(1, 2)
        memory = memory_features.transpose(1, 2)
        query = self.query_projection(current).reshape(
            batch, current_count, self.num_heads, self.head_channels)
        key = self.key_projection(memory).reshape(
            batch, memory_count, self.num_heads, self.head_channels)
        value = self.value_projection(memory).reshape(
            batch, memory_count, self.num_heads, self.head_channels)
        logits = torch.einsum("bnhd,bmhd->bhnm", query, key)
        logits = logits / math.sqrt(self.head_channels)

        delta = current_centers_xy[:, :, None, :] - \
            memory_centers_xy[:, None, :, :]
        ages = memory_ages_sec[:, None, :, None].expand(
            -1, current_count, -1, -1)
        velocity = delta / ages.clamp_min(1.0e-3)
        motion = torch.cat([velocity, ages], dim=-1)
        logits = logits + self.motion_bias(motion).permute(0, 3, 1, 2)

        distance = torch.linalg.vector_norm(delta, dim=-1)
        current_vehicle = current_labels == self.vehicle_label
        memory_valid = (
            (memory_labels == self.vehicle_label)
            & (memory_scores >= self.minimum_memory_score)
            & (memory_ages_sec > 0.0)
            & (memory_ages_sec <= self.maximum_memory_age_sec)
        )
        valid_pairs = (
            current_vehicle[:, :, None]
            & memory_valid[:, None, :]
            & (distance <= self.maximum_association_distance_m)
        )
        expanded_valid = valid_pairs[:, None, :, :]
        masked_logits = logits.masked_fill(~expanded_valid, -1.0e4)
        weights = torch.softmax(masked_logits, dim=-1) * expanded_valid
        weights = weights / weights.sum(dim=-1, keepdim=True).clamp_min(1.0e-8)
        pair_value = value[:, None, :, :, :].expand(
            -1, current_count, -1, -1, -1)
        if self.motion_value is not None:
            encoded_motion = self.motion_value(motion).reshape(
                batch, current_count, memory_count,
                self.num_heads, self.head_channels)
            pair_value = pair_value + encoded_motion
        context = torch.einsum("bhnm,bnmhd->bnhd", weights, pair_value)
        context = context.reshape(batch, current_count, channels)
        residual = self.output_projection(context)
        active = valid_pairs.any(dim=-1)
        residual = residual * active[:, :, None]
        motion_context = context * active[:, :, None]
        velocity_weights = weights.mean(dim=1)
        direct_velocity = torch.einsum(
            "bnm,bnmd->bnd", velocity_weights, velocity)
        direct_velocity = direct_velocity * active[:, :, None]
        scale = torch.sigmoid(self.residual_logit)
        output = current + scale * residual

        self.last_active_query_mask = active
        self.last_motion_context = motion_context.transpose(1, 2)
        self.last_direct_velocity = direct_velocity.transpose(1, 2)

        with torch.no_grad():
            self.last_active_query_fraction = active.float().mean()
            age_weights = weights.mean(dim=1).sum(dim=1)
            denominator = age_weights.sum().clamp_min(1.0e-8)
            self.last_mean_attention_age_sec = (
                age_weights * memory_ages_sec).sum() / denominator
        return output.transpose(1, 2)


@dataclass
class _MemoryFrame:
    features: Tensor
    centers_xy: Tensor
    scores: Tensor
    labels: Tensor
    timestamp_sec: float
    lidar_pose_world: Tuple[float, float, float, float]


class CausalQueryMemoryBank:
    """Inference/streaming FIFO with explicit scene and time reset policy."""

    def __init__(
        self,
        max_frames: int = 4,
        max_queries_per_frame: int = 64,
        maximum_age_sec: float = 0.60,
        reset_gap_sec: float = 0.30,
        vehicle_label: int = 0,
    ) -> None:
        self.max_frames = int(max_frames)
        self.max_queries_per_frame = int(max_queries_per_frame)
        self.maximum_age_sec = float(maximum_age_sec)
        self.reset_gap_sec = float(reset_gap_sec)
        self.vehicle_label = int(vehicle_label)
        self.reset()

    def reset(self) -> None:
        self._scene_id: Optional[str] = None
        self._last_timestamp_sec: Optional[float] = None
        self._frames: List[_MemoryFrame] = []

    def _ensure_context(self, scene_id: str, timestamp_sec: float) -> None:
        invalid_time = self._last_timestamp_sec is not None and (
            timestamp_sec <= self._last_timestamp_sec
            or timestamp_sec - self._last_timestamp_sec > self.reset_gap_sec
        )
        if self._scene_id != scene_id or invalid_time:
            self.reset()
            self._scene_id = str(scene_id)

    def prepare(
        self,
        scene_id: str,
        timestamp_sec: float,
        target_lidar_pose_world: Sequence[float],
        *,
        channels: int,
        device,
        dtype,
    ) -> Dict[str, Tensor]:
        self._ensure_context(str(scene_id), float(timestamp_sec))
        features, centers, scores, labels, ages = [], [], [], [], []
        for frame in self._frames:
            age = float(timestamp_sec) - frame.timestamp_sec
            if age <= 0.0 or age > self.maximum_age_sec:
                continue
            transform = planar_source_to_target_matrix(
                frame.lidar_pose_world, target_lidar_pose_world,
                device=frame.centers_xy.device)
            features.append(frame.features)
            centers.append(transform_query_centers(frame.centers_xy, transform))
            scores.append(frame.scores)
            labels.append(frame.labels)
            ages.append(frame.scores.new_full(frame.scores.shape, age))
        if not features:
            return {
                "features": torch.empty(1, channels, 0, device=device, dtype=dtype),
                "centers_xy": torch.empty(1, 0, 2, device=device, dtype=dtype),
                "scores": torch.empty(1, 0, device=device, dtype=dtype),
                "labels": torch.empty(1, 0, device=device, dtype=torch.long),
                "ages_sec": torch.empty(1, 0, device=device, dtype=dtype),
            }
        return {
            "features": torch.cat(features, dim=-1).unsqueeze(0).to(
                device=device, dtype=dtype),
            "centers_xy": torch.cat(centers, dim=0).unsqueeze(0).to(
                device=device, dtype=dtype),
            "scores": torch.cat(scores).unsqueeze(0).to(device=device, dtype=dtype),
            "labels": torch.cat(labels).unsqueeze(0).to(device=device),
            "ages_sec": torch.cat(ages).unsqueeze(0).to(device=device, dtype=dtype),
        }

    def push(
        self,
        scene_id: str,
        timestamp_sec: float,
        lidar_pose_world: Sequence[float],
        features: Tensor,
        centers_xy: Tensor,
        scores: Tensor,
        labels: Tensor,
    ) -> None:
        self._ensure_context(str(scene_id), float(timestamp_sec))
        if self.max_frames <= 0:
            self._frames = []
            self._scene_id = str(scene_id)
            self._last_timestamp_sec = float(timestamp_sec)
            return
        if features.ndim != 2 or features.shape[1] != centers_xy.shape[0]:
            raise ValueError("features must be [C,N] and align with centers")
        vehicle_indices = torch.where(labels == self.vehicle_label)[0]
        if len(vehicle_indices) > 0:
            order = torch.argsort(scores[vehicle_indices], descending=True)
            vehicle_indices = vehicle_indices[
                order[:self.max_queries_per_frame]]
        frame = _MemoryFrame(
            features=features[:, vehicle_indices].detach(),
            centers_xy=centers_xy[vehicle_indices].detach(),
            scores=scores[vehicle_indices].detach(),
            labels=labels[vehicle_indices].detach(),
            timestamp_sec=float(timestamp_sec),
            lidar_pose_world=tuple(map(float, lidar_pose_world)),
        )
        self._frames.append(frame)
        self._frames = self._frames[-self.max_frames:]
        self._scene_id = str(scene_id)
        self._last_timestamp_sec = float(timestamp_sec)
