"""Lightweight sweep-separated temporal BEV fusion for MORAI Base4."""

from __future__ import annotations

from typing import Dict, List, Optional

import torch
from torch import Tensor, nn

from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion import BEVFusion


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
class MoraiTemporalBEVFusion(BEVFusion):
    """BEVFusion variant that encodes causal LiDAR sweeps separately.

    Input points still use ``[x, y, z, intensity, age_sec]``. The loader has
    already transformed past XYZ into the current LiDAR frame. This class
    separates the constant-age groups, applies the shared voxel/sparse encoder
    to each group, then fuses their aligned BEV features before camera fusion.
    """

    def __init__(self, temporal_fusion: Optional[dict] = None, **kwargs) -> None:
        super().__init__(**kwargs)
        temporal_fusion = dict(temporal_fusion or {})
        temporal_fusion.setdefault("type", "MoraiTemporalBEVGate")
        self.temporal_fusion = MODELS.build(temporal_fusion)
        self.temporal_num_sweeps = self.temporal_fusion.num_sweeps
        self.temporal_age_tolerance = 1.0e-5

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

    def extract_pts_feat(self, batch_inputs_dict: Dict) -> Tensor:
        grouped = self._split_causal_sweeps(batch_inputs_dict["points"])
        sweep_features = [
            self._encode_aligned_sweep(sweep_points)
            for sweep_points in grouped
        ]
        return self.temporal_fusion(sweep_features)
