import torch
import torch.nn as nn
import torch.nn.functional as F

from .temporal_pointpillar_scatter import TemporalPointPillarScatter


class MATPMotionPointPillarScatter(TemporalPointPillarScatter):
    """
    MATP-v2:
    Explicit cross-frame motion feature modeling.

    Base branch:
        Same as T-Feature
        F(t-2), F(t-1), F(t)
        -> concat
        -> 1x1 temporal fusion
        -> F_base

    Motion branch:
        For each historical sweep k:

        signed_diff = F(t) - F(k)
        abs_diff    = |F(t) - F(k)|

        [signed_diff, abs_diff]
        -> shared Motion Encoder
        -> + actual-dt embedding
        -> M(k,t)

        M(t-2,t), M(t-1,t)
        -> residual fusion
        -> F_motion

    Final:
        F_temporal = F_base + F_motion

    The final residual projection is zero-initialized.
    Therefore the initial network is exactly equivalent to
    the T-Feature output before the motion branch learns.

    This version does NOT use the multiplicative MATP-v1
    time gate.
    """

    def __init__(self, model_cfg, grid_size, **kwargs):
        super().__init__(
            model_cfg=model_cfg,
            grid_size=grid_size,
            **kwargs
        )

        self.time_hidden_dim = int(
            self.model_cfg.get('TIME_HIDDEN_DIM', 32)
        )

        self.time_scale = float(
            self.model_cfg.get('TIME_SCALE', 10.0)
        )

        c = self.num_bev_features
        num_history = self.num_sweeps - 1

        if num_history < 1:
            raise ValueError(
                "MATP-v2 requires at least two sweeps"
            )

        # --------------------------------------------------
        # Explicit motion encoder.
        #
        # Input:
        #   signed difference C
        #   absolute difference C
        #
        # Total = 2C
        # --------------------------------------------------
        self.motion_encoder = nn.Sequential(
            nn.Conv2d(
                2 * c,
                c,
                kernel_size=1,
                stride=1,
                padding=0,
                bias=False
            ),
            nn.BatchNorm2d(
                c,
                eps=1e-3,
                momentum=0.01
            ),
            nn.ReLU(inplace=True)
        )

        # Shared actual-dt embedding.
        self.time_mlp = nn.Sequential(
            nn.Linear(1, self.time_hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(self.time_hidden_dim, c)
        )

        # --------------------------------------------------
        # Combine motion features from historical sweeps.
        #
        # Final Conv has zero initialization so the initial
        # motion residual is exactly zero.
        # --------------------------------------------------
        self.motion_residual = nn.Sequential(
            nn.Conv2d(
                num_history * c,
                c,
                kernel_size=1,
                stride=1,
                padding=0,
                bias=False
            ),
            nn.BatchNorm2d(
                c,
                eps=1e-3,
                momentum=0.01
            ),
            nn.ReLU(inplace=True),
            nn.Conv2d(
                c,
                c,
                kernel_size=1,
                stride=1,
                padding=0,
                bias=True
            )
        )

        nn.init.zeros_(
            self.motion_residual[-1].weight
        )
        nn.init.zeros_(
            self.motion_residual[-1].bias
        )

    def forward(self, batch_dict, **kwargs):

        pillar_features = batch_dict['pillar_features']
        coords = batch_dict['voxel_coords']

        if coords.shape[1] != 5:
            raise ValueError(
                "MATPMotionPointPillarScatter expects "
                "voxel_coords=[batch,z,y,x,sweep_id], "
                f"got {tuple(coords.shape)}"
            )

        if 'temporal_sweep_times' not in batch_dict:
            raise KeyError(
                "MATP-v2 requires temporal_sweep_times"
            )

        batch_size = int(batch_dict['batch_size'])

        sweep_times = batch_dict[
            'temporal_sweep_times'
        ]

        if not torch.is_tensor(sweep_times):
            sweep_times = torch.as_tensor(
                sweep_times,
                dtype=pillar_features.dtype,
                device=pillar_features.device
            )
        else:
            sweep_times = sweep_times.to(
                dtype=pillar_features.dtype,
                device=pillar_features.device
            )

        if sweep_times.shape != (
            batch_size,
            self.num_sweeps
        ):
            raise ValueError(
                "Expected temporal_sweep_times "
                f"[{batch_size},{self.num_sweeps}], "
                f"got {tuple(sweep_times.shape)}"
            )

        sweep_ids = coords[:, 4].long()

        if sweep_ids.numel() == 0:
            raise ValueError(
                "MATP-v2 received no temporal pillars"
            )

        if (
            int(sweep_ids.min().item()) < 0
            or
            int(sweep_ids.max().item())
            >= self.num_sweeps
        ):
            raise ValueError(
                "Invalid temporal sweep IDs"
            )

        # ==================================================
        # Per-sweep BEV
        # ==================================================
        sweep_bevs = []

        for sweep_id in range(self.num_sweeps):

            sweep_bev = self._scatter_one_sweep(
                pillar_features=pillar_features,
                coords=coords,
                batch_size=batch_size,
                sweep_id=sweep_id
            )

            sweep_bevs.append(sweep_bev)

        # ==================================================
        # T-Feature base branch
        # ==================================================
        temporal_features = torch.cat(
            sweep_bevs,
            dim=1
        )

        base_temporal = self.temporal_fusion(
            temporal_features
        )

        # Current sweep is always the final sweep:
        # sweep_times sorted oldest -> current.
        current_bev = sweep_bevs[-1]

        # ==================================================
        # Explicit cross-frame motion branch
        # ==================================================
        motion_features = []

        for sweep_id in range(
            self.num_sweeps - 1
        ):

            past_bev = sweep_bevs[sweep_id]

            signed_diff = (
                current_bev - past_bev
            )

            abs_diff = torch.abs(
                signed_diff
            )

            # Explicit feature variation.
            motion_input = torch.cat(
                [
                    signed_diff,
                    abs_diff
                ],
                dim=1
            )

            motion = self.motion_encoder(
                motion_input
            )

            # Actual elapsed time.
            dt = sweep_times[
                :,
                sweep_id:sweep_id + 1
            ]

            dt_scaled = (
                dt * self.time_scale
            )

            dt_embedding = self.time_mlp(
                dt_scaled
            )

            dt_embedding = (
                dt_embedding
                .unsqueeze(-1)
                .unsqueeze(-1)
            )

            # dt conditions the explicit motion feature,
            # rather than globally reweighting the BEV.
            motion = F.relu(
                motion + dt_embedding,
                inplace=False
            )

            motion_features.append(
                motion
            )

        motion_cat = torch.cat(
            motion_features,
            dim=1
        )

        motion_residual = self.motion_residual(
            motion_cat
        )

        # ==================================================
        # Motion-aware temporal representation
        # ==================================================
        spatial_features = (
            base_temporal
            + motion_residual
        )

        batch_dict[
            'spatial_features'
        ] = spatial_features

        # Diagnostics
        batch_dict[
            'matp_base_temporal'
        ] = base_temporal

        batch_dict[
            'matp_motion_residual'
        ] = motion_residual

        return batch_dict
