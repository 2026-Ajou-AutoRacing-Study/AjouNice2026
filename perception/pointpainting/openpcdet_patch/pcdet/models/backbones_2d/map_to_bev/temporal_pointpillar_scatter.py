import torch
import torch.nn as nn


class TemporalPointPillarScatter(nn.Module):
    """
    Simple feature-level temporal fusion baseline.

    Input voxel coordinates after collate_batch:
        [batch_idx, z, y, x, sweep_id]

    Pipeline:
        shared PillarVFE
            -> per-sweep BEV scatter
            -> concat [F_oldest, ..., F_current]
            -> 1x1 Conv + BN + ReLU
            -> spatial_features

    This module deliberately does NOT implement explicit motion modeling
    or explicit delta-t conditioning. Those belong to MATP, not T-Feature.
    """

    def __init__(self, model_cfg, grid_size, **kwargs):
        super().__init__()

        self.model_cfg = model_cfg
        self.num_bev_features = self.model_cfg.NUM_BEV_FEATURES
        self.num_sweeps = int(self.model_cfg.get('NUM_SWEEPS', 3))

        self.nx, self.ny, self.nz = grid_size
        assert self.nz == 1

        input_channels = self.num_bev_features * self.num_sweeps

        self.temporal_fusion = nn.Sequential(
            nn.Conv2d(
                input_channels,
                self.num_bev_features,
                kernel_size=1,
                stride=1,
                padding=0,
                bias=False
            ),
            nn.BatchNorm2d(
                self.num_bev_features,
                eps=1e-3,
                momentum=0.01
            ),
            nn.ReLU(inplace=True)
        )

    def _scatter_one_sweep(
        self,
        pillar_features,
        coords,
        batch_size,
        sweep_id
    ):
        batch_spatial_features = []

        for batch_idx in range(batch_size):
            spatial_feature = torch.zeros(
                self.num_bev_features,
                self.nz * self.nx * self.ny,
                dtype=pillar_features.dtype,
                device=pillar_features.device
            )

            batch_mask = coords[:, 0] == batch_idx
            sweep_mask = coords[:, 4] == sweep_id
            mask = batch_mask & sweep_mask

            this_coords = coords[mask, :]

            if this_coords.shape[0] > 0:
                # coords:
                # [batch, z, y, x, sweep]
                indices = (
                    this_coords[:, 1]
                    + this_coords[:, 2] * self.nx
                    + this_coords[:, 3]
                ).long()

                pillars = pillar_features[mask, :].t()
                spatial_feature[:, indices] = pillars

            batch_spatial_features.append(spatial_feature)

        batch_spatial_features = torch.stack(
            batch_spatial_features,
            dim=0
        )

        batch_spatial_features = batch_spatial_features.view(
            batch_size,
            self.num_bev_features * self.nz,
            self.ny,
            self.nx
        )

        return batch_spatial_features

    def forward(self, batch_dict, **kwargs):
        pillar_features = batch_dict['pillar_features']
        coords = batch_dict['voxel_coords']

        if coords.shape[1] != 5:
            raise ValueError(
                "TemporalPointPillarScatter expects voxel_coords "
                f"[batch,z,y,x,sweep_id], got shape={tuple(coords.shape)}"
            )

        sweep_ids = coords[:, 4].long()

        if sweep_ids.numel() == 0:
            raise ValueError("No temporal pillars were provided")

        min_id = int(sweep_ids.min().item())
        max_id = int(sweep_ids.max().item())

        if min_id < 0 or max_id >= self.num_sweeps:
            raise ValueError(
                f"Invalid sweep id range [{min_id}, {max_id}], "
                f"NUM_SWEEPS={self.num_sweeps}"
            )

        batch_size = int(batch_dict['batch_size'])

        sweep_bev_features = []

        # Temporal order is preserved:
        # oldest -> ... -> current
        for sweep_id in range(self.num_sweeps):
            sweep_bev = self._scatter_one_sweep(
                pillar_features=pillar_features,
                coords=coords,
                batch_size=batch_size,
                sweep_id=sweep_id
            )
            sweep_bev_features.append(sweep_bev)

        temporal_features = torch.cat(
            sweep_bev_features,
            dim=1
        )

        spatial_features = self.temporal_fusion(
            temporal_features
        )

        batch_dict['spatial_features'] = spatial_features

        return batch_dict
