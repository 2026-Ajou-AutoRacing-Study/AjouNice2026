"""Causal MORAI LiDAR sweep loader with explicit Ego-motion transforms."""

from pathlib import Path

import numpy as np

from mmcv.transforms import BaseTransform
from mmdet3d.registry import TRANSFORMS


@TRANSFORMS.register_module()
class MoraiLoadPointsFromCausalSweeps(BaseTransform):
    """Append past XYZI sweeps in the current LiDAR frame plus sweep age.

    The info builder stores a direct ``sweep2lidar`` SE(3) matrix. This keeps
    the online-reproducible operation explicit: only Ego motion is corrected;
    moving-object trails remain as velocity evidence.
    """

    def __init__(self, sweeps_num=2, pad_empty_sweeps=False):
        self.sweeps_num = int(sweeps_num)
        self.pad_empty_sweeps = bool(pad_empty_sweeps)
        if self.sweeps_num < 0:
            raise ValueError("sweeps_num must be >= 0")

    @staticmethod
    def _load_xyzi(path):
        values = np.fromfile(str(Path(path)), dtype=np.float32)
        if values.size % 4:
            raise ValueError("sweep XYZI float count is not divisible by 4")
        return values.reshape(-1, 4)

    @staticmethod
    def _new_temporal_point(reference, values):
        """Build a point object whose declared width includes sweep age."""
        attributes = dict(reference.attribute_dims or {})
        attributes["time"] = 4
        return type(reference)(
            values,
            points_dim=5,
            attribute_dims=attributes,
        )

    def transform(self, results):
        current = results["points"]
        current_values = current.numpy()
        if current_values.shape[1] != 4:
            raise ValueError(
                "current MORAI points must be XYZI before temporal loading"
            )
        current_5d = np.concatenate(
            [current_values, np.zeros((len(current_values), 1), np.float32)],
            axis=1,
        )
        point_sets = [self._new_temporal_point(current, current_5d)]
        sweeps = list(results.get("lidar_sweeps", []))[:self.sweeps_num]
        if not sweeps and self.pad_empty_sweeps:
            sweeps = [None] * self.sweeps_num
        for sweep in sweeps:
            if sweep is None:
                point_sets.append(
                    self._new_temporal_point(current, current_5d.copy())
                )
                continue
            values = self._load_xyzi(sweep["lidar_points"]["lidar_path"])
            transform = np.asarray(
                sweep["lidar_points"]["sweep2lidar"], dtype=np.float64
            )
            homogeneous = np.concatenate(
                [values[:, :3].astype(np.float64), np.ones((len(values), 1))],
                axis=1,
            )
            aligned = (transform @ homogeneous.T).T[:, :3].astype(np.float32)
            age = np.full(
                (len(values), 1), float(sweep["time_lag_sec"]), np.float32
            )
            point_sets.append(self._new_temporal_point(
                current,
                np.concatenate([aligned, values[:, 3:4], age], axis=1),
            ))
        results["points"] = current.cat(point_sets)
        results["num_pts_feats"] = 5
        return results
