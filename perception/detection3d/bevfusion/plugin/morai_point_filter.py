"""Opt-in MORAI no-return filtering before geometric augmentation."""

import torch
from mmcv.transforms import BaseTransform
from mmdet3d.registry import TRANSFORMS


@TRANSFORMS.register_module()
class MoraiFilterInvalidPoints(BaseTransform):
    """Remove nonfinite rows and exact XYZ-zero beam placeholders.

    Intensity zero alone is NOT invalid. There is no near-distance cutoff.
    Run immediately after loading, before translation can move placeholders.
    Empty clouds stay empty; never synthesize a fake point or modify GT.
    """

    def transform(self, data):
        points = data['points']
        tensor = points.tensor
        keep = torch.isfinite(tensor).all(dim=1)
        keep &= (tensor[:, :3] != 0).any(dim=1)
        data['points'] = points[keep]
        return data
