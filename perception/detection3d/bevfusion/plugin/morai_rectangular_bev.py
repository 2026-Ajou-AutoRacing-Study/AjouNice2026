"""Opt-in rectangular [Y,X] feature geometry; legacy square E25 stays untouched."""
import torch
from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion.depth_lss import DepthLSSTransform
from .morai_transfusion_head import MoraiTransFusionHead


@MODELS.register_module()
class MoraiRectangularDepthLSSTransform(DepthLSSTransform):
    def bev_pool(self, geom_feats, x):
        # Upstream camera pool emits [B,C,X,Y]; sparse LiDAR emits [B,C,Y,X].
        # Transpose before learned downsampling, not the metric bounds/coordinates.
        return super().bev_pool(geom_feats,x).transpose(-1,-2).contiguous()


@MODELS.register_module()
class MoraiRectangularTransFusionHead(MoraiTransFusionHead):
    def create_2D_grid(self,x_size,y_size):
        # Flattened feature index = y*x_size+x. Positional coordinate = (x,y).
        y,x=torch.meshgrid(torch.arange(y_size,dtype=torch.float32),
                           torch.arange(x_size,dtype=torch.float32),indexing='ij')
        return torch.stack((x+.5,y+.5),dim=-1).reshape(1,-1,2)
