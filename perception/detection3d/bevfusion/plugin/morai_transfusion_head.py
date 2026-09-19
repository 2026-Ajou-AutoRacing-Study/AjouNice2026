"""MORAI-safe initialization for the upstream BEVFusion TransFusion head."""

from torch import nn
from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion.transfusion_head import TransFusionHead


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
