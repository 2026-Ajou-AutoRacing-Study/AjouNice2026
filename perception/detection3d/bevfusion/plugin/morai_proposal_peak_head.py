"""Opt-in Base5 query peak experiment. No production class/default is changed."""

from mmdet3d.registry import MODELS
from .morai_rectangular_bev import MoraiRectangularTransFusionHead
from perception.detection3d.bevfusion.proposal_peak import (
    forward_single_with_explicit_peaks,
    validate_peak_kernels,
)


@MODELS.register_module()
class MoraiProposalPeakTransFusionHead(MoraiRectangularTransFusionHead):
    """Same trainable parameters and GT contract; explicit opt-in query masking.

    None deliberately calls the real inherited legacy forward_single. Even an
    empty list fails rather than implicitly guessing dataset/class semantics.
    Example ordered V/P/O kernels [3,1,3] enables obstacle peaks only.
    """
    def __init__(self, *args, proposal_peak_kernels=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.proposal_peak_kernels = (None if proposal_peak_kernels is None else
            validate_peak_kernels(proposal_peak_kernels, self.num_classes))

    def forward_single(self, inputs, metas):
        if self.proposal_peak_kernels is None:
            return super().forward_single(inputs, metas)
        return forward_single_with_explicit_peaks(
            self, inputs, metas, self.proposal_peak_kernels)
