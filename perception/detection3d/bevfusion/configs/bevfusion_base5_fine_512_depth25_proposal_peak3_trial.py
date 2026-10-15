"""Offline-only E24 query-peak3 trial; no production or training approval.

    Same weights/grid/depth/ROI/crop/classes. Only class-2 heatmap peaks change.
    Check near distinct actors and low-support recall, not just duplicate count.
"""
from copy import deepcopy
_base_ = './bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py'
custom_imports = deepcopy(_base_.custom_imports)
custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_proposal_peak_head')
model = dict(bbox_head=dict(type='MoraiProposalPeakTransFusionHead', proposal_peak_kernels=[3, 1, 3]))
