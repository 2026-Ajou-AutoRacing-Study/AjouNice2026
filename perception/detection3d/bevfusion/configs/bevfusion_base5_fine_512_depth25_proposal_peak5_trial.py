"""Offline-only query-peak5 trial. Larger close-object suppression risk than3.

    Separate opt-in config, never selected by production launch or a training
    initializer. Feature pitch remains0.4m; change is a5-cell peak neighborhood.
"""
_base_ = './bevfusion_base5_fine_512_depth25_proposal_peak3_trial.py'
model = dict(bbox_head=dict(proposal_peak_kernels=[3, 1, 5]))
