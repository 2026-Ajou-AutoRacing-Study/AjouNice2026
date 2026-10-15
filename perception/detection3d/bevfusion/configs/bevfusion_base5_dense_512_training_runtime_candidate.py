"""Opt-in training-runtime qualification; not the default or a main-run approval.

512x1056, original spatial grid/depth/augmentation/losses remain unchanged.
Use a NEW guarded work_dir; never resume an old source/config contract here.
"""
from copy import deepcopy
_base_ = './bevfusion_base5_dense_512_native_jpeg90.py'
custom_imports = deepcopy(_base_.custom_imports)
custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_training_runtime')
model = dict(type='MoraiTrainingRuntimeBEVFusion', view_transform=dict(
    pool_backward_mode='warp', pool_compact_indices=True,
    pool_geometry_mode='batched_checked'))
optim_wrapper = dict(optimizer=dict(fused=True, foreach=False))
