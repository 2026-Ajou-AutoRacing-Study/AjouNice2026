"""Opt-in experimental positive-camera-Z nearest hint; production unchanged."""
from copy import deepcopy
_base_ = './bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py'
# Inherits actual learned spatial/depth/image geometry and separated pool.
custom_imports = deepcopy(_base_.custom_imports)
custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_depth_raster')
model = dict(view_transform=dict(type='MoraiDepthRasterSeparatedDepthLSSTransform',
                                 depth_raster_policy='nearest_positive', depth_raster_backend='sorted'))
