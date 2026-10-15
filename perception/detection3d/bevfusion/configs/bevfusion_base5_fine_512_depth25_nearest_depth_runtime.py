"""E24 runtime qualification 2026-10-14; learned geometry/weights unchanged."""
from copy import deepcopy
_base_ = './bevfusion_base5_fine_512_depth25_pedestrian_merged_runtime.py'
custom_imports = deepcopy(_base_.custom_imports)
custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_depth_raster')
model = dict(view_transform=dict(type='MoraiDepthRasterSeparatedDepthLSSTransform',
                                 depth_raster_policy='nearest_positive', depth_raster_backend='sorted'))
