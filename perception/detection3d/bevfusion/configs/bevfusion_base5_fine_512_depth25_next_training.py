"""Fresh Base5 training preparation: eager camera and shared nearest depth hint.

The completed E24 training/resume configuration and production weights remain
unchanged. Use a new work directory and explicitly qualify data, initialization
and GPU cost before starting this configuration; it is not an E24 resume path.
"""

_base_ = './bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py'

required_next_training_policy = 'base5_eager_nearest_v1'
model = dict(compile_camera=False)
load_from = None
