"""Current Base5 input, 2026-10-11: preencoded native Q90 RGB in every split.

No JPEG re-encoding in train/val/analysis. Geometry augmentation stays enabled.
PNG is a local conversion source only; auxiliary lossless labels stay unchanged.
Not a production E25 change. Use the guarded direct-training runner.
"""
from copy import deepcopy

_base_ = './bevfusion_base5_dense_512_separated_candidate.py'

required_camera_input_policy = 'native_jpeg90'
train_pipeline = [t for t in deepcopy(_base_.train_pipeline)
                  if t['type'] != 'MoraiOnlineImageDomainAug3D']
train_dataloader = dict(dataset=dict(pipeline=train_pipeline))
