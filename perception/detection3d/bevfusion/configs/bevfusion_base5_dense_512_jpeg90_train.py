"""Historical PNG-to-JPEG conversion policy, 2026-10-10; not production E25.

Single-frame 4-class/7D, +/-100x30m, 512x1056, separated pooling.
Only native-resolution JPEG Q90 is enabled (100% of all views). Blur, exposure,
noise remain OFF. Val/test are clean; actual JPEG bags are separate dev-val.
This is pixel-domain adaptation, not proven AP improvement or training init.
Current all-JPEG input uses bevfusion_base5_dense_512_native_jpeg90.py instead.
"""
from copy import deepcopy

_base_ = './bevfusion_base5_dense_512_separated_candidate.py'

train_pipeline = deepcopy(_base_.train_pipeline)
# Remove inherited disabled post-crop augmentation; encode exactly once and
# BEFORE resizing, preserving native side-camera size and all calibration.
train_pipeline = [t for t in train_pipeline
                  if t['type'] != 'MoraiOnlineImageDomainAug3D']
image_aug_index = next(i for i, t in enumerate(train_pipeline)
                       if t['type'] == 'MoraiImageAug3D')
train_pipeline.insert(image_aug_index, dict(
    type='MoraiOnlineImageDomainAug3D', jpeg_prob=1.0,
    jpeg_quality=[90, 90], blur_prob=0.0, exposure_prob=0.0,
    noise_prob=0.0, color_order='rgb', native_resolution=True))
train_dataloader = dict(dataset=dict(pipeline=train_pipeline))
