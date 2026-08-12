"""21-frame ablation for a 0.5 m Fast-BEV detection grid.

The production baseline remains unchanged.  This config isolates the effect
of removing M2BevNeck's stride-2 downsampling before changing anchor shapes.
"""

_base_ = ["./fastbev_m0_morai_4cam_overfit20.py"]

model = dict(neck_3d=dict(stride=1))

runner = dict(type="EpochBasedRunner", max_epochs=30)
total_epochs = 30
checkpoint_config = dict(interval=10)
