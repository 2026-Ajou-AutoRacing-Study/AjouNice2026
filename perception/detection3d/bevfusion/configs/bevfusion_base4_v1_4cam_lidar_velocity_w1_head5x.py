"""Base4 V1 with unit velocity loss and a 5x LR for the new velocity head."""

_base_ = ['./bevfusion_base4_v1_4cam_lidar_velocity_w1.py']

# Base3 E25 has no velocity branch.  Preserve its trained spatial modules at
# the conservative fine-tuning LR while allowing only the random-initialized
# velocity branch to learn at the upstream BEVFusion LR.
optim_wrapper = dict(
    paramwise_cfg=dict(
        custom_keys={
            'bbox_head.prediction_heads.0.vel': dict(lr_mult=5.0),
        }))
