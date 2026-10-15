"""Matched Q1 control with the recurrent memory bank disabled.

This intentionally keeps the ordered batch-1 loader, identity global
augmentation, optimizer cadence, velocity loss, and head learning rate equal
to Q1-P0.  A zero-length bank makes every forward call memory-free, isolating
the effect of historical queries from the stronger velocity optimization.
"""

_base_ = ['./bevfusion_base4_q1_query_memory_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        memory_max_frames=0,
    ),
)

