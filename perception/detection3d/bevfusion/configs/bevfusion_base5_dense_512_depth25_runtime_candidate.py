"""Explicit depth-25cm preparation candidate, NOT a main-training approval.

Keep the validated 50cm base/default intact. The training-runtime improvements
are opt-in; camera compile remains OFF pending separate target-GPU checks.
"""
_base_ = './bevfusion_base5_dense_512_training_runtime_candidate.py'
model = dict(view_transform=dict(dbound=[1., 100., .25],
                                 pool_cache_limit_bytes=512 * 2**20))
