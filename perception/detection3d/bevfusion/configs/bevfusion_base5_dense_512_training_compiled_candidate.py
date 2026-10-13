"""Additional opt-in camera compile candidate; qualify on the target GPU first.

Compiler startup/retracing costs are not steady-state step latency. Keep the
ordinary training_runtime_candidate available; compiled is NOT the default.
"""
_base_='./bevfusion_base5_dense_512_training_runtime_candidate.py'
model=dict(compile_camera=True)
