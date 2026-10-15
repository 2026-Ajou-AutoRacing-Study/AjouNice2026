"""Fine-grid camera-backbone training compile opt-in; validate before main use."""
_base_ = './bevfusion_base5_fine_512_depth25_runtime_candidate.py'
model = dict(compile_camera=True)
