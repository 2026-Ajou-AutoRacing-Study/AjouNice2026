"""Inference/validation config for merged Base5; optimizations default OFF."""
_base_ = './bevfusion_base5_fine_512_depth25_pedestrian_merged.py'
model = dict(compile_camera=False)
