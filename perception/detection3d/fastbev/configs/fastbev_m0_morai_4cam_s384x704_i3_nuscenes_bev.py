_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

# Finished nuScenes Fast-BEV M0 checkpoint, filtered to image modules and every
# shape-compatible BEV-neck tensor. Temporal fuse and 10-class head mismatches
# are excluded by the preparation tool before this checkpoint is loaded.
load_from = (
    'artifacts/detection3d/fastbev/pretrained/prepared/'
    'i3_nuscenes_image_bev.pth'
)

wandb_config = dict(initialization='I3_nuScenes_image_BEV')
