_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

# Finished nuScenes Fast-BEV M0 checkpoint, filtered to image backbone/FPN.
load_from = (
    'artifacts/detection3d/fastbev/pretrained/prepared/'
    'i2_nuscenes_image.pth'
)

wandb_config = dict(initialization='I2_nuScenes_image')
