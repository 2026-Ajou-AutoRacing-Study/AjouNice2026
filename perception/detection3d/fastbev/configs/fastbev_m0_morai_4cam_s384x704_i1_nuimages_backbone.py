_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

# The Cascade Mask R-CNN checkpoint contains a 256-channel FPN, whereas
# Fast-BEV M0 uses a 64-channel FPN. Keep only the fully compatible R18
# backbone; the MORAI Fast-BEV FPN and later modules start from C0 defaults.
load_from = (
    'artifacts/detection3d/fastbev/pretrained/prepared/'
    'i1_nuimages_backbone.pth'
)

wandb_config = dict(initialization='I1_nuImages_backbone')
