"""Base4 V1: single-frame BEVFusion with MORAI planar velocity targets."""

_base_ = ['./bevfusion_base3_b0_4cam_lidar.py']

repo_root = '/workspace/AjouNice2026'
# MMEngine config inheritance cannot mix its lazy-import parser with ordinary
# Python imports.  Use the native environment-variable substitution syntax so
# this config remains a conventional (non-lazy) child of the Base3 config.
info_root = "{{$MORAI_BASE4_INFO_ROOT:/workspace/AjouNice2026/artifacts/datasets/morai_base4_temporal_highway_p24v1_raw}}"
load_from = "{{$BEVFUSION_BASE3_E25_CHECKPOINT:/workspace/AjouNice2026/artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth}}"

model = dict(
    bbox_head=dict(
        common_heads=dict(
            _delete_=True,
            center=[2, 2], height=[1, 2], dim=[3, 2], rot=[2, 2],
            vel=[2, 2]),
        train_cfg=dict(
            code_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                          0.2, 0.2]),
        bbox_coder=dict(code_size=10)))

train_dataloader = dict(
    dataset=dict(ann_file=info_root + '/morai_base3_infos_train.pkl'))
val_dataloader = dict(
    dataset=dict(ann_file=info_root + '/morai_base3_infos_val.pkl'))
test_dataloader = dict(
    dataset=dict(ann_file=info_root + '/morai_base3_infos_val.pkl'))

train_cfg = dict(by_epoch=True, max_epochs=4, val_interval=1)
param_scheduler = [
    dict(type='LinearLR', start_factor=0.1, by_epoch=False, begin=0, end=100),
    dict(
        type='CosineAnnealingLR', begin=0, end=4, T_max=4, by_epoch=True,
        eta_min=1.0e-6, convert_to_iter_based=True),
]
default_hooks = dict(
    checkpoint=dict(
        type='CheckpointHook', interval=1, max_keep_ckpts=4,
        save_best='val/center/mAP', rule='greater'))
