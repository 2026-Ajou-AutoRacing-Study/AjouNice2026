# Copyright 2018-2019 Open-MMLab. All rights reserved.
# Upstream attribution: modify from https://github.com/mit-han-lab/bevfusion
# Adapted from projects/BEVFusion/bevfusion/transfusion_head.py in the vendored
# mmdetection3d distribution, licensed under Apache-2.0 (third_party/mmdetection3d/LICENSE).
"""Explicit per-class TransFusion query peak masks for offline research only.

The runtime subclass delegates to the original head when no explicit kernels
are requested. This helper changes query selection, not rotated-box NMS, GT,
scores, grid geometry, state-dict keys or prediction-head parameters.
"""

import torch
from torch.nn import functional as F


def validate_peak_kernels(kernels, num_classes):
    if not isinstance(kernels, (tuple, list)) or len(kernels) != num_classes:
        raise ValueError('One explicit peak kernel per ordered model class is required')
    if any(type(k) is not int or k not in (1, 3, 5) for k in kernels):
        raise ValueError('Research peak kernels must be integer 1, 3 or 5')
    return tuple(kernels)


def explicit_class_peak_mask(heatmap, kernels):
    """Return the exact equality-to-local-max mask, with legacy zero borders.

    Kernel 1 keeps every cell. Tied maxima are ALL retained, not broken by
    spatial order. A larger kernel can erase two genuinely close objects;
    this opt-in candidate must not be treated as a free duplicate-removal fix.
    """
    if heatmap.ndim != 4:
        raise ValueError('Heatmap must have shape [B,classes,Y,X]')
    kernels = validate_peak_kernels(kernels, heatmap.shape[1])
    if any(k > min(heatmap.shape[-2:]) for k in kernels):
        raise ValueError('Peak kernel is larger than the BEV heatmap')
    local_max = torch.zeros_like(heatmap)
    for label, kernel in enumerate(kernels):
        values = heatmap[:, label:label + 1]
        if kernel == 1:
            local_max[:, label:label + 1] = F.max_pool2d(values, 1, stride=1, padding=0)
        else:
            padding = kernel // 2
            local_max[:, label:label + 1, padding:-padding, padding:-padding] = F.max_pool2d(
                values, kernel, stride=1, padding=0)
    return heatmap * (heatmap == local_max)


def forward_single_with_explicit_peaks(head, inputs, metas, kernels):
    """Upstream 2026-10-14 forward_single, with only peak-mask selection changed.

    No global monkey patch, mutable dataset name or new parameter is involved.
    The unchanged Waymo policy [3,1,1] is checked tensor-for-tensor against the
    actual vendored forward_single in the CPU regression tests.
    """
    batch_size = inputs.shape[0]
    fusion_feat = head.shared_conv(inputs)
    fusion_feat_flatten = fusion_feat.view(batch_size, fusion_feat.shape[1], -1)
    bev_pos = head.bev_pos.repeat(batch_size, 1, 1).to(fusion_feat.device)

    with torch.autocast('cuda', enabled=False):
        dense_heatmap = head.heatmap_head(fusion_feat.float())
    heatmap = dense_heatmap.detach().sigmoid()
    heatmap = explicit_class_peak_mask(heatmap, kernels)
    heatmap = heatmap.view(batch_size, heatmap.shape[1], -1)

    top_proposals = heatmap.view(batch_size, -1).argsort(
        dim=-1, descending=True)[..., :head.num_proposals]
    top_proposals_class = top_proposals // heatmap.shape[-1]
    top_proposals_index = top_proposals % heatmap.shape[-1]
    query_feat = fusion_feat_flatten.gather(
        index=top_proposals_index[:, None, :].expand(-1, fusion_feat_flatten.shape[1], -1), dim=-1)
    head.query_labels = top_proposals_class

    one_hot = F.one_hot(top_proposals_class, num_classes=head.num_classes).permute(0, 2, 1)
    query_cat_encoding = head.class_encoding(one_hot.float())
    query_feat += query_cat_encoding

    query_pos = bev_pos.gather(
        index=top_proposals_index[:, None, :].permute(0, 2, 1).expand(-1, -1, bev_pos.shape[-1]), dim=1)
    ret_dicts = []
    for i in range(head.num_decoder_layers):
        query_feat = head.decoder[i](
            query_feat, key=fusion_feat_flatten, query_pos=query_pos, key_pos=bev_pos)
        res_layer = head.prediction_heads[i](query_feat)
        res_layer['center'] = res_layer['center'] + query_pos.permute(0, 2, 1)
        ret_dicts.append(res_layer)
        query_pos = res_layer['center'].detach().clone().permute(0, 2, 1)

    ret_dicts[0]['query_heatmap_score'] = heatmap.gather(
        index=top_proposals_index[:, None, :].expand(-1, head.num_classes, -1), dim=-1)
    ret_dicts[0]['dense_heatmap'] = dense_heatmap
    if head.auxiliary is False:
        return [ret_dicts[-1]]

    new_res = {}
    for key in ret_dicts[0]:
        if key not in ('dense_heatmap', 'dense_heatmap_old', 'query_heatmap_score'):
            new_res[key] = torch.cat([r[key] for r in ret_dicts], dim=-1)
        else:
            new_res[key] = ret_dicts[0][key]
    return [new_res]
