"""Experimental, reversible stream scheduling; no default/runtime promotion.

`legacy` is a diagnostic reproduction. `voxel_main` completes voxelization
on the caller stream before scheduling sparse convolution on a side stream.
Neither path changes model weights, geometry, kernels or decoding thresholds.
`camera_side` keeps sparse convolution on the default stream because cumm
0.4.11 Tensor.cpu(ctx) ignores ctx in its device-to-host copy. Only an explicitly
qualified serial inference service may use this wrapper.
"""
import threading

import torch


class ParallelBranches:
    def __init__(self, model, mode='camera_side'):
        if model.training or mode not in ('legacy', 'voxel_main', 'camera_side'):
            raise ValueError('Parallel branches require eval and a known mode')
        if model.fusion_layer is None:
            raise ValueError('Parallel branches require camera/LiDAR fusion')
        self.model, self.mode = model, mode
        self.extract, self.points = model.extract_feat, model.extract_pts_feat
        self.images = model.extract_img_feat
        self.fusion = model.fusion_layer.forward
        self.pending = None
        self.streams = {}
        self.lock = threading.Lock()
        self.enabled = True
        model.extract_feat, model.extract_pts_feat = self.extract_parallel, self.join_points
        if mode == 'camera_side':
            model.extract_img_feat, model.fusion_layer.forward = self.side_images, self.join_fusion

    def extract_parallel(self, inputs, metas, **kwargs):
        if self.model.training or torch.is_grad_enabled():
            raise RuntimeError('Parallel branches are inference-only')
        if inputs.get('imgs') is None:
            return self.extract(inputs, metas, **kwargs)
        device = inputs['imgs'].device
        if device.type != 'cuda':
            raise ValueError('Parallel branches require CUDA')
        with self.lock:
            main = torch.cuda.current_stream(device)
            if self.mode == 'camera_side' and main != torch.cuda.default_stream(device):
                raise RuntimeError('Sparse encoder must remain on the default CUDA stream')
            if device not in self.streams:
                self.streams[device] = torch.cuda.Stream(device=device)
            side = self.streams[device]
            if self.mode in ('voxel_main', 'camera_side'):
                # The installed hard voxel kernel has device-wide barriers.
                # Put these before overlap, and avoid a GPU scalar batch size.
                with torch.autocast('cuda', enabled=False):
                    feats, coords, sizes = self.model.voxelize([p.float() for p in inputs['points']])
                batch_size = len(inputs['points'])
                if self.mode == 'camera_side':
                    self.pending = (inputs, feats, coords, batch_size, side)
                    try:
                        return self.extract(inputs, metas, **kwargs)
                    finally:
                        main.wait_stream(side)
                        self.pending = None
                side.wait_stream(main)
                feats.record_stream(side)
                coords.record_stream(side)
                with torch.cuda.stream(side):
                    features = self.model.pts_middle_encoder(feats, coords, batch_size)
            else:
                side.wait_stream(main)
                for points in inputs['points']:
                    points.record_stream(side)
                with torch.cuda.stream(side):
                    features = self.points(inputs)
            features.record_stream(main)
            self.pending = (inputs, features, side)
            try:
                return self.extract(inputs, metas, **kwargs)
            finally:
                main.wait_stream(side)
                self.pending = None

    def join_points(self, inputs):
        if self.pending is None:
            return self.points(inputs)
        if self.mode == 'camera_side':
            expected, feats, coords, batch_size, side = self.pending
            if inputs is not expected:
                raise RuntimeError('Parallel branch input identity differs')
            return self.model.pts_middle_encoder(feats, coords, batch_size)
        expected, features, side = self.pending
        if inputs is not expected:
            raise RuntimeError('Parallel branch input identity differs')
        torch.cuda.current_stream(features.device).wait_stream(side)
        return features

    def side_images(self, *args, **kwargs):
        if self.pending is None:
            return self.images(*args, **kwargs)
        side = self.pending[-1]
        main = torch.cuda.current_stream(side.device)
        side.wait_stream(main)
        def record(value):
            if torch.is_tensor(value) and value.is_cuda:
                value.record_stream(side)
            elif isinstance(value, (tuple, list)):
                for item in value:
                    record(item)
            elif isinstance(value, dict):
                for item in value.values():
                    record(item)
        record(args)
        record(kwargs)
        with torch.cuda.stream(side):
            result = self.images(*args, **kwargs)
        result.record_stream(main)
        return result

    def join_fusion(self, *args, **kwargs):
        if self.pending is not None:
            torch.cuda.current_stream(self.pending[-1].device).wait_stream(self.pending[-1])
        return self.fusion(*args, **kwargs)

    def disable(self):
        if self.enabled:
            self.model.extract_feat, self.model.extract_pts_feat = self.extract, self.points
            if self.mode == 'camera_side':
                self.model.extract_img_feat, self.model.fusion_layer.forward = self.images, self.fusion
            self.enabled = False
