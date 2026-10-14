# Copyright 2018-2019 Open-MMLab. All rights reserved.
# Projection equations adapted from projects/BEVFusion/bevfusion/depth_lss.py,
# originally modified from https://github.com/mit-han-lab/bevfusion.
# Apache-2.0 terms: third_party/mmdetection3d/LICENSE.
"""Opt-in camera-axis depth hint rasterization, never a GT or ROI rewrite.

Positive finite camera Z is axial distance along the optical axis, not the
Euclidean LiDAR range. Legacy CUDA duplicate-index writes are not reproduced
here; the runtime subclass delegates to its real parent for the default None.
"""

import torch


def reduce_depth_pixels(coords, depth, image_size, policy='nearest_positive', backend='sorted'):
    """Rasterize [row,column] points into one positive finite Z per pixel.

    Sorted mode emits unique destinations before writing, avoiding duplicate
    index_put_ behavior. Equal-depth tie winner is stable original order.
    scatter_reduce mode is a research comparator: PyTorch documents possible
    CUDA nondeterminism, so its output still needs device qualification.
    With distinct minima, point gradients reach only the selected winner;
    scatter_reduce splits exact tied extrema, sorted mode selects first tie.
    """
    if policy not in ('nearest_positive', 'farthest_positive', 'unique_last_positive'):
        raise ValueError('Unknown explicit depth raster policy')
    if backend not in ('sorted', 'scatter_reduce'):
        raise ValueError('Unknown depth raster backend')
    if len(image_size) != 2 or any(type(v) is not int or v <= 0 for v in image_size):
        raise ValueError('Positive integer image H,W required')
    if coords.ndim != 2 or coords.shape[-1] != 2 or depth.ndim != 1 or len(coords) != len(depth):
        raise ValueError('Depth samples require Nx2 coordinates and N depths')
    if not torch.is_floating_point(depth) or coords.device != depth.device:
        raise ValueError('Depth must be floating and coordinates on the same device')
    height, width = image_size
    valid = torch.isfinite(depth) & (depth > 0) & torch.isfinite(coords).all(dim=-1)
    valid &= (coords[:, 0] >= 0) & (coords[:, 0] < height)
    valid &= (coords[:, 1] >= 0) & (coords[:, 1] < width)
    coords, values = coords[valid].long(), depth[valid]
    indices = coords[:, 0] * width + coords[:, 1]
    empty = depth.new_zeros(height * width)
    if not len(indices):
        # Preserve a zero differentiable dependency for all-invalid input.
        return (empty + depth[valid].sum()).reshape(height, width)
    if backend == 'scatter_reduce' and policy != 'unique_last_positive':
        nearest = policy == 'nearest_positive'
        initial = depth.new_full((height * width,), float('inf') if nearest else -float('inf'))
        reduced = initial.scatter_reduce(0, indices, values, 'amin' if nearest else 'amax', include_self=True)
        return torch.where(torch.isfinite(reduced), reduced, empty).reshape(height, width)
    if policy == 'unique_last_positive':
        # Stable grouped original index order; choose the final valid sample.
        order = torch.argsort(indices, stable=True)
        destinations = indices[order]
        winner = torch.ones_like(destinations, dtype=torch.bool)
        winner[:-1] = destinations[:-1] != destinations[1:]
    else:
        # Stable sort by Z, then pixel id: each group's first is its extremum.
        depth_order = torch.argsort(values, descending=policy == 'farthest_positive', stable=True)
        grouped_order = torch.argsort(indices[depth_order], stable=True)
        order = depth_order[grouped_order]
        destinations = indices[order]
        winner = torch.ones_like(destinations, dtype=torch.bool)
        winner[1:] = destinations[1:] != destinations[:-1]
    selected = order[winner]
    # Every destination is unique; no undefined repeated writes or averaging.
    return empty.index_put((indices[selected],), values[selected], accumulate=False).reshape(height, width)


def project_positive_camera_z(points, lidar2image, img_aug_matrix, lidar_aug_matrix, image_size):
    """Project exactly the existing pinhole/augmentation axes without mutation.

    Returns per-batch, per-camera (pixel row/column, pre-clamp camera Z).
    Filtering invalid/nonpositive Z is explicit and applies to the hint only,
    not LiDAR voxels, labels, perception ROI or calibration matrices.
    """
    projected = []
    for batch, point in enumerate(points):
        xyz = point[:, :3] - lidar_aug_matrix[batch, :3, 3]
        xyz = torch.inverse(lidar_aug_matrix[batch, :3, :3]).matmul(xyz.transpose(1, 0))
        camera = lidar2image[batch, :, :3, :3].matmul(xyz)
        camera = camera + lidar2image[batch, :, :3, 3].reshape(-1, 3, 1)
        depth = camera[:, 2].clone()
        camera = camera.clone()
        camera[:, 2] = camera[:, 2].clamp(1e-5, 1e5)
        camera[:, :2] = camera[:, :2] / camera[:, 2:3]
        camera = img_aug_matrix[batch, :, :3, :3].matmul(camera)
        camera = camera + img_aug_matrix[batch, :, :3, 3].reshape(-1, 3, 1)
        coords = camera[:, :2].transpose(1, 2)[..., [1, 0]]
        sample = []
        for xy, z in zip(coords, depth):
            valid = torch.isfinite(xy).all(dim=-1) & torch.isfinite(z) & (z > 0)
            valid &= (xy[:, 0] >= 0) & (xy[:, 0] < image_size[0])
            valid &= (xy[:, 1] >= 0) & (xy[:, 1] < image_size[1])
            sample.append((xy[valid].long(), z[valid]))
        projected.append(sample)
    return projected


def rasterize_positive_camera_z(points, lidar2image, img_aug_matrix, lidar_aug_matrix,
                               image_size, policy='nearest_positive', backend='sorted'):
    values = project_positive_camera_z(points, lidar2image, img_aug_matrix, lidar_aug_matrix, image_size)
    return torch.stack([torch.stack([reduce_depth_pixels(xy, z, image_size, policy, backend)
                                    for xy, z in sample]) for sample in values]).unsqueeze(2)
