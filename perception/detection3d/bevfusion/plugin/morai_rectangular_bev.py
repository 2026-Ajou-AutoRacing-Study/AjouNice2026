"""Opt-in rectangular [Y,X] feature geometry; legacy square E25 stays untouched."""
import torch
from mmdet3d.models import draw_heatmap_gaussian, gaussian_radius
from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion.depth_lss import DepthLSSTransform
from projects.BEVFusion.bevfusion.sparse_encoder import BEVFusionSparseEncoder
from .morai_transfusion_head import MoraiTransFusionHead
from perception.detection3d.bevfusion.view_geometry import restore_view_geometry


@MODELS.register_module()
class MoraiRectangularSparseEncoder(BEVFusionSparseEncoder):
    """Custom BEVFusion voxelizer emits b,x,y,z; this encoder emits B,C,Y,X.

    Do not apply this to the legacy square E25 model: even there, swapping axes
    changes its learned orientation. New rectangular models use it explicitly.
    """
    def forward(self, voxel_features, coors, batch_size):
        # sparse_shape is [Y,X,Z]. Preserve voxel features and all valid points.
        coors = coors[:, [0, 2, 1, 3]].contiguous()
        return super().forward(voxel_features, coors, batch_size)


@MODELS.register_module()
class MoraiRectangularDepthLSSTransform(DepthLSSTransform):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # The inherited constructor truncates fractional floating grid counts.
        # Fresh Base5 training must own the same rounded grid as inference,
        # without relying on an inference-only post-checkpoint restoration.
        restore_view_geometry(self)

    def bev_pool(self, geom_feats, x):
        # Upstream camera pool emits [B,C,X,Y]; sparse LiDAR emits [B,C,Y,X].
        # Transpose before learned downsampling, not the metric bounds/coordinates.
        return super().bev_pool(geom_feats,x).transpose(-1,-2).contiguous()


@MODELS.register_module()
class MoraiRectangularTransFusionHead(MoraiTransFusionHead):
    """Base5 [Y,X] features, (x,y) positions and matching dense targets.

    Keep the parent's query assignment/regression contract. Its legacy dense
    target swaps x/y, so replace only that returned target for this opt-in head.
    No parameters, prediction path or legacy E25 head are changed.
    """

    def create_2D_grid(self,x_size,y_size):
        # Flattened feature index = y*x_size+x. Positional coordinate = (x,y).
        y,x=torch.meshgrid(torch.arange(y_size,dtype=torch.float32),
                           torch.arange(x_size,dtype=torch.float32),indexing='ij')
        return torch.stack((x+.5,y+.5),dim=-1).reshape(1,-1,2)

    def build_dense_heatmap_targets(self, gt_instances_3d, device=None):
        """Return [1,classes,Y,X]; the Gaussian utility takes (x,y), not (y,x).

        Use the box coder's metric origin and pitch. Out-of-grid centers fail
        explicitly instead of being clamped to an unrelated boundary cell.
        The dataset's semantic range filter runs before this padded-grid target.
        """
        boxes = gt_instances_3d.bboxes_3d.tensor
        if device is not None:
            boxes = boxes.to(device)
        labels = gt_instances_3d.labels_3d.to(boxes.device)
        grid = self.train_cfg['grid_size'][:2]
        stride = self.train_cfg['out_size_factor']
        if stride <= 0 or any(size <= 0 or size % stride for size in grid):
            raise ValueError('Base5 target grid must be divisible by head stride')
        x_size, y_size = (int(size // stride) for size in grid)
        heatmap = boxes.new_zeros((self.num_classes, y_size, x_size))
        if len(boxes) != len(labels):
            raise ValueError('Base5 target boxes and labels differ in length')
        if not len(boxes):
            return heatmap[None]
        if not torch.isfinite(boxes).all() or (boxes[:, 3:6] <= 0).any():
            raise ValueError('Base5 target boxes must be finite with positive sizes')
        if labels.dtype != torch.long or ((labels < 0) | (labels >= self.num_classes)).any():
            raise ValueError('Base5 target labels outside the configured classes')
        pitch = boxes.new_tensor(self.train_cfg['voxel_size'][:2]) * stride
        if not torch.isfinite(pitch).all() or (pitch <= 0).any():
            raise ValueError('Base5 target pitch must be finite and positive')
        origin = boxes.new_tensor(self.train_cfg['point_cloud_range'][:2])
        centers = (boxes[:, :2] - origin) / pitch
        limit = boxes.new_tensor([x_size, y_size])
        if ((centers < 0) | (centers >= limit)).any():
            raise ValueError('Base5 target center outside padded BEV; check range filtering')
        centers_int = centers.to(torch.int32)
        sizes = boxes[:, 3:5] / pitch
        for index in range(len(boxes)):
            radius = gaussian_radius(
                (sizes[index, 1], sizes[index, 0]),
                min_overlap=self.train_cfg['gaussian_overlap'])
            radius = max(self.train_cfg['min_radius'], int(radius))
            draw_heatmap_gaussian(heatmap[labels[index]], centers_int[index], radius)
        return heatmap[None]

    def get_targets_single(self, gt_instances_3d, preds_dict, batch_idx):
        targets = super().get_targets_single(gt_instances_3d, preds_dict, batch_idx)
        heatmap = self.build_dense_heatmap_targets(
            gt_instances_3d, device=targets[-1].device)
        return (*targets[:-1], heatmap)
