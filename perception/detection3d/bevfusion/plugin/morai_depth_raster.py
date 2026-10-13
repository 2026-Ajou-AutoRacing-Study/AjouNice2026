"""Research-only depth hint policy. Default None delegates to actual legacy."""

from mmdet3d.registry import MODELS
from .morai_separated_depth_lss import MoraiSeparatedRectangularDepthLSSTransform
from perception.detection3d.bevfusion.depth_raster import rasterize_positive_camera_z


@MODELS.register_module()
class MoraiDepthRasterSeparatedDepthLSSTransform(MoraiSeparatedRectangularDepthLSSTransform):
    def __init__(self, *args, depth_raster_policy=None, depth_raster_backend='sorted', **kwargs):
        super().__init__(*args, **kwargs)
        if depth_raster_policy not in (None, 'nearest_positive', 'farthest_positive', 'unique_last_positive'):
            raise ValueError('Explicit experimental depth hint policy required')
        if depth_raster_backend not in ('sorted', 'scatter_reduce'):
            raise ValueError('Unknown experimental depth hint backend')
        self.depth_raster_policy = depth_raster_policy
        self.depth_raster_backend = depth_raster_backend

    def forward(self, img, points, lidar2image, cam_intrinsic, camera2lidar,
                img_aug_matrix, lidar_aug_matrix, metas, **kwargs):
        if self.depth_raster_policy is None:
            return super().forward(img, points, lidar2image, cam_intrinsic, camera2lidar,
                                   img_aug_matrix, lidar_aug_matrix, metas, **kwargs)
        depth = rasterize_positive_camera_z(points, lidar2image, img_aug_matrix, lidar_aug_matrix,
            self.image_size, self.depth_raster_policy, self.depth_raster_backend)
        geom = self.get_geometry(camera2lidar[..., :3, :3], camera2lidar[..., :3, 3],
                                 cam_intrinsic[..., :3, :3], img_aug_matrix[..., :3, :3],
                                 img_aug_matrix[..., :3, 3], extra_rots=lidar_aug_matrix[..., :3, :3],
                                 extra_trans=lidar_aug_matrix[..., :3, 3])
        features = self.get_cam_feats(img, depth)
        return self.downsample(self.bev_pool(geom, features))
