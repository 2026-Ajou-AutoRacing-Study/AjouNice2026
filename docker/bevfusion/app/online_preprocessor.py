"""In-memory OpenCV preprocessing matching the Base3 BEVFusion test pipeline."""

from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import torch
from mmdet3d.structures import Det3DDataSample, LiDARInstance3DBoxes


def preprocessing_options_from_config(config):
    """Use the deterministic test augmentation contract, including new sizes.

    Refuse unsupported crops/rotations instead of silently serving pixels with
    a projection matrix different from training. This does not extend model ROI.
    """
    pipeline = config['test_dataloader']['dataset']['pipeline']
    transforms = [item for item in pipeline if item['type'] == 'MoraiImageAug3D']
    if len(transforms) != 1:
        raise ValueError('runtime requires exactly one MoraiImageAug3D')
    aug = transforms[0]
    for key in ('bot_pct_lim', 'rot_lim'):
        if tuple(aug.get(key, (0.0, 0.0))) != (0.0, 0.0):
            raise ValueError('runtime requires zero ' + key)
    if aug.get('rand_flip', False) or aug.get('is_train', False):
        raise ValueError('runtime requires deterministic test augmentation')
    resize_lim = tuple(aug['resize_lim'])
    if len(resize_lim) != 2 or resize_lim[0] != resize_lim[1]:
        raise ValueError('runtime requires a fixed resize factor')
    final_dim = tuple(aug['final_dim'])
    if any(d <= 0 or d % 32 for d in final_dim):
        raise ValueError('Swin/depth fusion requires image dimensions divisible by 32')
    view = config['model']['view_transform']
    if tuple(view['image_size']) != final_dim:
        raise ValueError('augmentation and view-transform image sizes differ')
    if tuple(view['feature_size']) != tuple(d // 8 for d in final_dim) or any(d % 8 for d in final_dim):
        raise ValueError('runtime requires stride-8 view-transform features')
    base5_valid_returns = any(item['type'] == 'MoraiFilterInvalidPoints'
                             for item in pipeline)
    return dict(final_dim=final_dim, resize=float(resize_lim[0]),
                reference_width=aug.get('reference_width', 1280),
                point_cloud_range=config['point_cloud_range'],
                remove_zero_returns=base5_valid_returns,
                strict_point_range=base5_valid_returns)


class Base3OnlinePreprocessor:
    """Build MMDetection3D inputs without temporary image or point files."""

    def __init__(
        self,
        calibration,
        camera_workers=4,
        point_dim=4,
        final_dim=(256, 704),
        resize=0.55,
        reference_width=1280,
        point_cloud_range=(-30.0, -30.0, -3.0, 30.0, 30.0, 2.0),
        remove_zero_returns=False,
        strict_point_range=False,
        rgb_after_crop=False,
    ):
        self.calibration = calibration
        self.final_dim = tuple(int(value) for value in final_dim)
        self.resize = float(resize)
        self.reference_width = float(reference_width)
        self.point_cloud_range = np.asarray(point_cloud_range, dtype=np.float32)
        self.camera_workers = max(1, int(camera_workers))
        self.point_dim = int(point_dim)
        # Explicitly config-driven for Base5; production E25 stays unchanged.
        self.remove_zero_returns = bool(remove_zero_returns)
        self.strict_point_range = bool(strict_point_range)
        self.rgb_after_crop = bool(rgb_after_crop)
        if self.point_dim not in (4, 5):
            raise ValueError("BEVFusion runtime point_dim must be 4 or 5")
        self.executor = (
            ThreadPoolExecutor(
                max_workers=self.camera_workers,
                thread_name_prefix="bevfusion_camera",
            )
            if self.camera_workers > 1
            else None
        )

        self.camera_intrinsics = []
        self.lidar2cam = []
        self.cam2lidar = []
        self.lidar2img = []
        self.img_aug_matrix = []
        for camera in calibration["cameras"]:
            cam2img = np.eye(4, dtype=np.float32)
            cam2img[:3, :3] = np.asarray(camera["cam2img"], dtype=np.float32)
            lidar2cam = np.asarray(camera["lidar2cam"], dtype=np.float32)
            self.camera_intrinsics.append(cam2img)
            self.lidar2cam.append(lidar2cam)
            self.cam2lidar.append(
                np.asarray(camera["cam2lidar"], dtype=np.float32)
            )
            self.lidar2img.append(cam2img @ lidar2cam)
            self.img_aug_matrix.append(self._augmentation_matrix(camera))

        self.camera_intrinsics = np.stack(self.camera_intrinsics)
        self.lidar2cam = np.stack(self.lidar2cam)
        self.cam2lidar = np.stack(self.cam2lidar)
        self.lidar2img = np.stack(self.lidar2img)
        self.img_aug_matrix = np.stack(self.img_aug_matrix)

    def prepare(
        self,
        points,
        image_blobs,
        sample_idx=0,
        images_preprocessed=False,
        timestamp_sec=0.0,
        lidar_pose_world=None,
        scene_id="runtime",
    ):
        """Return a pseudo-collated batch accepted directly by model.test_step."""
        cameras = self.calibration["cameras"]
        if images_preprocessed:
            images = [
                self._load_preprocessed(camera, image_blobs)
                for camera in cameras
            ]
        else:
            arguments = [(camera, image_blobs) for camera in cameras]
            if self.executor is None:
                images = [self._decode_resize(argument) for argument in arguments]
            else:
                images = list(self.executor.map(self._decode_resize, arguments))

        image_tensor = torch.from_numpy(
            np.ascontiguousarray(np.stack(images).transpose(0, 3, 1, 2))
        )
        points = self._filter_points(points)
        point_tensor = torch.from_numpy(points)

        data_sample = Det3DDataSample()
        lidar2global = self._lidar2global(lidar_pose_world)
        data_sample.set_metainfo({
            "sample_idx": str(sample_idx),
            "scene_id": str(scene_id),
            "timestamp": float(timestamp_sec),
            "lidar2global": lidar2global,
            "num_pts_feats": self.point_dim,
            "num_views": len(cameras),
            "box_type_3d": LiDARInstance3DBoxes,
            "cam2img": self.camera_intrinsics,
            "ori_cam2img": self.camera_intrinsics,
            "lidar2cam": self.lidar2cam,
            "cam2lidar": self.cam2lidar,
            "lidar2img": self.lidar2img,
            "img_aug_matrix": self.img_aug_matrix,
            "lidar_aug_matrix": np.eye(4, dtype=np.float32),
        })
        return {
            "inputs": {"points": [point_tensor], "img": [image_tensor]},
            "data_samples": [data_sample],
        }

    @staticmethod
    def _lidar2global(lidar_pose_world):
        matrix = np.eye(4, dtype=np.float32)
        if lidar_pose_world is None:
            return matrix
        values = np.asarray(lidar_pose_world, dtype=np.float64).reshape(-1)
        if values.size != 4 or not np.isfinite(values).all():
            raise ValueError(
                "lidar_pose_world must contain finite x, y, z, yaw")
        x, y, z, yaw = values.tolist()
        cosine, sine = np.cos(yaw), np.sin(yaw)
        matrix[0, 0] = cosine
        matrix[0, 1] = -sine
        matrix[1, 0] = sine
        matrix[1, 1] = cosine
        matrix[:3, 3] = (x, y, z)
        return matrix

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(wait=True)

    def _decode_resize(self, argument):
        camera, image_blobs = argument
        frame_id = camera["frame_id"]
        blob_name = "image/{}".format(frame_id)
        if blob_name not in image_blobs:
            raise ValueError("request is missing {}".format(blob_name))
        image = cv2.imdecode(
            np.frombuffer(image_blobs[blob_name], dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise ValueError("could not decode image for {}".format(frame_id))
        expected_shape = (int(camera["height"]), int(camera["width"]))
        if image.shape[:2] != expected_shape:
            raise ValueError(
                "{} image shape {} differs from Sensor.json {}".format(
                    frame_id, image.shape[:2], expected_shape
                )
            )
        if not self.rgb_after_crop:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resize_dims, crop = self._resize_crop(camera)
        image = cv2.resize(image, resize_dims, interpolation=cv2.INTER_CUBIC)
        left, top, right, bottom = crop
        image = image[top:bottom, left:right]
        if self.rgb_after_crop:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if image.shape[:2] != self.final_dim:
            raise ValueError(
                "{} preprocessed shape {} differs from {}".format(
                    frame_id, image.shape[:2], self.final_dim
                )
            )
        # Keep RGB pixels uint8 across CPU and PCIe. MMDetection3D casts to
        # float and applies the configured mean/std on GPU.
        return np.ascontiguousarray(image)

    def _load_preprocessed(self, camera, image_blobs):
        blob_name = "image/{}".format(camera["frame_id"])
        if blob_name not in image_blobs:
            raise ValueError("request is missing {}".format(blob_name))
        final_height, final_width = self.final_dim
        expected = final_height * final_width * 3
        values = np.frombuffer(image_blobs[blob_name], dtype=np.uint8)
        if values.size != expected:
            raise ValueError(
                "{} RGB8 payload has {} bytes, expected {}".format(
                    camera["frame_id"], values.size, expected
                )
            )
        return values.reshape(final_height, final_width, 3)

    def _resize_crop(self, camera):
        width = int(camera["width"])
        height = int(camera["height"])
        final_height, final_width = self.final_dim
        scale = self.resize * self.reference_width / width
        new_width, new_height = int(width * scale), int(height * scale)
        crop_top = new_height - final_height
        crop_left = max(0, new_width - final_width) // 2
        crop = (
            crop_left,
            crop_top,
            crop_left + final_width,
            crop_top + final_height,
        )
        if crop_top < 0 or crop[2] > new_width:
            raise ValueError(
                "camera {} cannot produce final size {} after resize {}".format(
                    camera["frame_id"], self.final_dim, (new_height, new_width)
                )
            )
        return (new_width, new_height), crop

    def _augmentation_matrix(self, camera):
        width = int(camera["width"])
        scale = self.resize * self.reference_width / width
        _resize_dims, crop = self._resize_crop(camera)
        matrix = np.eye(4, dtype=np.float32)
        matrix[0, 0] = scale
        matrix[1, 1] = scale
        matrix[0, 3] = -float(crop[0])
        matrix[1, 3] = -float(crop[1])
        return matrix

    def _filter_points(self, points):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != self.point_dim:
            raise ValueError(
                "BEVFusion expects Nx{} points, got {}".format(
                    self.point_dim, points.shape)
            )
        lower, upper = self.point_cloud_range[:3], self.point_cloud_range[3:]
        keep = np.isfinite(points).all(axis=1)
        if self.remove_zero_returns:
            keep &= (points[:, :3] != 0).any(axis=1)
        if self.strict_point_range:
            # Match MMDetection3D PointsRangeFilter's open interval exactly.
            keep &= (points[:, :3] > lower).all(axis=1)
            keep &= (points[:, :3] < upper).all(axis=1)
        else:
            keep &= (points[:, :3] >= lower).all(axis=1)
            keep &= (points[:, :3] <= upper).all(axis=1)
        filtered = np.ascontiguousarray(points[keep])
        if not len(filtered):
            raise ValueError("no finite LiDAR points remain inside the Base3 range")
        return filtered
