"""Camera-shape-aware image augmentation for MORAI's mixed resolutions."""

import cv2
import numpy as np
import torch
from mmcv.transforms import BaseTransform
from mmdet3d.registry import TRANSFORMS
from projects.BEVFusion.bevfusion.transforms_3d import ImageAug3D


@TRANSFORMS.register_module()
class MoraiImageAug3D(ImageAug3D):
    """Undo loader padding and resize every view to a common target width.

    MORAI uses 1280x720 front/back cameras and 640x480 side cameras. The
    upstream multi-view loader pads all views to the largest source shape.
    Applying one scale afterward would leave half of each side view black.
    This transform crops each view back to its recorded source resolution and
    applies a resolution-relative scale. The returned per-camera homography
    keeps BEVFusion's projection geometry consistent.
    """

    def __init__(self, *args, reference_width=1280, **kwargs):
        super().__init__(*args, **kwargs)
        self.reference_width = reference_width

    def img_transform(self, img, rotation, translation, resize, resize_dims,
                      crop, flip, rotate):
        """Apply the same OpenCV cubic path used by the online ROS runtime."""
        # The upstream PIL implementation quantizes the loader's float32
        # array back to uint8 before resizing. Preserve that contract so the
        # online RGB8 path is pixel-identical while transferring 4x less data.
        img = img.astype(np.uint8, copy=False)
        img = cv2.resize(img, resize_dims, interpolation=cv2.INTER_CUBIC)
        left, top, right, bottom = crop
        img = img[top:bottom, left:right]
        if flip:
            img = cv2.flip(img, 1)
        if rotate:
            height, width = img.shape[:2]
            affine = cv2.getRotationMatrix2D(
                (width * 0.5, height * 0.5), rotate, 1.0)
            img = cv2.warpAffine(
                img,
                affine,
                (width, height),
                flags=cv2.INTER_CUBIC,
                borderMode=cv2.BORDER_CONSTANT,
            )

        rotation *= resize
        translation -= torch.tensor(crop[:2], dtype=translation.dtype)
        if flip:
            matrix = torch.tensor(
                [[-1, 0], [0, 1]], dtype=rotation.dtype)
            offset = torch.tensor(
                [crop[2] - crop[0], 0], dtype=translation.dtype)
            rotation = matrix.matmul(rotation)
            translation = matrix.matmul(translation) + offset
        theta = rotate / 180.0 * np.pi
        matrix = torch.tensor(
            [[np.cos(theta), np.sin(theta)],
             [-np.sin(theta), np.cos(theta)]],
            dtype=rotation.dtype,
        )
        center = torch.tensor(
            [crop[2] - crop[0], crop[3] - crop[1]],
            dtype=translation.dtype,
        ) / 2.0
        offset = matrix.matmul(-center) + center
        rotation = matrix.matmul(rotation)
        translation = matrix.matmul(translation) + offset
        return np.ascontiguousarray(img), rotation, translation

    def transform(self, data):
        images_meta = list(data['images'].values())
        if len(images_meta) != len(data['img']):
            raise ValueError('camera metadata/image count mismatch')

        new_imgs, transforms = [], []
        fH, fW = self.final_dim
        for padded_img, camera in zip(data['img'], images_meta):
            height, width = int(camera['height']), int(camera['width'])
            img = padded_img[:height, :width]
            if self.is_train:
                base_resize = np.random.uniform(*self.resize_lim)
                bot_pct = np.random.uniform(*self.bot_pct_lim)
                rotate = np.random.uniform(*self.rot_lim)
                flip = self.rand_flip and bool(np.random.choice([0, 1]))
            else:
                base_resize = float(np.mean(self.resize_lim))
                bot_pct = float(np.mean(self.bot_pct_lim))
                rotate = 0.0
                flip = False
            resize = base_resize * self.reference_width / width
            resize_dims = (int(width * resize), int(height * resize))
            newW, newH = resize_dims
            crop_h = int((1 - bot_pct) * newH) - fH
            crop_w = int(max(0, newW - fW) / 2)
            crop = (crop_w, crop_h, crop_w + fW, crop_h + fH)
            post_rot, post_tran = torch.eye(2), torch.zeros(2)
            new_img, rotation, translation = self.img_transform(
                img, post_rot, post_tran, resize, resize_dims, crop, flip,
                rotate)
            transform = torch.eye(4)
            transform[:2, :2] = rotation
            transform[:2, 3] = translation
            new_imgs.append(np.asarray(new_img, dtype=np.float32))
            transforms.append(transform.numpy())
        data['img'] = new_imgs
        data['img_aug_matrix'] = transforms
        return data


@TRANSFORMS.register_module()
class MoraiOnlineImageDomainAug3D(BaseTransform):
    """Mild camera-domain augmentation for MORAI online JPEG input.

    Base3 is captured as lossless PNG while the live ROS camera topics are
    JPEG-compressed.  This transform changes pixels only; calibration and box
    geometry remain untouched.  One exposure/noise draw is shared by all
    views in a sample, while JPEG quality and blur are sampled per camera.
    """

    def __init__(self,
                 jpeg_prob=0.8,
                 jpeg_quality=(72, 96),
                 blur_prob=0.12,
                 exposure_prob=0.5,
                 gain=(0.92, 1.08),
                 bias=(-6.0, 6.0),
                 noise_prob=0.2,
                 noise_std=(0.0, 1.5)):
        self.jpeg_prob = float(jpeg_prob)
        self.jpeg_quality = tuple(int(v) for v in jpeg_quality)
        self.blur_prob = float(blur_prob)
        self.exposure_prob = float(exposure_prob)
        self.gain = tuple(float(v) for v in gain)
        self.bias = tuple(float(v) for v in bias)
        self.noise_prob = float(noise_prob)
        self.noise_std = tuple(float(v) for v in noise_std)

    def transform(self, data):
        images = data['img']
        apply_exposure = np.random.random() < self.exposure_prob
        gain = np.random.uniform(*self.gain) if apply_exposure else 1.0
        bias = np.random.uniform(*self.bias) if apply_exposure else 0.0
        apply_noise = np.random.random() < self.noise_prob
        noise_std = np.random.uniform(*self.noise_std) if apply_noise else 0.0

        augmented = []
        for image in images:
            image = np.asarray(image, dtype=np.float32)
            if apply_exposure:
                image = image * gain + bias
            if noise_std > 0.0:
                image = image + np.random.normal(
                    0.0, noise_std, image.shape).astype(np.float32)
            image = np.clip(image, 0.0, 255.0).astype(np.uint8)

            if np.random.random() < self.blur_prob:
                image = cv2.GaussianBlur(image, (3, 3), 0.0)
            if np.random.random() < self.jpeg_prob:
                quality = int(np.random.randint(
                    self.jpeg_quality[0], self.jpeg_quality[1] + 1))
                ok, encoded = cv2.imencode(
                    '.jpg', image,
                    [cv2.IMWRITE_JPEG_QUALITY, quality])
                if ok:
                    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                    if decoded is not None:
                        image = decoded
            augmented.append(np.ascontiguousarray(image, dtype=np.float32))

        data['img'] = augmented
        return data
