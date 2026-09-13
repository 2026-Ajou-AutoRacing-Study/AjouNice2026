"""Camera-shape-aware image augmentation for MORAI's mixed resolutions."""

import numpy as np
import torch
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
