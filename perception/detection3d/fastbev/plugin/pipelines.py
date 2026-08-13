"""Four-camera pipeline primitives for Fast-BEV."""

import cv2
import numpy as np

from mmdet.datasets.builder import PIPELINES
from mmdet.datasets.pipelines import Compose
from mmdet3d.datasets.pipelines.transforms_3d import (
    RandomAugImageMultiViewImage,
)


@PIPELINES.register_module()
class MoraiMultiViewPipeline:
    """Load all four views in the fixed front/left/right/back order."""

    def __init__(self, transforms, n_images=4, **_unused):
        if n_images != 4:
            raise ValueError("MoraiMultiViewPipeline requires n_images=4")
        self.transforms = Compose(transforms)

    def __call__(self, results):
        if len(results["img_info"]) != 4:
            raise AssertionError("Expected four MORAI camera images")
        images, extrinsics = [], []
        last = None
        for index in np.arange(4).tolist():
            current = {
                "img_prefix": results["img_prefix"][index],
                "img_info": results["img_info"][index],
            }
            last = self.transforms(current)
            images.append(last["img"])
            extrinsics.append(results["lidar2img"]["extrinsic"][index])
        for key, value in last.items():
            if key not in ("img", "img_prefix", "img_info"):
                results[key] = value
        results["img"] = images
        results["lidar2img"]["extrinsic"] = extrinsics
        return results


@PIPELINES.register_module()
class MoraiMultiViewArrayPipeline:
    """Accept four already-decoded image arrays for online inference.

    The training/evaluation pipeline normally starts with
    :class:`MoraiMultiViewPipeline`, which reads image paths from disk.  A ROS
    node already owns decoded camera frames, so serialising them only to make
    ``LoadImageFromFile`` read them again would add avoidable latency and disk
    traffic.  This adapter creates the same image metadata as the file loader
    and leaves all resize/crop/normalisation steps to the existing test
    pipeline.
    """

    def __init__(self, n_images=4, **_unused):
        if n_images != 4:
            raise ValueError("MoraiMultiViewArrayPipeline requires n_images=4")

    def __call__(self, results):
        images = results.pop("camera_images")
        if len(images) != 4:
            raise AssertionError("Expected four MORAI camera arrays")

        prepared = []
        for image in images:
            image = np.asarray(image)
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(
                    "Each camera image must have shape HxWx3, got "
                    f"{image.shape}")
            # Keep OpenCV/MMCV's BGR channel order and a contiguous uint8
            # layout so online and disk-loaded inputs are byte-for-byte
            # equivalent before the existing augmentation/normalisation.
            prepared.append(np.ascontiguousarray(image, dtype=np.uint8))

        results["img"] = prepared
        results["filename"] = [None] * 4
        results["ori_filename"] = [None] * 4
        results["ori_shape"] = [image.shape for image in prepared]
        results["img_shape"] = [image.shape for image in prepared]
        results["pad_shape"] = [image.shape for image in prepared]
        results["scale_factor"] = 1.0
        results["img_norm_cfg"] = dict(
            mean=np.zeros(3, dtype=np.float32),
            std=np.ones(3, dtype=np.float32),
            to_rgb=False,
        )
        return results


@PIPELINES.register_module()
class MoraiAugImageMultiViewImage(RandomAugImageMultiViewImage):
    """Fast-BEV image geometry with a selectable pixel backend.

    Fast-BEV's original transform uses Pillow for resize/crop/flip/rotate.
    The OpenCV backend preserves the same sampled geometry and projection
    update while replacing only the pixel operations.  Keeping both paths
    makes the original training reproducible and lets deployment-matched
    experiments select OpenCV explicitly.
    """

    def __init__(self, backend="pil", **kwargs):
        super().__init__(**kwargs)
        self.backend = str(backend).lower()
        if self.backend not in ("pil", "opencv"):
            raise ValueError(
                "MoraiAugImageMultiViewImage backend must be 'pil' or "
                f"'opencv', got {backend!r}")

        # Upstream Fast-BEV exposes image flipping as a boolean and therefore
        # hard-codes a 50% probability whenever it is enabled. MORAI's camera
        # roles and right-hand-traffic distribution are asymmetric, so retain
        # the legacy boolean behaviour while allowing a smaller probability.
        flip_prob = self.data_config.get("flip_prob")
        if flip_prob is not None:
            flip_prob = float(flip_prob)
            if not 0.0 <= flip_prob <= 1.0:
                raise ValueError(
                    "data_config.flip_prob must be in [0, 1], got "
                    f"{flip_prob}")
            self.data_config["flip_prob"] = flip_prob

    def sample_augmentation(self, H, W):
        sampled = super().sample_augmentation(H, W)
        if not self.is_train or "flip_prob" not in self.data_config:
            return sampled

        resize, resize_dims, crop, _flip, rotate, pad = sampled
        enabled = bool(self.data_config.get("flip", False))
        flip = enabled and np.random.rand() < self.data_config["flip_prob"]
        return resize, resize_dims, crop, flip, rotate, pad

    @staticmethod
    def _crop_with_border(image, crop):
        """Match Pillow crop size and zero-fill out-of-image regions."""
        left, top, right, bottom = map(int, crop)
        width, height = right - left, bottom - top
        if width <= 0 or height <= 0:
            raise ValueError(f"Invalid image crop: {crop}")
        output = np.zeros((height, width, image.shape[2]), dtype=image.dtype)
        src_left, src_top = max(left, 0), max(top, 0)
        src_right, src_bottom = min(right, image.shape[1]), min(
            bottom, image.shape[0])
        if src_right > src_left and src_bottom > src_top:
            dst_left, dst_top = src_left - left, src_top - top
            output[
                dst_top:dst_top + src_bottom - src_top,
                dst_left:dst_left + src_right - src_left,
            ] = image[src_top:src_bottom, src_left:src_right]
        return output

    def img_transform_core(self, img, resize_dims, crop, flip, rotate, pad):
        if self.backend == "pil":
            return super().img_transform_core(
                img, resize_dims, crop, flip, rotate, pad)

        image = cv2.resize(
            np.asarray(img), tuple(map(int, resize_dims)),
            interpolation=cv2.INTER_LINEAR)
        image = self._crop_with_border(image, crop)
        if flip:
            image = cv2.flip(image, 1)
        if rotate:
            height, width = image.shape[:2]
            matrix = cv2.getRotationMatrix2D(
                (width / 2.0, height / 2.0), float(rotate), 1.0)
            image = cv2.warpAffine(
                image,
                matrix,
                (width, height),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )

        top, right, bottom, left = map(int, pad[0])
        if any((top, right, bottom, left)):
            image = cv2.copyMakeBorder(
                image,
                top,
                bottom,
                left,
                right,
                cv2.BORDER_CONSTANT,
                value=pad[1],
            )
        return image
