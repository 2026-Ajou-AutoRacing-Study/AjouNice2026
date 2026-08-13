"""Four-camera pipeline primitives for Fast-BEV."""

import numpy as np

from mmdet.datasets.builder import PIPELINES
from mmdet.datasets.pipelines import Compose


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
