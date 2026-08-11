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
