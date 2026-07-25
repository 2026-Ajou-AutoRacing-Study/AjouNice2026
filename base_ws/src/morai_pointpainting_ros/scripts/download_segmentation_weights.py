#!/usr/bin/env python3
"""Pre-download torchvision LRASPP weights before the competition."""

from torchvision import __version__ as torchvision_version
from torchvision.models.segmentation import (
    lraspp_mobilenet_v3_large,
)


VOC_CATEGORIES = [
    "__background__",
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
]


def main():
    try:
        from torchvision.models.segmentation import (
            LRASPP_MobileNet_V3_Large_Weights,
        )
    except ImportError:
        LRASPP_MobileNet_V3_Large_Weights = None

    if LRASPP_MobileNet_V3_Large_Weights is not None:
        weights = LRASPP_MobileNet_V3_Large_Weights.DEFAULT
        lraspp_mobilenet_v3_large(weights=weights).eval()
        categories = list(weights.meta.get("categories", []))
        api = "weights enum"
    else:
        lraspp_mobilenet_v3_large(pretrained=True).eval()
        categories = VOC_CATEGORIES
        api = "legacy pretrained=True"

    print("Torchvision:", torchvision_version)
    print("API:", api)
    print("LRASPP MobileNetV3 Large weights are ready.")
    print("Categories:", categories)


if __name__ == "__main__":
    main()
