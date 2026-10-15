#!/usr/bin/env python3
"""Run with the actual inference interpreter, without a ROS master or MORAI."""
import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="신호등 Python / GPU / 모델 / ROS import 검사")
    parser.add_argument("--device", default="0", help="GPU 번호 또는 cpu")
    parser.add_argument("--check-ros", action="store_true")
    parser.add_argument("--require-team-versions", action="store_true")
    parser.add_argument("--isolate-python", action="store_true", help="가상환경 검사: 사용자 site-packages 제외")
    parser.add_argument("--weights", type=Path, default=Path(__file__).resolve().parents[2] /
                        "base_ws/src/perception/traffic_light_detection/weights/best.pt")
    args = parser.parse_args()
    if args.device != "cpu" and not (args.device.isascii() and args.device.isdecimal()):
        parser.error("device는 cpu 또는 GPU 번호입니다")
    os.environ["YOLO_OFFLINE"] = "true"
    user_site = str(Path.home() / ".local/lib/python{}.{}".format(*sys.version_info[:2]) / "site-packages")
    if args.isolate_python:
        sys.path[:] = [entry for entry in sys.path if entry != user_site]
    elif Path(user_site).is_dir() and user_site not in sys.path:
        # Match the existing traffic_light_node system-Python/user-site policy.
        sys.path.insert(0, user_site)
    os.environ.setdefault("MPLCONFIGDIR", str(Path.home() / ".cache/ajounice/matplotlib"))
    os.environ.setdefault("YOLO_CONFIG_DIR", str(Path.home() / ".cache/ajounice/ultralytics"))
    for name in ("MPLCONFIGDIR", "YOLO_CONFIG_DIR"):
        Path(os.environ[name]).mkdir(parents=True, exist_ok=True)
    report = {"python": sys.executable, "python_version": sys.version.split()[0],
              "device": args.device, "weights": str(args.weights)}
    try:
        versions = {}
        for name in ("torch", "torchvision", "ultralytics", "numpy", "opencv-python"):
            try:
                versions[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                versions[name] = None  # e.g. system apt OpenCV has no pip metadata.
        report["versions"] = versions
        if args.require_team_versions:
            expected = {"torch": "2.7.1+cu128", "torchvision": "0.22.1+cu128",
                        "ultralytics": "8.4.143", "numpy": "1.26.4", "opencv-python": "4.8.1.78"}
            if versions != expected or sys.version_info[:2] != (3, 11):
                raise RuntimeError("공용 환경 버전 불일치: expected={} / actual={}".format(expected, versions))
        import torch
        import torchvision
        import cv2
        import numpy as np
        from ultralytics import YOLO
        report["opencv_import_version"] = cv2.__version__
        if args.device != "cpu":
            index = int(args.device)
            if not torch.cuda.is_available() or index >= torch.cuda.device_count():
                raise RuntimeError("CUDA GPU를 사용할 수 없습니다. nvidia-smi / 드라이버 / 선택한 Python을 확인하세요")
            report["gpu"] = {"name": torch.cuda.get_device_name(index),
                             "capability": list(torch.cuda.get_device_capability(index)),
                             "torch_arches": torch.cuda.get_arch_list()}
            # Real CUDA and torchvision kernel checks, not just is_available().
            device = "cuda:{}".format(index)
            boxes = torch.tensor([[0., 0., 1., 1.]], device=device)
            torchvision.ops.nms(boxes, torch.ones(1, device=device), 0.5)
            torch.cuda.synchronize(index)
        if args.check_ros:
            import rospy
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String, UInt64
            from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray
            report["ros_imports"] = "OK"
        if not args.weights.is_file():
            raise RuntimeError("신호등 가중치가 없습니다: {}".format(args.weights))
        start = time.monotonic()
        model = YOLO(str(args.weights))
        model.predict(source=np.zeros((480, 640, 3), dtype=np.uint8), device=args.device, verbose=False)
        report["smoke_elapsed_sec"] = round(time.monotonic() - start, 3)
        report["status"] = "PASS"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        report.update(status="FAIL", error="{}: {}".format(type(error).__name__, error))
        print(json.dumps(report, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
