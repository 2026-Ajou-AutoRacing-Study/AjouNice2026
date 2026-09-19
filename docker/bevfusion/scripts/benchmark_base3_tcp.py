#!/usr/bin/env python3
"""Benchmark the persistent Base3 TCP runtime with real MMDet3D info rows."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import pickle
import socket
import statistics
import time
from pathlib import Path

import cv2
import numpy as np

from app.protocol import TYPE_REQUEST, TYPE_RESPONSE, recv_message, send_message


CAMERAS = {
    "camera_front": "front",
    "camera_front_left": "front_left",
    "camera_front_right": "front_right",
    "camera_back": "back",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--info", required=True)
    parser.add_argument("--sensor-config", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29500)
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--discard", type=int, default=3)
    parser.add_argument("--score-threshold", type=float, default=0.2)
    return parser.parse_args()


def load_rows(path, count):
    with Path(path).expanduser().open("rb") as info_file:
        payload = pickle.load(info_file)
    rows = payload.get("data_list", payload) if isinstance(payload, dict) else payload
    if not rows:
        raise ValueError("info file has no data rows")
    return rows[: min(int(count), len(rows))]


def preprocess_image(path):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not read {}".format(path))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height, width = image.shape[:2]
    scale = 0.55 * 1280.0 / width
    new_width, new_height = int(width * scale), int(height * scale)
    image = cv2.resize(
        image, (new_width, new_height), interpolation=cv2.INTER_CUBIC
    )
    top = new_height - 256
    left = max(0, new_width - 704) // 2
    image = np.ascontiguousarray(image[top:top + 256, left:left + 704])
    if image.shape != (256, 704, 3):
        raise ValueError("preprocessed image has shape {}".format(image.shape))
    return image.tobytes()


def percentile(values, quantile):
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def summarize(name, values):
    mean = statistics.mean(values)
    print(
        "{}: mean={:.2f} ms p50={:.2f} ms p95={:.2f} ms rate={:.2f} Hz".format(
            name, mean, percentile(values, 50), percentile(values, 95), 1000.0 / mean
        )
    )


def main():
    args = parse_args()
    rows = load_rows(args.info, args.samples)
    sensor_bytes = Path(args.sensor_config).expanduser().read_bytes()
    sensor_hash = hashlib.sha256(sensor_bytes).hexdigest()
    host_times, roundtrip_times, inference_times = [], [], []

    with ThreadPoolExecutor(max_workers=4) as executor:
        with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
            connection.settimeout(30.0)
            for sequence, row in enumerate(rows):
                host_start = time.perf_counter()
                points = np.fromfile(
                    row["lidar_points"]["lidar_path"], dtype=np.float32
                ).reshape((-1, 4))
                paths = [
                    row["images"][dataset_name]["img_path"]
                    for dataset_name in CAMERAS.values()
                ]
                images = list(executor.map(preprocess_image, paths))
                host_ms = (time.perf_counter() - host_start) * 1000.0
                blobs = {"points": points.astype("<f4", copy=False).tobytes()}
                for runtime_name, image_blob in zip(CAMERAS, images):
                    blobs["image/{}".format(runtime_name)] = image_blob
                metadata = {
                    "operation": "inference",
                    "sequence": sequence,
                    "stamp": float(sequence),
                    "lidar_frame": "lidar1",
                    "camera_order": list(CAMERAS),
                    "sensor_config_sha256": sensor_hash,
                    "intrinsic_source": "horizontal_fov",
                    "points_blob": "points",
                    "point_dim": 4,
                    "point_count": len(points),
                    "image_encoding": "rgb8",
                    "image_height": 256,
                    "image_width": 704,
                    "image_channels": 3,
                    "score_threshold": args.score_threshold,
                    "max_detections": 100,
                }
                request_start = time.perf_counter()
                send_message(connection, TYPE_REQUEST, metadata, blobs)
                message_type, response, _blobs = recv_message(connection)
                roundtrip_ms = (time.perf_counter() - request_start) * 1000.0
                if message_type != TYPE_RESPONSE:
                    raise RuntimeError("server response: {}".format(response))
                if sequence >= args.discard:
                    host_times.append(host_ms)
                    roundtrip_times.append(roundtrip_ms)
                    inference_times.append(float(response["inference_ms"]))

    summarize("host_preprocess", host_times)
    summarize("tcp_roundtrip", roundtrip_times)
    summarize("model_inference", inference_times)
    stage_ms = max(statistics.mean(host_times), statistics.mean(roundtrip_times))
    serial_ms = statistics.mean(host_times) + statistics.mean(roundtrip_times)
    print("staged_capacity={:.2f} Hz".format(1000.0 / stage_ms))
    print("individual_latency_estimate={:.2f} ms plus ROS jitter".format(serial_ms))


if __name__ == "__main__":
    main()
