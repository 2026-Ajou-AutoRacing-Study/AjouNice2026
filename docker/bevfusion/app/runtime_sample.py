"""Materialize one in-memory ROS frame as a RAM-backed MMDetection3D sample."""

import os
import pickle

import numpy as np


class RuntimeSampleWriter:
    def __init__(self, runtime_dir, calibration):
        self.runtime_dir = runtime_dir
        self.calibration = calibration
        os.makedirs(runtime_dir, exist_ok=True)
        self.points_path = os.path.join(runtime_dir, "points.bin")
        self.info_path = os.path.join(runtime_dir, "sensor4_runtime_info.pkl")

    def write(self, metadata, blobs):
        images = {}
        for camera in self.calibration["cameras"]:
            frame_id = camera["frame_id"]
            blob_name = "image/{}".format(frame_id)
            if blob_name not in blobs:
                raise ValueError("request is missing {}".format(blob_name))
            filename = "{}.jpg".format(frame_id)
            image_path = os.path.join(self.runtime_dir, filename)
            _atomic_write(image_path, blobs[blob_name])
            images[frame_id] = {
                "img_path": filename,
                "cam2img": camera["cam2img"].tolist(),
                "lidar2cam": camera["lidar2cam"].tolist(),
            }

        points_blob_name = metadata.get("points_blob", "points")
        if points_blob_name not in blobs:
            raise ValueError("request is missing point cloud blob")
        point_dim = int(metadata.get("point_dim", 5))
        if point_dim != 5:
            raise ValueError("BEVFusion runtime expects five float32 point fields")
        points = np.frombuffer(blobs[points_blob_name], dtype="<f4")
        if points.size % point_dim:
            raise ValueError("point cloud payload does not align to point_dim")
        points = points.reshape((-1, point_dim))
        _atomic_write(self.points_path, points.astype("<f4", copy=False).tobytes())

        timestamp = float(metadata.get("stamp", 0.0))
        info = {
            "metainfo": {"dataset": "MORAI Sensor_4 runtime"},
            "data_list": [
                {
                    "sample_idx": str(metadata.get("sequence", 0)),
                    "timestamp": timestamp,
                    "lidar_points": {
                        "lidar_path": self.points_path,
                        "num_pts_feats": point_dim,
                    },
                    "images": images,
                    "sweeps": [],
                }
            ],
        }
        _atomic_pickle(self.info_path, info)
        return self.points_path, self.runtime_dir, self.info_path, points


def _atomic_write(path, value):
    temporary = path + ".tmp"
    with open(temporary, "wb") as output:
        output.write(value)
    os.replace(temporary, path)


def _atomic_pickle(path, value):
    temporary = path + ".tmp"
    with open(temporary, "wb") as output:
        pickle.dump(value, output, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(temporary, path)

