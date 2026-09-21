#!/usr/bin/env python3
"""Serve Sensor_4 projection checks and BEVFusion inference over localhost TCP."""

import argparse
import logging
import socket
import time

import cv2
import numpy as np

from app.protocol import (
    TYPE_ERROR,
    TYPE_PING,
    TYPE_PONG,
    TYPE_REQUEST,
    TYPE_RESPONSE,
    ProtocolError,
    recv_message,
    send_message,
)
from app.sensor4_calibration import load_sensor4_calibration, serializable_calibration


LOGGER = logging.getLogger("bevfusion_server")


class InferenceServer:
    def __init__(self, args):
        self.args = args
        self.calibration = load_sensor4_calibration(
            args.sensor_config, args.camera_order, args.intrinsic_source
        )
        self.model = None
        if not args.projection_only:
            from app.bevfusion_model import BevFusionModel

            LOGGER.info("Loading BEVFusion model from %s", args.checkpoint)
            self.model = BevFusionModel(
                args.config,
                args.checkpoint,
                self.calibration,
                args.device,
                use_amp=args.amp,
                preprocess_workers=args.preprocess_workers,
                warmup_iterations=args.warmup_iterations,
            )
            LOGGER.info("BEVFusion model is ready")

    def serve_forever(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.args.bind, self.args.port))
            server.listen(self.args.backlog)
            LOGGER.info("Listening on %s:%d", self.args.bind, self.args.port)
            while True:
                connection, address = server.accept()
                LOGGER.info("Client connected: %s", address)
                with connection:
                    connection.settimeout(self.args.client_timeout)
                    self._serve_client(connection)

    def _serve_client(self, connection):
        while True:
            try:
                message_type, metadata, blobs = recv_message(connection)
            except (EOFError, ConnectionError, socket.timeout):
                LOGGER.info("Client disconnected")
                return
            except ProtocolError as error:
                LOGGER.warning("Protocol error: %s", error)
                return
            try:
                if message_type == TYPE_PING:
                    send_message(
                        connection,
                        TYPE_PONG,
                        {
                            "ready": True,
                            "model_loaded": self.model is not None,
                            "calibration": serializable_calibration(self.calibration),
                        },
                    )
                elif message_type == TYPE_REQUEST:
                    self._handle_request(connection, metadata, blobs)
                else:
                    raise ValueError("unsupported message type {}".format(message_type))
            except Exception as error:  # Keep the server alive for the next request.
                LOGGER.exception("Request failed")
                send_message(connection, TYPE_ERROR, {"error": str(error)})

    def _handle_request(self, connection, metadata, blobs):
        request_start = time.monotonic()
        if metadata.get("sensor_config_sha256") != self.calibration["config_sha256"]:
            raise ValueError("ROS and Docker are not using the same sensor config")
        if metadata.get("camera_order") != [
            camera["frame_id"] for camera in self.calibration["cameras"]
        ]:
            raise ValueError("ROS and Docker camera order differs")
        if metadata.get("intrinsic_source") != self.calibration["intrinsic_source"]:
            raise ValueError("ROS and Docker intrinsic_source differs")
        points = parse_points(metadata, blobs)
        operation = metadata.get("operation", "inference")
        if operation == "projection":
            projection_blobs = self._project(points, blobs)
            send_message(
                connection,
                TYPE_RESPONSE,
                {
                    "operation": operation,
                    "stamp": metadata.get("stamp", 0.0),
                    "sequence": metadata.get("sequence", 0),
                    "frame_id": self.calibration["lidar_frame"],
                    "server_ms": (time.monotonic() - request_start) * 1000.0,
                },
                projection_blobs,
            )
            return
        if operation != "inference":
            raise ValueError("unsupported operation {!r}".format(operation))
        if self.model is None:
            raise RuntimeError("server was started in projection-only mode")
        image_encoding = metadata.get("image_encoding", "compressed")
        if image_encoding not in ("compressed", "rgb8"):
            raise ValueError(
                "unsupported image_encoding {!r}".format(image_encoding)
            )
        detections, preprocess_ms, inference_ms = self.model.infer(
            points,
            blobs,
            metadata.get("sequence", 0),
            metadata.get("score_threshold", 0.2),
            metadata.get("max_detections", 100),
            images_preprocessed=(image_encoding == "rgb8"),
        )
        send_message(
            connection,
            TYPE_RESPONSE,
            {
                "operation": operation,
                "stamp": metadata.get("stamp", 0.0),
                "sequence": metadata.get("sequence", 0),
                "frame_id": self.calibration["lidar_frame"],
                "detections": detections,
                "preprocess_ms": preprocess_ms,
                "inference_ms": inference_ms,
                "server_ms": (time.monotonic() - request_start) * 1000.0,
            },
        )

    def _project(self, points, request_blobs):
        homogeneous = np.ones((points.shape[0], 4), dtype=np.float64)
        homogeneous[:, :3] = points[:, :3]
        outputs = {}
        for camera in self.calibration["cameras"]:
            frame_id = camera["frame_id"]
            image = cv2.imdecode(
                np.frombuffer(request_blobs["image/{}".format(frame_id)], np.uint8),
                cv2.IMREAD_COLOR,
            )
            if image is None:
                raise ValueError("could not decode image for {}".format(frame_id))
            camera_points = (camera["lidar2cam"] @ homogeneous.T).T
            depth = camera_points[:, 2]
            visible = depth > 0.5
            pixels = (camera["cam2img"] @ camera_points[:, :3].T).T
            pixels[:, 0] /= np.maximum(pixels[:, 2], 1e-6)
            pixels[:, 1] /= np.maximum(pixels[:, 2], 1e-6)
            visible &= pixels[:, 0] >= 0
            visible &= pixels[:, 0] < image.shape[1]
            visible &= pixels[:, 1] >= 0
            visible &= pixels[:, 1] < image.shape[0]
            indices = np.flatnonzero(visible)[:: max(1, self.args.projection_stride)]
            for index in indices:
                normalized_depth = min(float(depth[index]) / 60.0, 1.0)
                color = (int(255 * (1.0 - normalized_depth)), 0, int(255 * normalized_depth))
                cv2.circle(
                    image,
                    (int(round(pixels[index, 0])), int(round(pixels[index, 1]))),
                    2,
                    color,
                    -1,
                )
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("failed to encode projection image")
            outputs["projection/{}".format(frame_id)] = encoded.tobytes()
        return outputs


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=29500)
    parser.add_argument("--backlog", type=int, default=1)
    parser.add_argument("--client-timeout", type=float, default=30.0)
    parser.add_argument("--sensor-config", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--intrinsic-source", default="horizontal_fov")
    parser.add_argument("--camera-order", nargs=4, default=None)
    parser.add_argument("--projection-only", action="store_true")
    parser.add_argument("--projection-stride", type=int, default=4)
    parser.add_argument("--amp", action="store_true")
    parser.add_argument("--preprocess-workers", type=int, default=4)
    parser.add_argument("--warmup-iterations", type=int, default=2)
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def parse_points(metadata, blobs):
    blob_name = metadata.get("points_blob", "points")
    if blob_name not in blobs:
        raise ValueError("request is missing point cloud blob")
    point_dim = int(metadata.get("point_dim", 4))
    if point_dim not in (4, 5):
        raise ValueError(
            "BEVFusion runtime expects 4 or 5 float32 point fields, got {}".format(
                point_dim)
        )
    values = np.frombuffer(blobs[blob_name], dtype="<f4")
    if values.size % point_dim:
        raise ValueError("point cloud payload does not align to point_dim")
    points = values.reshape((-1, point_dim))
    expected_count = metadata.get("point_count")
    if expected_count is not None and len(points) != int(expected_count):
        raise ValueError(
            "point_count {} differs from payload {}".format(
                expected_count, len(points)
            )
        )
    return points


def main():
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    InferenceServer(args).serve_forever()


if __name__ == "__main__":
    main()
