#!/usr/bin/env python3
"""Serve Sensor_4 projection checks and BEVFusion inference over localhost TCP."""

import argparse
import hashlib
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


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class InferenceServer:
    def __init__(self, args):
        self.args = args
        self.fp32_reference = bool(getattr(args, 'fp32_reference', False))
        if self.fp32_reference and args.amp:
            raise ValueError('FP32 reference does not enable AMP')
        self.calibration = load_sensor4_calibration(
            args.sensor_config, args.camera_order, args.intrinsic_source
        )
        self.model = None
        self.runtime_identity = None
        if not args.projection_only:
            self.runtime_identity = {
                'checkpoint_sha256': file_sha256(args.checkpoint), 'config_sha256': file_sha256(args.config),
                'amp': args.amp, 'parallel_camera': bool(getattr(args, 'parallel_camera', False)),
                'compile_parts': list(getattr(args, 'compile_parts', ())),
                'base5_runtime': bool(getattr(args, 'base5_runtime', False)),
            }
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
                temporal_feature_cache=args.temporal_feature_cache,
                temporal_maximum_age_sec=args.temporal_maximum_age_sec,
                base5_runtime=getattr(args, "base5_runtime", False),
                compile_parts=getattr(args, "compile_parts", ()),
                fuse_dense_bn=getattr(args, "fuse_dense_bn", False),
                camera_cuda_graph=getattr(args, "camera_cuda_graph", False),
                eager_camera_numerics=getattr(args, "eager_camera_numerics", False),
                rgb_after_crop=getattr(args, "rgb_after_crop", False),
                compile_reference_layout=getattr(args, "compile_reference_layout", False),
                parallel_camera=getattr(args, "parallel_camera", False),
                point_intensity_source=getattr(args, "point_intensity_source", "native"),
            )
            LOGGER.info("BEVFusion model is ready")
            if self.fp32_reference:
                import torch
                torch.backends.cuda.matmul.allow_tf32 = False
                torch.backends.cudnn.allow_tf32 = False
                torch.backends.cudnn.benchmark = False

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
                            "runtime_identity": self.runtime_identity,
                            "model_loaded": self.model is not None,
                            "classes": list(getattr(self.model, "classes", ())),
                            "point_intensity_source": getattr(getattr(self.model, "preprocessor", None), "point_intensity_source", None),
                            "image_final_dim": list(getattr(getattr(self.model, "preprocessor", None), "final_dim", ())),
                            "fp32_reference": self.fp32_reference,
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
        declared_source = metadata.get('point_intensity_source')
        if declared_source is not None and declared_source != self.model.preprocessor.point_intensity_source:
            raise ValueError('ROS/server intensity source contract differs')
        declared_size = metadata.get('model_image_final_dim')
        if declared_size is not None and tuple(declared_size) != self.model.preprocessor.final_dim:
            raise ValueError('ROS/server model image size contract differs')
        image_encoding = metadata.get("image_encoding", "compressed")
        if image_encoding not in ("compressed", "rgb8"):
            raise ValueError(
                "unsupported image_encoding {!r}".format(image_encoding)
            )
        if image_encoding == 'rgb8' and (
                int(metadata.get('image_height', -1)),
                int(metadata.get('image_width', -1))) != self.model.preprocessor.final_dim:
            raise ValueError('ROS RGB crop size differs from the model input contract')
        detections, preprocess_ms, inference_ms = self.model.infer(
            points,
            blobs,
            metadata.get("sequence", 0),
            metadata.get("score_threshold", 0.2),
            metadata.get("max_detections", 100),
            images_preprocessed=(image_encoding == "rgb8"),
            timestamp_sec=metadata.get("stamp"),
            lidar_pose_world=metadata.get("lidar_pose_world"),
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
                "query_memory_diagnostics": (
                    self.model.last_query_memory_diagnostics),
                "preprocess_ms": preprocess_ms,
                "inference_ms": inference_ms,
                "postprocess_ms": getattr(self.model, "last_runtime_timings", {}).get("postprocess_ms", 0.0),
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
    parser.add_argument("--fp32-reference", action="store_true",
                        help="Disable AMP/TF32/autotuning for the offline/ROS reference gate; not a latency preset")
    parser.add_argument("--preprocess-workers", type=int, default=4)
    parser.add_argument("--warmup-iterations", type=int, default=2)
    parser.add_argument("--temporal-feature-cache", action="store_true")
    parser.add_argument("--temporal-maximum-age-sec", type=float, default=0.30)
    parser.add_argument("--base5-runtime", action="store_true",
                        help="Explicit single-frame Base5 contract; not E25 deployment")
    parser.add_argument("--point-intensity-source", choices=('native', 'ros255'), default='native',
                        help="Base5 only: ros255 applies fixed /255; native preserves checkpoint-native intensity")
    parser.add_argument("--compile-parts", nargs="+", choices=("camera", "neck", "bev"), default=())
    parser.add_argument("--parallel-camera", action="store_true",
                        help="Experimental Base5-only camera side stream; sparse encoder stays on default stream")
    parser.add_argument("--fuse-dense-bn", action="store_true",
                        help="Experimental eval-only dense Conv/BN folding")
    parser.add_argument("--camera-cuda-graph", action="store_true",
                        help="Opt-in serial fixed-shape camera eager-kernel replay")
    parser.add_argument("--eager-camera-numerics", action="store_true",
                        help="Keep camera LayerNorm/GELU eager within camera compilation")
    parser.add_argument("--rgb-after-crop", action="store_true",
                        help="Base5-only pixel-equivalent late BGR-to-RGB conversion")
    parser.add_argument("--compile-reference-layout", action="store_true",
                        help="Disable compiler layout optimization and shape padding")
    parser.add_argument("--cpu-threads", type=int, default=None)
    parser.add_argument("--opencv-threads", type=int, default=None)
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
    if args.cpu_threads is not None:
        if args.cpu_threads < 1: raise ValueError("cpu-threads must be positive")
        import torch
        torch.set_num_threads(args.cpu_threads)
    if args.opencv_threads is not None:
        if args.opencv_threads < 1: raise ValueError("opencv-threads must be positive")
        cv2.setNumThreads(args.opencv_threads)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    InferenceServer(args).serve_forever()


if __name__ == "__main__":
    main()
