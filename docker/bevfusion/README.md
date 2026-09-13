# AjouNice BEVFusion runtime

This directory builds and runs the Base3 four-camera MORAI BEVFusion inference
process. Sensor geometry comes only from `configs/sensors/Sensor.json` and the
model contract comes from
`perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py`.
Algorithm runtime policy comes from
`configs/perception/bevfusion_sensor4.yaml`.

The ROS host connects to a port published only on `127.0.0.1`. The simulator
boundary remains the competition-provided UDP interface. See
`docs/bevfusion/260720_morai_four_camera_zero_shot_full.md` for the protocol,
competition-rule caveat, and the Unix-domain-socket fallback.

```bash
# Create the local Compose environment file once.
cd ~/AjouNice2026/docker/bevfusion
cp .env.example .env

# Start the model server.
docker compose up -d inference
docker compose logs -f inference

# Check protocol and model readiness.
PYTHONPATH=. python3 scripts/healthcheck.py
```

Before production use, set `BEVFUSION_CHECKPOINT` to the best Base3 training
checkpoint. The checkpoint in `.env.example` is only the architecture-adapted
nuScenes initialization and is not a trained MORAI detector.

Set `PROJECTION_ONLY=true` in `.env` to validate calibration without loading
the model checkpoint.

## Online execution contract

- ROS chooses the closest four camera messages for each LiDAR timestamp.
- The ROS host decodes and resizes the four images concurrently with OpenCV.
- It sends four contiguous RGB8 tensors and one contiguous Nx4 XYZI tensor.
- The server constructs the MMDetection3D batch in memory; no temporary files
  are written or reread.
- FP16 autocast is enabled by default. Host preprocessing and server inference
  are separate latest-only stages, so they overlap without an unbounded queue.

The online OpenCV tensor was compared against the Base3 offline test transform:
image pixels, points and augmentation matrices match exactly (floating-point
matrix inverse differences were below `1e-6`).

## RTX 5060 Ti feasibility result

Measured with 20 real Base3 samples and the compatible BEVFusion checkpoint:

| Segment | Mean | p95 | Interpretation |
|---|---:|---:|---|
| Host four-camera preprocessing | 19.1 ms | 22.1 ms | Runs concurrently with inference |
| Docker tensor preprocessing | 2.6 ms | 3.3 ms | No filesystem round trip |
| FP16 model inference | 61.4 ms | 64.3 ms | Preloaded samples |
| Persistent TCP server round trip | 72.3 ms | 76.5 ms | 13.8 Hz server-side capacity |

The staged steady-state capacity is governed by the slower stage, currently
the 72.3 ms server round trip, rather than by adding 19.1 ms serially. This is
enough to consume a nominal 12 Hz LiDAR stream with modest margin. Individual
sensor-to-result latency remains roughly 90 ms plus ROS scheduling jitter.
Final measurements must be repeated with the trained Base3 checkpoint and live
MORAI topics; monitor `dropped_raw` and `dropped_prepared` in the status topic.

With the server running, the real-data TCP benchmark is reproducible with:

```bash
cd ~/AjouNice2026/docker/bevfusion
PYTHONPATH=. python3 scripts/benchmark_base3_tcp.py \
  --info ~/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_v1/morai_base3_infos_val.pkl \
  --sensor-config ~/AjouNice2026/configs/sensors/Sensor.json \
  --samples 50
```
