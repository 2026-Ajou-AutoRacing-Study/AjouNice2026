# Fast-BEV / PointPainting runtime optimization roadmap

## Scope and measurement rules

This document records optimization candidates after the 2026-08-14 through
2026-08-16 ROS bag benchmarks.  Throughput, drop rate, and single-frame
input-to-publish latency must remain separate metrics.  A pipeline stage can
increase throughput by overlapping adjacent frames without reducing the
latency of one frame.

The current same-host reference at roughly 34.6 Hz camera input is:

| Detector | Backend | Detection rate | Compute p50 / p95 | Input-to-publish p50 / p95 |
|---|---|---:|---:|---:|
| Fast-BEV | OpenCV staged | 34.196 Hz | 36.96 / 41.25 ms | 40.08 / 52.24 ms |
| PointPainting | PIL, 4 camera workers | 14.965 Hz | 54.89 / 59.51 ms | 58.51 / 72.48 ms |
| PointPainting | OpenCV, 4 camera workers | 14.965 Hz | 45.21 / 48.87 ms | 45.97 / 52.97 ms |

PointPainting output is capped by the 15 Hz LiDAR.  Fast-BEV is camera-only
and can follow the camera rate, so output Hz alone is not a fair model-runtime
comparison.

## PointPainting priorities

### 1. Keep four camera CPU workers

Four workers reduced the four-camera JPEG decode p50 from about 27.3 ms to
13.7--14.2 ms.  It also removed the sustained queue backlog at a 15 Hz LiDAR
input.  This is the lowest-risk production optimization because it does not
change image values.

### 2. Separate decode optimization from resize-backend changes

JPEG decode and image transformation are different problems.

- A decode implementation that returns the same RGB pixels should not change
  model AP.  Candidate work includes moving BGR-to-RGB conversion after
  downsampling, eliminating unnecessary full-resolution copies, reusing
  buffers, and benchmarking libjpeg-turbo/TurboJPEG.
- Reduced-resolution JPEG decode, GPU nvJPEG, or a different resize kernel can
  change input pixels.  These require accuracy validation even when they are
  faster.
- Raw camera transport removes JPEG decode but substantially increases ROS
  bandwidth and memory copies.  It is only useful after an end-to-end network
  and CPU budget test on the competition machines.

The current PointPainting decode timer includes `cv2.imdecode`, a
full-resolution BGR-to-RGB conversion, and a copy.  Fast-BEV moves its color
conversion to the smaller resized image, so applying the same boundary change
to PointPainting is the first implementation candidate.

### 3. OpenCV resize/normalize is promising but is not accuracy-neutral yet

OpenCV with four workers reduced resize/tensor/normalize p50 from about
12.0 ms to 3.2 ms and reduced input-to-publish p50 from 58.5 ms to 46.0 ms.
However, the LRASPP checkpoint was trained with the PIL/torchvision path.
PIL antialiased bilinear and OpenCV `INTER_LINEAR` produced normalized tensor
MAE of roughly 0.010--0.020 depending on camera.

This small pixel error does not mean the 3D AP loss will have the same small
percentage.  Sparse LiDAR points can lie on semantic boundaries, so a few
changed semantic labels can affect painting.  Before making OpenCV the
production default:

1. evaluate LRASPP mIoU on the fixed validation split;
2. build painted validation features with both backends;
3. evaluate the same PointPillars checkpoint with the same center and IoU
   metrics;
4. if the difference is material, fine-tune LRASPP with the OpenCV transform
   and regenerate painted train/validation features.

Expected risk is modest, not zero.  Pure decode-path optimization that
preserves pixels is safer than changing interpolation.

### 4. Defer frame-level two-stage pipelining unless GPU contention appears

Overlapping frame `n+1` CPU preprocessing with frame `n` GPU inference can
increase sustainable throughput.  It does not remove preprocessing from the
latency of frame `n`.  The current four-worker implementation already follows
15 Hz without backlog, so the extra queue/state complexity is not justified
until segmentation, traffic-light detection, or another GPU model runs at the
same time.

### 5. Later high-effort candidates

- TensorRT/ONNX for LRASPP and PointPillars, measured separately.
- CUDA Graphs and preallocated/pinned host tensors for stable shapes.
- GPU JPEG decode only after measuring transfer and launch overhead for four
  images per LiDAR frame.
- Lower semantic resolution only with matching LRASPP retraining and a
  distance-binned 3D AP check.
- Do not expect more than 15 Hz fresh PointPainting detections without a
  higher-rate LiDAR; prediction/tracking can publish between LiDAR updates but
  that is not a new detector observation.

## Fast-BEV priorities

### Current production state

Fast-BEV already uses an OpenCV inference transform and a staged CPU/GPU
pipeline.  At a 30 Hz camera input it recorded no steady-state drop, compute
p50 36.66 ms, and input-to-publish p50 38.08 ms.  At 34.6 Hz stress it still
followed the input at about 34.2 Hz.

### Next candidates

1. Preserve the current OpenCV training/inference transform contract and
   periodically compare against PIL on a fixed validation set.
2. Reuse decoded-image and normalized tensor buffers and pin host memory to
   reduce allocation and H2D variance.
3. Profile feature extraction, view transformation, BEV encoder, and head
   separately before choosing a TensorRT target.
4. Evaluate TensorRT/ONNX or CUDA Graphs only after the native PyTorch result
   and metric parity are frozen.
5. Image resolution or camera ROI reduction is a model-quality ablation, not
   a free runtime optimization.  It needs retraining and near/far, per-camera
   AP evaluation.
6. Measure p95/p99 latency and drop while all competition perception nodes run
   together.  Single-model headroom is not a guarantee under GPU contention.

## Accuracy-comparison guardrails

- Use the same train/validation/test frame IDs and the same class mapping.
- Compare the sealed 222-frame test split only after model selection on the
  228-frame validation split.
- Use identical center thresholds, score floor, evaluation range, and IoU
  implementation.
- Report center AP together with mATE, mASE, mAOE, and IoU AP.  Center AP alone
  does not prove box size or yaw quality.
- Report 0--10, 10--20, 20--30, and 30--50 m bins with GT counts.  A bin with
  only a handful of GT boxes is not evidence of long-range performance.

## Related evidence

- `perception/detection3d/docs/runtime_comparison_260816.md`
- `perception/pointpainting/docs/runtime_benchmark_260816.md`
- `perception/detection3d/fastbev/docs/obstacle_regression_roadmap.md`
