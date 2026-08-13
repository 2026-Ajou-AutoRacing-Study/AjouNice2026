# PointPainting output interface

## Topic

Production:

`/perception/object_recognition/detection/objects`

Type:

`autoware_perception_msgs/DetectedObjects`

Legacy diagnostics:

`/pointpainting/detections`

Type:

`morai_pointpainting_ros/PointPaintingDetectionArray`

## Coordinate and time contract

- `header.stamp`: original LiDAR scan timestamp.
- `header.frame_id`: original LiDAR frame, expected `lidar1`.
- Position: metres in the LiDAR coordinate frame.
- Orientation: quaternion generated from OpenPCDet yaw.
- Size:
  - `size.x`: box length along local x.
  - `size.y`: box width along local y.
  - `size.z`: box height.
- `detection_id`: unique only inside one message. It is not a tracking ID.
- Downstream tracking must assign persistent IDs.
- Production tracking uses the standard detection header and transforms the
  measurement to `map` at this timestamp.

## Timing fields

- `camera_age_max_ms`: oldest selected image relative to the LiDAR timestamp.
- `camera_span_ms`: timestamp spread among the four selected images.
- `queue_delay_ms`: time spent waiting for the processing worker.
- `segmentation_ms`: four-camera LRASPP inference.
- `painting_ms`: LiDAR projection and semantic feature concatenation.
- `detection_ms`: PointPillars preprocessing and inference.
- `total_ms`: node processing time from worker start to output construction.
- `dropped_lidar_frames`: scans overwritten by a newer scan while busy.
- `skipped_lidar_frames`: scans rejected because cameras or points were invalid.

## Current validated operating point

- Model: PointPainting PointPillars, epoch 20.
- Score threshold: 0.20.
- Validated detection output range: up to 20 m.
- Validation TP BEV centre error:
  - mean 0.145 m
  - p90 0.225 m
  - maximum 0.296 m
- Current validation precision/recall:
  - precision 0.227
  - recall 0.423

These numbers came from one MORAI rosbag and are pipeline-validation results, not final generalisation claims.
