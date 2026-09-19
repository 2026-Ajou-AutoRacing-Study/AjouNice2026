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

- `header.stamp`: representative rolling-LiDAR object acquisition time in
  production (`object_points_median`), or the original scan timestamp when
  `lidar_measurement_stamp_mode=header`.
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
  measurement to `map` at this timestamp. Object-specific point times are
  reduced to one robust frame timestamp because `DetectedObjects` has one
  shared header.

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

- Detector: Base3-Static `B3S-PP1 E34`.
- Semantic model: Base3 LRASPP E30.
- Range: 30 m.
- Thresholds: Vehicle `0.40`, Pedestrian `0.15`, Obstacle `0.10`.
- Input: single-frame 9-feature PointPainting.
- Detector velocity: disabled.
- Intra-scan coordinate deskew: disabled.

The previous epoch-20/threshold-0.20 figures were an early single-bag pipeline
smoke test and are no longer the production operating point. Current model
selection and rosbag metrics are recorded in
`perception/pointpainting/docs/legacy/production_runtime_contract_260913.md`.
