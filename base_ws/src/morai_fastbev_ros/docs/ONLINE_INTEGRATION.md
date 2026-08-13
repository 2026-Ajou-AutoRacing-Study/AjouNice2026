# Fast-BEV online ROS integration

## 좌표와 box 계약

- Fast-BEV 학습 GT와 온라인 calibration은 모두 `Sensor_6.json`에서 생성한다.
- 출력 좌표는 학습 당시 LiDAR 좌표계(`x forward, y left, z up`)다.
- 기본 ROS frame은 `velodyne`이며 실제 TF 이름이 다르면 launch의
  `output_frame_id`를 수정한다.
- `size`는 `(length x, width y, height z)`, yaw는 ROS 표준 CCW radian이다.
- class 순서는 `vehicle=0, pedestrian=1, obstacle=2`이며 기존 detection
  메시지의 `class_id`에는 1부터 시작하는 값으로 기록한다.

## T&P 연결

Fast-BEV는 두 형태를 동시에 발행한다.

1. `/fastbev/detections`
   - 기존 PointPainting용 lightweight tracking adapter와 호환된다.
   - `tracking_from_fastbev.launch`로 `/tracking/objects`를 만들 수 있다.
2. `/hmi/perc/jsk_pillar_objects`
   - `ekf-multi-object-tracking`의 정식 detection 입력이다.
   - class label은 tracker enum에 맞춰 vehicle=CAR(1), pedestrian=3,
     obstacle=UNKNOWN(0)으로 변환한다.

운용 경로는 2번 EKF 경로를 권장한다. 이는 지속 ID와 속도 추정을 제공하며,
이후 `/tracking/objects`, `/prediction/objects`,
`/perception/object_recognition/objects`로 이어진다.

## 동기화와 지연 정책

- 네 카메라는 ApproximateTimeSynchronizer로 묶는다.
- callback과 CUDA 추론을 분리한다.
- 추론 중 새 세트가 들어오면 가장 최신 세트 하나만 남기고 이전 대기 세트를
  버린다. 따라서 처리율이 입력률보다 낮아져도 지연이 누적되지 않는다.
- `dropped_lidar_frames`, `skipped_lidar_frames`는 기존 메시지 호환 때문에
  유지한 이름이다. Fast-BEV에서는 각각 dropped/skipped 4-camera set 수다.

## 검증 순서

```bash
rostopic hz --wall-time /fastbev/detections
rostopic hz --wall-time /hmi/perc/jsk_pillar_objects
rostopic echo -n 1 /fastbev/detections
rostopic echo -n 1 /tracking/objects
rostopic echo -n 1 /prediction/objects
rostopic echo -n 1 /perception/object_recognition/objects
```

확인할 핵심 값은 camera span, preprocessing/inference/total ms, Detection Hz,
dropped/skipped set, 각 class 검출 수, TF/RViz box 정렬이다.
