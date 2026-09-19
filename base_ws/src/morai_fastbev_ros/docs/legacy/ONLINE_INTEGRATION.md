# Fast-BEV online ROS integration

## 좌표와 box 계약

- Fast-BEV 학습 GT와 온라인 calibration은 모두 `Sensor_6.json`에서 생성한다.
- 출력 좌표는 학습 당시 LiDAR 좌표계(`x forward, y left, z up`)다.
- `Sensor_6.json`과 UDP LiDAR의 기본 ROS frame은 `lidar1`이며 Fast-BEV
  `DetectedObjects`도 동일한 frame을 사용한다. 다른 센서 구성을 사용할
  때만 launch의 `output_frame_id`를 함께 수정한다.
- `size`는 `(length x, width y, height z)`, yaw는 ROS 표준 CCW radian이다.
- class 순서는 `vehicle=0, pedestrian=1, obstacle=2`이며 기존 detection
  메시지의 `class_id`에는 1부터 시작하는 값으로 기록한다.

## T&P 연결

Fast-BEV는 표준 객체와 detector-neutral 진단·시각화 출력을 발행한다.

1. `/perception/object_recognition/detection/objects`
   - `autoware_perception_msgs/DetectedObjects`
   - EKF의 유일한 production 입력이다.
2. `/fastbev/diagnostics`
   - `diagnostic_msgs/DiagnosticArray`
   - camera span, queue, preprocessing, inference, latency와 drop count를 담는다.
3. `/hmi/perc/jsk_pillar_objects`, `/fastbev/detection_markers`
   - RViz/debug용이다.

표준 입력은 `/perception/object_recognition/tracking/objects`를 거쳐 최종
`/perception/object_recognition/objects`로 이어진다. Marker text나 JSK box를
데이터 전달에 사용하지 않는다.

## 동기화와 지연 정책

- 네 카메라는 ApproximateTimeSynchronizer로 묶는다.
- ApproximateTimeSynchronizer 허용폭은 기본 15 ms다. 35 Hz 스트레스 재생에서
  50 ms를 쓰면 서로 이웃한 카메라 프레임이 묶일 수 있어 축소했다.
- callback, 4-camera CPU 전처리, GPU 추론을 분리한다.
- raw와 prepared 단계마다 가장 최신 세트 하나만 남긴다. 따라서 처리율이
  입력률보다 낮아져도 과거 프레임 backlog가 누적되지 않는다.
- diagnostics의 `dropped_raw_sets`, `dropped_prepared_sets`, `skipped_sets`로
  각 단계의 최신-frame 정책과 처리 실패를 구분한다.

## 검증 순서

```bash
rostopic hz --wall-time /fastbev/diagnostics
rostopic hz --wall-time /perception/object_recognition/detection/objects
rostopic echo -n 1 /fastbev/diagnostics
rostopic echo -n 1 /perception/object_recognition/tracking/objects
rostopic echo -n 1 /perception/object_recognition/objects
```

확인할 핵심 값은 camera span, preprocessing/inference/total ms, Detection Hz,
dropped/skipped set, 각 class 검출 수, TF/RViz box 정렬이다.

diagnostics의 `total_ms`는 전처리와 추론 계산량의 합계다. staged mode에서는 CPU와 GPU가
겹쳐 실행되므로 실제 steady-state 처리율은 `1 / max(preprocess, inference)`에
가깝고, 로그의 `latency`가 한 세트의 end-to-end wall latency다.
