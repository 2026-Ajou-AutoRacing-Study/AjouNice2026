# configs/sensors

- 특별한 언급이 없으면, `Sensor_n.json`에서 숫자 n이 클수록 최신의 버전별 config임.
- `Sensor_competition_local.json`은 개인 PC 단독 실행용 정식 config임. 대회 포트
  계약을 그대로 사용하고 UDP IP만 `127.0.0.1`로 설정함.
- `Sensor.json`은 기존 스크립트 호환을 위해 남겨 둔 mirror이며
  `Sensor_competition_local.json`과 항상 동일해야 함.
- 심볼릭 링크를 사용하지 않고, 최신 `Sensor_n.json`의 내용을
  `Sensor_competition_local.json`과 호환 mirror `Sensor.json`에 명시적으로 복사함.
- 새 버전을 만들 때는 `Sensor_n.json`을 먼저 확정한 뒤 위 두 실행 파일을 같은
  내용으로 갱신함.
- 기존 YAML의 `source_sensor_config: Sensor.json`은 호환성을 위해 유지함.
- 261016 기준 `Sensor_competition_local.json`, `Sensor.json`, `Sensor_6.json`은 동일함.
- 30 Hz `sensorPeriod`는 현장 MORAI가 저장한 float32 표현
  `0.03333333507180214`로 통일함. `Sensor_competition_lan.json`은 현장 원본
  `morai_field_reference/AjouNice_sensor.json`과 JSON 값이 완전히 동일하고,
  local profile은 그 원본에서 UDP IP만 `127.0.0.1`로 바꾼 형태를 유지함.
- 현재 `Sensor_6.json`에는 카메라 고정 3대 + 후방 자유 1대, VLP-16 LiDAR 1대, GPS 1대, IMU 1대가 배치되어 있음.
- `Sensor_6.json`은 `Sensor_5.json`과 동일한 센서 구성·extrinsic을 유지하고 카메라 4대의 주기만 20 Hz에서 30 Hz로 변경함.
- 현재 센서 주기는 Camera 30 Hz, VLP-16 15 Hz, GPS 30 Hz, IMU 50 Hz임.
- ROS UDP 파이프라인은 VLP-16을 `900 RPM = 15 Hz`로 사용하며, FAST-LIO의 `scan_rate`도 15 Hz를 사용함.
- 현장 원본과 동일하게 VLP-16의 `frequencyIndex`는 `2`, MORAI 내부 PointCloud 시각화는
  `isVizPointCloud=false`를 사용함. 후자는 UDP 출력을 끄는 설정이 아니라 시뮬레이터의
  렌더링 부하를 줄이는 설정임.
- ROS UDP launch의 개인 PC 기본값은 `Sensor_competition_local.json`을 사용함.
  일부 기존 localization/perception 도구는 호환 mirror인 `Sensor.json`을 계속 참조함.
- `Sensor_localization_gt_lightweight.json`은 한 PC에서 MR 또는 MRP와
  localization/planning/control의 시간축을 검증하기 위한 로컬 전용 profile임.
  GPS 30 Hz와 IMU 50 Hz만 유지하고 Camera와 LiDAR를 MORAI sensor 구성에서 제거함.
  `/Ego_topic`, `/Competition_topic`, `/Object_topic`, `/ctrl_cmd`는 Sensor JSON이
  아니라 기존 Ego Network JSON을 사용하므로 그대로 유지됨.
- 위 경량 profile은 대회 sensor 구성을 대체하지 않으며 `udp_localization_light`와
  `sensing_localization_light`를 명시적으로 실행할 때만 사용함.
