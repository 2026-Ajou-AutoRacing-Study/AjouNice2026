# configs/sensors

- 특별한 언급이 없으면, `Sensor_n.json`에서 숫자 n이 클수록 최신의 버전별 config임.
- `Sensor.json`은 ROS launch와 일반 실행 스크립트가 참조하는 최신 실행용 config임.
- 심볼릭 링크를 사용하지 않고, 최신 `Sensor_n.json`의 내용을 `Sensor.json`에 명시적으로 복사함.
- 새 버전을 만들 때는 `Sensor_n.json`을 먼저 확정한 뒤 `Sensor.json`을 같은 내용으로 갱신함.
- YAML에서 `source_sensor_config`는 `Sensor.json`으로 쓰고, 파일 상단 주석에 갱신 날짜와 대응하는 `Sensor_n.json`을 기록함.
- 260916 기준 `Sensor.json`은 `Sensor_6.json`과 동일함.
- 현재 `Sensor_6.json`에는 카메라 고정 3대 + 후방 자유 1대, VLP-16 LiDAR 1대, GPS 1대, IMU 1대가 배치되어 있음.
- `Sensor_6.json`은 `Sensor_5.json`과 동일한 센서 구성·extrinsic을 유지하고 카메라 4대의 주기만 20 Hz에서 30 Hz로 변경함.
- 현재 센서 주기는 Camera 30 Hz, VLP-16 15 Hz, GPS 30 Hz, IMU 50 Hz임.
- ROS UDP 파이프라인은 VLP-16을 `900 RPM = 15 Hz`로 사용하며, FAST-LIO의 `scan_rate`도 15 Hz를 사용함.
- 현장 원본과 동일하게 VLP-16의 `frequencyIndex`는 `2`, MORAI 내부 PointCloud 시각화는
  `isVizPointCloud=false`를 사용함. 후자는 UDP 출력을 끄는 설정이 아니라 시뮬레이터의
  렌더링 부하를 줄이는 설정임.
- ROS UDP launch, localization, perception과 sensor static TF 기본값은 `Sensor.json`을 사용함.
