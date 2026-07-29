# configs/sensors

- 특별한 언급이 없으면, `Sensor_n.json`에서 숫자 n이 클 수록 최신의 config임.
- 현재 기본값인 `Sensor_5.json`에는 카메라 고정 3대 + 후방 자유 1대, VLP-16 LiDAR 1대, GPS 1대, IMU 1대가 배치되어 있음.
- `Sensor_5.json`은 `Sensor_4.json`과 동일한 센서 구성·extrinsic을 유지하고 GPS 주기만 20 Hz에서 30 Hz로 변경함.
- `Sensor_5.json`에 저장된 센서 주기는 Camera 20 Hz, VLP-16 15 Hz, GPS 30 Hz, IMU 50 Hz임.
- ROS UDP 파이프라인은 VLP-16을 `900 RPM = 15 Hz`로 사용하며, FAST-LIO의 `scan_rate`도 15 Hz를 사용함.
- ROS UDP launch, localization, perception과 sensor static TF 기본값은 `Sensor_5.json`을 사용함.
