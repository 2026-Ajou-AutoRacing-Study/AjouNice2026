# configs/sensors

- 특별한 언급이 없으면, `Sensor_n.json`에서 숫자 n이 클 수록 최신의 config임.
- 현재 `Sensor_4.json`에는 카메라 고정 3대 + 후방 자유 1대, VLP-16 LiDAR 1대, GPS 1대, IMU 1대가 배치되어 있음.
- `Sensor_4.json`에 저장된 센서 주기는 Camera 20 Hz, VLP-16 15 Hz, GPS 20 Hz, IMU 50 Hz임.
- 대회 LiDAR 10 Hz 이하 권고에 따라 ROS UDP 파이프라인은 VLP-16을 `600 RPM = 10 Hz`로 묶으며, FAST-LIO의 `scan_rate`도 10 Hz를 사용함.
- ROS UDP launch 기본값과 센서 static TF 기본값은 `Sensor_4.json`을 사용함.
