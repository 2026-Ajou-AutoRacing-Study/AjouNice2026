# topic interface
- docs/external_docs에 topic 종류와, msgs 타입들 종류 다 적혀져 있음.
- 여기는 주로 사용하게 될 주요 topic들만 정리
- 나중에 표로 다시 정리하든가 하기

## SIM -> ROS
- `/sim/process/state/msg/MoraiSimProcStatus`: `morai_msgs/MoraiSimProcStatus`
    - MORAI 시뮬레이터 실행 상태
- `/Ego_topic`: `morai_msgs/EgoVehicleStatus`
    - Ego 차량 현재 상태
- `/Object_topic`: `morai_msgs/ObjectStatusList`
    - 주변 객체 GT 정보

- `/GetTrafficLightStatus`: `morai_msgs/GetTrafficLightStatus`
    - 신호등 상태
- `/VehicleCollisionData`: `morai_msgs/VehicleCollisionData`
    - NPC 차량 간 충돌 정보
- `/CollisionData`: `morai_msgs/CollisionData`
    - Ego 차량과 객체 간 충돌 정보
- `/ReplayInfo_topic`: `morai_msgs/ReplayInfo`
    - Rosbag/Replay 관련 정보
- `/SyncModeInfo`: `morai_msgs/SyncModeInfo`
  - Synchronous mode 상태
- `/IntscnTL_topic`: `morai_msgs/IntscnTL`
  - 교차로 신호등 상태 배열
- `/tf`: `tf2_msgs/TFMessage`
    - MORAI가 publish하는 좌표계 정보
- `/clock`: `rosgraph_msgs/Clock`
    - MORAI 시뮬레이터 시간

### sensors
- 자세한 건 configs/sensors/sensor_layout.yaml 참고
- `/sensor/imu`: `sensor_msgs/Imu`
- `/sensor/gps`: `morai_msgs/GPSMessage`
- `/sensor/camera`: `sensor_msgs/CompressedImage`
- `/sensor/lidar1`: `sensor_msgs/PointCloud2`
- `/sensor/lidar2`: `sensor_msgs/PointCloud2`


## ROS -> SIM
- `/ctrl_cmd`: `morai_msgs/CtrlCmd`
  - Ego 차량 제어 명령
  - ROS 쪽에서 MORAI 차량을 움직이라고 보내는 제어 명령 토픽임.
  - MORAI 내부에서 키보드로 wasd 입력하는 그거랑 다름!! 얘는 자율주행 제어 토픽.

- `/ego_setting`: `morai_msgs/MultiEgoSetting`
  - Multi Ego 설정
- `/InsnControl`: `morai_msgs/IntersectionControl`
  - 교차로 상태 제어
- `/ScenarioLoad`: `morai_msgs/ScenarioLoad`
  - 시나리오 로드
- `/SaveSensorData`: `morai_msgs/SaveSensorData`
  - 센서 데이터 저장 요청
- `/SensorPosControl`: `morai_msgs/SensorPosControl`
  - 센서 위치/자세 제어
- `/SetTrafficLight`: `morai_msgs/SetTrafficLight`
  - 신호등 상태 제어
- `/lamps`: `morai_msgs/Lamps`
  - 방향지시등/비상등 제어
