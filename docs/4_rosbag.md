# morai - rosbag 만들기
## 계획
- 무상실에 매번 morai를 키는 게 어려움
- rosbag으로 morai 센서만 담아놓고 배포하면, 그걸 local에서 ros만 구동시켜서 테스트 및 개발이 가능할지도?

### rosbag 제작
```bash
source ~/AjouNice2026/scripts/rosbag_maker.sh
```
- 도심로 10초 녹화함
  - 정지 상태에서 직진 가속도 주행, 왼쪽 및 오른쪽 살짝식 조향
```bash
rosbag info morai_local_valid_ego_sensors.bag

path:        morai_local_valid_ego_sensors.bag
version:     2.0
duration:    9.7s
start:       Jun 07 2026 18:22:14.23 (1780824134.23)
end:         Jun 07 2026 18:22:23.94 (1780824143.94)
size:        81.8 MB
messages:    663
compression: none [73/73 chunks]
types:       morai_msgs/EgoVehicleStatus [b3d9f65cf9e5cbc5fd966757559f49b2]
             morai_msgs/GPSMessage       [6ab9511aaf9d33b721630ddffc1f7c5f]
             morai_msgs/ObjectStatusList [97d744183b4ec364d0c8dac1cd520ef1]
             sensor_msgs/CompressedImage [8f7a12909da2c9d3332d540a0977563f]
             sensor_msgs/Imu             [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/PointCloud2     [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage          [94810edda583a504dfda3829e70d7eec]
topics:      /Ego_topic               109 msgs    : morai_msgs/EgoVehicleStatus
             /Object_topic            114 msgs    : morai_msgs/ObjectStatusList
             /gps                      43 msgs    : morai_msgs/GPSMessage      
             /image_jpeg/compressed   104 msgs    : sensor_msgs/CompressedImage
             /sensor/imu              107 msgs    : sensor_msgs/Imu
             /lidar3D                  72 msgs    : sensor_msgs/PointCloud2    
             /tf                      114 msgs    : tf2_msgs/TFMessage
```

- 근데 /clock은 왜 없지
  - real-time mode 켜져 있는데 흠...
  - `rostopic hz /clock` 해 봐도 `no new messages` 나오네 -> 일단 나중에 생각(당장 없어도 되긴 하니까..)


### rosbag 테스트
```bash
# 재생
rosbag play morai_local_valid_ego_sensors.bag --clock
rostopic hz /Ego_topic
```

![alt text](attachments/image-9.png)
- rviz에서도 잘 재생됨
- 일단 내 local에서는 bag 끄고 morai 기반으로 하자(topic 겹치니까)
