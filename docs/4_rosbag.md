# morai - rosbag 만들기
## 계획
- 무상실에 매번 morai를 키는 게 어려움
- rosbag으로 morai 센서만 담아놓고 배포하면, 그걸 local에서 ros만 구동시켜서 테스트 및 개발이 가능할지도?

### rosbag 제작
```bash
bash ~/AjouNice2026/scripts/rosbag_maker.sh
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
topics:      /Ego_topic               109 msgs    : morai_msgs/EgoVehicleStatus
             /Object_topic            114 msgs    : morai_msgs/ObjectStatusList
             /gps                      43 msgs    : morai_msgs/GPSMessage      
             /image_jpeg/compressed   104 msgs    : sensor_msgs/CompressedImage
             /sensor/imu              107 msgs    : sensor_msgs/Imu
             /lidar3D                  72 msgs    : sensor_msgs/PointCloud2    
```

현재 record 스크립트는 `/tf`, `/tf_static`, `/morai/tf_raw`를 기록하지 않는다.
TF는 bag 재생 시점에 `maptf_morai_bag.launch` 또는 `maptf.launch`로 다시 만든다.

- 근데 /clock은 왜 없지
  - real-time mode 켜져 있는데 흠...
  - `rostopic hz /clock` 해 봐도 `no new messages` 나오네 -> 일단 나중에 생각(당장 없어도 되긴 하니까..)


### rosbag 테스트
```bash
# 재생
rosbag play morai_local_valid_ego_sensors.bag --clock
rostopic hz /Ego_topic
```

Loop replay를 할 때도 bag의 `/tf`, `/tf_static`는 같이 재생하지 않는다.
`rosbag play -l`은 bag 끝에서 `/clock`이 다시 처음 시간으로 되감기기 때문에,
TF buffer, RViz, localization 노드가 이전 loop의 미래 timestamp/state를 들고 있으면
정합이 깨져 보일 수 있다. 특히 FAST-LIO처럼 내부 map/PCD 상태를 누적하는 노드는
loop 경계에서 자동 초기화되지 않으므로 반복 재생 대신 한 번씩 재시작해서 확인하는
쪽이 안전하다.

현재 localization 전 단계에서는 `/Ego_topic`을 임시 GT localization으로 사용한다.
bag 안에는 `/tf`를 넣지 않고, 별도 터미널에서 아래 launch를 같이 띄운다.

```bash
roslaunch bringup maptf_morai_bag.launch
```

이 launch는 rosbag의 `/Ego_topic`으로 `/localization/odom`과
`morai_map -> base_link` TF를 만들고, `map -> morai_map` static TF까지 같이
publish한다. GPS-only bag은 `maptf.launch`를 사용한다.

![alt text](attachments/image-9.png)
- rviz에서도 잘 재생됨
- 일단 내 local에서는 bag 끄고 morai 기반으로 하자(topic 겹치니까)
