# ego - ros bridge 만들기
## 왜 fast-lio - ros bridge가 아니라 ego - ros를 먼저 만드냐?
- 판단 팀에서 개발을 시작할 수 있도록 하는 목적
- 바로 fast-lio를 붙이면 (아마) 값이 많이 튈 확률이 높음
- 그러니 GT인 /Ego_tipic을 보고 그걸로 판단 단에서 주행 로직 개발 가능
- 즉, 이후 목적은 저 Ego_topic을 fast-lio 기반 loc 결과로 교체한 버전의 bridge를 만드는 것
  - 당연하겠지만 Ego_topic은 GT(정답)이니까 대회에선 거의 무조건 제공 X
## bridge의 목적
- 본격적으로 bridge 제작 착수해서, 우리 스택들이 morai에 종속되지 않게 하고
- 판단 단에서 우리 내부 topic명만 보고 개발 가능하게 만들기
```
입력:
  /Ego_topic
  morai_msgs/EgoVehicleStatus

출력:
  /localization/odom
  nav_msgs/Odometry
```


### config 만들기
- fast lio 안에 `config/velodyne.yaml`가 있긴 한데 서브모듈로 관리할 거니까 우리 쪽에 config 따로 만들기
```bash
mkdir -p ~/AjouNice2026/configs/localization
gedit ~/AjouNice2026/configs/localization/fast_lio_morai_velodyne.yaml
```
- 나중에 만들자 일단 bridge부터

### fast-lio launch wrapper 만들기
- config 우리쪽 꺼 쓰기로 했으니까 그거에 맞춰서 launch도 새로 만들어줘야 함
- 나중에 일단 ego부터 bridge 만들기

### morai_ego -> 우리 ros odem bridge 만들기
- `~/AjouNice2026/base_ws/src/morai_ros_bridge/morai_ego_to_odom.py`
- notes:
  - morai에서 heading 값이 deg로 들어왔었음(ex. 58.123) -> 이거 ros에선 rad 쓰니까 변환해줘야함.

- `~/AjouNice2026/base_ws/src/morai_ros_bridge/launch/ego_to_odom.launch`



### bridge test

- rosbag로 bridge 테스트해보자
```bash
# 터미널1
cd ~/AjouNice2026/rosbags
rosparam set use_sim_time true # ROS 노드들이 컴퓨터 현재 시간이 아니라 /clock을 기준 시간으로 사용함
# 이거 우리 rosbag에 /clock이 안 담겼었음 그래서 clock도 새로 만들어서 publish해주게 함
rosbag play morai_local_valid_ego_sensors.bag --clock --loop # loop: 반복
```
```bash
# 터미널2
roslaunch morai_ros_bridge ego_to_odom.launch
```

```bash
# 터미널3
$ rostopic list | grep localization
/localization/odom

$ rostopic hz /localization/odom
subscribed to [/localization/odom]
WARNING: may be using simulated time
average rate: 12.524
        min: 0.010s max: 0.135s std dev: 0.04129s window: 13
average rate: 12.522
        min: 0.000s max: 0.135s std dev: 0.04563s window: 25
average rate: 12.178
        min: 0.000s max: 0.135s std dev: 0.04770s window: 37
average rate: 12.935
        min: 0.041s max: 0.114s std dev: 0.03659s window: 3
average rate: 10.964
        min: 0.020s max: 0.134s std dev: 0.03442s window: 14
average rate: 9.884
        min: 0.020s max: 0.174s std dev: 0.03426s window: 23
average rate: 10.687
        min: 0.010s max: 0.175s std dev: 0.04408s window: 34

$ rostopic echo -n 1 /localization/odom
header: 
  seq: 83
  stamp: 
    secs: 1780824019
    nsecs: 820000000
  frame_id: "map"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: -13.645261764526367
      y: -223.20413208007812
      z: 28.821598052978516
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.5179688743105172
      w: 0.8553994653058276
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.021008485928177834
      y: 6.491318345069885e-07
      z: 1.8019623894360848e-05
    angular: 
      x: -0.008628355339169502
      y: -0.00030235349549911916
      z: 0.0003966026706621051
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
- 잘 됐음
- 이제 rviz 테스트

```bash
# 터미널3
rviz
# Fixed Frame: 3D LiDAR-1
# TF, /localization/odom, lidar3D
```
![alt text](attachments/image-10.png)
- odem의 화살표가 제대로 차량 앞쪽으로 잘 나와줬음.

### bringup에 통합
- `~/AjouNice2026/base_ws/src/bringup/launch/localization_baseline.launch`
- 이거 해줘야 나중에 fast-lio도 그렇고 저 launch 파일에 하나씩 추가하면서 한 번에 launch 가능