# ajou_trajectory_recorder

MORAI SIM에서 수동으로 주행한 ego 차량 궤적을 ROS1 Noetic에서 기록하고, CSV로 저장한 뒤 다시 기준 궤적으로 publish하는 패키지입니다. 이 패키지는 제어기를 구현하지 않습니다. MORAI 연결 확인, 수동 주행 궤적 기록, 기록된 궤적 publish, RViz 시각화까지만 담당합니다.

## 개념

- MORAI SIM은 시뮬레이터 역할만 합니다.
- MORAI가 ego 차량 상태를 ROS의 `/Ego_topic`으로 publish합니다.
- ROS는 `/Ego_topic`을 구독해서 차량 위치, heading, 속도를 CSV로 기록합니다.
- 기록된 CSV는 이후 제어 검증용 기준 궤적(reference trajectory)으로 다시 publish됩니다.

## 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x ~/catkin_ws/src/ajou_trajectory_recorder/scripts/*.py
```

## ROS-MORAI 연결 확인 명령

```bash
rostopic list
rostopic echo /Ego_topic
rostopic hz /Ego_topic
rostopic info /Ego_topic
```

`/Ego_topic` 타입은 `morai_msgs/EgoVehicleStatus`여야 합니다.

## 연결 체크 노드 실행

```bash
roslaunch ajou_trajectory_recorder check_connection.launch
```

이 노드는 `/Ego_topic`을 구독해서 현재 위치, heading/yaw, 속도를 출력합니다. 2초 이상 메시지가 들어오지 않으면 경고를 출력합니다. `/ctrl_cmd` 토픽 존재 여부도 ROS master에서 확인하지만, 제어 명령은 publish하지 않습니다.

## 수동 주행 궤적 기록

1. `roscore`를 실행합니다.
2. MORAI SIM을 실행합니다.
3. MORAI가 ROS master로 ego status를 publish하도록 네트워크를 설정합니다.
4. ego 차량 제어 모드를 Manual Keyboard 또는 Manual GameWheel로 설정합니다.
5. 기록 노드를 실행합니다.

```bash
roslaunch ajou_trajectory_recorder record_trajectory.launch
```

CSV 경로를 바꿔 기록하려면:

```bash
roslaunch ajou_trajectory_recorder record_trajectory.launch output_csv:=/absolute/path/to/recorded_trajectory.csv
```

6. MORAI에서 수동으로 주행합니다.
7. 주행이 끝나면 `Ctrl+C`로 launch를 종료합니다.
8. CSV 파일을 확인합니다.

```bash
head -n 5 ~/catkin_ws/src/ajou_trajectory_recorder/data/recorded_trajectory.csv
```

기본 CSV 컬럼은 다음과 같습니다.

```text
time,x,y,z,yaw,velocity
```

## 기록된 궤적 publish

```bash
roslaunch ajou_trajectory_recorder publish_trajectory.launch
```

CSV 경로를 지정해서 publish하려면:

```bash
roslaunch ajou_trajectory_recorder publish_trajectory.launch input_csv:=/absolute/path/to/recorded_trajectory.csv
```

publish되는 토픽:

- `/recorded_trajectory_path`: `nav_msgs/Path`
- `/recorded_trajectory/target_speed`: `std_msgs/Float32`
- `/recorded_trajectory_marker`: `visualization_msgs/MarkerArray`

확인:

```bash
rostopic echo /recorded_trajectory_path
rostopic hz /recorded_trajectory_path
```

## RViz 확인

RViz에서 다음 display를 추가합니다.

- Path: `/recorded_trajectory_path`
- Marker: `/recorded_trajectory_marker`

Fixed Frame은 `map`으로 설정합니다.

## 주요 파라미터

`config/recorder_params.yaml`에서 수정합니다.

```yaml
ego_topic: /Ego_topic
output_csv: data/recorded_trajectory.csv
input_csv: data/recorded_trajectory.csv
record_rate: 10.0
min_distance: 0.2
min_speed_to_record: 0.1
path_topic: /recorded_trajectory_path
target_speed_topic: /recorded_trajectory/target_speed
frame_id: map
publish_rate: 10.0
default_target_speed: 5.0
marker_topic: /recorded_trajectory_marker
```

상대 경로인 `data/recorded_trajectory.csv`는 이 패키지 루트 기준으로 해석됩니다.

## 문제 해결

- `/Ego_topic`이 보이지 않으면 MORAI 네트워크 설정과 ROS master 주소를 확인합니다.
- `/Ego_topic`은 보이지만 데이터가 들어오지 않으면 `ROS_IP`, `ROS_MASTER_URI`, WSL2 네트워크 설정을 확인합니다.
- CSV가 header만 있고 비어 있으면 차량 속도와 `min_speed_to_record` 값을 확인합니다.
- 너무 적은 점만 저장되면 `min_distance` 값을 낮춥니다.
- RViz에서 경로가 보이지 않으면 Fixed Frame이 `map`인지 확인합니다.

## 최종 사용 흐름

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
roslaunch ajou_trajectory_recorder check_connection.launch
```

Terminal 3:

```bash
roslaunch ajou_trajectory_recorder record_trajectory.launch
```

수동 주행 후 `Ctrl+C`로 기록을 종료합니다.

그 다음:

```bash
roslaunch ajou_trajectory_recorder publish_trajectory.launch
```

검증:

```bash
rostopic echo /recorded_trajectory_path
rostopic hz /recorded_trajectory_path
rviz
```

## 제어기까지 연결

기록된 CSV를 제어기에 연결하려면 `ajou_control`의 통합 launch를 사용합니다.

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0
```

검증 CSV까지 같이 남기려면:

```bash
roslaunch ajou_control recorded_path_stack.launch target_speed_mps:=3.0 enable_validator:=true
```

처음에는 `target_speed_mps:=3.0`처럼 낮은 고정 속도로 시작하고, 추종이 안정된 뒤에만 `use_target_speed_topic:=true`를 켜서 기록 속도 평균을 사용합니다.
