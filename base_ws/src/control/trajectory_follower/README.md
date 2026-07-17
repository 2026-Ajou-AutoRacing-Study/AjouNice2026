# trajectory_follower

## 실행 준비

```bash
cd /home/jihun/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps trajectory_follower
source devel/setup.bash
```

`--only-pkg-with-deps`는 package whitelist를 유지한다. 전체 workspace를 다시 구성할 때는
`catkin_make -DCATKIN_WHITELIST_PACKAGES=""`로 해제한다.

실행 전 다음 입력이 실제로 publish되고 있어야 한다.

```bash
rostopic hz /control/trajectory
rostopic hz /localization/kinematic_state
```

- `/control/trajectory`: `trajectory_generator` 또는 호환 planner가 제공한다.
- `/localization/kinematic_state`: 선택한 localization provider가 `map` 기준
  `base_link` pose와 twist를 제공한다. MORAI GT에서는 `localization_morai_gt` 또는
  `maptf_morai_deg_correction.launch`를 실행한다.

## 사용법

### 1. Follower 실행

```bash
roslaunch trajectory_follower pp_pi.launch
```

등록된 단축 명령은 다음과 같다.

```bash
traj_fol
```

튜닝 파일을 바꾸어 실행할 수도 있다.

```bash
roslaunch trajectory_follower pp_pi.launch \
  common_config:=$(rospack find trajectory_follower)/config/trajectory_follower.yaml \
  pp_config:=$(rospack find trajectory_follower)/config/purepursuit.yaml \
  pi_config:=$(rospack find trajectory_follower)/config/longi_pi.yaml
```

### 2. MORAI UDP 제어 연결

`/ctrl_cmd`를 MORAI로 보내려면 UDP interface의 control sender가 함께 실행되어야 한다.
이미 실행 중인 UDP interface가 있다면 중복 실행하지 않는다.

```bash
roslaunch sensor_interface morai_udp_interface.launch \
  enable_sensors:=false \
  enable_lidar:=false \
  enable_ego_network:=true \
  enable_control:=true
```

### 3. 동작 확인

```bash
rostopic hz /control/command/steering
rostopic hz /ctrl_cmd
rostopic echo -n 1 /ctrl_cmd
rostopic info /ctrl_cmd
```

정상 출력은 `longlCmdType: 1`이며 `accel`, `brake`, `steering`을 사용한다.
`/ctrl_cmd` publisher는 `longi_pi` 하나만 있어야 한다.

## 기능과 역할

- `purepursuit`: trajectory와 공통 kinematic state로 조향각을 계산해
  `/control/command/steering`으로 publish한다.
- `longi_pi`: 목표 속도와 kinematic state 속도로 accel/brake를 계산하고 조향 명령을 결합해
  `/ctrl_cmd`를 단독 publish한다.
- 입력 timeout, 잘못된 값, 마지막 point 도달 시 type 1 정지 명령을 출력한다.

| 구분 | Topic | Type |
| --- | --- | --- |
| 입력 | `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| 입력 | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| 내부 출력 | `/control/command/steering` | `std_msgs/Float64` |
| 최종 출력 | `/ctrl_cmd` | `morai_msgs/CtrlCmd` |

튜닝은 `config/purepursuit.yaml`과 `config/longi_pi.yaml`, 공통 안전 설정은
`config/trajectory_follower.yaml`에서 변경한다. 세부 제어 규칙은
[trajectroy_follower.md](trajectroy_follower.md)를 참고한다.
