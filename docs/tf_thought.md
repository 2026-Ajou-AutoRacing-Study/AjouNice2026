```bash
fast lio publish frame, tf:
header.frame_id: camera_init
child_frame_id: body
tf: camera_init -> body
```

- 위처럼 fast lio는, fast lio 초기 시작 지점을 (0, 0)으로 두고, 거기서부터의 imu 위치를 body로 둬서 pose를 내보냄.

# Perception에서 필요한 것:

1. 기본 사용에는 fast lio와 아무런 연관이 x. 왜냐하면 입력받는 건 lidar raw data지, fast lio 결과가 아니기 때문.
2. ~~만약 bevfusion의 multi sweep(Temporal modeling)을 위해 현재 차량 pose를 써야 한다면 planning에서처럼 map → base_link 변환이 필요해 보임.~~
    1. → 아님 local odometry만으로도 가

# Planning에서 필요한 것:

- `map → base_link` 로 변환된 pose가 필요
    - 이는 tf 행렬곱으로 분해됨
    - `map -> morai_map -> camera_init -> body -> base_link`

```bash
planning을 위해 내보내줘야 하는 토픽 frame 형태:
header.frame_id = map
child_frame_id = base_link
```

## Localization provider별 공통 인터페이스

- FAST-LIO mode: `roslaunch bringup localization_fast_lio.launch`
  - TF: `map -> morai_map -> camera_init -> body -> base_link`
- MORAI GT mode: `roslaunch bringup localization_morai_gt.launch`
  - TF: `map -> morai_map -> base_link`
- 두 mode 모두 Planning에 동일한 출력을 제공함.
  - TF lookup: `map -> base_link`
  - topic: `/localization/kinematic_state`
- 두 localization launch는 동시에 실행하지 않음.

```bash
즉,
tf: map -> base_link
=
map -> morai_map
* morai_map -> camera_init
* camera_init -> body
* body -> base_link

Fast-LIO 시작 시점의 pose를 저장
같은 시점의 morai_map에서의 초기 pose를 측정
그 차이를 morai_map -> camera_init 초기 정렬값으로 고정
```

## camera_init -> body

- fast lio가 제공함

## body → base_link

```bash
base_link:
  차량 기준점

body / imu:
  IMU 장착 위치
```

- 이라서, 저 둘 사이에 static tf를 만들어줘야 할 수 있음.
    - morai의 base_link가 어디 기준인지 확인할 것.
    - eval에선 단순 position 비교만 하니 필요 x
- 현재 MORAI Sensor_1/2 config의 IMU 위치는 `[1.528, 0.0, 0.200]`이고 rotation은 없다. 따라서 역변환인 `[-1.528, 0.0, -0.200]`가 `body -> base_link` TF가 됨.

## morai_map -> camera_init

- fast lio는 시작 지점을 camera_init 원점으로 잡음.
    - 그래서 시작 시점의 차량 morai_map 기준 pose를 알면, 그 pose를 기준으로 camera_init을 morai_map에 붙이는 TF를 계산 가능
- 이를 계산하기 위해선 초기
    1. morai_map에서의 초기 position과 yaw
    2. camera_init 에서의 초기 position과 yaw
- 가 필요함.
    1. 1번은 아래의 방법들로 **직접 취득해야 함**.
    2. 2번은 (0, 0) / yaw = 0 degree 임. 즉 이미 주어진 값.

## TF 변환 코드 구현 예시

```bash
선택지1)
output topic: 
/localization/odom_map # planning에서 받아 사용할 map pose

선택지2)
output topic: 
/tf # morai_map -> camera_init
왜냐하면 기존 map -> morai_map TF와 morai_map -> camera_init TF가 있으면 planning에서 실시간으로 map → base_link 복원 가능
```

    

## 정리

- planning을 위해 우리가 구해야 하는 것:
    - morai_map에서의 차량 초기 position과 yaw
    - 고정된 `map -> morai_map` 정렬을 통해 최종 map pose를 복원

# 1. 초기 position

## 1) 대회에서 시작 position을 줄 경우

- 그대로 사용

## 2) GPS 측정값 사용

- gps 좌표계 → map 좌표계 변환이 필요할 수 있음.

## 3) NDT Scan matching을 사용

- 이러면 map-based localization이 들어가는 것이므로, fast lio로 local odometry를 만들고 map에 붙이려는 기존 계획과 역할이 겹칠 수 있음.
- 만약 이 방식을 쓴다면, 그리고 pcd map file이 제공되지 않는다면, 우리가 LiDAR를 달고 주행하면서 pseudo map을 만들어야 함.

# 2. 초기 head yaw 각도

## 1) 대회에서 초기 yaw를 줄 경우

- 그대로 사용

## 2) 두 시점에서의 GPS pose를 가지고 각도를 계산해 head yaw로 사용

- 그러나 그냥 시작 두 프레임 잡으면, 정지 상태에서는 불안정함.
    - 따라서 일정 거리 이상 움직임이 발생한 뒤에, 그 움직임의 직선 거리 yaw를 사용하는 식으로 가능

## 3) NDT Scan matching을 사용

- 위와 동일

# 3) 이후 global correction

- 초기 `morai_map -> camera_init` 의 오차 + fast lio drift 등의 이유로, 초기 이후에도 보정이 필요할 수 있음
- 우리가 궁극적으로 목표하는 https://app.notion.com/p/395b49145fb98026b2c9ed070079412b?pvs=25#395b49145fb980ce82eafeb43588610b 가 gnss로 이러한 global correction을 하는 걸로 보임.
    - 해당 gnss + wheel fusion을 먼저 적용해 보는 게 좋을 듯.
    - 또는 NDT Scan matching으로도 가능
- GNSS로 global correction을 한다고 생각하면 될 듯함. gnss로 잘 안 되면 다른 global correction 방법을 찾아보는 것보단 local odometry를 잘 다듬고, 시작 위치 pose matching을 더 신경쓰는 게 맞을 듯.

## 대회에서 시작 position과 yaw를 줄까?

- 줄 수도 있지만, 안 준다고 생각하고 가는 게 맞아 보임.
    - 이번주 주말 기술세미나 때 질문해보는 게 좋아 보인다.
- +) 센서에 노이즈 넣는지도 질문
    - 노이즈 넣어서 테스트 gnss
    - 각 센서별 노이즈 정도 질문

## 정리

- 시작 pose matching 초기 개발 방향:
    - position: gnss
    - yaw: gnss 두 점 잇기
    - global correction: gnss (+ wheel) fusion으로 drift 제어
