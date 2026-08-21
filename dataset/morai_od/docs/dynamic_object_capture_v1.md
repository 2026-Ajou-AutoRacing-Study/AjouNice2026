# MORAI Dynamic Object Capture V1

## 결정

학습 데이터 본 수집은 rosbag이 아니라 MORAI `/SaveSensorData`를 사용한다.
한 요청에서 생성된 4-camera RGB·Depth·Semantic·Instance와 LiDAR 파일은 공통
MORAI timestamp stem으로 묶고, 모든 필수 artifact가 있을 때만 frame으로 쓴다.

rosbag은 온라인 재현, latency, drop, tracking/prediction 평가에만 사용한다.
학습 수집에 쓰지 않는 이유는 센서 재동기화, 대용량 저장, 별도 추출과
Semantic/Instance GT 처리 비용 때문이다.

## 대상과 split

| family | smoke 수 | 동작 |
|---|---:|---|
| `roundabout_sample_spawn` | 2 | 공식 로터리 정지 spawn 이후 16~22 km/h 가속. |
| `roundabout_circulation` | 4 | 16~22 km/h 목표속도로 시작하는 안정 순환 대리 장면. |
| `highway_merge_sample_spawn` | 3 | 공식 40·50·60 km/h spawn 직후 가속을 차로별로 분리. |
| `highway_merge_cruise` | 6 | 공식 세 차로에서 목표속도로 시작하는 합류 순항 대리 장면. |
| `highway_cruise_overtake` | 9 | 고주로 Ego 3차로 × NPC 3차로 조합. |
| `crosswalk_pedestrian` | 4 | 공식 path의 방향과 속도를 변형. |
| `static_obstacle` | 3 | 공식 장애물 구간 정적 obstacle. |
| `gps_shadow_obstacle` | 3 | GPS 음영 OBB 내부 도로 정적 obstacle. |

GPS 음영 끼어들기 차량은 검증된 MORAI spawn/waypoint template이 없어 아직
생성하지 않는다. 수동 저장 template을 확보한 뒤 family로 추가한다.

위 count는 각 구현 경로를 고르게 live 검증하기 위한 smoke quota이며 최종 학습
비율이 아니다. 본 수집에서는 아래 motion-phase 권장 비율로 다시 sampling한다.

frame이 아니라 scenario clip 전체를 train/val/test 중 하나에 배정한다.

현재 V1 미션 family는 의도적으로 class-pure하게 유지한다.

- 로터리·고주로 vehicle family에는 동적 차량만 배치한다.
- `crosswalk_pedestrian`: 주 동적 보행자만 배치한다.
- `static_obstacle`, `gps_shadow_obstacle`: 정적 obstacle만 배치한다.
- 따라서 보행자 미션이 아닌 위치에 random pedestrian을 추가하지 않는다.

첫 10,000-frame 학습에서는 `structured_random_dynamic`을 보류한다. 미션 family
자체를 무작위 혼합으로 바꾸지 않고 로터리·고주로·횡단보도의 pure한 상호작용을
유지한다. 과거 Batch001의 미션 50%/정적 random 35%/hard 15%와 정적 내부 obstacle
40%는 1,500-frame single-frame 인수용 구성으로만 보존한다. 신규 temporal V2의
production 기준은 `temporal_sequence_capture_v2_plan.md`의 동적 차량 45%, 동적
보행자 15%, 정적 temporal 25%, hard temporal 15%와 정적 내부 obstacle-focused
70%다. Obstacle 종류 부족은 동적 mission clip에 임의 배치하지 않고 asset·형상·거리
quota가 있는 정적/hard family로 해결한다. structured dynamic은 첫 학습에서 일반
도로 동적 운동이나 문맥 shortcut 문제가 확인될 때만 별도 pilot으로 재검토한다.

### 고주로 장면 분리

`highway_merge_sample_spawn`은 공식 샘플의 NPC spawn 직후 가속을 수집한다.
`highway_merge_cruise`는 같은 위치·차로에서 NPC를 목표속도로 시작시켜 실제 합류
시점의 순항 상태를 더 많이 수집한다.
`highway_cruise_overtake`는 합류 이후 다차선 주행을 별도로 수집한다.

고주로 합류에서 Ego도 정지 출발하지 않는다. 두 merge family 모두 Ego를 global
route의 합류 접근부에 배치하고 20~35 km/h 초기·목표속도를 부여한다. 목표속도의
80%에 도달하기 전에는 캡처하지 않는다. 즉 spawn family와 cruise family의 비교는
Ego 출발 방식이 아니라 NPC motion phase만 다르다.

T&P 분석에서는 spawn 가속이 all-motion p90/p95 tail을 만들었지만, Ego와 실제로
상호작용할 때에는 NPC가 대부분 순항에 도달했다. 따라서 prediction production은
CV를 유지하고 spawn 가속은 주 분포가 아닌 별도 hard case로 취급한다. 반면 OD,
velocity regression과 향후 temporal OD에는 저속→가속 전이, rolling LiDAR 왜곡,
거리·point 밀도 변화가 유용하므로 제거하지 않는다.

- `highway_merge_sample_spawn`은 고주로 vehicle clip의 15~20% 이하를 권장한다.
- 10,000-frame 전체 데이터에서는 약 3~5%, 즉 300~500 frame을 상한으로 삼는다.
- 나머지 고주로 vehicle 표본은 합류 순항과 합류 이후 following·병렬·추월에 둔다.
- metadata의 `motion_phase`와 `sampling_role`로 두 분포를 분리하여 sampler와
  평가기가 독립적으로 선택할 수 있게 한다.

다만 Batch001 live production에서는 이 family의 Ego 초기 pose가 capture
route에서 1.6--15.8 m 벗어나 hard clip 18개 중 11개가 실패했다. 따라서 생성
실험과 향후 경로 수정 대상으로는 유지하지만 현재 production hard quota에서는
제외한다. Batch001 hard dynamic은 검증된 `roundabout_sample_spawn` 12개와
`crosswalk_pedestrian` 6개로 교체했고 216 frame 모두 통과했다.

- 공식 40·50·60 km/h spawn이 있는 세 MGeo 차선을 모두 사용한다.
- Ego 차선 3개와 NPC 차선 3개를 조합해 총 9개 smoke scene을 만든다.
- 같은 차선에서는 Ego와 NPC 속도를 같게 두어 안정 following을 수집한다.
- 다른 차선에서는 Ego가 NPC보다 8~15 km/h 빠르게 주행해 접근·병렬·추월
  상대 위치를 수집한다.
- NPC의 `Min/MaxSpawnVelocity_Custom`과
  `Min/MaxDesiredVelocity_Custom`을 동일하게 설정해 spawn 가속을 제거한다.
- Ego도 시나리오 초기속도와 외부 제어 목표속도를 동일하게 요청한다. 실제
  런처가 초기속도를 복원하지 않더라도 목표속도의 80%에 도달하기 전에는
  SaveSensorData를 요청하지 않는다.
- 각 scene은 선택된 MGeo 차선 중심선과 자연스러운 후속 link로 만든 전용
  waypoint를 사용한다. 따라서 하나의 대회 global path 차선에 고정되지 않는다.

현재 방식은 Ego가 실제로 차선을 변경하는 maneuver를 생성하지 않는다. 서로
다른 차선에서 더 빠른 Ego가 NPC를 지나치는 상대 운동을 먼저 확보하는 단계다.
실제 lane-change trajectory는 별도의 검증된 경로 family로 추가한다.

### 로터리 spawn과 상호작용 분리

로터리도 공식 sample spawn을 제거하지 않되 주 분포로 사용하지 않는다.

- `roundabout_sample_spawn`: 정지에 가까운 spawn 이후 약 16~22 km/h로 가속하는
  transition을 수집한다. 로터리 vehicle clip의 10~15%만 권장한다.
- `roundabout_circulation`: NPC spawn velocity와 target velocity를 같게 설정한
  안정 순환 대리 장면이다. 단일-frame OD와 velocity regression의 주 표본이다.
- 향후 `roundabout_merge_interaction`: 순환 중인 NPC가 Ego 합류점을 통과하는
  시점을 의도적으로 맞춘다.
- 향후 `roundabout_exit_branch`: 순환 유지와 출구 진출을 분리한다.

마지막 두 family는 현재 공식 spawn JSON만으로 정확한 상호작용 시점을 보장할 수
없어 아직 생성하지 않는다. `roundabout_circulation`도 목표속도 시작을 보장하는
대리 장면이지, 이미 로터리 내부를 충분히 주행한 NPC라고 단정하지 않는다. live
smoke에서 spawn 이후 위치와 Ego 합류 지점 도달시간을 확인한 뒤 MGeo 기반 시작
arc 또는 warm-up 정책으로 승격한다.

#### `dhc002` 첫 live smoke 결과

2026-08-20 첫 실행에서는 시나리오의 Ego 초기속도 약 56 km/h가 실제로
적용되었다. 기존 준비 검사는 Ego가 초기 좌표에 정지해 있다고 가정했기 때문에,
15초 동안 Ego가 약 157 m 이동한 뒤 `Ego position error`로 실패했다. 이 결과는
초기속도 필드가 무시된 것이 아니라 정상 적용됐음을 뜻한다.

같은 실행에서 장면 override의 `ego_target_speed_kph`가 수집기 내부
`speed_kph`로 연결되지 않아 제어 목표가 5.4 km/h로 남는 mapping bug도
확인했다. 다음과 같이 수정했다.

- 고주로 순항 scene은 최초 절대좌표 대신 선택한 MGeo route 위에 있는지로
  준비 상태를 확인한다. 움직이는 Ego가 정확한 초기 좌표를 벗어나도 정상이다.
- `ego_target_speed_kph`를 실제 CtrlCmd 목표 `speed_kph`에 연결한다.
- capture 시작 전 Ego 속도 80% gate와 최대 130 m 이동 제한은 유지한다.
- 일반 저속·정적 scene은 기존 fixed-pose 검증을 그대로 사용한다.

수정 후 live smoke에서 `Scene motion policy`가 약 56 km/h로 표시되고,
시나리오가 즉시 검증되며, 첫 trigger 이전 속도 gate를 통과하는지 확인해야 한다.

## 수집 정책

- 모드: `timed-grouped`.
- frame 수: 장면당 12개.
- 요청 간격: 최소 0.5초, 즉 nominal 2 Hz keyframe.
- 첫 frame gate: Ego가 경로를 0.30 m 이상 실제로 진행한 뒤 수집을 시작한다.
- frame 간 gate: 0.5초 경과와 직전 frame 대비 0.20 m 이상 진행을 모두 요구한다.
- motion stall: 5초 안에 진행 gate를 만족하지 못하면 정지 중인 중복 frame을
  저장하지 않고 해당 scene을 실패 처리한다.
- nominal clip span: 5.5초.
- actor 확인 후 warm-up: 0.2초.
- 횡단 보행자는 생성된 Ego 초기 거리보다 `activeDistance`를 5 m 크게 두고,
  `/Object_topic`에서 실제 속도 또는 위치 변화를 확인한 뒤 clip을 시작한다.
- Ego 속도: 5.4 km/h, 곡선에서는 3.6 km/h까지 감속.
- 2 Hz 안정성을 확인한 뒤 필요하면 3~5 Hz를 별도 시험한다.

2 Hz는 센서 원래 발행률이 아니라 학습용 keyframe 간격이다. MORAI가 고주기
SaveSensorData를 안정적으로 처리하는지 보기 전에는 15~30 Hz를 요구하지 않는다.
곡선 감속이나 짧은 제어 지연으로 진행 gate가 늦게 충족되면 실제 frame 간격은
0.5초보다 길어질 수 있다. temporal 학습은 고정 0.5초로 가정하지 않고 frame
metadata의 실제 timestamp를 사용한다.

### Dynamic keyframe 간격과 temporal 3-sweep.

현재 동적 생성기의 설정값은 장면당 12 frame, `capture_period_sec=0.5`초다. 이 값은
SaveSensorData 요청의 wall/ROS-time 최소 간격이며 LiDAR 원래 발행률이나 고정된
simulator-time 간격이 아니다. 개인용 MORAI가 real time보다 느리게 실행된 기존
1,500-frame 수집에서는 인접 저장 frame의 복원된 simulator-time lag p50이 약
0.147초, 두 frame 전 lag p50이 약 0.293초였다.

따라서 temporal PointPainting의 `0.15초`는 설계에서 강제한 촬영 주기가 아니라
기존 데이터에서 관측된 결과다. 3-sweep early temporal 입력은 다음처럼 동작한다.

1. 현재 frame과 같은 clip의 직전 두 저장 frame을 선택한다.
2. 각 frame의 실제 `time_lag_sec`를 point feature로 추가한다.
3. 저장된 Ego pose로 과거 sweep 전체를 현재 LiDAR 좌표계에 정합한다.
4. 정합한 세 point cloud를 하나로 합쳐 PointPillars에 입력한다.

모델은 정확히 0.15초 간격만 받을 필요는 없지만, 지나치게 긴 간격은 학습 분포를
바꾸고 association을 어렵게 한다. 현재 정적 `rolling-grouped`는 5.4 km/h Ego가
3 m씩 이동한 뒤 촬영하므로 인접 frame이 물리적으로 약 2초 떨어진다. 이 3장은
temporal 3-sweep이 아니라 같은 배치를 서로 다른 거리에서 본 single-frame 표본
3장으로 취급한다.

정적 객체를 temporal 학습에도 포함하려면 기존 거리 기반 3장을 대체하지 않고,
별도의 짧은 keyframe burst와 frame별 timestamp/Ego pose sidecar를 추가한다. 정적
객체의 world velocity target은 계속 `(0,0)`이며, 센서에서 보이는 상대 이동은 Ego
pose 정합으로 처리한다.

### 1,500-frame production의 정적 구성.

Primary 1,500 frame 중 일반 정적은 525 frame, hard static은 15 frame으로 전체
정적 비율은 36%다. 일반 정적은 175 scene을 장면당 3 frame으로 수집한다.

| 정적 scene type | Scene | Frame | 일반 정적 내 비율 | 배치 내용 |
| --- | ---: | ---: | ---: | --- |
| `realistic_mixed` | 61 | 183 | 34.9% | NPC 2--5, pedestrian 1--4, obstacle 1--3. |
| `vehicle_centric` | 18 | 54 | 10.3% | NPC 5--8, pedestrian 0--2, obstacle 0--2. |
| `pedestrian_centric` | 17 | 51 | 9.7% | NPC 0--2, pedestrian 6--10, obstacle 0--2. |
| `obstacle_centric` | 70 | 210 | 40.0% | NPC 0--2, pedestrian 0--2, obstacle 5--7. |
| `sparse_empty_background` | 9 | 27 | 5.1% | NPC 0--1, pedestrian 0, obstacle 0. |

Hard static 5 scene, 15 frame은 camera seam/occlusion 2 scene, dense obstacle
occlusion 2 scene, sparse hard-negative 1 scene으로 구성한다. 예비 static 50 scene,
150 frame은 실패 교체용이며 primary 1,500 frame에는 자동으로 포함하지 않는다.

## 출력과 metadata

```text
<capture-dir>/
├── CAMERA_1..4/
├── LIDAR_5/
├── dynamic_metadata/
│   ├── drs001.frames.jsonl
│   ├── drs001.timing.jsonl
│   └── drs001.clip.json
├── scenes.jsonl
├── failures.jsonl
└── summary.json
```

각 frame에는 다음을 저장한다.

- clip/scene/frame ID, split, mission family.
- 요청 wall/monotonic time, schedule lateness, MORAI timestamp stem.
- 실제 센서 파일 상대 경로와 artifact 완전성.
- Ego position, heading, raw velocity/acceleration과 control 상태.
- actor class, stable ID, asset, position, heading, size.
- MORAI raw velocity/acceleration.
- clip 내 중앙차분 velocity, acceleration, yaw rate와 motion phase.
- Ego/Object callback age와 actor 존재 여부.
- 가장 가까운 `/clock`, Ego, Object, 선택적 sensor-header sample ID와 시간차.

`*.timing.jsonl`은 clip 앞뒤 0.25초를 포함한 연속 상태 이력을 저장한다.

- `/clock` simulator time.
- Ego pose·heading·velocity·acceleration.
- 모든 actor의 pose·heading·size·velocity·acceleration.
- ROS 센서 토픽이 존재할 때 LiDAR·4-camera header와 payload 형상 metadata.
- 이미지나 point cloud payload는 중복 저장하지 않는다.

원시 velocity는 임의 변환하지 않는다. 파생 m/s 값과 함께 남겨 launcher별 단위
차이를 감사한다. calibration은 `configs/sensors/Sensor_6.json`을 참조한다.

개인용 launcher가 real time보다 느리게 실행될 때 ROS header/wall time 차분은
물리 속도를 축소한다. 학습 변환 단계에서는 Ego/actor 위치 차이와 MORAI raw
physical speed로 scene simulator time을 복원한다. 기존 원본 metadata는 감사
가능성을 위해 그대로 보존한다. 구현과 A/B 결과는
`../../../perception/pointpainting/docs/temporal_sweep_alignment_azimuth_deskew_ab.md`
를 따른다.

## 생성·검증·설치

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_dynamic_object_capture_set.py \
  --group dynamic_object_v1_smoke

python3 dataset/morai_od/scripts/validate_dynamic_capture_set.py \
  dataset/morai_od/manifests/dynamic_object_v1_smoke.json

python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
  dataset/morai_od/manifests/dynamic_object_v1_smoke.json \
  --network-template \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/EgoNetwork/od_s13p6_186_MN.json"
```

설치기는 실행용 `manifests/dynamic_object_v1_smoke_runtime.json`도 만든다. 여기에는
sidecar SHA, 필수 ROS topic과 설치 경로가 포함된다.

## Live actor 검증

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

python3 dataset/morai_od/scripts/verify_dynamic_scene_live.py \
  --manifest dataset/morai_od/manifests/dynamic_object_v1_smoke.json \
  --scene drs001
```

확인 순서는 `drs001`, `drc001`, `dhm001`, `dmc001`, `dhc002`, `dpd001`,
`dso001`, `dgo001`이다. 각 spawn/cruise 쌍에서 실제 초기속도가 서로 다른지,
보행자 설정 speed와 실제 변위의 단위 관계도 확인한다.

2026-08-20 live 검증에서 `dhm001`, `dmc001`, `dhc002`, `dpd001`도 모두
통과했다. `dhm001`은 실제 0 속도에서 가속했고, `dmc001`은 첫 관측부터 약
40 raw speed로 움직였다. `dhc002`는 약 41-48 raw speed의 다차선 순항 차량,
`dpd001`은 약 13.8 m를 이동한 횡단 보행자를 확인했다. 네 장면의 실제
SaveSensorData 4-frame capture도 artifact/metadata validator에서 errors 0,
warnings 0으로 통과했다.

## Production batch 명령

시나리오 생성·검증·설치는 다음 한 명령으로 묶는다. 동적 component는 단순 JSON
복사로 끝내지 않고 검증된 Ego-only ROS sidecar와 runtime manifest를 생성한다.

```bash
python3 dataset/morai_od/scripts/generate_dynamic_dataset_batch.py \
  --batch-index 1 \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

전체 primary component를 순서대로 수집하려면 ROS 환경을 source한 뒤 실행한다.
예비 scene은 기본적으로 수집하지 않는다.

```bash
python3 dataset/morai_od/scripts/capture_dynamic_dataset_batch.py \
  --batch-index 1 \
  --capture-root /home/luv/morai_cap
```

한 component만 재개·검증할 때는 `--component mission`, `static`,
`hard_dynamic`, `hard_static` 중 하나를 지정한다. 예비 세트는 실패 원인을 확인한
뒤 해당 reserve component를 명시적으로 실행한다.

## 첫 SaveSensorData smoke

터미널 1에서 ROS master를 실행한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
roscore
```

터미널 2에서 rosbridge를 실행한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

그다음 MORAI Ego Network의 기존 ROS profile을 Load/Connect한다. Simulator
Network는 연결하지 않아도 된다. Camera 1~4와 LiDAR 5 Capture Mode 및 필요한
GT 출력을 켠다.

터미널 3에서 실제 rosbridge 연결을 점검한다. `rostopic list`만으로는 연결
여부를 알 수 없으므로 topic의 subscriber와 service provider를 확인한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rosparam set /use_sim_time false
rostopic info /ScenarioLoad
rostopic info /SaveSensorData
rostopic info /ctrl_cmd
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /Object_topic
rosservice info /Service_MoraiEventCmd
```

`/ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`의 Subscribers와
`/Service_MoraiEventCmd`의 Providers에 `/rosbridge_websocket`이 보여야 한다.

터미널 4에서 수집기를 실행한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/dynamic_object_v1_smoke_runtime.json \
  --capture-dir /home/luv/morai_cap/dynamic_v1_drs001_smoke \
  --motion-mode timed-grouped \
  --scenario drs001 \
  --captures-per-scene 4 \
  --capture-period-sec 0.5 \
  --capture-timeout-sec 15 \
  --scenario-load-attempts 1 \
  --continue-on-scene-error
```

4-frame smoke에서 다음을 확인한다.

- `Timed-grouped motion armed after ...` 로그가 첫 trigger보다 먼저 나온다.
- 첫 trigger의 이동거리는 0.30 m 이상이고 이후 frame 간 진행은 0.20 m 이상이다.
- timestamp group과 metadata frame이 각각 4개다.
- CAMERA_1..4와 LIDAR_5 artifact가 완전하다.
- actor stable ID, raw velocity/acceleration이 frame마다 있다.
- derived velocity가 비정상적으로 튀지 않는다.
- rejected frame 없이 summary가 완료된다.

`CtrlCmd` topic 연결만 살아 있고 MORAI 차량에 명령이 적용되지 않으면 5초 뒤
`Ego did not arm timed-grouped capture`로 실패해야 정상이다. 이 경우 정지 상태의
센서 파일은 학습 데이터로 인정하지 않는다.

통과 후 `--captures-per-scene` override를 제거하면 기본 12개로 수집한다. 누락이
생기면 요청 주기를 늘리고 불완전 파일을 정상 frame으로 강제 포함하지 않는다.

수집 직후에는 출력 자체를 다시 검증한다.

```bash
python3 dataset/morai_od/scripts/validate_dynamic_capture_output.py \
  /home/luv/morai_cap/<실제_timestamp_prefix가_붙은_capture_dir>
```

### 동적 clip의 빈 LiDAR 객체 목록 처리

ScenarioLoad 직후 개인용 launcher의 센서 writer를 초기화하기 위해 실제 clip과
분리된 prime snapshot을 한 번 저장한다. prime은 `sensor_primes/<scene_id>`로
이동하며 학습 frame과 temporal metadata에 포함하지 않는다.

`_Instance.txt`와 `_Instance_8Points.txt`는 해당 keyframe에서 LiDAR로 보이는
객체가 없으면 정상적으로 0 byte일 수 있다. 따라서 파일 존재와 고정 크기
`_Intensity.bin`, `_Semantic.bin`, `_Instance.bin` payload는 엄격히 검사하되,
텍스트 객체 목록의 빈 내용만으로 저장 실패라고 판정하지 않는다. Camera 또는
LiDAR payload 누락·0 byte는 계속 scene 실패다. 이 구분은
`batch001_m_dmc003` 4-frame 재검증에서 errors 0, warnings 0으로 확인했다.

## rolling-LiDAR timing sidecar

현재 SaveSensorData `LIDAR_5/*_Intensity.bin`은 `x,y,z,intensity`만 저장하며
point별 측정 시각을 포함하지 않는다. 실제 표본은 19,200 point, 307,200 byte로
확인됐다. 현재 수집한 3,000 frame은 동적 객체의 frame-level GT 속도를 이용한
single-frame `vx,vy` regression에는 사용할 수 있지만, 정확한 point-time feature와
point별 Ego-motion deskew 학습에는 정보가 부족하다.

현재 capture runner는 lightweight timing sidecar를 기본 생성한다.

1. SaveSensorData trigger 주변의 `/clock`, `/Ego_topic`, `/Object_topic` 연속 이력을
   보존한다.
2. `/sensor/lidar1`과 4-camera compressed topic이 존재할 때 payload 없이 header,
   field/shape, byte-size metadata만 보존한다.
3. 각 frame은 가장 가까운 timing sample ID와 trigger 대비 시간차를 참조한다.
4. Capture Mode에서 센서 ROS topic이 없으면 실패시키지 않고 clip metadata에
   `available: false`로 기록한다.

정확한 point-time 연결은 아래 조건을 만족할 때 추가 검증한다.

1. SaveSensorData XYZI와 ROS scan의 point count·좌표·순서가 같은지 검증한다.
2. 동일 scan으로 확인된 frame에만 point-time과 scan 구간 Ego pose를 연결한다.
3. 같은 원본에서 raw와 Ego-deskew painted point를 함께 생성해 재수집 없이 A/B할 수
   있게 한다.

이 sidecar는 rosbag 기반 수집으로 회귀하는 것이 아니다. Camera 4대·LiDAR·GT의
canonical frame은 계속 SaveSensorData가 담당하고, ROS는 SaveSensorData가 버리는
rolling-scan timing 정보만 보완한다. 구현과 검증 근거는
`perception/pointpainting/docs/rolling_lidar_ego_deskew_ab_260820.md`를 따른다.

Deskew를 production에 적용하려면 학습과 online inference가 같은 reference time,
LiDAR extrinsic, camera matching 정책을 사용해야 한다. 현재 raw-scan Epoch 65
checkpoint의 production 기본값 `lidar_deskew_mode=off`는 이 A/B가 통과할 때까지
유지한다.

### Timing sidecar live smoke (`260821_095908`).

- `drs001` 2-frame SaveSensorData capture가 완료됐고 frame당 25개, 총 50개 sensor
  artifact를 저장했다.
- validator 결과는 errors 0, warnings 1, status pass다.
- clip 주변 1.05초 동안 Ego 22개와 Object 22개 sample을 저장하여 둘 다 약 20 Hz다.
- capture trigger와 nearest Ego/Object sample의 절대 시간차는 약 5~8 ms다.
- NPC 이동량 1.593 m와 raw speed 변화 11.460→14.108을 연속 이력에서 확인했다.
- Capture Mode에서는 `/clock`, `/sensor/lidar1`, 4-camera compressed topic payload가
  발행되지 않아 `available: false`로 기록됐다. Ego/Object message header timestamp는
  정상이며 frame-level temporal alignment에는 사용 가능하다.
- 따라서 sweep 간 Ego 정합과 actor velocity label 검증에는 sidecar를 사용할 수 있다.
  반면 exact scan-internal point-time deskew는 이 capture profile만으로 해결되지 않는다.
