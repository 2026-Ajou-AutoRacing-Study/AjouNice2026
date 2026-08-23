# Base3-10K route-wide static temporal dataset.

## 목적.

- 동적 family는 MORAI NPC route가 live 검증된 대회 미션 구간을 유지한다.
- 정적 family는 대회 preferred route 전역을 독립적으로 순회한다.
- 중앙분리대·가드레일·표지판 기둥 등 배경 오검출을 줄이기 위한 무객체/sparse
  hard-negative를 포함한다.
- Base2와 같은 8-frame, nominal 12 Hz sequence 계약을 유지한다.
- single-frame 재현용 데이터가 아니라 velocity와 temporal modeling을 위한 canonical
  clip을 만든다.

## Base3 primary batch.

한 batch는 188 clips, 1,504 frames다. 1,500은 8로 나누어지지 않으므로 가장 가까운
완전 clip 수를 사용한다.

| Group | Family | Clips | 전체 비율 |
| --- | --- | ---: | ---: |
| Mission dynamic | roundabout_circulation | 14 | 7.45% |
| Mission dynamic | highway_merge_cruise | 10 | 5.32% |
| Mission dynamic | highway_cruise_overtake | 26 | 13.83% |
| Mission dynamic | highway_lane_change_overtake | 4 | 2.13% |
| Mission dynamic | crosswalk_pedestrian | 25 | 13.30% |
| Static temporal | static_obstacle | 44 | 23.40% |
| Static temporal | static_highway_obstacle | 8 | 4.26% |
| Static temporal | realistic_mixed_static | 8 | 4.26% |
| Static temporal | sparse_background | 12 | 6.38% |
| Static temporal | standing_pedestrian | 4 | 2.13% |
| Static temporal | stopped_vehicle | 4 | 2.13% |
| Hard transition | roundabout_sample_spawn | 8 | 4.26% |
| Hard transition | highway_merge_sample_spawn | 8 | 4.26% |
| Hard transition | gps_shadow_obstacle | 6 | 3.19% |
| Hard transition | gps_shadow_vehicle_interaction | 7 | 3.72% |

- Mission dynamic 79 clips, 42.02%.
- Static temporal 80 clips, 42.55%.
- Hard transition 29 clips, 15.43%.
- Obstacle-bearing static은 60/80 clips다.
- Static obstacle placement budget은 batch당 320개로 Base2와 동일하다.
- `sparse_background` 12개 중 8개는 진짜 무객체, 4개는 actor 1개 sparse scene이다.

## 공간 sampling.

- sparse global path는 대회 진행 순서를 고르는 guide로만 사용한다.
- 실제 route-wide sampling과 Ego 제어는 MGeo directed-link graph에서 복원한 연속
  centerline을 사용한다.
  - 39 links, 4,392 points, 2,184.61 m 폐루프다.
  - 최대 waypoint 간격은 0.50 m, 폐루프 closing gap은 0 m다.
  - lane-change용 synthetic lazy link는 제외하고 실제 주행 link만 연결한다.
- competition route 전체 arc를 기본 24개 stratum으로 나눈다.
- family별 prime-stride schedule과 stratum 내부 연속 jitter를 사용한다.
- Val/Test가 route 뒤쪽 연속 영역이 되지 않게 variant 순서와 공간 순서를 분리한다.
- batch별 phase offset을 바꿔 작은 family도 7-batch 누적 시 위치가 고정되지 않게 한다.
- 정확히 같은 clip은 한 split에만 속한다. 위치 자체를 split label로 사용하지 않는다.
- `static_highway_obstacle`은 검증된 세 MGeo lane path를 사용한다.
  - 오른쪽/가운데/왼쪽 45/35/20%.
  - 8 clips 단위 반올림은 batch마다 교차하고 7-batch 합계는 25/20/11,
    즉 44.6/35.7/19.6%다.
  - 각 lane 유효 arc의 8--82%를 stratified sampling한다.
- `standing_pedestrian`은 competition route에서 18 m 이내인 MGeo crosswalk만 사용한다.
  - 7-batch campaign에서 route 인접 crosswalk 6곳을 모두 사용한다.
  - launcher 공통 positive pedestrian asset deck을 사용하며 Man1에 고정하지 않는다.
- `stopped_vehicle`은 Ego가 정지한다는 뜻이 아니다.
  - 상대 차량이 정지하고 Ego는 temporal clip 동안 이동한다.
  - 교차/평행 도로가 가까운 구간에서는 도로 양쪽과 종방향 후보를 함께 탐색하여
    Ego swept path에서 최소 3 m clearance를 확보한다.

## Temporal 계약.

- clip length 8 frames.
- nominal trigger period 1/12 sec.
- actual timestamp와 `time_lag_sec`를 frame metadata에 기록한다.
- 현재 frame과 과거 두 sweep을 causal input으로 구성한다.
- sweep 간 Ego pose alignment를 적용한다.
- point time이 없는 scan 내부 방위각 근사 deskew는 적용하지 않는다.
- static actor의 raw velocity는 0이며 finite-difference derived velocity도 검증한다.
- 모든 frame, retry, replacement는 clip 단위 split을 유지한다.

## Offline 검증 결과.

- Base3 batch 1: 188 clips/1,504 frames 생성 성공.
- split: train 129, val 29, test 30 clips.
- 기존 validator: errors 0.
- 25 warnings는 기존 dynamic pedestrian speed unit live-audit 안내다.
- 7-batch generation: 1,316 clips/10,528 frames 전부 생성·검증 성공.
- Reserve 245 clips까지 포함한 14 manifests는 총 1,561개의 고유 scene ID이며
  static validator error 0이다.
- 폐루프 route-wide scene 588개는 실행 waypoint seam을 각 Ego 시작점보다 약 12 m
  뒤로 회전한다.
  - controller가 waypoint 배열 끝에서 lookahead를 뒤로 잡는 것을 방지한다.
  - MGeo link 순서와 geometry는 바꾸지 않는다.
  - 실행 anchor arc는 11.58--12.45 m, 최대 segment 0.50 m, closing gap 0 m다.
- static_obstacle은 train/val/test 각각 24개 route bin 전체를 커버한다.
- mixed와 sparse는 7-batch 누적 시 24개 route bin 전체를 커버한다.
- standing pedestrian는 crosswalk 6곳과 positive pedestrian asset 8종 이상을 사용한다.
- Batch 5 반복 생성에서 교차도로 stopped-vehicle clearance corner case를 발견했고,
  양방향/종방향 안전 후보 탐색으로 수정한 뒤 재검증했다.

## Live 검증 원칙.

- `ScenarioLoad`, `SaveSensorData`, `CtrlCmd`, Ego/Object topic 연결을 먼저 확인한다.
- MRP Camera 4대와 LiDAR의 Capture Mode가 켜져 SensorData 디렉터리가 존재해야 한다.
- 생성기 검증을 느슨하게 하여 ROS/Capture Mode 문제를 숨기지 않는다.
- representative smoke는 obstacle, highway, mixed, empty+sparse, pedestrian asset,
  stopped vehicle을 포함한다.
- 대량 1,504-frame batch는 representative smoke와 artifact completeness 검증 이후 만든다.

## 2026-08-24 representative live smoke.

- 초반 `ScenarioLoad` 실패는 생성기 문제가 아니라 일시적인 MRP ROS 연결 불안정이었다.
  - 10k 수집에서 검증된 runner의 subscriber gate를 완화하지 않았다.
  - Ego Network 재연결 뒤 원래 runner로 ScenarioLoad와 SaveSensorData가 정상 동작했다.
- route-wide 정적 scene은 Base2의 mission-local scene보다 급곡선 위치를 더 자주 포함한다.
  - 기존 target speed의 85% gate에서는 정상적인 adaptive-curve 주행도 실패할 수 있었다.
  - Base3 route-wide 정적 family만 65% floor, maximum motion 90 m, readiness 15 s로 조정했다.
  - stable-Ego gate는 유지하여 강한 가속 중 clip이 시작되는 것을 막는다.
- `NCAP_EBT` 같은 pedestrian dummy는 MRP가 scenario를 받아도 ObjectStatus에서 누락되는
  live case가 있었다.
  - route-wide standing/mixed positive pool은 launcher 공통 `person` asset으로 제한했다.
- 수정 후 representative 6 clips/48 frames가 모두 live capture에 성공했다.
  - static obstacle, three-lane highway obstacle, realistic mixed static.
  - true empty background, standing pedestrian, stopped vehicle.
  - artifact validator errors 0, sequence rate 11.43--11.79 Hz.
  - clip별 warning은 `/clock` timing sample 부재뿐이며 Ego/Object header timestamp는 보존됐다.

## 2026-08-24 RViz 공간 분포 재검토.

- 첫 preview에는 정적 장애물 구간과 동적 보행자 구간 사이 및 START/END seam 주변에
  Ego 표본 공백이 있었다.
  - 공식 global-path waypoint에는 `(-59.907, -105.529)`와
    `(-60.156, -34.510)` 사이 약 71.02 m 구간이 한 segment로 비어 있다.
  - MGeo에는 이 구간의 정상 도로 link가 존재하지만, 기존 route arc sampler는 두
    waypoint 사이를 하나의 점프로 취급하여 중간 위치를 만들 수 없었다.
  - 단순 직선 보간은 도로 centerline을 보장하지 않으므로 최종안에서 폐기했다.
  - sparse global path의 연속 segment를 MGeo link anchor로 대응시킨 뒤 directed-link
    graph의 최단 연결을 구한다.
  - 누락 구간은 실제 `A2256W000202`, `A2256W000728` link로 복원한다.
  - 잘못 연결될 수 있는 synthetic `A2256W000009-A2256W000202` lazy link는 제외한다.
  - 공식 route 파일과 mission-dynamic 정책은 변경하지 않고 Base3 route-wide static
    scene에만 복원 경로를 사용한다.
  - closed-loop seam margin도 제거하고 slot schedule을 family 사이에 interleave한다.
- 수정 전 전체 Ego 표본의 최대 원형 arc gap은 START 인근 약 212.5 m였다.
  - 최종 MGeo Batch 001의 route-wide 72 clips 기준 최대 gap은 약 68.37 m,
    평균 gap은 약 30.34 m다.
  - 기존 공백에는 static obstacle, realistic mixed, sparse background,
    standing pedestrian 표본이 새로 들어간다.
- GPS 음영 obstacle이 도로 진행 방향에 수직으로 늘어선 문제도 실제 좌표계 버그였다.
  - MORAI shaded-area의 긴 `size.x` 축은 표시 yaw가 아니라 `yaw - 90 deg` 방향이다.
  - 기존 방식은 대회 route tangent와 약 92.7 deg 어긋났다.
  - `shade_local_to_world`, static obstacle heading, validator를 같은 축 정의로 수정했다.
  - 수정 preview의 obstacle heading과 실제 Ego route tangent 차이는 약 2.7--19.5 deg다.
- GPS obstacle은 방향 수정 직후 Ego 경로 위 4--12 m에 놓이면 저장 중 충돌했다.
  - Ego가 capture-speed gate까지 11--21 m 주행하고, 8-frame 파일 저장 중에도 이동하기
    때문이다.
  - 초기 간격을 42--57 m로 두어 실제 첫 capture 시 약 20--40 m 범위가 되게 했다.
  - 근거리/원거리 대표 2 clips, 16 frames live smoke가 충돌 없이 통과했다.
  - artifact validator errors 0, sequence rate 11.60--11.78 Hz다.

## 2026-08-24 MGeo route live 검증.

- 첫 MGeo smoke에서 START/END seam 직전 scene이 heading 178.5 deg 오류로 중단됐다.
  - MGeo route 복원 오류나 ROS 불안정이 아니었다.
  - online waypoint controller가 배열 끝에서 index 0으로 wrap하지 않고 lookahead를
    마지막 점에 고정하는 실행 경로 표현 문제였다.
  - heading safety gate는 완화하지 않았다.
- 각 폐루프 scene의 동일 MGeo waypoint 순서를 Ego보다 약 12 m 뒤에서 시작하도록
  회전시켰다.
  - Ego 앞에는 약 2.17 km 이상의 연속 경로가 남으므로 90 m motion limit 안에서
    배열 끝에 닿지 않는다.
  - canonical route arc와 link identity는 metadata에 그대로 보존한다.
- START seam, 71 m 복원 구간, END 구간, sparse empty, standing pedestrian의
  5 clips/40 frames를 MRP에서 실제 SaveSensorData로 검증했다.
  - 전부 8/8 frames 성공했다.
  - sequence rate는 11.46--12.00 Hz다.
  - capture artifact validator error 0이다.

## 후속 hard-negative mining.

- 첫 Base3 모델을 객체가 거의 없는 전체 주행 rosbag에 실행한다.
- false positive의 timestamp, 위치, class, confidence, bbox를 저장한다.
- 중앙분리대, 가드레일, 기둥처럼 실제로 틀린 위치만 targeted add-on으로 수집한다.
- 무작위 empty 비율을 계속 늘리지 않고 모델이 틀리는 배경에 학습량을 집중한다.

## RViz generated coverage 확인.

Audit publisher는 sparse guide가 아니라 audit가 내보낸 실제 MGeo
`generated_route.csv`를 기본 표시한다.

```bash
# Terminal 1.
source /opt/ros/noetic/setup.bash
roscore
```

```bash
# Terminal 2. audit 결과에 맞는 Ego/actor/route marker 발행.
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
python3 dataset/morai_od/scripts/publish_generated_scenario_coverage.py \
  --audit-dir /tmp/base3_mgeo_final/audits/batch001_primary
```

```bash
# Terminal 3. 저장된 Base3 RViz 구성 실행.
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roslaunch lanelet_map_visualizer \
  base3_generated_dataset_coverage_rviz.launch
```
