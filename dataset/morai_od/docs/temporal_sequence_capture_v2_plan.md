# MORAI temporal sequence capture V2 계획.

## 런처 약칭.

- `mrp`: 개인용 MORAI 런처. 2026년도 버전이며 경로는
  `~/MoraiLauncher_Lin_per`이다.
- `mr`: 대회용 MORAI 런처. 2025년도 버전이며 경로는
  `~/MoraiLauncher_Lin`이다.
- 이후 문서와 작업 대화에서 별도 설명이 없으면 위 약칭을 사용한다.

## ROS bag 기본 계약.

- 별도 설명이 없으면 ROS bag은 `mrp`에서 녹화한다.
- `mrp`에서 녹화하는 모든 ROS bag에는 `/ReplayInfo_topic`을 반드시 포함한다.
- 센서 benchmark·online 평가용 bag은 기존 목적별 sensor topic에
  `/ReplayInfo_topic`을 추가한다. `/ReplayInfo_topic`이 다른 raw sensor topic을
  대체하지는 않는다.
- MORAI Rosbag Replay만을 위한 최소 world-motion bag은
  `/ReplayInfo_topic` 하나로 생성할 수 있다. 다만 map, scenario, sensor config,
  weather, time-of-day와 traffic-light 상태를 완전히 설명하지 않으므로 해당 설정
  파일과 metadata를 sidecar로 함께 보존한다.
- 재현성 검증용 world-motion bag에는 `/clock`, `/Ego_topic`, `/Object_topic`,
  `/ctrl_cmd`, collision topic을 함께 기록한다. MORAI Replay의 필수 payload는
  `/ReplayInfo_topic`이고 나머지는 원본과 replay 궤적의 차이를 검증하기 위한
  보조 자료다.

## Production cadence 결정.

- 2026-08-22부터 canonical capture cadence는 `12 Hz`, 즉 `1/12초`로 고정한다.
- 8-frame clip의 nominal span은 첫 frame부터 마지막 frame까지 `7/12초`, 약
  `0.583초`다.
- 3-sweep PointPainting 입력의 nominal lag는 `0`, `-1/12`, `-2/12초`다.
- 실제 frame metadata에는 nominal 값과 별개로 측정된 timestamp와
  `time_lag_sec`를 보존한다.
- 부분 전체주행 bag의 LiDAR는 99.35초 동안 1,182 frame, 평균 `11.89 Hz`였고
  header 간격 p50은 79.3 ms, p95는 106.2 ms였다. 따라서 12 Hz를 중심 분포로
  사용하되 실제 지터와 누락은 timestamp 기반 loader에서 처리한다.
- 이후 대회 클라이언트 PC 실측 분포가 달라지면 새 dataset을 수집한다. 현재
  production dataset을 미리 10 Hz로 낮춰 수집하지 않는다.
- 아래 2026-08-21 `0.15초` live gate 결과는 구형 저속 pilot의 역사적 검증이며,
  신규 production cadence를 뜻하지 않는다.

## 실주행 bag 기반 held-out test 계획.

- Procedural generator의 train/val/test는 scene ID가 분리돼도 같은 생성 분포를
  공유한다. 따라서 이 test는 `synthetic IID test`로만 해석하고 최종 대회 일반화
  지표로 단독 사용하지 않는다.
- 최종 주 지표는 mrp에서 사람이 실제 대회 경로를 주행해 녹화한 raw bag을
  LiDAR frame 기준으로 parsing한 `real-drive held-out test`로 구성한다.
- 모델 입력 camera는 online production과 동일하게 LiDAR보다 늦지 않은 최신 frame을
  causal selection한다. GT Object/Ego 상태는 평가 label이므로 앞뒤 고주기 메시지를
  LiDAR timestamp로 보간할 수 있다.
- 하나의 연속 주행을 frame 단위로 train/val/test에 나누지 않는다. Bag/run 전체가
  하나의 split group이다.
- 반복 튜닝에 사용할 `real-drive dev`와 마지막까지 보지 않을 `real-drive final test`를
  서로 다른 주행으로 녹화한다. Generated validation은 epoch/checkpoint 선택에 유지한다.
- 전체 평균만 내면 긴 배경 구간이 rare mission을 덮으므로 roundabout, highway merge,
  highway cruise/overtake, pedestrian, static obstacle, GPS-shadow 구간별 지표와
  mission macro average를 함께 기록한다.

### 두 test 생성 방식 비교.

- MORAI rosbag replay 뒤 SaveSensorData를 다시 요청하는 방식.
  - 현재 canonical SaveSensorData는 frame당 약 4.82 MB다.
  - 센서별 label이 바로 생성되는 장점이 있다.
  - 다만 replay는 simulator가 원 bag을 그대로 반환하는 것이 아니라 actor 상태를
    재생해 센서를 다시 렌더링하는 방식이다.
  - 현재 `replay_target_option`에는 Ego/NPC/pedestrian만 있고 static obstacle bit가
    없으므로, 원 scenario를 함께 정확히 load하지 않으면 obstacle test가 달라질 수 있다.
  - 원본 packet timing/drop을 보존하지 않으므로 최종 primary test로 쓰지 않는다.
- Raw bag 직접 parsing 방식.
  - 원본 JPEG, PointCloud2, timestamp와 실제 drop/jitter를 그대로 보존한다.
  - `/Object_topic`, `/Ego_topic`, calibration으로 canonical 3D label을 생성해야 한다.
  - GT box의 sensor range, camera visibility, LiDAR point-hit filtering을 구현·검증해야
    하지만 최종 배포 조건과 가장 가까우므로 primary test로 사용한다.
- Replay SaveSensorData는 raw-bag label converter의 소규모 대조·검증용 secondary
  benchmark로만 사용한다. 전체 bag을 두 방식으로 중복 변환하지 않는다.

### 실측 크기 추정.

| Bag | 길이 | 12 Hz frame | SaveSensorData 예상 | Raw bag | Raw parsing payload 예상 |
|---|---:|---:|---:|---:|---:|
| 최신 부분 주행 | 99.47초 | 약 1,194 | 약 5.75 GB | 1.12 GB | 실측 약 0.64 GB |
| 기존 full high-speed | 155.72초 | 약 1,869 | 약 9.00 GB | 1.59 GB | 약 1.00 GB |
| 기존 full slow | 282.02초 | 약 3,384 | 약 16.31 GB | 2.86 GB | 약 1.80 GB |

- SaveSensorData 1회는 live smoke에서 wall time 약 0.65초를 사용했다. 따라서 replay
  전체 변환은 각각 대략 13분, 20분, 37분이고 launcher 상태에 따라 더 길어질 수 있다.
- 최신 99.47초 bag은 LiDAR 1,182 frame, 11.887 Hz였다. Raw parsing에서는 억지로
  1,194 frame을 만들지 않고 실제 1,182개 LiDAR frame을 anchor로 쓴다.
- 같은 bag의 LiDAR 대비 nearest camera header 차이는 99.92%가 50 ms 이내였고,
  p95는 camera별 3.6~5.4 ms였다. Online과 같은 causal selection에서는 front/back
  p95 age가 약 38~40 ms, side camera는 약 5.4 ms였다. Object/Ego causal age p95는
  약 21 ms로, GT interpolation을 구현할 근거가 충분하다.

## 구현 상태.

- 2026-08-21 기준 V2 offline generator와 validator를 구현했다.
- Canonical pilot은 38 clip x 8 frame = 304 frame이다. `1,500` 단위 production은
  clip을 자르지 않기 위해 188 clip x 8 = 1,504 frame으로 생성한다.
- `temporal_sequence_capture_v2_pilot304.json`은 Sunny 13시에서 로터리, 고주로
  합류·순항·차선 변경, 횡단 보행자, 일반·고주로 static obstacle과 GPS 음영 hard
  case를 포함한다.
- GPS 음영 랜덤 미션의 `장애물 회피`뿐 아니라 `끼어들기`도 빠뜨리지 않는다.
  `gps_shadow_vehicle_interaction`은 추월 없는 동일방향 차량 상호작용으로 만들고,
  진입 전·음영 전반·후반·출구 네 구간에 capture anchor를 분산한다. 규정만으로
  급격한 측면 cut-in을 확정할 수 없으므로 이를 임의로 학습 prior로 만들지 않는다.
  NPC를 목표속도로 즉시 생성하면 ScenarioLoad 검증과 Ego 가속 gate 동안 먼저
  구간을 벗어나므로, NPC는 0 km/h에서 목표속도로 가속하고 Ego capture preroll은
  3~6 m로 제한한다. MORAI pause setup은 spawn-point actor가 생성되지 않아 기각했다.
  거리 역산 하드코딩도 초기 충돌 위험이 있어 사용하지 않는다. 대신 터널 NPC를
  10~20 km/h, 초기 선행 간격을 6~12 m로 두어 준비 지연 후 frame 0에서도
  검출·상호작용 범위에 남게 한다.
- Personal launcher에서 자동 가능한 obstacle 30종을 V2 asset deck으로 순환시킨다.
  Pilot 69 unique placement에서 30종이 모두 최소 1회 선택되는 것을 검증했다.
- Production batch는 아래 1,415-placement quota를 smooth weighted round-robin으로
  interleave한다. 따라서 한 asset quota를 모두 소진한 뒤 다음 asset으로 넘어가지 않고,
  1,504-frame 첫 batch에서도 30종이 전부 등장한다.
- 고주로 lane-change route는 세 MGeo centerline을 arc-length로 대응시키고 quintic
  smoothstep으로 연결한다. Entry/middle/exit clip의 capture arm 위치를 따로 둔다.
- Static highway clip은 clip 단위 right/middle/left 60/25/15 weighted lane policy를
  사용한다. 8-clip production batch에서는 5/2/1 clip으로 반올림된다.

### 2026-08-22 공간 분산·GPS 차량 live smoke.

- GPS 차량 최초안은 15~30 km/h 목표속도와 12~28 m 선행 간격을 사용했다.
  ScenarioLoad, scene settle, sensor writer prime, Ego arm 동안 NPC가 먼저 이동해
  frame 0 거리가 약 85 m가 됐으므로 기각했다.
- NPC를 0 km/h에서 가속하고 Ego arm을 3~6 m로 줄인 중간안은 약 53 m였다.
- ScenarioLoad pause는 pause 상태에서 spawn-point actor가 생성되지 않고
  EventCmd로 정상 resume되지 않아 production에서 제거했다.
- 최종안은 GPS 터널 NPC 10~20 km/h, initial lead 6~12 m를 사용한다.
  Ego 시작점과 NPC center 간 최소 계획 간격은 preroll을 포함해 8 m 이상인지
  validator가 확인한다.
- GPS 4-frame live smoke는 4/4 frame, 92 sensor artifact, validation error 0이었다.
  측정 간격은 약 11.0 Hz였고, 해당 상단속도 seed의 frame 0~3 거리는
  35.21~35.26 m, 횡방향 차이는 0.11~0.14 m였다.
- Roundabout exit smoke는 실제 route index 1760~1763으로 계획 zone 1741~1780
  안에 있었다. Highway settled smoke는 2125~2130으로 계획 zone 2061~2130
  안에 있었다. 두 clip 모두 4/4 frame과 validation error 0을 기록했다.
- 188 clip/1,504 frame production dry-run은 error 0으로 통과했다.
  split은 train/val/test 129/29/30 clip이고 GPS 차량 7 clip은 네 spatial phase를
  1/2/2/2로 포함한다. 남은 25 warning은 기존 pedestrian speed 단위의 live
  displacement audit 안내이며 새 GPS/공간 분산 오류가 아니다.
- Scene별 `capture_period_sec`, motion arm, 최소 frame 진행거리, warm-up과 속도 설정이
  실제 capture runtime에 적용된다.
- Stable family는 목표속도, 최소 진행거리뿐 아니라 최근 0.5초 Ego acceleration gate를
  통과해야 capture를 시작한다. Spawn·횡단·전이 family에는 이 gate를 강제하지 않는다.
- Frame/clip metadata에 motion phase, sampling role, 8-frame sequence contract,
  PointPainting target lag와 clip split group을 저장한다.
- Dynamic clip schema V3는 `dynamic_metadata/calibration.json`을 clip 공용 파일로
  저장한다. Camera intrinsic, distortion, `sensor_to_ego`/`ego_to_sensor`,
  LiDAR extrinsic과 행렬·축 convention을 명시한다. 각 frame에는
  `ego_to_global`/`global_to_ego`, 이전 frame 및 clip 첫 frame과의 Ego 상대변환을
  저장한다. SaveSensorData에 노출되지 않는 실제 exposure/point time은 추정값으로
  위조하지 않고 limitation으로 기록한다.
- `generate_temporal_sequence_v2_batch.py`는 batch별 seed와 scenario prefix를 분리하고
  primary 1,504 frame 및 reserve 224 frame을 생성한다. Pedestrian reserve 10 clip은
  여섯 crosswalk ID를 모두 포함하여 다른 위치의 clip으로 실패를 대체하지 않게 한다.
- Offline smoke는 188-clip primary batch까지 생성했고 static validation은 통과했다.
- MORAI live smoke는 static obstacle, crosswalk pedestrian, highway lane-change에서
  실제 launcher load, CtrlCmd 추종과 SaveSensorData 완전성을 통과했다.

현재 asset deck은 asset별 quota까지 구현했지만 거리·형상·방향·가시성의 교차 quota는
아직 실제 부족 bin을 feedback해 고르는 scheduler가 아니다. 10,000-frame production
전에는 아래 조건부 quota scheduler를 추가해야 한다. 따라서 pilot selection count를
최종 obstacle 공간 분포로 해석하지 않는다.

## 상태와 목표.

- 상태: 설계 확정 전 구현 대기.
- 기존 static 3 m `rolling-grouped`와 dynamic V1을 즉시 폐기하지 않는다.
- 기존 Epoch 65 single-frame geometry 성능은 재현 기준으로 보존한다.
- 신규 수집의 주 목표는 vehicle/pedestrian velocity, temporal consistency와 향후
  StreamPETR 계열 camera memory 학습이다.
- 신규 canonical 원본은 독립 frame이 아니라 연속 scenario clip으로 저장한다.
- single-frame dataset은 sequence에서 시간·거리 기준으로 downsample해 파생한다.

## Production 후보 입력 계약.

- PointPainting temporal baseline은 raw 3-sweep과 sweep 간 Ego-pose 정합을 사용한다.
- 현재, 약 0.083초 전, 약 0.167초 전 sweep을 실제 timestamp로 선택한다.
- point feature에 실제 `time_lag_sec`를 추가한다.
- scan 내부 방위각 근사 Ego deskew는 production에서 끈다.
- SaveSensorData와 현재 MORAI PointCloud2에는 point별 time field가 없어 exact
  point-level deskew는 현재 대회 입력 계약으로 구현할 수 없다.
- 원본 XYZI는 변경하지 않고 보존하여 향후 driver가 point time을 제공할 때 다시
  변환할 수 있게 한다.

## Sequence-first 원칙.

신규 정적·동적 데이터는 모두 연속 clip으로 수집한다.

- PointPainting single-frame: clip에서 충분히 떨어진 frame만 선택한다.
- PointPainting 3-sweep: 목표 lag에 가장 가까운 current/past-1/past-2를 선택한다.
- StreamPETR: camera 4대의 연속 frame과 `prev_exists`, Ego pose를 사용한다.
- train/val/test split 단위는 frame이 아니라 전체 clip이다.
- 같은 scenario seed, 재촬영과 실패 대체 clip은 같은 split group에 둔다.

기존 static 3 m 데이터는 geometry replay와 baseline 재현에만 유지한다. 신규 static을
single-frame용으로 다시 따로 촬영하지 않는다.

## Capture phase.

시나리오 Load 직후 저속 frame이 전체 sequence를 지배하지 않게 stable과 transition을
분리한다.

### Stable clip.

- 전체의 약 70--80%를 목표로 한다.
- actor ready, route와 heading 검사를 통과해야 한다.
- Ego가 family 목표속도의 85% 이상에 도달해야 한다.
- 최근 0.5초 `|acceleration| < 1.0 m/s^2`를 기본 안정 조건으로 한다.
- 저속·중속·고속 family에서 각각 최소 3 m, 5 m, 10 m 진행 후 capture를 arm한다.

### Transition clip.

- 전체의 약 20--30%를 상한으로 한다.
- `spawn_acceleration`, `ego_acceleration`, `braking`, `stopped`, `restart`를 metadata로
  구분한다.
- stable gate를 의도적으로 해제하되 family quota로 과대표집을 막는다.

## Ego 속도 분포.

규정은 전 구간 제한속도를 명시하지 않지만 주행 시간 경쟁, 공식 고주로 NPC
40/50/60 km/h와 팀의 빠른 일반구간 주행 목표를 고려한다. 기존 5.4 km/h는
single-frame 안전 수집 속도이며 신규 temporal production의 주 분포로 사용하지 않는다.

| Ego 속도 | 전체 권장 비율 | 대표 상황 |
| --- | ---: | --- |
| 5--15 km/h | 10% | 정지·출발, 횡단보도, 장애물 근접. |
| 15--30 km/h | 25% | 로터리, 장애물 접근, GPS 음영. |
| 30--45 km/h | 40% | 일반 도로 안정 순항. |
| 45--60 km/h | 25% | 고주로 합류 이후와 추월·순항. |

Family별 목표속도는 다음 범위에서 정한다.

- 횡단 보행자·blocking obstacle: 10--25/30 km/h.
- 로터리: 15--30 km/h.
- GPS 음영 mission: 20--35 km/h.
- 일반 static roadside/background: 30--45 km/h.
- 고주로 합류: 25--45 km/h.
- 고주로 합류 이후 오른쪽 차로: 35--50 km/h.
- 고주로 합류 이후 가운데 차로: 45--55 km/h.
- 고주로 합류 이후 왼쪽 차로: 50--60 km/h.

고속 capture는 기존 10.5--15 m 회랑을 재사용하지 않는다. 전체 clip의 예상 이동거리로
route, collision corridor, 최대 motion과 객체 visibility 구간을 다시 만든다.

## 고주로 Ego 차로와 추월 sequence.

공식 preferred route의 고주로 구간은 기본적으로
`201087 -> 201086 -> 200101 -> 200128 -> 200102 -> 200134`를 따른다. 현재
Behavior Path Planner에는 느린 앞차만으로 추월 차로를 자동 선택하는 완성된 policy가
없고, preferred lane 변경 또는 외부 lane-change 요청이 있어야 차선 변경이 발생한다.
따라서 모든 차로를 균등하게 뽑거나 가운데·왼쪽을 과대표집하지 않는다.

고주로 합류 family와 합류 이후 family를 분리한다.

- `highway_merge_*`: 진입로와 가장 오른쪽 합류 차로를 그대로 사용한다. Ego 차로를
  randomize하지 않는다.
- `highway_lane_keep`: 합류 이후 오른쪽·가운데·왼쪽 차로의 고정 순항을 수집한다.
- `highway_lane_change_overtake`: 오른쪽→가운데, 가운데→왼쪽과 두 복귀 방향을
  실제 연속 waypoint로 수집한다.

공식 샘플 시나리오와 현재 Planning prior를 다시 대조한 결과, Batch001의 합류 이후
36 clip은 다음과 같이 고정한다. 합류 접근 22 clip은 이 표와 별개이며 모두 공식
global route와 가장 오른쪽 합류 차로를 따른다.

| Ego route 유형 | Clip | 합류 이후 비율 |
| --- | ---: | ---: |
| right lane 유지 | 20 | 55.6% |
| middle lane 유지 | 9 | 25.0% |
| left lane 유지 | 3 | 8.3% |
| 실제 lane-change sequence | 4 | 11.1% |

고정 순항의 차로별 Ego 목표속도는 right 35--50, middle 45--55, left 50--60 km/h로
제한한다. Ego가 NPC보다 왼쪽 차로에 있을 때만 추월 속도 우위를 적극적으로 만들고,
오른쪽 차로에 있을 때에는 나란히 달리거나 뒤따르는 표본으로 제한한다. 따라서
오른쪽 차로에서 55--60 km/h로 반복 추월하는 비현실적인 shortcut을 학습하지 않는다.
고정 순항 시작점은 각 고주로 centerline의 arc 70--300 m, lane-change 중심은
100--240 m에서 뽑는다. 합류 직후 곡선부에서 고속 초기화하면 control mode 전환 전에
Ego가 접선 방향으로 이탈하므로, 이 구간은 merge family에 맡기고 cruise/overtake는
세 차로가 평행해진 이후부터 수집한다.
Planning에 자동 추월이 production으로 들어오면 middle/left와 lane-change 비율을 다시
높일 수 있다. NPC는 절대 차로 번호와 별개로 `relative_lane`,
ahead/alongside/behind와 relative speed quota를 관리한다.

### Lane-change route 생성.

MORAI scenario JSON에 Ego maneuver를 하드코딩하지 않는다. 현재 capture stack과 같이
초기 pose·NPC는 scenario JSON에 저장하고, Ego는 별도 waypoint sidecar를
`ros_scenario_autocapture_poc.py`가 `/ctrl_cmd`로 추종한다.

1. MGeo의 source/target adjacent lane centerline을 arc length로 대응시킨다.
2. 차선 변경 전 20--40 m 직진, 부드러운 횡이동, 변경 후 20--40 m 직진을 연결한다.
3. 횡이동은 quintic smoothstep 또는 기존 control trajectory generator의 곡률 기반
   geometry를 재사용하여 시작·끝 위치, yaw와 곡률을 연속으로 만든다.
4. 차선 변경 시간은 약 4초로 두고 길이를
   `clamp(ego_speed_mps * 4 sec, 35 m, 75 m)`로 정한다. 30/40/50/60 km/h에서는
   각각 약 33/44/56/67 m가 기준이다.
5. 두 차로를 한 번에 건너지 않는다. 연속 두 번 변경할 때에는 사이에 최소
   20--30 m의 안정 직진을 둔다.

한 maneuver 전체를 8 frame에 억지로 압축하지 않는다. `lane_change_entry`,
`lane_change_middle`, `lane_change_exit` 중 하나를 clip phase로 선택하고,
동일 maneuver에서 파생된 clip은 같은 split group에 둔다. Metadata에는 source/target
lane, direction, 0--1 progress, 목표·실제 lateral offset, Ego speed/acceleration/yaw
rate와 NPC relative state를 저장한다.

구현 gate는 MGeo adjacency audit, geometry unit test와 RViz overlay, NPC 없는
30/45/60 km/h Ego smoke, NPC 상호작용 추가, phase별 8-frame SaveSensorData smoke
순서다. Route lateral error 0.5 m 이하, 목표속도 85% 이상, 도로 영역 유지, 충돌 없음과
목표 객체 6/8 frame 이상 유효를 일반 clip acceptance 조건으로 사용한다.

### 정적 temporal의 고주로 차로 정책.

신규 V2 정적 생성기도 고주로에서 공식 오른쪽 route 하나에만 고정하지 않는다.
그렇게 하면 가운데·왼쪽 차로 주행 시 정적 객체와 정지 차량의 camera sector,
가림과 LiDAR 입사각 분포가 빠진다. 다만 매 frame 차로를 독립 randomize하거나 세
차로를 균등 배분하지 않는다.

- 차로는 clip 시작 시 선택하고 clip 전체에서 유지한다.
- 기본 static highway clip은 right 60%, middle 25%, left 15%로 둔다.
- 정적 객체를 둔 Ego lane-change clip은 static highway의 10--15%를 상한으로 하고,
  위 `highway_lane_change_overtake`/hard family와 중복 집계하지 않는다.
- stopped vehicle은 same/adjacent/two-lanes-away를 균형화한다.
- 일반 obstacle은 roadside·공식 obstacle 문맥에 둔다. 고주로 주행 차로 한가운데의
  obstacle은 realistic static에 섞지 않고 별도 hard/stress family로만 만든다.
- 차로별 scene 수보다 relative lane, 거리, camera sector와 asset unique placement
  quota를 우선 검사한다.

즉 정적 generator의 고주로 차로 다양화는 필요하지만, 현재 global route prior를
유지한 weighted randomization으로 구현한다. 기존 3 m single-frame static generator는
재현용으로 보존하고 이 정책은 신규 temporal V2 generator에만 적용한다.

## 신규 10,000-frame production 초안.

Canonical clip은 우선 8 frame으로 설계한다. PointPainting은 이 안에서 실제 timestamp가
목표 lag에 가장 가까운 3 sweep을 사용하고, 향후 StreamPETR은 연속 8 camera frame을
사용할 수 있다. 10,000을 8-frame clip 단위로 정확히 나누기 위해 명목 비율에서 몇
frame만 조정한다.

| 구성 | Frame | 8-frame clip | 비율 |
| --- | ---: | ---: | ---: |
| 동적 차량 sequence | 3,744 | 468 | 37.44% |
| 동적 보행자 sequence | 1,248 | 156 | 12.48% |
| 정적 temporal sequence | 3,504 | 438 | 35.04% |
| Hard temporal sequence | 1,504 | 188 | 15.04% |
| 합계 | 10,000 | 1,250 | 100% |

Frame 수 외에도 clip 수, simulator-time 길이, unique placement, stable actor track,
속도 bin, 거리 bin과 visibility duration을 함께 보고한다. 연속 frame을 독립 장면처럼
집계하지 않는다.

Obstacle은 velocity regression에서 제외하고 online output을 `(0,0)`으로 고정하는
방향을 우선한다. Vehicle과 pedestrian은 stationary/moving 비율과 속도 bin을 클래스별로
관리하여 zero-velocity collapse를 막는다.

### 정적 temporal 내부 구성.

정적 35%는 velocity가 0인 same-class vehicle/pedestrian, obstacle과 background를
학습하기 위한 절충안이다. 기존 1,500-frame 학습에서 확인한 obstacle 표본 부족을
해결하면서도 mission dynamic 50%와 hard 15%를 유지한다. 정적 내부
obstacle-focused 70%는 전체 데이터의 약 24.5%이며 hard와 mixed 표본까지 합치면
obstacle 관련 frame은 전체의 약 28--30%가 된다.

| 정적 family | Frame | Clip | 정적 내 비율 |
| --- | ---: | ---: | ---: |
| obstacle-focused | 2,448 | 306 | 69.9% |
| stopped vehicle | 528 | 66 | 15.1% |
| standing pedestrian | 176 | 22 | 5.0% |
| realistic mixed | 176 | 22 | 5.0% |
| sparse/background | 176 | 22 | 5.0% |
| 합계 | 3,504 | 438 | 100% |

Hard temporal 중 304 frame, 38 clip을 원거리 작은 obstacle, 부분 가림, camera seam,
길거나 높은 형상과 밀집 배치에 사용한다. 이는 mission family 안에 무관한 obstacle을
섞는 방식이 아니라 별도 hard family다.

같은 clip의 연속 8 frame은 8개의 독립 사례로 세지 않는다. Obstacle-focused clip당
5--7개, 평균 6개, mixed clip당 평균 2개, hard obstacle clip당 평균 3개를 사용하면
예상 unique placement는 다음과 같다.

| 출처 | 계산 | unique placement |
| --- | ---: | ---: |
| obstacle-focused | 306 clip x 6 | 1,836 |
| realistic mixed | 22 clip x 2 | 44 |
| hard obstacle | 38 clip x 3 | 114 |
| 합계 |  | 약 1,994 |

30개 자동 가능 asset을 독립 균등 난수로만 선택하면 asset당 평균은 약 66.5회지만,
로드 실패와 가시성 실패까지 더해지므로 pure random sampling은 사용하지 않는다.
부족한 asset을 우선하는 quota deck을 사용하고 실패 placement는 quota 완료로
집계하지 않는다.

### Obstacle catalog와 asset quota.

현재 catalog 33종 중 두 launcher에서 자동 사용 가능한 것은 30종이다. `Scooter_01`은
competition-only manual, `Speed_Bump`와 `Tree08`은 정책 제외다. `hole-cover`는 자동
생성할 수 있지만 얇은 road-surface 객체라 20개 audit 뒤 positive label 유지 여부를
결정한다.

| 그룹 | Asset | Asset당 unique quota | 그룹 합계 |
| --- | --- | ---: | ---: |
| box | `CargoBox`, `WoodBox` | 72 | 144 |
| barrel/drum | `PE_Drum`, `TrafficBarrel`, `RedBarrel`, `YellowBarrel` | 72 | 288 |
| barrier | `concrete-barrier`, `Bollard01`, `Bollard04`, `Concrete_Barricade01`, `Concrete_Barricade02`, `Lane_Separation_Bar`, `Steel_Barricade`, `PE_Firewall_Orange`, `PE_Firewall_White` | 62 | 558 |
| unoccupied two-wheeler | `Bicycle_city`, `Bicycle_mountain`, `Scooter_02` | 45 | 135 |
| road sign | `Road_Sign_Mark`, `KR_Sign_SL_30`, `40`, `50`, `60`, `70`, `80`, `90`, `100`, `110` | 25 | 250 |
| roadside context | `fr-streetlight-2` | 20 | 20 |
| surface audit | `hole-cover` | 20 | 20 |
| 합계 | 자동 대상 30종 |  | 1,415 |

예상 1,458 placement 중 1,415개를 quota에 쓰고 43개를 내부 여유로 둔다. 생성
단계에서는 MORAI asset 누락과 가시성 실패를 고려해 별도로 10--15% reserve scene을
만든다. Quota 72/62/45/25/20은 70/15/15 split 후 train에 대략
50/43/31/17--18/14 unique placement를 남긴다. 개별 asset AP를 신뢰하기에는 작은
수이므로 asset 평가는 오류 진단용이고, production 평가는 obstacle 전체와
compact/tall/long 및 거리 구간을 기준으로 한다.

### Obstacle 조건별 quota.

Asset quota를 먼저 만족시키고, 그 안에서 다음 조건 quota가 부족한 placement를
우선 생성한다. 이름으로 형상을 추정하지 않고 생성된 MORAI GT의 실제
`length/width/height`로 compact/tall/long을 분류한다.

| 축 | 목표 분포 |
| --- | --- |
| 거리 | 0--10 m 25%, 10--20 m 40%, 20--30 m 35%. 30 m 이상은 stress set. |
| 형상 | compact 최소 35%, tall 최소 25%, long 최소 35%, surface/context 최대 5%. |
| 방향 | 전방 60%, 좌측방 20%, 우측방 15%, 후방 5%. |
| 가시성 | clear 60%, partial occlusion 25%, camera seam 15%. |

예를 들어 quota 72인 barrel/box asset 하나는 0--10/10--20/20--30 m에서 각각
18/29/25개 정도를 확보한다. 기존 데이터에서 가장 부족했던 20--30 m와
compact/long 조합을 우선 채운다. 하나의 placement는 일반 clip에서는 8 frame 중
최소 6 frame에서 유효해야 하며, 의도적 occlusion hard clip은 별도 기준과 metadata를
사용한다.

## Randomization과 mission specialization.

동적 생성기를 static 생성기처럼 완전 무작위화하지 않는다. 동적 객체는 위치뿐 아니라
차선 연결, 진행 방향, 속도 profile과 Ego interaction timing이 의미를 가지므로
비현실적인 random motion은 velocity와 prediction에 잘못된 prior를 만든다.

권장 방식은 `structured randomization`이다.

- 로터리, 고주로, 횡단보도, GPS 음영 등 mission family는 유지한다.
- Family 내부에서 spawn arc, lane, asset, 속도, acceleration, Ego gap, camera sector와
  occlusion을 randomize한다.
- 일반 도로 lane-following, stationary vehicle과 background sequence를 별도 family로
  포함해 mission 위치만 외우는 shortcut을 막는다.
- 공식 sample JSON의 exact actor path·seed·capture는 내부 mission-holdout에서 격리한다.

### 동적 보행자 위치·난이도 production 정책.

MGeo grouped crosswalk 28개와 2,184.6 m competition waypoint를 polygon 기준으로
교차 검사했다. Waypoint가 직접 통과하는 곳은 `CW000000`, `CW000002`, `CW000013`,
`CW000015` 네 곳이며, 같은 교차로의 측면에서 각각 2.96 m와 5.61 m 떨어진
`CW000001`, `CW000014` 두 곳도 관측 다양성 후보로 사용한다. 공식 sample pedestrian은
`CW000000`에 있다.

동적 pedestrian 1,248 frame/156 clip은 다음 location quota를 사용한다.

| 위치 | 비율 | 역할과 split |
| --- | ---: | --- |
| `CW000000` | 25% | 공식 sample 직접 통과, train. |
| `CW000002` | 25% | 다른 직접 통과, train. |
| `CW000013` | 15% | 다른 직접 통과, location-disjoint val. |
| `CW000015` | 15% | 다른 직접 통과, location-disjoint test. |
| `CW000001` | 10% | waypoint 인접 측면 관측, train. |
| `CW000014` | 10% | waypoint 인접 측면 관측, train. |

따라서 공식 sample 위치는 충분한 25%를 유지하지만 나머지 위치가 75%여서 한 배경에
과적합되지 않는다. 같은 crosswalk ID는 split을 넘지 않는다. 공식 sample의 exact
capture를 그대로 복제하지 않고 Ego gap, 속도와 capture seed를 바꾼다.

304-frame smoke set의 pedestrian 6 clip은 production 비율을 흉내 내지 않고 위 6개
crosswalk를 정확히 한 번씩 사용한다. 이 단계의 목적은 빈도 학습이 아니라 생성 경로,
Ego 접근, 실제 가시성과 SaveSensorData artifact를 위치별로 검증하는 것이다. Smoke를
통과한 뒤 production batch부터 위 25/25/15/15/10/10 quota를 적용한다.

난이도 quota는 normal visible 35%, fast crossing 25%, occluded run-out 25%, partial
visibility transition 15%다. 공식 sample path의 알려진 가림 시작 방향은 run-out에서
보존한다. 다른 횡단보도의 `occluded_runout`과 `partial_visibility_transition`은 이름만으로
성공 처리하지 않고 live camera projection에서 실제 가림→등장 변화가 확인돼야 accepted
clip으로 승격한다. 지도 구조물이 부적합하면 현실적인 정차 차량 가림을 사용하고,
무관한 obstacle을 횡단보도에 임의 배치하지 않는다.

### Generic structured-random vehicle pilot.

Mission vehicle만 학습하면 모델이 로터리·고주로 배경과 motion을 결합하는 shortcut을
가질 수 있으므로 일반 도로 vehicle family도 후보로 유지한다. 다만 첫 1,504-frame
pipeline gate에는 넣지 않는다. 먼저 mission/static/hard 기준선을 만든 뒤 별도
304--504 frame pilot으로 다음을 검증한다.

- Competition waypoint corridor의 유효 MGeo lane에서만 Ego와 NPC를 배치한다.
- NPC는 연결된 centerline을 따라 lane-following하며 역주행·임의 횡단은 만들지 않는다.
- 일반도로 제한속도 60 km/h와 고주로 예외를 지키면서 initial/target speed, acceleration,
  same/adjacent lane, longitudinal gap, relative approach와 camera sector를 randomize한다.
- 공식 로터리·고주로 spawn anchor와 crosswalk mission anchor는 generic 후보에서 제외한다.
- Clip split은 spawn link와 route segment group 기준으로 묶어 같은 공간이 split을 넘지
  않게 한다.

Pilot이 mission-holdout의 vehicle center AP, velocity error와 1--3초 prediction을
개선하고 mission recall을 손상시키지 않으면 최종 10,000 frame에서 전체의 5%를
generic vehicle로 배정한다. 이 5%는 동적 vehicle 37.44% 내부에서 가져와 mission
vehicle 32.44% + generic vehicle 5%로 구성한다. 효과가 명확할 때만 generic을 최대
8%까지 높이고, 개선이 없으면 기존 mission vehicle 37.44%를 유지한다.

### 2026-08-22 전체 quota 재검토 결론.

- 동적 vehicle 37.44%는 로터리·합류·순항·추월을 모두 담으면서 static과 pedestrian을
  압도하지 않으므로 유지한다. Generic 5%는 위 pilot 통과 후 이 내부에서 재배분한다.
- 동적 pedestrian 12.48%는 대회 run-out 위험과 velocity 학습에 충분하므로 유지하고,
  비율을 늘리는 대신 6개 crosswalk와 난이도 quota로 다양화한다.
- 정적 temporal 35.04%는 정지 vehicle/pedestrian의 zero-velocity target과 obstacle
  geometry를 함께 학습해야 하므로 유지한다.
- 정적 내부 obstacle-focused 약 70%는 30종 asset의 unique placement 부족을 해결하는
  직접적인 quota이므로 유지한다. Mission dynamic clip에는 무관한 obstacle을 섞지 않는다.
- Hard temporal 15.04%는 spawn acceleration, GPS 음영 obstacle·vehicle interaction,
  원거리·가림·camera seam을 격리해 평균 분포를 오염시키지 않는 역할이므로 유지한다.
- 첫 1,504-frame batch의 GPS 전용 13 clip은 obstacle 6 clip과 같은 방향 vehicle
  interaction 7 clip으로 나눈다. 일반 static obstacle은 다른 family에서 충분히
  확보하므로 GPS 랜덤 미션 두 종류의 누락 방지를 우선한다.
- 모든 canonical clip은 8 frame, 12 Hz이며 frame이 아니라 clip/location/link group으로
  split한다.

대회 맵과 공개 sample family를 학습에 활용하는 것은 허용된 domain specialization이며
숨겨진 본 대회 시나리오에 대한 평가 누수와 같지 않다. 반면 내부 test로 선언한 exact
sample clip을 반복 튜닝에 사용하면 내부 성능을 과대평가하는 실제 누수다.

## Split과 평가 세트.

하나의 test만 두지 않고 세 축을 유지한다.

1. `ID random holdout`.
   - 학습과 같은 structured-random generator의 새 seed와 clip을 사용한다.
   - 일반화와 overfitting을 측정한다.
2. `Mission holdout`.
   - 공식 sample scenario의 exact trajectory·capture와 대회 미션 재현 bag을 사용한다.
   - Model selection 동안 학습에 넣지 않는다.
3. `Stress/OOD holdout`.
   - 속도, 거리, occlusion, camera seam과 frame drop tail을 강화한다.
   - 평균 성능보다 failure mode를 진단한다.

최종 제출 모델은 실험 선택이 끝난 뒤 공개 mission data를 학습에 포함한
mission-adapted variant와 계속 격리한 general variant를 별도로 비교할 수 있다. 이때
숨겨진 실제 대회가 최종 test이므로 공개 sample을 최종 fine-tuning에 쓰는 것 자체를
데이터 누수라고 부르지 않는다. 다만 내부 mission-holdout 점수는 더 이상 공정한
최종 보고값으로 사용하지 않는다.

## 다음 구현 gate.

1. V2 generator를 바로 10,000장으로 실행하지 않는다.
2. 304-frame pilot을 vehicle dynamic, pedestrian dynamic, static temporal과 hard
   transition으로 구성한다.
3. simulator-time lag, speed phase, Ego pose, stable ID, velocity GT와 camera 4대
   artifact 완전성을 검사한다.
4. 3-sweep PointPainting dataset과 StreamPETR metadata loader를 모두 smoke한다.
5. SaveSensorData 저속 pilot을 먼저 검증하고 안정적일 때 canonical 12 Hz pilot을 수행한다.
6. 기존 Epoch 65 geometry, tracker-only velocity와 비교한 뒤 V2 비율을 확정한다.

## 실행 명령.

Pilot 생성·설치 전 정적 검증은 다음과 같다.

```bash
python3 dataset/morai_od/scripts/generate_dynamic_object_capture_set.py \
  --config dataset/morai_od/configs/temporal_sequence_capture_v2_pilot304.json \
  --output-root dataset/morai_od \
  --group temporal_sequence_v2_pilot304

python3 dataset/morai_od/scripts/validate_dynamic_capture_set.py \
  dataset/morai_od/manifests/temporal_sequence_v2_pilot304.json

python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
  dataset/morai_od/manifests/temporal_sequence_v2_pilot304.json \
  --network-template \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/EgoNetwork/od_s13p6_186_MN.json" \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

ROS/MORAI가 연결된 뒤 한 clip을 live smoke하려면 다음처럼 실행한다. Pilot manifest의
기본 capture policy가 8 frame, 1/12초이므로 CLI에서 다시 덮어쓰지 않는다.

```bash
python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/temporal_sequence_v2_pilot304_runtime.json \
  --capture-dir /home/luv/morai_cap/temporal_v2_pilot_smoke \
  --motion-mode timed-grouped \
  --scenario dlc001 \
  --scenario-load-attempts 1 \
  --continue-on-scene-error
```

SaveSensorData는 한 snapshot을 쓰는 동안 MORAI wall-clock topic을 약 0.6초까지
늦출 수 있다. 따라서 V2의 1/12초는 host wall-clock 간격이 아니다. 캡처기는 Ego의
이동거리와 측정 속도를 적분한 `sequence_time_sec`을 물리 simulation-time 근사로
사용한다. `triggered_unix`와 MORAI 파일명 시간은 provenance와 I/O 진단용으로만
보존한다. 이 분리가 없으면 저장 I/O 지연을 실제 객체 운동 시간으로 오인하여
velocity GT가 과소 계산된다.

Capture가 끝나면 다음으로 sensor artifact, timing sidecar, frame 간 진행거리와 V2
sequence contract를 검사한다.

```bash
python3 dataset/morai_od/scripts/validate_dynamic_capture_output.py \
  /home/luv/morai_cap/<실제_timestamp_prefix가_붙은_capture_dir>
```

첫 production chunk는 명목 1,500장이 아니라 canonical 1,504장으로 생성한다.

```bash
python3 dataset/morai_od/scripts/generate_temporal_sequence_v2_batch.py \
  --batch-index 1
```

## 2026-08-21 live gate 결과.

- `dso001` static obstacle.
  - 8/8 frame과 Camera 4대·LiDAR artifact가 완전했다.
  - 6개 obstacle track이 모든 frame에 유지됐다.
  - sequence interval 중앙값은 0.151초였다.
- `dpd001` crosswalk pedestrian.
  - 최초 구현은 sensor writer priming과 Ego arm 동안 보행자가 path 끝에 도달해
    stationary clip이 됐다.
  - pedestrian family만 priming과 warm-up을 끄고 capture arm을 0.3m로 낮췄다.
  - 수정 후 동일 pedestrian ID가 8 frame 유지됐고 1.051초 동안 2.949m 이동했다.
- `dhc010` left-lane highway cruise.
  - 최초 18--45 m 시작점은 합류 직후 곡선부여서, 50 km/h로 초기화된 Ego가 control
    mode 전환 전에 접선 방향으로 3.93 m 이탈했다.
  - cruise 시작 범위를 평행 직선부 70--300 m로 옮긴 뒤 scenario verification과
    8/8 SaveSensorData frame이 통과했다.
  - 캡처는 왼쪽 차로 centerline에서 44.56 km/h에 arm됐고 1.061초 동안 13.49 m
    진행했다. 두 NPC track도 8 frame 전체에서 유지됐다.
- `dlc001` highway lane change.
  - Ego target 52.9km/h, capture 시작 45.0km/h 조건을 통과했다.
  - 동적 vehicle 2대가 각각 8 frame 유지됐다.
  - sequence interval 중앙값은 0.150초였고 route deviation·collision은 없었다.
- 세 clip 모두 validator 오류는 0개였다.
  - MORAI Capture Mode가 `/clock`과 Camera/LiDAR ROS topic을 발행하지 않아
    `/clock` availability 경고만 남는다.
  - sensor payload는 SaveSensorData artifact, state time은 Ego/Object header,
    temporal 간격은 `sequence_time_sec`으로 각각 역할을 분리한다.

Production batch 001은 이 수정본으로 primary 188 clips/1,504 frames와 reserve
28 clips/224 frames를 생성했고 두 manifest 모두 정적 검증 오류 0개를 통과했다.
Primary quota는 mission dynamic 93 clips/744 frames, static temporal 66 clips/528
frames, hard transition 29 clips/232 frames다. Static 내부 explicit obstacle family는
45/66 clips로 약 68.2%다.
생성 manifest·scenario·route·validation report는 재현 가능한 산출물이므로 Git에는
넣지 않고 `dataset/morai_od/.gitignore`로 제외한다.

## Runtime 실패 triage 원칙.

- 첫 live 실패 한 번만으로 생성 quota, arm threshold나 actor activation 정책을 바꾸지
  않는다.
- 먼저 같은 manifest와 scenario를 MORAI Ego Network 재연결 후 한 번 더 실행한다.
- 두 실행의 ROS subscriber 복구 시간, control-mode 응답, 실제 Ego 진행거리·속도와 actor
  상태를 비교한다.
- 동일 조건에서 재현되면 그때 network/control, scenario geometry, capture policy 순으로
  원인을 분리한다.
- 안전 검증 완화나 생성기 정책 변경은 재현 로그 또는 artifact 근거가 있을 때만 적용한다.

`dpd001` 첫 12 Hz smoke는 10초 동안 Ego가 14.455 m 이동했지만 speed gate를 넘지 못해
실패했다. 그러나 변경되지 않은 기존 runtime manifest를 다시 로드한 두 번째 실행은 초기
rosbridge 복구 후 0.782초 만에 10.33 km/h로 arm되고 8/8 frame을 완료했다. 따라서 이
사례는 생성기 결함으로 확정하지 않고 일시적인 MORAI/ROS control 적용 상태로 분류한다.

## 2026-08-22 six-crosswalk live gate와 deferred activation.

첫 `dpd002` 실행은 일시적인 ROS 문제가 아니라 재현 가능한 geometry 결함이었다. 생성된
Ego 시작점은 pedestrian과 11.84 m 떨어져 있었지만 pedestrian `activeDistance`는
21.00 m여서 actor가 시나리오 로드 직후부터 횡단했다. 그 결과 Ego가 생성된 횡단보도
중심으로부터 약 0.10 m 떨어진 곳에서 `Man1`과 충돌했다. 수정은 공식 sample인
`dpd001`의 동작을 보존하고 비공식 횡단보도에만 다음과 같이 적용했다.

- `dpd001`은 공식 path와 알려진 출발 방향을 재현하므로 `active_at_load`를 유지한다.
- 나머지 횡단보도 scene은 `deferred_ego_approach`를 사용한다.
- Activation distance는 Ego route clearance와 초기 Ego-actor 거리 사이에 두고 1.5 m
  pre-roll을 확보한다.
- Dynamic-actor readiness는 처음 정지한 pedestrian을 허용하지만, capture arm은 실제
  pedestrian motion을 요구한다. Actor가 움직이기 전에 정지 frame 8장을 저장하지 않는다.
- Route clearance 계산은 10 m보다 큰 불연속 waypoint jump를 제외한다. Sparse global
  route가 거짓 collision-safe window를 만들지 않게 한다.

수정 후 6개 위치 smoke 결과는 다음과 같다.

| Scene | Crosswalk 역할 | 8-frame rate | Actor 이동 | Derived speed | 최소 Ego 거리 |
| --- | --- | ---: | ---: | ---: | ---: |
| `dpd001` | 공식 sample | 11.368 Hz | 0.810 m | 1.289--1.331 m/s | 9.532 m |
| `dpd002` | direct train | 11.997 Hz | 1.506 m | 1.632--3.534 m/s | 5.163 m |
| `dpd003` | direct validation | 11.814 Hz | 1.577 m | 2.631--2.690 m/s | 9.970 m |
| `dpd004` | direct test | 11.728 Hz | 1.397 m | 2.203--2.981 m/s | 7.735 m |
| `dpd005` | adjacent-side train | 11.600 Hz | 1.081 m | 1.786--1.798 m/s | 8.004 m |
| `dpd006` | adjacent-side train | 11.586 Hz | 0.838 m | 1.365--1.407 m/s | 14.360 m |

6개 clip 모두 collision 없이 8/8 SaveSensorData frame을 완료했다. Output validator 오류는
0개였고 유일한 경고는 `/clock` 부재였다. 이 경우 message header time과
`sequence_time_sec`을 계속 사용한다. 4-camera contact sheet를 눈으로 검사했을 때 모든
설정 횡단보도에서 pedestrian이 실제 camera coverage에 들어왔다.

`dpd004/005`의 `occluded_runout`과 `partial_visibility_transition` 명칭은 아직 실제
occlusion 성공으로 인정하지 않는다. Smoke 영상에서 횡단과 부분 camera 관측은 확인됐지만,
구조물 뒤 완전 가림에서 등장하는 전환은 명확하지 않았다. 따라서 geometry smoke로는
유효하지만, production occlusion quota에는 전용 visibility audit를 통과하거나 실제적인
정차 차량 occluder가 추가된 clip만 집계한다.

추가 Instance-mask audit에서도 이 판단이 확인됐다. `dpd004` pedestrian은 CAMERA_1과
CAMERA_2의 8/8 frame에 모두 Instance pixel이 있었고, CAMERA_2 후반 감소는 image-right
boundary clipping이었다. `dpd005`도 CAMERA_3의 8/8 frame에 계속 보였으며
visible-pixel/BBox 비율이 약 0.35--0.46으로 유지됐다. 가림→등장 전환이 아니므로 첫
production batch에서는 두 profile weight를 0으로 비활성화하고 normal visible 60%, fast
crossing 40%로 재배분한다. 실제 occluder placement와 camera-projection gate가 구현된 뒤에만
occlusion profile을 다시 활성화한다.

이 수정본으로 batch 001을 재생성하고 개인용 MORAI launcher에 설치했다. Source manifest와
설치된 runtime manifest가 모두 정적 검증을 통과했다.

- Primary는 188 clips/1,504 frames이며 오류 0개, pedestrian speed-unit audit 경고
  25개다.
- Reserve는 28 clips/224 frames이며 오류 0개, 같은 경고 10개다.
- Speed-unit 경고는 MORAI pedestrian JSON 값의 단위가 schema에 명시되지 않아 유지한다.
  실제 live displacement를 acceptance 근거로 사용한다.

## 2026-08-22 production representative live gate.

Batch 001 설치본에서 고위험 family 8개, 총 64개 저장 frame을 live gate로 검사했다.
최초 실행에서는 `dhc001`, `dlc001`, `dso001`, `dsh001`이 통과했고 `drc001`,
`dmc001`, `drs001`, `dhm001`이 route 검증에서 실패했다. 실패 4개를 변경 없이 다시
실행해 모두 재현한 뒤 원인을 수정했다.

원인은 scenario JSON에 저장한 11--29 km/h Ego 초기 속도였다. ScenarioLoad 직후
rosbridge와 external control이 복구되기 전에 Ego가 접선 방향으로 이동해 route를
1.5--12.3 m 벗어났다. 학습에 필요한 캡처 속도 다양성과는 별개인 load-time race이므로
다음 정책으로 변경했다.

- 모든 Temporal V2 scenario는 load 시 Ego 속도 0 km/h로 시작한다.
- Family별 nominal initial speed는 metadata의 `nominal_ego_initial_speed_kph`에 보존한다.
- External control이 연결된 뒤 기존 target speed까지 가속한다.
- 기존 minimum speed, route progress와 stability gate를 통과한 뒤에만 frame 0을 저장한다.

이 수정으로 `dmc001`, `drs001`, `dhm001`은 8/8 frame을 완료했다. `drc001`은 route
이탈은 사라졌지만 stability gate가 로터리의 정상 횡가속을 가감속으로 오인해 두 번
실패했다. Stability gate를 world acceleration magnitude가 아니라 velocity 방향에 투영한
종방향 acceleration으로 바꿨다. 실제 종방향 가감속은 계속 차단하고 steady-speed turn의
구심가속만 허용한다. 수정 후 `drc001`도 15.88 km/h에서 arm되어 8/8 frame을 완료했다.

최종 대표 family live 결과는 다음과 같다.

| Family scene | 역할 | 결과 |
| --- | --- | --- |
| `drc001` | 로터리 순항 | 8/8, 11.433 Hz. |
| `dmc001` | 고주로 합류 순항 | 8/8, 11.515 Hz. |
| `dhc001` | 고주로 순항·추월 | 8/8, 11.345 Hz. |
| `dlc001` | 고주로 차선 변경 | 8/8, 11.500 Hz. |
| `dso001` | 일반 정적 obstacle | 8/8, 11.593 Hz. |
| `dsh001` | 고주로 정적 obstacle | 8/8, 11.519 Hz. |
| `drs001` | 로터리 spawn transition | 8/8, 11.635 Hz. |
| `dhm001` | 고주로 spawn transition | 8/8, 11.385 Hz. |

세 capture directory를 각각 output validator로 검사한 결과 총 8 clips/64 frames,
오류 0개였다. 경고 8개는 모두 Capture Mode의 `/clock` 부재이며 sensor artifact,
frame metadata와 sequence contract 결손은 없었다.

## 2026-08-22 Batch 001 초기 중단과 로터리 readiness 보정.

첫 primary 실행은 14개 scene을 시도해 3개만 성공하고 11개가 실패하여 중단했다. 대형
console 원문 대신 `summary.json`과 `failures.jsonl`의 작은 집계만 사용한 결과,
실패는 readiness 9건과 collision 2건이었다. 대표 실패 `drc005`를 변경 없이 단독
재실행했을 때 Ego가 34.37 m 이동하고 CtrlCmd, actor motion과 sensor writer가 모두
정상이었지만, 10초 시점 속도 16.43 km/h가 목표의 85%인 17.79 km/h보다 낮고 stable
gate도 충족하지 못해 동일하게 실패했다. 반면 `drc011`은 같은 조건에서 timeout 직전인
9.92초에 겨우 arm되어 8/8 frame을 저장했다. 따라서 ROS 연결 문제가 아니라 곡선
제어와 readiness 정책의 불일치로 확정했다.

로터리 production 정책은 다음과 같이 보정했다.

- `roundabout_circulation`의 capture 최소속도를 target의 85%에서 65%로 낮췄다.
- 로터리에서 constant-speed만 강제하던 0.5초 stable acceleration gate를 제거했다.
  3 m motion arm, actor motion, collision, route와 artifact 검증은 유지한다.
- 로터리 NPC는 18--22 km/h인데 adaptive curve가 Ego 명령을 12 km/h까지 낮추면서
  뒤따르던 NPC가 약 `(-102, 325)`에서 Ego를 추돌했다. 두 roundabout family에만
  `minimum_curve_speed_kph=16`을 적용해 이 비현실적인 catch-up을 줄였다.
- Family별 curve minimum을 scenario runtime override로 전달하도록 generator를
  보강했다. 다른 family의 곡선 속도 정책은 바꾸지 않았다.

수정 후 기존 실패 `drc005`는 3.21초, 13.64 km/h에서 arm되어 8/8, 11.776 Hz로
완료했다. 최종 runtime manifest로 충돌 이력이 있던 `drc002`, `drc004`, `drc007`과
hard-transition `drs001`을 연속 실행한 결과 4/4 clip, 32/32 frame이 성공했고 sequence
rate는 11.254--11.873 Hz였다. 장면 전환 중 rosbridge reconnect 경고 한 번은 자동
복구됐으며 collision, readiness와 artifact 실패는 없었다. Primary 188 scenes와 reserve
28 scenes runtime manifest도 정적 validator 오류 0건을 유지했다.

## 2026-08-22 Batch 001 공간 anchor 감사와 mission-wide 보정.

첫 A 실험을 368 frame에서 중단한 뒤 이미지 payload 전체를 읽지 않고 `summary.json`과
frame별 Ego/Object metadata만 집계했다. 46개 성공 clip 중 당시 완료된 세 vehicle
family의 실제 capture 시작점 최대 간격은 다음과 같았다.

| Family | 완료 clip | 실제 capture 시작점 최대 간격 | 판단 |
| --- | ---: | ---: | --- |
| `roundabout_circulation` | 17 | 9.0 m | 한 짧은 구간에 편향. |
| `highway_merge_cruise` | 10 | 41.9 m | 일부 분산됐지만 접근 초반이 부족. |
| `highway_cruise_overtake` | 19 | 214.5 m | 충분한 연속 공간 분산. |

기존 로터리 18개 scene의 scenario Ego start도 global-route index `1651--1668`에
집중됐다. 모든 scene이 공식 NPC spawn 4의 고정 위치를 route에 투영하고 그보다
18--27 m 앞에서 Ego를 시작했기 때문이다. 더구나 scenario load 속도 0, 3 m motion
arm과 최소속도 gate가 실제 capture 위치를 downstream으로 더 모았다. 목표속도
randomization만으로는 공간 분포가 넓어지지 않았다.

Temporal V2의 네 simple spawn vehicle family를 capture-anchor 중심으로 변경했다.

- `roundabout_sample_spawn`, `roundabout_circulation`은 접근, 합류·진입, 순환, 이탈의
  네 route-index 구간에 exact quota를 배정한다. 각 구간 내부 좌표는 연속 랜덤이다.
- `highway_merge_sample_spawn`, `highway_merge_cruise`는 접근, 합류, 합류 후 안정의 세
  구간에 같은 정책을 쓴다.
- 먼저 원하는 `planned_capture_anchor`를 뽑고, target speed에 도달할 preroll 거리를
  upstream으로 역산해 scenario Ego start와 `capture_motion_arm_m`을 함께 정한다.
  따라서 속도 gate가 캡처 위치를 임의로 결정하지 않는다.
- 로터리 NPC는 공식 `144→133→134→135→102` MGeo 경로를 유지하면서 capture anchor
  주변 phase로 이동한다. 여러 합법 후보 중 Ego와 약 14--28 m 떨어진 후보를 우선해
  객체 소실과 즉시 충돌을 모두 방지한다.
- 기존에 공간 분포가 충분한 고주로 순항·추월, lane-change, 여섯 횡단보도 정책은
  변경하지 않았다. GPS shadow와 static hard family의 좁은 범위는 미션 구역 자체가
  제한적인 의도된 분포이므로 동일 결함으로 보지 않는다.

재생성된 primary의 계획 capture anchor 결과는 다음과 같다.

| Family | clip | 공간 phase quota | anchor 최대 간격 |
| --- | ---: | --- | ---: |
| `roundabout_sample_spawn` | 8 | 2/2/2/2 | 64.3 m |
| `roundabout_circulation` | 18 | 4/5/5/4 | 69.5 m |
| `highway_merge_sample_spawn` | 8 | 2/3/3 | 72.9 m |
| `highway_merge_cruise` | 14 | 3/6/5 | 73.8 m |

로터리 계획 capture anchor와 NPC spawn의 초기 거리는 primary에서 최소 15.6--17.3 m,
중앙값 25.3--29.6 m, 최대 38.9--41.3 m였다. Primary 188 clips와 reserve 28 clips는
source/runtime manifest 모두 validator 오류 0개를 유지했고 개인 MORAI launcher에
재설치했다. 이 변경 중 ROS master가 종료된 상태여서 신규 공간 phase의 live
SaveSensorData smoke는 다음 MORAI 연결 시 수행해야 한다. 기존 368-frame A 폴더는 같은
scene ID의 구형 공간 정책을 포함하므로 신규 runtime manifest로 resume하지 않는다.
