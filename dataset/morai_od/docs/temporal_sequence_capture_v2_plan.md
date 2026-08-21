# MORAI temporal sequence capture V2 계획.

## 구현 상태.

- 2026-08-21 기준 V2 offline generator와 validator를 구현했다.
- Canonical pilot은 38 clip x 8 frame = 304 frame이다. `1,500` 단위 production은
  clip을 자르지 않기 위해 188 clip x 8 = 1,504 frame으로 생성한다.
- `temporal_sequence_capture_v2_pilot304.json`은 Sunny 13시에서 로터리, 고주로
  합류·순항·차선 변경, 횡단 보행자, 일반·고주로 static obstacle과 GPS 음영 hard
  case를 포함한다.
- Personal launcher에서 자동 가능한 obstacle 30종을 V2 asset deck으로 순환시킨다.
  Pilot 69 unique placement에서 30종이 모두 최소 1회 선택되는 것을 검증했다.
- Production batch는 아래 1,415-placement quota를 smooth weighted round-robin으로
  interleave한다. 따라서 한 asset quota를 모두 소진한 뒤 다음 asset으로 넘어가지 않고,
  1,504-frame 첫 batch에서도 30종이 전부 등장한다.
- 고주로 lane-change route는 세 MGeo centerline을 arc-length로 대응시키고 quintic
  smoothstep으로 연결한다. Entry/middle/exit clip의 capture arm 위치를 따로 둔다.
- Static highway clip은 clip 단위 right/middle/left 60/25/15 weighted lane policy를
  사용한다. 8-clip production batch에서는 5/2/1 clip으로 반올림된다.
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
  primary 1,504 frame 및 reserve 160 frame을 생성한다.
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
- 현재, 약 0.15초 전, 약 0.30초 전 sweep을 실제 timestamp로 선택한다.
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
5. SaveSensorData 5 Hz 상당을 먼저 검증하고 안정적일 때만 10 Hz pilot을 수행한다.
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
기본 capture policy가 8 frame, 0.15초이므로 CLI에서 다시 덮어쓰지 않는다.

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
늦출 수 있다. 따라서 V2의 0.15초는 host wall-clock 간격이 아니다. 캡처기는 Ego의
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
20 clips/160 frames를 생성했고 두 manifest 모두 정적 검증 오류 0개를 통과했다.
Primary quota는 mission dynamic 93 clips/744 frames, static temporal 66 clips/528
frames, hard transition 29 clips/232 frames다. Static 내부 explicit obstacle family는
45/66 clips로 약 68.2%다.
생성 manifest·scenario·route·validation report는 재현 가능한 산출물이므로 Git에는
넣지 않고 `dataset/morai_od/.gitignore`로 제외한다.
