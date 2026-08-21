# MORAI temporal sequence capture V2 계획.

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
- 고주로 순항·추월: 40--60 km/h.

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

현재 Planning prior를 반영한 고주로 순항·추월 clip의 초기 quota는 다음과 같다.

| Ego route 유형 | 비율 |
| --- | ---: |
| 공식 global route/right lane 유지 | 55% |
| middle lane 유지 | 20% |
| left lane 유지 | 10% |
| 실제 lane-change sequence | 15% |

Lane-change frame까지 포함한 최종 차로 점유는 대략 right 55--60%, middle 27--30%,
left 12--15%를 목표로 한다. Planning에 자동 추월이 production으로 들어오면
45/35/20 수준으로 재조정할 수 있으나, 현 단계에서 35/40/25는 middle/left를
과대표집하므로 사용하지 않는다. NPC는 절대 차로 번호와 별개로
`relative_lane`, ahead/alongside/behind와 relative speed quota를 관리한다.

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
| 동적 차량 sequence | 4,504 | 563 | 45.04% |
| 동적 보행자 sequence | 1,496 | 187 | 14.96% |
| 정적 temporal sequence | 2,496 | 312 | 24.96% |
| Hard temporal sequence | 1,504 | 188 | 15.04% |
| 합계 | 10,000 | 1,250 | 100% |

Frame 수 외에도 clip 수, simulator-time 길이, unique placement, stable actor track,
속도 bin, 거리 bin과 visibility duration을 함께 보고한다. 연속 frame을 독립 장면처럼
집계하지 않는다.

Obstacle은 velocity regression에서 제외하고 online output을 `(0,0)`으로 고정하는
방향을 우선한다. Vehicle과 pedestrian은 stationary/moving 비율과 속도 bin을 클래스별로
관리하여 zero-velocity collapse를 막는다.

### 정적 temporal 내부 구성.

정적 25%는 velocity가 0인 same-class vehicle/pedestrian, obstacle과 background를
학습하기 위한 절충안이다. 그보다 낮으면 기존 1,500-frame 학습에서 확인한 obstacle
표본 부족이 반복되고, 크게 높이면 신규 수집의 주 목적인 moving velocity 학습을
침범한다. 정적 내부 obstacle-focused 70%는 전체 데이터의 약 17.5%이며 hard와 mixed
표본까지 합치면 obstacle 관련 frame은 전체의 약 20--22%가 된다.

| 정적 family | Frame | Clip | 정적 내 비율 |
| --- | ---: | ---: | ---: |
| obstacle-focused | 1,752 | 219 | 70.2% |
| stopped vehicle | 376 | 47 | 15.1% |
| standing pedestrian | 128 | 16 | 5.1% |
| realistic mixed | 120 | 15 | 4.8% |
| sparse/background | 120 | 15 | 4.8% |
| 합계 | 2,496 | 312 | 100% |

Hard temporal 중 304 frame, 38 clip을 원거리 작은 obstacle, 부분 가림, camera seam,
길거나 높은 형상과 밀집 배치에 사용한다. 이는 mission family 안에 무관한 obstacle을
섞는 방식이 아니라 별도 hard family다.

같은 clip의 연속 8 frame은 8개의 독립 사례로 세지 않는다. Obstacle-focused clip당
5--7개, 평균 6개, mixed clip당 평균 2개, hard obstacle clip당 평균 3개를 사용하면
예상 unique placement는 다음과 같다.

| 출처 | 계산 | unique placement |
| --- | ---: | ---: |
| obstacle-focused | 219 clip x 6 | 1,314 |
| realistic mixed | 15 clip x 2 | 30 |
| hard obstacle | 38 clip x 3 | 114 |
| 합계 |  | 약 1,458 |

30개 자동 가능 asset을 독립 균등 난수로만 선택하면 asset당 평균은 약 48.6회지만,
30종 중 하나 이상이 35회 미만으로 선택될 확률이 약 39%다. 로드 실패와 가시성
실패까지 더해지므로 pure random sampling은 사용하지 않는다. 부족한 asset을 우선하는
quota deck을 사용하고 실패 placement는 quota 완료로 집계하지 않는다.

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
2. 300-frame pilot을 vehicle dynamic, pedestrian dynamic, static temporal과 hard
   transition으로 구성한다.
3. simulator-time lag, speed phase, Ego pose, stable ID, velocity GT와 camera 4대
   artifact 완전성을 검사한다.
4. 3-sweep PointPainting dataset과 StreamPETR metadata loader를 모두 smoke한다.
5. SaveSensorData 5 Hz 상당을 먼저 검증하고 안정적일 때만 10 Hz pilot을 수행한다.
6. 기존 Epoch 65 geometry, tracker-only velocity와 비교한 뒤 V2 비율을 확정한다.
