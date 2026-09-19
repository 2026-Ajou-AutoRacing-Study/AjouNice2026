# Temporal V2 Base2 Batch 001--003 감사와 후속 구현 계획

이 문서는 2026-08-23 기준 `base2` Batch 001--003 실수집 결과와 이후 대화에서
확정한 구현 방향을 보존한다. Batch 004--007 수집 이후 다음 작업을 정할 때 이 문서를
우선 context로 사용한다.

## 1. 실수집 상태

수집 session은 다음과 같다.

```text
/home/luv/morai_cap/260822_212455_temporal_v2_base2_10k
```

| Batch | 요청 clip | 성공 clip | 실패 clip | 성공 frame |
| --- | ---: | ---: | ---: | ---: |
| 001 | 188 | 182 | 6 | 1,456 |
| 002 | 188 | 181 | 7 | 1,448 |
| 003 | 188 | 180 | 8 | 1,440 |
| 합계 | 564 | 543 | 21 | 4,344 |

- 성공 clip은 모두 8 frames다.
- Sequence rate는 평균 약 11.65 Hz다.
- Source/runtime validator는 오류 0개다.
- Batch 003의 `t2base2b003p_dmx006`에서 `Road_Sign_Mark` 하나가 지면 아래로
  낙하했다. 같은 asset의 나머지 배치는 안정적이므로 asset 전체를 제거하지 않는다.
- Capture runner는 metadata상 static actor가 생성 위치에서 3 m 이상 이탈하거나,
  capture 전 0.5초 동안 0.10 m 이상 움직이면 scene을 실패 처리하는 stability gate를
  적용한다.
- Batch 004--007은 같은 `base2`, seed base `260826000`, 동일 baseline signature로
  생성·설치됐다. 기존 session에 `--resume`로 연결한다.

## 2. Velocity GT의 현재 저장 상태

현재 SaveSensorData artifact 자체에는 velocity label이 없지만, capture runner가
`/Ego_topic`과 `/Object_topic`을 구독해 `dynamic_metadata/*.frames.jsonl`에 함께
저장한다.

Batch 001--003의 실제 completeness는 다음과 같다.

- Ego `velocity_raw`: 4,344/4,344 frames, 누락 0개.
- Actor `morai_velocity_raw`: 11,857/11,857 actor-frames, 누락 0개.
- `EgoVehicleStatus.velocity`의 메시지 정의 단위는 m/s다.
- `ObjectStatus.velocity`의 메시지 정의 단위는 km/h다.
- 연속 위치 차분으로 Ego와 actor의 `derived_velocity_mps`도 clip finalize 단계에서
  생성한다.

Raw와 derived는 다음처럼 구분한다.

| 값 | 의미 | Production 역할 |
| --- | --- | --- |
| MORAI raw velocity | Simulator가 제공한 순간 velocity | 단위·좌표 변환 후 primary GT 후보 |
| Derived velocity | 동일 ID의 연속 위치를 sequence time으로 차분 | 검증, fallback, confidence와 auxiliary loss |

Derived velocity는 raw velocity가 없어서 만든 대체값이 아니다. 일정한 state 지연은 위치
차분에서 상쇄될 수 있지만, 개별 sensor timestamp에 actor state를 직접 보간한 값도 아니다.

Batch 001--003의 clip 양 끝 frame을 제외하고 raw speed를 단위 변환해 derived speed와
비교한 결과는 다음과 같다.

| Class | 표본 | 절대 차이 p50 | 절대 차이 p90 | 0.1 m/s 이내 |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 1,772 | 0.019 m/s | 1.769 m/s | 80.3% |
| Pedestrian | 447 | 0.025 m/s | 0.349 m/s | 86.1% |

순항 구간 중앙값은 매우 잘 맞지만 vehicle tail은 크다. 급가감속, spawn transition,
가변 callback age, ID 변화와 sequence-time 근사 오차를 분리하지 않고 derived velocity를
단독 정답으로 사용하지 않는다.

## 3. SaveSensorData timestamp 한계

현재 frame 본문의 Ego/Object GT는 SaveSensorData trigger 직전에 callback된 최신 상태다.

```text
latest EgoStatus ------> frame.ego
latest ObjectStatus ---> frame.objects
SaveSensorData trigger -> Camera/LiDAR snapshot 요청
```

- Ego와 Object message timestamp는 서로 다를 수 있다.
- `ego_state_age_sec`, `object_state_age_sec`로 trigger 대비 age를 저장한다.
- Timing sidecar는 trigger와 callback monotonic time이 가장 가까운 Ego, Object, Clock,
  Camera와 LiDAR ROS sample을 별도로 연결한다.
- Frame 본문의 Ego/Object를 이 근접 sample로 교체하거나 sensor 시각까지 외삽하지는 않는다.
- SaveSensorData 파일명의 timestamp stem은 같은 snapshot artifact grouping에는 쓸 수 있지만,
  Camera/LiDAR의 독립적인 실제 exposure timestamp라고 검증되지 않았다.
- Capture Mode에서는 Camera/LiDAR ROS topic이 나오지 않을 수 있으므로 sensor header를
  항상 얻을 수도 없다.

따라서 현재 dataset에서는 정확한 `sensor-aligned GT`를 주장하지 않는다. 재수집 없이
구현 가능한 최선은 timing sidecar의 앞뒤 Ego/Object sample을 이용한
`trigger-aligned GT`이며, 이것도 SaveSensorData sensor exposure와 동일하다고 가정하지
않는다.

향후 rosbag 기반 dataset으로 전환하면 LiDAR header timestamp를 기준으로 Ego/Object
history를 보간하고 Camera는 각 header timestamp의 Ego pose로 변환하는 구조를 사용한다.

## 4. Production velocity label builder 계획

Batch 001--007 수집이 끝난 뒤 geometry 학습용 dataset을 만들기 전에 다음 builder를
구현한다.

1. SaveSensorData native BBox와 position annotation은 geometry GT로 유지한다.
2. Trigger에 가장 가까운 MORAI raw velocity를 읽는다.
3. Actor raw km/h를 m/s로 변환하고 MORAI actor frame, `morai_map`, Ego/LiDAR frame의
   좌표 변환을 명시적으로 검증한다.
4. Trigger 앞뒤 동일 ID sample이 모두 짧은 허용시간 안에 있을 때만 velocity를
   제한적으로 보간한다.
5. Raw와 derived의 speed·direction 일치도를 계산한다.
6. 다음 필드를 detector dataset annotation에 저장한다.

```yaml
velocity_target_mps: {x: 0.0, y: 0.0}
velocity_source: morai_raw_nearest
velocity_reference: save_sensor_trigger
velocity_time_offset_sec: 0.0
velocity_confidence: 1.0
derived_velocity_mps: {x: 0.0, y: 0.0}
raw_derived_speed_error_mps: 0.0
velocity_valid: true
```

7. 동일 ID가 최소 3 frames 유지되지 않거나 위치가 점프하면 velocity loss를 mask한다.
8. Static actor는 raw와 derived가 모두 0에 가깝고 static stability gate를 통과할 때만
   확정적인 zero velocity label을 준다.
9. Timestamp offset과 raw-derived 불일치가 클수록 velocity loss weight를 낮춘다.
10. Derived-only label은 production 후보가 아니라 진단용 ablation으로만 유지한다.

추천 학습 ablation은 다음과 같다.

| 실험 | Velocity target | 목적 |
| --- | --- | --- |
| V0 | 기존 tracker-only, OD velocity 없음 | 현재 production 대조군 |
| V1 | 변환한 MORAI raw velocity | Velocity head 기본 효과 |
| V2 | Raw + timestamp/consistency confidence mask | Production 후보 |
| V3 | Derived velocity only | Label source 진단용 |

## 5. 객체 상대 방향 분포 감사

성공한 543 clips, 4,344 frames에서 Ego 50 m 안의 frame-object observation을 집계했다.
방향 sector는 전방 ±60도, 측방 60--120도, 후방 120도 이상이다.

| Class | 관측 수 | 전방 | 측방 | 후방 |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 1,942 | 67.4% | 5.5% | 27.1% |
| Pedestrian | 992 | 76.9% | 15.9% | 7.2% |
| Obstacle | 8,149 | 62.9% | 13.2% | 23.9% |

Vehicle track을 clip 내 중앙 위치로 한 번만 집계해도 전방 67.9%, 측방 4.9%, 후방
27.2%로 거의 같다. 전체 dataset이 후방 차량에 지배되는 상태는 아니다.

그러나 움직이는 vehicle을 mission별로 분리하면 로터리 편향이 확인된다.

| 구간 | 전방 | 측방 | 후방 |
| --- | ---: | ---: | ---: |
| 로터리 | 38.2% | 4.5% | 57.3% |
| 고주로 | 72.9% | 6.1% | 21.0% |
| GPS 음영 vehicle interaction | 100.0% | 0.0% | 0.0% |

Batch 004--007은 중단하지 않는다. 전체 수집 후 로터리 전방·측방 add-on을 추가하며
기존 후방 clip도 후방카메라와 합류 이후 상황을 위해 유지한다.

로터리 add-on의 목표는 다음과 같다.

| Phase | 목표 share | 내용 |
| --- | ---: | --- |
| 접근·양보 | 30% | Ego 진입 전, 순환 NPC가 전방·좌측에서 접근 |
| 합류 직전 conflict | 25% | Ego 진입점 근처, NPC가 전방·측방 통과 |
| 내부 순환 | 30% | 합류 후 앞차·인접 차량 |
| 탈출·후방 | 15% | 후방 차량과 exit 상황 |

최종 moving vehicle 방향 목표는 전방 50--55%, 측방·나란히 10--15%, 후방
30--35%다. 전체 10k 감사 후 약 40--80 clips의 targeted add-on부터 검토하며,
전체 dataset 재수집을 기본안으로 두지 않는다.

## 6. Ego 공간 coverage 검증 계획

Static temporal 비율을 늘린 목적 중 하나는 competition route 전역의 Ego 위치와 배경을
일반화하는 것이다. Generator의 planned anchor만으로 이 목적 달성을 판단하지 않는다.
Ego 가속, motion arm, stable-speed gate와 scene failure 때문에 실제 frame-0 위치는
계획과 달라질 수 있다.

Batch 001--007 완료 후 read-only 후처리 도구를 구현한다.

- 성공 clip의 frame-0 Ego 위치를 한 번씩 집계한다.
- 8-frame Ego 궤적은 짧은 선으로 표시하되 spatial quota에는 중복 집계하지 않는다.
- Lanelet map 위 RViz Marker를 mission dynamic, static temporal, hard transition,
  family와 Train/Val/Test namespace로 분리한다.
- Ego heading은 작은 화살표, speed는 색상 또는 별도 histogram으로 표시한다.
- Route arc-length를 10--20 m bin으로 나눠 clip 수, family 구성, 최대 미관측 gap을
  계산한다.
- Lanelet ID별 clip 수와 Train/Val/Test 공간 분포를 계산한다.
- 2D heatmap과 family별 overlay를 생성한다.
- Ego 위치뿐 아니라 heading, speed, actor 상대 방향·거리, motion phase를 함께 본다.

Coverage가 부족하면 전체를 다시 수집하지 않고 비어 있는 route bin과 family만 targeted
add-on으로 생성한다. 분포가 충분하면 추가 수집 없이 velocity label builder와 학습으로
넘어간다.

## 7. 다음 스텝 요청 시 제안 순서

사용자가 다음 작업을 물으면 다음 순서로 제안한다.

1. Batch 004--007 수집 완료 여부와 disk/artifact/failure 감사.
2. Batch 001--007 전체 class, family, split, range, direction과 motion 분포 산출.
3. Ego Lanelet RViz marker, spatial heatmap과 route-bin coverage 구현.
4. 로터리 전방·측방 targeted add-on 필요량 확정 및 생성.
5. MORAI raw velocity 단위·좌표계 변환 smoke와 trigger-nearest offset 감사.
6. Production velocity label builder와 loader smoke.
7. V0--V3 velocity ablation 준비.
8. 사용자가 직접 장시간 학습을 시작하기 직전에 명령어와 W&B logging을 확정.

이 순서는 수집 분포의 결함을 모른 채 모델부터 학습하거나, 정확한 sensor timestamp가 없는
데이터에 과도한 보간 정밀도를 가정하는 것을 방지한다.
