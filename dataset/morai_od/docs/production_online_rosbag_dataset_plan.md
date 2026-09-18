# Production-Online ROS Bag Dataset 계획

마지막 갱신: 2026-09-18.

## 목적

Base3 시나리오 생성기의 공간·객체 다양성은 유지하면서 센서 취득 경로를
`SaveSensorData`에서 실제 production과 같은 UDP/ROS online 경로로 바꾼다.
Base3 offline val은 높지만 mission rosbag에서 성능이 하락하는 원인을 단순 객체 수가
아닌 입력 domain 차이까지 포함해 줄이는 것이 목적이다.

가칭은 `Base3-Online`이다. 기존 문서의 3-sweep velocity용 `Base4-Temporal`과 같은
이름을 쓰지 않는다. Pilot 결과 후 최종 dataset 이름을 확정한다.

## 핵심 방식

1. `rosbag record`를 실행한 상태에서 기존 시나리오 생성기·주입기를 순차 실행한다.
2. MORAI가 실제로 발행하는 압축 카메라, LiDAR, Ego, Object 토픽을 저장한다.
3. `SaveSensorData` artifact는 학습 입력으로 사용하지 않는다.
4. 각 sample은 LiDAR header timestamp를 기준으로 production synchronizer와 동일한
   카메라 선택 정책을 적용해 만든다.
5. Ego/Object 상태는 LiDAR 시각으로 보간해 LiDAR 좌표계 3D GT를 만든다.

이 방식으로 다음 online 특성을 학습 데이터에 포함한다.

- JPEG 압축 영상과 production 색상·resize 전처리.
- 카메라--LiDAR timestamp 차이.
- UDP 수신 jitter와 실제 ROS callback 순서.
- Ego와 동적 객체의 연속 motion.
- production과 동일한 sensor configuration과 calibration.

## 기록 토픽 계약

최소 기록 대상은 다음과 같다.

```text
/sensor/lidar1
/sensor/camera/front/compressed
/sensor/camera/front_left/compressed
/sensor/camera/front_right/compressed
/sensor/camera/back/compressed
/Ego_topic
/Object_topic
/tf
/tf_static
/dataset/scenario_marker
```

필요하면 `/clock`, `/ReplayInfo_topic_out`, simulator process state를 진단용으로
추가하되 학습 sample의 기준 시각은 LiDAR header다.

## Scene marker 계약

하나의 bag에 여러 scene을 기록하므로 시나리오 전환 프레임을 자동 제외할 수 있어야
한다. 자동 수집기는 `/dataset/scenario_marker`에 최소 다음 event를 발행한다.

```json
{"event":"scene_loading","scene_id":"...","family":"..."}
{"event":"capture_start","scene_id":"...","family":"..."}
{"event":"capture_end","scene_id":"...","family":"..."}
```

`capture_start`는 ScenarioLoad 완료만으로 발행하지 않는다. Ego pose, expected object,
LiDAR, 카메라 네 개가 모두 안정화되고 최소 settling window를 통과한 뒤 발행한다.
`capture_end` 이후 다음 `scene_loading`까지의 데이터는 학습에서 제외한다.

## 자동 수집기 변경 방향

기존 `/SaveSensorData` 성공을 기다리는 실행 경로를 무조건 우회하지 않는다. 명시적인
backend를 추가한다.

```text
--capture-backend savesensordata   # 기존 Base3 계약
--capture-backend rosbag-online    # 신규 online 계약
```

`rosbag-online`은 다음 상태 순서를 가진다.

```text
scene load
  -> expected Ego/Object 및 센서 안정화
  -> capture_start marker
  -> rolling Ego 또는 동적 객체 운행
  -> 유효 관찰 window
  -> capture_end marker
  -> next scene
```

ROS 연결 일시 불안정은 생성기 검증 로직을 임의로 강화해 숨기지 않는다. 기존 운영
원칙대로 MORAI--rosbridge 재연결을 먼저 요청하고 동일 scene을 재시도한다.

## Dataset builder 계약

1. marker의 유효 window 밖 프레임을 제거한다.
2. LiDAR timestamp를 기준으로 각 카메라 ring buffer에서 production과 동일한 정책으로
   가장 가까운 이미지를 선택한다.
3. 허용 camera age를 넘긴 sample은 제외하고 원인을 집계한다.
4. `/Ego_topic`과 `/Object_topic` history를 LiDAR 시각에 보간한다.
5. scenario manifest의 asset, class, 크기와 Object 상태를 결합해 3D GT를 만든다.
6. 기존 LiDAR visibility 정책을 유지한다.
   - 0-point GT 제거.
   - 1--4-point sparse GT는 positive supervision에서 제외.
   - 5-point 이상을 정상 positive로 사용.
7. frame별 원시 Object 위치 미분값을 학습 GT처럼 노출하지 않는다. 필요할 때만
   진단 namespace에 기록한다.

## 중복과 용량 제어

30 Hz 카메라를 scene 전체에서 모두 독립 single-frame sample로 쓰지 않는다.

- 정적 scene은 scene당 대표 LiDAR frame 3--8장만 선택한다.
- 동적 Vehicle/Pedestrian은 motion phase를 보존하되 시간적으로 지나치게 가까운
  frame을 제거한다.
- train/val/test는 frame random split이 아니라 scene/run 단위로 분리한다.
- bag은 batch 또는 약 4 GiB 단위로 LZ4 split한다.
- 긴 ScenarioLoad 대기 구간과 전환 구간은 bag에 남아도 dataset index에서 제외한다.

같은 rosbag을 반복 재생해 sample 수만 늘리는 것은 데이터 다양성 증가로 인정하지
않는다. 매 scene의 Ego/object/asset/geometry randomization이 실제로 달라야 한다.

## 실행 단계

### Phase 0. 현재 baseline 고정

- BEVFusion E1--E40 corrected-range30 validation과 best checkpoint 선정을 완료한다.
- 기존 8-bag production 기준을 고정한다.

### Phase 1. 30--50 scene pilot

- Vehicle, Pedestrian, Obstacle, empty/hard-negative를 포함한다.
- 정적 scene과 실제로 움직이는 Pedestrian/Vehicle scene을 모두 넣는다.
- rosbag topic rate, timestamp, marker, frame coverage, disk rate를 감사한다.
- 임의 sample을 RViz와 카메라 projection으로 확인한다.

### Phase 2. 변환기 검증

- production synchronizer가 선택한 카메라와 dataset builder 선택 결과가 같은지
  sample 단위로 비교한다.
- GT box를 raw LiDAR와 RViz에 중첩해 위치, yaw, 크기를 확인한다.
- 시나리오 전환·teleport 프레임이 index에 0개인지 확인한다.
- 동일 scene 연속 frame 과대표집 통계를 낸다.

### Phase 3. 작은 controlled fine-tuning

- 약 1,000--2,000 independent frame으로 시작한다.
- Base3 corrected best를 초기값으로 짧게 fine-tuning한다.
- Base3-only와 Base3+Online을 동일 offline val 및 동일 8개 mission bag에서 비교한다.
- GT sampling, 해상도 변경, temporal modeling을 첫 실험에 동시에 섞지 않는다.

### Phase 4. 확대 여부 결정

다음 조건을 만족할 때만 본 수집으로 확대한다.

- online bag에서 class별 AP/recall 또는 false positive가 반복적으로 개선됨.
- Base3 frozen val과 box geometry가 허용 범위 이상 퇴화하지 않음.
- runtime latency와 input contract가 production과 동일함.
- 개선이 특정 bag 하나의 반복 frame에만 의존하지 않음.

## 성공 판단

offline val 상승만으로 성공 처리하지 않는다. 같은 threshold와 evaluator를 사용한
mission rosbag에서 다음을 본다.

- Vehicle/Pedestrian/Obstacle AP, precision, recall, F1.
- mATE, mASE, mAOE와 거리·방향 slice.
- 정적 배경 hard-negative false positive.
- tracker F1/MOTA/AVE와 Map-CV prediction FDE.
- inference p50/p95 latency, output Hz, drop/skip, sensor-stamp age.

## 보류 사항

- 실제 대회 bag에는 `/Object_topic` GT가 없으므로 자동 정답 dataset으로 바로 쓰지
  않는다. 수동 라벨, hard-negative audit, 제한적 pseudo-label 용도로 분리한다.
- ReplayInfo-only bag을 MORAI에서 복원하는 방식은 `/Ego_topic` 복원 및 loop boundary
  문제가 있으므로 본 수집의 기본 경로로 확정하지 않는다.
- LiDAR scan 내부 Ego/Object motion distortion는 timestamp 보간만으로 완전히 해결되지
  않는다. 필요하면 별도 deskew/rolling-scan 연구로 분리한다.
