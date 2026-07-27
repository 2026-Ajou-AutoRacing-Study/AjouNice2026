# Tracking & Prediction 파이프라인 v1 (GT 기반)

## 목표
Object Detection(BEVFusion/pointpainting)이 아직 파인튜닝 중이라 성능이 낮으므로,
MORAI GT(`/Object_topic`)를 "이미 검출된 결과"인 것처럼 취급해서 Tracking →
Prediction 파이프라인을 먼저 완성한다. 이후 OD가 준비되면 `tracking_node`의
입력만 실제 OD 결과로 바꾼다 (출력 인터페이스는 그대로 유지).

이 설계는 localization 팀이 `Ego_topic`과 실제 localization 결과를 모두
`localization/kinematic_state`라는 동일한 토픽명으로 발행해서, 하위(planning)
노드가 소스를 신경 안 써도 되게 한 방식을 그대로 따른 것이다.

## 패키지 구조
```
tracking_prediction/
├── msg/
│   ├── TrackedObject.msg / TrackedObjectArray.msg
│   └── PredictedTrajectoryPoint.msg / PredictedObject.msg / PredictedObjectArray.msg
├── scripts/
│   ├── tracking_node.py       # /Object_topic -> /tracking/objects (dummy ID)
│   ├── prediction_node.py     # /tracking/objects -> /prediction/objects (CTRA)
│   └── eval_ctra_prediction_v2.py  # rosbag 오프라인 CTRA 정확도 평가
└── launch/
    ├── tracking.launch
    ├── prediction.launch
    └── tracking_and_prediction.launch   # bringup에서 include용
```

## 토픽 흐름
```
/Object_topic (morai_msgs/ObjectStatusList, GT)
        │  tracking_node.py (dummy ID 배정, 재식별 로직 없음)
        ▼
/tracking/objects (tracking_prediction/TrackedObjectArray)
        │  prediction_node.py (CTRA로 여러 horizon 예측)
        ▼
/prediction/objects (tracking_prediction/PredictedObjectArray)
        │
        ▼
   planning (예정)
```

## Tracking 설계 의도
실제 3D detector는 프레임마다 안정적인 track_id를 주지 않기 때문에 (그게 바로
tracking이 풀어야 할 문제), GT를 detection인 것처럼 다루는 이 단계에서도 일부러
**진짜 데이터연관(재식별) 로직 없이** 매 프레임 보이는 순서대로 1,2,3...으로
dummy ID를 부여한다. 이렇게 해야 나중에 실제 OD 결과로 교체했을 때 파이프라인
구조가 그대로 유지된다.

MOT 표준 평가지표(MOTA/MOTP 등)는 이번 v1에서는 만들지 않았다 (후순위, 파이프라인
완성이 우선).

## Prediction 설계 — CTRA (등yaw rate + 등가속도)
- 상태: `[x, y, theta, v, omega, a]`
- 딥러닝 기반 예측은 이번 단계에서 배제 (물리 모델로 우선 검증)
- `omega`(yaw rate)는 GT에 직접 없어서, 직전 프레임과 위치가 가장 가까운
  같은 클래스 객체를 찾아 heading 변화량으로 유한차분 추정한다 (`prediction_node.py`
  내부에서만 쓰는 보조 로직, tracking의 dummy id와는 무관).
- 수식 근거: Autoware.Auto `motion_model` 문서(CATR)
  https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/motion-model-design.html
  - `omega == 0`(직진) 분기는 고전 등가속도 공식(`거리=v*t+1/2*a*t²`)으로 정확히 환원됨 (구현 검증용 sanity check로 씀)
  - `omega != 0`(회전) 분기는 `1/omega`, `1/omega²` 항이 있어 omega가 0에
    가까우면 수치적으로 불안정 → `OMEGA_EPS`(현재 1e-3) 이하면 강제로 직진
    공식으로 전환

## 알려진 한계 / TODO
1. **map-based 필요 여부 미확정**: `eval_ctra_prediction_v2.py`로 회전 구간
   (yaw_rate 임계값 초과)과 직진 구간의 오차를 분리해서 측정하도록 만들어뒀음.
   교차로/로터리 rosbag(`sensor_cam_raw_roundabout_1/2`)으로 실측 후 결정 예정.
2. **감쇠 가속도(decaying acceleration) 미적용**: Autoware 공식
   `map_based_prediction`은 "예측구간 내내 가속도 일정 가정은 비현실적"이라
   판단해 지수함수로 감쇠시키는 모델을 쓴다. v1에서는 순수 CTRA(가속도 상수)만
   구현했고, 평가 결과를 보고 필요하면 감쇠 모델을 추가하는 것을 다음 버전에서
   검토한다.
3. **heading 단위 가정 미검증**: MORAI `heading`이 degree라고 가정하고
   radian으로 변환했음 (`MORAI_HEADING_IN_DEGREES = True`). 실측으로 재확인 필요.
4. **position/velocity 좌표계**: `/Object_topic`을 그대로 통과시키고 있어서,
   world 좌표계인지 ego-relative인지는 launch에서 어떤 소스를 물리느냐에 따라
   달라짐. 나중에 실제 OD 결과(라이다 프레임 기준일 가능성 높음)로 바꿀 때
   좌표계 일치 여부를 확인해야 함.
5. **CTRA용 EKF 자코비안 미구현**: 지금은 상태를 그대로 예측만 하는 결정론적
   버전이라 공분산(불확실성)을 전파하지 않음. 실제 노이즈가 있는 OD 결과를
   붙일 때는 EKF로 확장이 필요할 수 있음 (자코비안은 SymPy로 자동 생성 권장,
   `omega!=0` 분기가 삼각함수+`omega`의 2,3제곱까지 있어 손 유도는 오류 위험).

## 참고 자료
- Houenou et al., 2013 — Vehicle trajectory prediction based on motion model and maneuver recognition (CTRA 표준 baseline)
- SAPI, 2024 — Surroundings-Aware Vehicle Trajectory Prediction at Intersections (arxiv.org/html/2306.01812)
- MHA-JAM, 2020 (arxiv.org/pdf/2005.02545) — 여러 물리모델 앙상블(physics oracle)
- Autoware `multi_object_tracker` — enable_delay_compensation/publish_rate (ROS2 전용이라 로직만 참고)
- Autoware.Auto `motion_model` — CVTR/CATR 클래스 및 수식 유도
