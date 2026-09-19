# Tracking·Prediction production 현재 상태

상태: **CURRENT**
마지막 코드 대조: 2026-09-20, `planning/autoware-universe-to-ros1`

## Production 계약

```text
BEVFusion B3-BF0-F40 E25 DetectedObjects
  -> EKF tracker
  -> UNKNOWN stationary policy
  -> Lanelet2 Map-CV predictor
  -> /perception/object_recognition/objects
```

운영 진입점은 `docs/common_alias.txt`의 `tracking`이다. detector와 localization은 별도
프로세스이며 동시에 다른 detector나 tracker를 같은 canonical topic에 발행하지 않는다.

| 구간 | 현재값 |
|---|---|
| Detection | `/perception/object_recognition/detection/objects` |
| Measurement track | `/perception/object_recognition/tracking/measurement_objects` |
| Motion-policy track | `/perception/object_recognition/tracking/objects` |
| Prediction | `/perception/object_recognition/objects` |
| World frame | `map` |

## Tracker

- motion model: CTRV(`prediction_model=1`).
- detector position measurement noise: `0.40 m`.
- detector velocity fusion: `off`; production E25는 7D detector다.
- detector measurement timestamp state를 current time으로 강제 보상하지 않는다.
- EKF 내부 scheduler가 30 Hz로 prediction copy를 발행한다.
- production `tracking` alias의 maximum measurement age는 `0.50 s`다. generic
  `object_pipeline.launch`와 standalone `tracking_prediction.launch` 기본값 `0.25 s`는
  연구/직접 실행 호환값이며 운영 alias를 대체하지 않는다.
- Vehicle yaw는 1.0 m/s 이상에서 motion 방향과 canonicalize하고 enter/exit
  hysteresis는 `100/80 deg`다.
- 외부 fixed-rate adapter는 `0 Hz`, 즉 비활성이다.

0.50초는 detector frame 지연이나 간헐적 skip에서 track이 즉시 끊기지 않게 하는
운영 허용치다. 0.50초 동안 새 관측 없이 정확하다는 뜻이 아니므로 detector age와
prediction coverage를 함께 감시한다.

## Obstacle/UNKNOWN 정책

`force_unknown_stationary=true`다. 최신 관측은 pose와 box를 갱신하지만 tracker가 만든
가짜 속도·가속도·yaw rate는 prediction 전에 0으로 만든다. 따라서 정적 obstacle은
현재 위치가 반복되는 stationary path가 되고, Vehicle/Pedestrian은 이 정책의 영향을
받지 않는다.

대회에서는 `/Object_topic`이 없으므로 `od_gt`는 production fallback이 아니다. 로컬
부하 분리와 통제 실험에서만 detector replacement로 사용한다.

## Map-CV

- map: `kcity_map/lanelet_map/lanelet2_map_v84.osm`.
- horizon/time step: `3.0 s / 0.1 s`, 현재 pose 포함 31개 pose.
- minimum map-following speed: `0.5 m/s`.
- lane match: 거리 최대 `3.0 m`, yaw 오차 최대 `45 deg`.
- centerline lateral convergence: `1.5 s`.
- start lanelet 최대 3개, 최종 path 최대 6개.
- 같은 downstream Lanelet이며 horizon 전체가 `0.75 m` 이내인 중복 궤적은 하나의
  물리 hypothesis로 묶는다.
- confidence는 heuristic route score를 남은 hypothesis 사이에서 정규화한다. 실제
  calibrated probability로 해석하지 않는다.
- 저속, off-lane, wrong-direction, non-vehicle은 Cartesian CV fallback을 사용한다.

현재 predictor는 CTRV 곡률로 lane rank를 고르는 구조가 아니다. 현재 위치·yaw로
start Lanelet을 점수화하고 routing graph successor와 연결성을 따라 경로를 열거한다.
따라서 고주로 인접 차선 Top-1 의도 추정은 tracker history 없이 완전히 해결되지 않는다.

## 검증과 미구현 항목

모델 선택은 Detection만 보지 않고 같은 mission bag의 Tracking center/AVE, speed error,
Map-CV 1/2/3초 top-1 및 minFDE-K를 함께 본다. 최신 detector 및 tracker 근거는
`docs/bevfusion/README.md`와
`docs/bevfusion/260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`를 따른다.

다음은 계획일 뿐 현재 production에 구현되지 않았다.

- 기본 horizon 5초와 로터리 3~4초의 map-zone-aware 전환.
- tracker history를 이용한 temporal lane association과 Top-1 안정화.
- detector 9D velocity를 tracker update에 fusion.
- Planning의 추월 gap 판단을 위한 track-age/confidence gate.

위 항목은 localization 및 Base4 velocity pilot과 동시에 바꾸지 않는다. 기존 v60
PointPainting/Map-CV 수치와 초기 도입 계획은 [`legacy/`](legacy/)에만 보존한다.
