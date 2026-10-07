# Base5 정적 높이 보완 — EVIDENCE / 2026-10-07

## 결론

정적 배치에도 자산별 초기 높이 후보와 전 객체 지면 검증을 적용했다. 모든 mesh의 접지를
해결했다는 뜻은 아니다. 본10K는 재시작하지 않았으며 구8354셀 deck도 새 정책으로 검증된
본수집용 목록이라고 간주하지 않는다. Production detector/tracker/localization은 미변경이다.

## 코드 정책

- `base5_static_height.py`: F2 원본 저장 pivot Z에서 원본 위치의 가까운 MGeo 높이를 뺀
  자산별 높이를 새 위치의 지도 높이에 더하는 **초기 후보**다. 원본이 지도점1m 밖이면
  추정하지 않으며, 주변 지도 Z span>10cm 또는 offset 범위 밖도 거부한다.
  차량의 기존+0.36m를 후보와 중복 적용하지 않는다. 원본 scenario/map checksum을 기록한다.
  122종 중69종 후보/53종 fallback이다. 69종 접지 인증이라는 뜻은 아니다.
- 새 generator와 동적 준비의 `ground_support_check.required_uids`는 전체 actor UID다.
  20m 밖 정적 객체도 필수 검사이며, annotation 누락/지면 근거 부족은 review다.
- 지면 평면은 중심뿐 아니라 전체 component BBox corner의 XY를 지지해야 한다.
  zero XYZ/Ego mesh 제외, 충분한 return·평면 품질·최대75cm hull 외삽 제한을 유지한다.
- 입력 Z downward retry에 정적 vehicleList도 포함했다. 실제 initPosition.pos와 metadata를
  같이 수정하며, 누적 보정은 최대75cm로 제한한다. raw cloud/Instance GT는 고치지 않는다.
- 수정 후에도 높이가 거의 변하지 않으면 `actor_input_z_correction_ineffective`로 기록한다.
  이는 관측된 annotation/terrain response이며 MORAI 버그의 증명이 아니다.
- sparse 근접 검사는 문제 UID 근처의 높이가 호환되는 지도 도로에 Ego를 Park해 빈 지면을
  촬영한다. 임의 객체 pivot XY/Z에 Ego를 놓지 않는다. 지도 근거가 없으면 보류/동일quota 대체다.
  최대2회 recovery와 안전 Park/충돌/통신 gate를 유지하며, 혼합 객체의 이전 지면 witness를 보존한다.

## 실제 검증

Git 비추적 근거: `/home/luv/morai_cap/base5_v2_261007`.

| 세션 | 결과 |
| --- | --- |
| static_ground_live_v60 | 초기4조건; 빈 지면 deck에 source required UID가 남는 결함 발견. 성공률 근거로 사용하지 않음 |
| static_ground_retry_v61 | TrafficBarrel 근접 혼합 재촬영 통과; 빈 지면에는 required UID를 비움 |
| static_far_live_v62 | 원거리 Kona 입력Z 수정 후에도 review. 후속 근접 비교 전 부유 확정 아님 |
| static_dog_live_v61/v63 | Dog2 자체 near gap 약4cm, 혼합 secondary terrain 미확인으로 전체 review. 실제 부유라고 단정하지 않음 |
| static_ground_final_v64 | 강화 gate: TrafficBarrel 동일quota 대체/Porter2/Scooter2개 혼합3조건 통과, Kona1조건 보류 |
| static_initializer_live_v65 | 자산별 초기 높이 후보 적용: 동일4조건 중3통과/원거리Kona1보류 |
| static_ground_infos_v65 | train2/val1, Vehicle1+Obstacle4 GT; 성공3장만 CPU 변환, raw 보존 |
| static_kona_reference_live_v66 | 같은 원거리 Kona를 가까이서 관측: ground gap −5.84cm/검사 통과, calibration_only로 학습 제외 |

## 원거리 높이 결과 해석의 수정

Kona의 원거리 입력 Z를 약20cm 낮춘 진단에서는 gap 약17cm가 남았으나, 같은 원본 배치를
가까이서 보면 gap −5.84cm다. 초기 원거리/근접 reference의 Object_topic 위치는 XY 동일,
Z 차이는 약0.000002m, heading도 동일했다. 즉 원거리 결과만으로 실제 차량 부유를 확정하면 안 된다.

`ego_to_world`는 Ego yaw와 XYZ만 사용하고 물리적 roll/pitch는 복원하지 않는다.
서로 다른 위치에서 얻은 평면을 world로 옮겨 장거리 annotation과 비교하는 과정에 높이 오차가
생길 가능성이 있다. **정확한 원인은 아직 분리 검증되지 않았다.** 이 때문에 yaw-only
cross-view witness로20m 밖 객체를 낮추는 처리를 금지했다. 원거리 자체의 local cloud 평면
검사는 계속 필수이며, 근거가 부족하면 보류한다. 가까운 reference 통과를 원거리 학습 quota로
자동 복사하지 않는다. 안전한 full-attitude 또는 동일 실제 배치의 hashed 근접 witness를
검증하는 별도 방법이 남아 있다.

## 한계 / 다음

- Annotation 최하단−지면 허용 범위는 기존 −30~+15cm다. margin/관절/지형 proxy이므로
  literal mesh contact나 collider clearance의 인증이 아니다.
- 새 초기 높이는 실제 지면 측정값이 아닌 저장 pivot과 지도 높이 차이 후보다.
  근거 없는53종에는 일괄 상향/하향 offset을 주지 않는다.
- 원거리/도로 외부/sparse 혼합의 ground coverage와 대량 수집 비용은 아직 미해결이다.
  특히 yaw-only world 변환의 장거리 오판을 해소하기 전 본10K readiness를 선언하지 않는다.
- Base5 회귀170 tests PASS 당시 결과. 이후 최종 회귀 결과는 CURRENT를 참고한다.
- 새 script `prepare_base5_static_ground_validation.py`는1–8셀 정적 검증용이고,
  `prepare_base5_static_height_reference.py`는 근접 관찰용 calibration deck이며 본수집 명령이 아니다.

다음은 근접 witness와 원거리 local 검사를 비교해 좌표계/평면 검사의 오차를 분리하고,
그 뒤 미확인 자산·경사·혼합 배치 coverage를 보강하는 것이다. 원거리 분포 quota를
임의로 없애거나 near frame으로 대체하지 않는다.
