# Base5 정적 높이 보완 — EVIDENCE / 2026-10-07

## 결론

현재 구현/판정은 아래 **v113 최종 XY 보완**이 우선한다. 그 위의 v60–v66 내용은
당시 근거와 해석 변경 이력이며 현재 후보 수/원거리 QA 계약과 혼동하지 않는다.

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

## 최종 XY 기준 보완 — v113

### 문제와 구현

좌우·혼합 배치 일부는 pose의 XY만 바꾸고 기존 anchor Z를 유지했다. 특히 정지 Dog2
`(-75.7541,-77.9886)`의 초기 Z28.3607m와 해당 위치 근처 도로 높이는 수십 cm 달랐다.
과거 실제 local-plane gap은+31.27cm였다. 이것은 모든 정적 자산의 높이가 잘못됐다는 뜻이 아니다.
최근 5개 세션 첫 정적 촬영35scene/75actor 중 직접 plane 측정이 가능했던35actor는
gap 절대값 p50 1.503cm/p95 6.127cm, 최대+5.334cm였다. 미측정40actor는 정상 인증하지 않았다.

`base5_map_surface.py`는2m 이하 간격의 검색용 인덱스를 만들되, 높이는 원본 도로 선분 XYZ에서
정확히 투영·보간한다. 해당 link에 연결된 lane boundary만 방향/높이/거리 검사를 거쳐 횡경사에
쓴다. 지도점을 단순히 더 잘게 복사하거나 전역 삼각분할로 보도/다른 높이를 연결하지 않는다.
`lazy_init`/lane-change link208개는 도로 표면이 아닌 경로 shortcut이므로 제외한다.
동일 위치의 서로 다른 도로 높이가25cm 넘게 모호하면 추정을 거부한다. 도로8m 밖도 unsupported다.
연결 경계 tie는 ID순으로 고정하여 process별 proof가 달라지는 비결정성을 제거했다.

F2 저장 pivot도 같은 도로 표면 방식으로 계산한다. source가 road strip/near centerline일 때만
자산 offset 후보를 쓰며 범위−0.5~1m를 유지한다. 81/122종 후보/41fallback이다.
최종 XY 조회 → 자산 pivot → 차량 legacy0.36m 중복 제거 순서이며 blanket +Z margin은 없다.
다섯 footprint sample/지도 source SHA와 confidence를 기록한다. footprint의 실제 접촉 인증은 아니다.

### 같은 위치 실제 A/B

원본과 신버전의 XY/yaw/asset은 같다. Dog는 첫 actor만 격리했다. 별도 원본/캡처는 보존한다.
수치는 raw annotation 최하단과 local LiDAR plane 간격이며 mesh/collider의 실제 penetration량이 아니다.

| 조건 | 기존 gap | 새 gap | 판정 |
|---|---:|---:|---|
| 정지 Dog2, 원래 부유했던 위치 |+31.274cm|−10.089cm|review→accepted|
| 정적 Ped_Pet1 근접 |−1.186cm|−2.345cm|양쪽 accepted, 사람+동물 component 유지|
| 정적 ElectricScooter2 근접 |+0.958cm|+1.592cm|양쪽 accepted, 차량+사람 component 유지|
| 혼합 Lane_Separation_Bar/Firewall/Girl1/Steel_Barricade |측정3actor 정상|측정3actor 정상|Girl1 plane 부족만 남음|

Dog 최종 serial A/B는4.806/4.783초, no-contact/종료Park 통과다.
초기 A/B의 Dog 격리 metadata에 제거한8501 required UID가 남는 시험 deck 결함을 발견해
수정·새 output 재촬영했다. 초기 해당 review를 정상/실패율 근거로 사용하지 않는다.
두 검사 프로세스가 잠시 겹친 중단 시험도 증거에서 제외하고 Park를 확인했다.
이후 서로 다른 output을 포함한 동일 ROS master 단일 live lock을 추가했으며 최종 시험은 직렬 실행했다.

혼합은 `final_xy_height_v113_mixed_quality_serial`: initial review → Girl1 near witness1회
→ original raw re-audit accepted, 총13.567초. 원본 재촬영0이며 센서 bytes/GT는 그대로다.
기존−30~+15cm gate, raw GT·CollisionData·Park·가시성 기준을 완화하지 않았다.

### 제한된 재취득과 실제 남은 실패

child quality-loop당 measured-height 재촬영1회/배치 대체1회/witness 최대8회다.
같은 UID+동일 scenario SHA에는 witness를1회만 시도한다. 배치가 바뀌면 새 proof가 필요하다.
단순 plane 부족은 높이 실패가 아니며 임의 downward retry를 하지 않는다. 비용 상한 도달 시
실패 이유·budget 카운터를 deficit에 남긴다. outer quota 기본2round는 그대로여서 한도를 구분한다.
실제 collision/contact·hash 손상·통신/Park 미확인 기존 정책도 유지한다.

새 frozen 목록 `final_xy_ground_v113_final_smoke4b`는4셀 중3accepted/1deficit,66.306초다.
GV80 교차/Soul 양보/DEFAULT_WAGON 양보3셀은 ground/no-contact/Park 통과했다.
근접 OBJ_Kona는 primary gap−0.443cm로 정상이나, secondary plane/reference pose 대조가 불충분해
quota 대체까지 했어도 `static_ground_reference_pose_or_view_mismatch`로 남았다.
완료 resume은 추가촬영0이고3/1 상태를 그대로 유지했다. 잘 나온 near frame으로 quota를 바꾸지 않았다.

첫 별도 Mixer 셀(ordinal1258)은 캡처 전 `Stationary Ego drift exceeded`로 halt했고 Park 확인했다.
원인을 높이 보완이라고 확정하지 않는다. stationary pose 미확인을 성공/자동 continue로 완화하지 않았다.
입력높이 변화 최대+1.207m인 animated 혼합(ordinal4549)은 static calibration에서
PE_Firewall_White gap+23.807cm가 남았고, 다른 actor ground도 부족했다.
높이가 호환되는 도로 witness를 찾지 못해14.477초 후 deficit, 학습캡처0이다.
즉 roadside의 가장 가까운 도로 높이는 terrain mesh 높이의 완전한 대체가 아니다.

### 동결 목록·검증·비용 결론

`final_5k_final_xy_ground_v113_final/manifest.json` SHA256:
`86f2d7e7c8dd2c319623ecb8343ac203caa951c60049727f98b1594ab92bd758`.
5000 unique/OFFLINE_PASS/errors0, planned122종 최소4, train4382/val618/test0이다.
Ego/actor XY/yaw/spec/split/이동 경로 전수 보존, animated1271actor 완전 불변이다.
9371static/standing actor 중7671개 Z 변경, abs delta p50 약0.002cm/p95 7.763cm/max1.207m다.
road strip2067/near centerline5187/roadside1138/unsupported979이며 마지막 두 그룹은 지면 인증이 아니다.
`revise_base5_ground_placement.py`는 새 디렉터리만 허용하며 원본 v112 파일·계약을 변경하지 않는다.

CPU 변환: 승인 혼합1장 GT4(Obstacle3/Pedestrian1), frozen 차량3장 GT4(Vehicle4), 제외GT0/raw보존.
calibration·review·부족분은 넣지 않는다. Base5 회귀261 tests PASS, compile/diff check 통과다.
단위검사에 source/proof 변조 거부, source파일·XY 보존, 가상 link 제외, 횡경사/층 모호성,
동일 master 이중 제어 차단, budget 초과 시 추가촬영/false pass 없는 deficit를 포함한다.

warm map lookup2000회 평균0.214ms, footprint까지 보통 actor당6조회다.
지도 인덱스/초기 높이 계산은 생성·대체 준비 때이며 정상 capture마다 새 map scan을 하지 않는다.
Dog 실제 capture 시간은 증가하지 않았다. 대부분 기존 정적 높이가 정상이므로 전체5K 시간을
크게 단축했다고 주장하지 않는다. 비용 절감 대상은 문제 장면의 불필요한 재촬영/반복 QA다.
구13.4h/45.7h는 이전 분포의 조건부 외삽이며 이번 정확한 ETA가 아니다.
본5K 수집/학습은 미시작. 지면 evidence 부족/일부 near reference mismatch/Ego 드리프트는 남는다.
최대 가능한 지도 초기화와 안전한 품질 분리가 완료됐으며 모든 지형·자산 접지 해결은 아니다.
