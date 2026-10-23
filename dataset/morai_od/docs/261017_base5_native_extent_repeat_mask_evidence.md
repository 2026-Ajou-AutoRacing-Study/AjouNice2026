# Base5 혼합 반복의 native 치수와 보이는 마스크 대조

문서 상태: EVIDENCE. 확인: 2026-10-17. 원본·native GT·QA 판정·배치 정책은 수정하지 않았다.

## 결론과 범위

동일한 물리 payload에서 **보이는 RGB/instance 외곽은 유지되지만 native 8점 치수가 25% 바뀐 사례**를 확인했다. 이는 해당 사례에서 실제 보이는 mesh가 25% 확대됐다는 설명보다 native annotation bound의 불안정을 강하게 지지한다. 정확한 mesh·collider·내부 cache 원인을 확정하거나 GT를 자동 보정하는 근거는 아니다. 일부 다른 객체는 실제 위치와 보이는 마스크도 달라졌다. 모든 혼합 객체의 mesh가 불변이라고 일반화하지 않는다.

요청한 all-Red-unit/fresh-UID/빈 배경 뒤 fresh-UID 각 4장, 합계 12장에 예전 mixed 6장을 더해 18 raw를 전수 비교했다. 각 조건의 독립 물리 배치는 두 개뿐이며 반복·조건 간 조합은 독립 통계 표본이 아니다. all-Red-unit에도 CargoBox/PE_Drum/YellowBarrel의 nonunit이 남아 있으므로 unit-only cold 조건으로 해석하지 않는다.

| 조건 | 장면 | 원형 Red nominal / extra 1.25 | Cargo 요청 1.25 nominal / extra 1.25 |
|---|---:|---:|---:|
| all-Red-unit | 4 | 8 / 4 | 2 / 2 |
| fresh object UID | 4 | 8 / 4 | 2 / 2 |
| 빈 배경 로드 뒤 fresh UID | 4 | 8 / 4 | 2 / 2 |

각 조건의 native 확대 Red는 4회 모두 한 부모씩 나타났다. Cargo는 04192의 같은 요청 scale 1.25인 두 부모 중 한 부모만 추가 1.25를 보였고, 반복에 따라 부모가 바뀌었다. 요청 1.25에 추가 1.25가 붙으므로 unit 기준 유효 비율은 약 1.5625다. 이 현상은 Red 또는 하나의 고정 UID만의 문제라고 제한할 수 없다. UID 변경·빈 장면 로드가 이번 네 장에서 회복을 만들지 못했다는 범위만 말할 수 있다.

## proxy 없이 확인한 강한 대조

아래 쌍은 objectList UID만 ordinal로 정규화한 **전체 scenario payload SHA와 calibration SHA가 동일**하다. actor 대응은 원래 UID↔fresh UID 계보와 objectList 순서·DataID·name으로 검증했다. mask는 BBox2D의 parent UID/RGBA로 원문 PNG에서 직접 추출했다. native 3D projection·원통·depth·box 내부 점 선택을 사용하지 않았다.

| 동일 부모·반복 | native 치수 비율 | camera | 보이는 외곽 W×H | mask 면적 | mask IoU | RGB 채널 MAE (0~255) |
|---|---|---|---|---|---:|---:|
| old mixed 02170 UID8500, repeat 0→1 | 요청 1.5에서 1.5→1.875 | 1 | 148×243 → 동일 | 33,268→33,259 | 0.999609 | 0.5868 |
| 위와 같음 | 위와 같음 | 3 | 91×79 → 동일 | 5,280→5,280 | 1 | 0.4924 |
| all-Red-unit 02170 UID8500, repeat 0→1 | 요청 1에서 1.25→1 | 1 | 97×165 → 동일 | 14,865→14,862 | 0.998857 | 0.7954 |
| 위와 같음 | 위와 같음 | 3 | 57×53 → 동일 | 2,257→2,256 | 0.999557 | 0.8009 |
| fresh UID 02170 원래8502, 60002→60302 | 요청 1에서 1.25→1 | 4 | 28×48 → 동일 | 1,330→1,330 | 1 | 0.5574 |
| all-Red-unit 04192 Cargo8501 | 요청 1.25에서 1.5625→1.25 | 1 | 26×15 → 동일 | 325→326 | 0.984756 | 1.4106 |

old mixed UID8500의 실제 ObjectInfo 위치 차이는 0m, Ego 위치 차이는 0.00001144m, 실제 heading 차이는 0도다. pivot 거리로 계산한 단순 역거리 비율은 0.999999899다. all-Red-unit UID8500도 실제 actor 위치 차이는 0m, Ego 차이는 0.00023460m이며 역거리 비율 1.000002080이다. fresh UID 원래8502의 위치 차이는 0.00003412m이며 역거리 비율 0.999997500이다. 이런 차이로 치수 25% 변화에 비례하는 외형 확대를 설명할 수 없다. 다만 ObjectInfo는 정확한 노출 시각 pose가 아니며 이 역거리 계산은 full projection의 대체가 아니다.

BBox2D 자체도 거의 같다. old mixed UID8500 camera1의 실수형 높이는 242.8835→242.8832px, all-Red-unit UID8500은 165.2275→165.2211px다. 마스크 bounds뿐 아니라 RGB도 매우 유사하므로 단순 RGBA 색 번호 재사용이나 camera UID 오연결로 설명되는 사례가 아니다.

반대 근거를 남긴다. all-Red-unit Cargo8502는 actual pose가 0.23483m 달라졌고 mask는 460→539px, IoU 0.651240이다. 같은 조건의 일부 Red도 실제 pose가 약 0.06467m 달라졌다. 이런 쌍은 mesh-only/annotation-only를 이번 자료만으로 강하게 분리하지 않는다. 위 표의 안정된 부모 근거를 모든 actor에 확대 적용하지 않는다.

## 별도 surface projection 대조

12 raw의 Red 36부모를 기존 `report_base5_surface_bounds.run`으로 따로 분석했다. 모든 unit 확대의 축별 native 치수는 작은 pinned 원형 기준 약 1.25다. 대표 가까운 unit Red의 camera1은 observed/native projected 높이 0.798221, native/1.25 가설에서는 1.000282였다. fresh UID 원래8502 camera4는 0.794904→0.993916이다. PNG의 초록선은 실제 mask, 빨강은 native 원통 proxy, 하늘색은 native/1.25, 노랑은 배치 reference다.

이는 직접 반복 마스크 대조와 일치하지만 보조 근거다. 원통은 정확 mesh가 아니며, occlusion·재질·rasterization·camera calibration의 잔여 오차가 있다. 예를 들어 부분 가림이 남은 79px짜리 camera2 unit Red는 native/1.25에서도 높이 비율 0.796654에 불과해 단순 fit 승인의 근거가 아니다. 단일 median 또는 proxy fit으로 원본 GT를 덮어쓰지 않는다.

## 원문 매핑·보존·재현

신규 `diagnose_base5_native_extent_repeats.py`는 다음을 fail-closed로 검사한다.

- 원본 metadata/scenario/audit/calibration과 정확히 같은 timestamp의 4 camera RGB·mask·BBox2D 및 LiDAR Instance/8Points를 핀한다.
- objectList/runtime actor의 ordered UID·DataID·name·container, 실제 ObjectInfo UID/name, fresh UID의 일대일 원래 부모 계보를 검증한다.
- 독립 원문 Instance line 파싱과 기존 parser의 size를 대조한다. Instance/8Points의 parent UID/class/component 수·center·전체 radian RPY·8개 꼭짓점을 엄격히 검산한다. scenario scale을 native 치수에 재곱하지 않는다.
- 전체 scenario에서 objectList UID 외 어떤 값도 제거하지 않고 동일 payload를 판별한다. 이 검사에서 축소·이동·Ego/sensor 변경은 동일 쌍이 되지 않는다.
- BBox2D의 RGBA가 다른 UID/class에 중복된 경우 거부한다. 직접 boolean mask의 면적·bounds·IoU/XOR와 실제 RGB 오차를 계산한다. 노출 외곽을 정밀 mesh 복원으로 해석하지 않는다.

18 raw 분석은 5.056초, source SHA 328개 전후 불변이었다. 성공 상태나 raw metadata에 reference를 소급 주입하지 않았고 18장 모두 기존 review 판정을 유지했다. 관련 신규 테스트 8개가 통과했다.

| 산출물 | 경로·SHA256 |
|---|---|
| 직접 반복 대조 최종본 | `artifacts/evaluation/base5_scale_qualification_261017/native_extent_repeat_masks_v2/report.json` / `110c504abd64279edb7693a6bd4c30564c69eb3be4ccac832f31be51ba21e6c9` |
| surface 독립 산출물 | `artifacts/evaluation/base5_scale_qualification_261017/red_unit_mixed_surface_v1/report.json` / `92da1229a1ffce293cc6b326b7c3b544e11fcc961385eca5ec09242a94d2d4c5` |
| 원형만인 cold/hot 공통 준비본 | `artifacts/evaluation/base5_scale_qualification_261017/all_physical_unit_mixed_v1/manifest.json` / `6d8256058bbf3d3e52d83d034035c48fc7578f3d40080a7c7b9bfa3a6a5c2ead` |

`native_extent_repeat_masks_v1`은 RGB 정량 추가 전 중간본으로 보존한다. 최종 해석은 v2를 사용한다. 두 보고서의 원문 source 목록·해시는 각각 JSON에 있다. [이전 native scale 근거](261016_base5_native_scale_and_extent_evidence.md), [10/17 검증](261017_base5_scale_split_qualification.md), [크기 reference 네 층 구분](261017_base5_geometry_reference_audit.md)의 과거 관측을 덮어쓰지 않는다.

## 다음 최소 대조

새 all-physical-unit 준비본은 기존 all-Red-unit 두 layout×2회를 재사용하되 Cargo/PE/Yellow까지 모두 scale1로 만든다. 26 actors 전부 원형이며 기존 unit actor의 raw pose/Z/height initializer는 보존했다. 확대→원형만 pinned unit reference와 기존 qualified native bottom proxy로 Z를 조정했고, 실제 mesh 접지 인증은 아니다. 원문 보수적 배치 envelope 전체를 nativeGT로 바꾸지 않았다. source plan/policy는 명시적 counterfactual 계보에 보존하고 active 확대 plan/정책은 제거했다. native19종 QA 및 정상 계약 검증은 유지한다.

이 준비본의 cold-unit-before-scale와 hot-unit-after-scale를 같은 frozen payload로 비교하되 실제 simulator 재시작·선행 scale 노출은 operator가 기록한다. 재시작 없이 cold라고 부르거나, unit mismatch를 숨기거나, 같은 실패를 자동 반복해 정상 장면으로 승격하지 않는다. 4회는 두 배치의 반복일 뿐 안정성 통계·본수집 승인·120장 안정성 검사를 대신하지 않는다.

## 재시작 후 대조 결과

사용자가 MORAI와 ROS를 모두 재시작했다. 새 MORAI 프로세스는 PID 3211363,
시작 시각은 2026-10-17 13:48:00 KST였다. 첫 `cold_v1`은 시작 통신 검사에서
중단되어 실제 로드·촬영이 없었고, 종료 Park를 확인했다. 이후 새 session 결과는 다음과 같다.

| 순서 | 실제 취득 | 결과와 범위 |
|---|---|---|
| 1 | `capture_all_physical_unit_cold_v2` | 원형 4/4 승인. 같은 두 배치를 각각 두 번 확인 |
| 2 | `capture_stability_large_a_v1` | 새 최종 규격의 원형/확대 pilot 2/2 승인 후 계획적으로 종료 |
| 3 | `capture_all_physical_unit_after_final_pair_v1` | 순서 1과 같은 원형 4/4 승인 |

각 session의 종료 `park_verified=true`를 확인했다. 이 결과만으로 내부 캐시 원인이나
완전한 해결을 확정하지 않는다. 과거 혼합 시험의 실패 원문·판정은 그대로 보존한다.
현재 후보는 1.25배를 사용하지 않지만, 그 차이가 원인이라는 통제 실험도 아니다.
pilot 2장은 최종 통합 사전등록보다 먼저 취득했으므로 공식 paired120의 분모·승인·비용에
합산하지 않는다. 이후 새 source/runtime 등록과 별도 전용 campaign으로 확인한다.
