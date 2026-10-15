# Base5 데이터와 수집의 현재 기준

문서 상태: CURRENT. 근거 확인: 2026-10-15.
Base5-only 학습용 Sunny13 5K는 완료했고, 다른 다섯 환경의 새 25K는 생성·준비만 완료했다.
학습 모델·checkpoint·추론 설정은 [모델 CURRENT](../../../docs/bevfusion/CURRENT.md)에서 관리한다.

<a id="data"></a>

## 승인 데이터와 계획

| 항목 | 현재 상태 |
|---|---|
| 완료 환경 | Sunny13, 승인 5,000장 / 부족 0 |
| Canonical raw | `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140` |
| Canonical 선택 | 위 경로의 selected ledger 하나. 부모 성공분을 다시 연결하지 않음 |
| 완료 경위 | 기존 4,974장 보존 + 같은 패밀리의 새 정상 장면 26장. 실제 대체 내용은 adopted manifest/mapping |
| 실제 동작 구성 | 정적 4,822장 / 실제 이동 178장 |
| Split | train 4,255 / val 745. 장면·group 계보를 보존 |
| 다음 환경 | sunny11/15, foggy11/13/15 각각 새 배치 5K. 총 30K 계획이며 실제 30K 완료가 아님 |
| 다음 준비본 | `~/morai_cap/base5_obstacle_improvement_261013/candidate_v150c/future_suite`의 독립 25K offline 계획. v142/v149 보존 |
| Sunny13 동결 release | 같은 실험 root의 `release_v150_sunny13/release.json`. 버스 4/4, 정적 보충 64/64, 이동 8/8, `bulk_ready=true` |
| 본수집 상태 | 미실행. 전방·원거리·복합 진단 70장과 v151 5K CPU 계획만 확인. 다른 날씨와 새 release 승격은 별도 필요 |

완료 기준과 부모 보존은 [v140 최종 근거](261010_base5_flexible_deficit_replacement.md)를 따른다.
v150의 `bulk_ready`는 10/13 당시 source의 보존된 검증 snapshot이다. v151·현재 source를 새로 동결한 승인으로 해석하지 않는다.
다섯 환경의 새 geometry·CPU preflight·소량 live 검증은
[수집기 근거](261010_base5_postcollection_collector_work.md)에서 확인한다.
다른 환경의 상태가 바뀌면 실제 journal을 확인한 뒤 이 표를 갱신한다.

## 배치 계약

| 구역·객체군 | 범용 | 미션 |
|---|---:|---:|
| 고주로 차량 | 740 | 600 |
| 로터리 차량 | 0 | 350 |
| 기타 차량 | 80 | 50 |
| 장애물 계열 | 770 | 600 |
| 사람·동물·복합 계열 | 830 | 250 |
| 독립 GPS 음영 구간 | 0 | 500 |
| 배경 | 230 | 0 |
| 합계 | 2,650 | 2,350 |

배경 230장까지 범용 합계에 포함한 장면 수다. 일반/미션 카테고리와 실제 유효 GT 수를 혼동하지 않는다.
고주로·로터리는 보조 actor도 일반 차량만 사용한다. 음영은 별도 family이며 안의 객체를
바깥에서 보는 접근 장면도 포함한다. 정적 회피·음영은 허용된 전체 asset 후보를 고려한다.
범용 혼합의 배치 수·크기 budget과 실제 보이는 객체/유효 component GT 수는 다르다.
122종 primary coverage는 해당 release와 adopted manifest의 실제 할당으로 확인한다.

현재 보강은 비율·primary 할당·거리/방향·미션 phase·split과 기존 actor 목록을 유지한다.
범용 regular/dense의 일부에 19종 물리 장애물을 최대 하나 더하고 기존 0~20/20~30m 보조를 보호한다.
장애물 보강은 30m 이상 보조만 근·중거리로 옮기며 미션·고주로·로터리·음영·single·배경은 변경하지 않는다.
같은 승인 ID 43장 대조에서 근거리 GT 64→98, LiDAR 20점 이상 39→55이고 Vehicle 36/Pedestrian 69는 유지됐다.
이는 배치·센서 지원의 증거이며 재학습 AP 개선의 인증은 아니다.

별도 긴 NPC 보완은 정적 회피·음영의 길이 8m 이상 정지 물리 차량에만 적용한다. 차체의 도로 지지를
추정하지 못한 배치만 다시 생성하고 필요한 actor의 yaw fallback을 명시한다. 새 Sunny13에서
24장 재배치·15개 actor 방향 fallback이며 4,976개 payload는 유지했다. 자산·거리/방향 계약·split은 보존한다.
고주로·로터리의 경로 기반 차량, 일반 범용 차량, passive OBJ·사람·동물·복합체의 자유 yaw와 이동 경로는 유지한다.
[정지 NPC 검증](261013_base5_parked_npc_support_evidence.md)에 실제 근거와 지도 추정의 한계를 기록했다.

v142 baseline의 공간 정책을 유지하므로 범용은 고정 기준점만 반복하는 대신
허용 도로를 따라 연속 Ego 위치를 샘플링한다.
같은 slot의 환경 A/B라고 부르지 않으며 물리 link·final support·100m 시야 중복을 따로 감사한다.
보강 배치·새 정책은 현재 성공분을 폐기하는 이유가 아니다.

다음 25K는 [현재 후보 런북](261013_base5_obstacle_support_runbook.md)과 [보강 근거](261013_base5_obstacle_support_experiments.md)를 따른다.
[원거리 개선 계획](../../../docs/bevfusion/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)의 비차량 0~40m 집중·40~50m 소량 유지·50m 이상 quota 축소는 후속 후보이며 v150에 적용하지 않았다.
개수·거리 비율은 실측으로 수정하되 실제 보이는 원거리 GT를 삭제하거나 Vehicle로 바꾸지 않는다.
10/15 v151은 추가 장애물 338개만 전방 지원으로 재배치한 별도 후보다. [70장 소량 근거·복합 진단·남은 승격 조건](261015_base5_front_far_compound_experiments.md)을 따른다.
`fresh_suite_final_v142`는 이전 baseline으로 보존한다. 새 Sunny13는 원래 30K에 자동 합산하지 않는다.

<a id="capture"></a>

## 수집과 복구

현재 보강 후보는 [보강 런북](261013_base5_obstacle_support_runbook.md)을 따른다.
구 v142 통신·대시보드의 baseline 이력은 [fresh 런북](261010_base5_fresh_environment_runbook.md)에 보존한다.
기존 Sunny13 수집/보충 재현은 당시 source·runtime 계약을 사용하며 현재 코드로 강제 resume하지 않는다.

- SaveSensorData 수집과 UDP/ROS 실주행 streaming을 구분한다. 기존 master/guarded bridge를 중복 실행하지 않는다.
- MR UI의 날씨와 시간을 확인한다. manifest 생성만으로 UI 환경이 변경되지는 않는다.
- release·코드·manifest·동결 QA 해시를 확인한다. `--complete`는 본수집 뒤 유한 보충까지 연결한다.
- 성공분·실패 raw·부모 journal을 보존하고 새 후보의 변경 내용을 mapping에 기록한다.
- 동일 실패 payload 반복·보충 상한 초기화·부모 ledger 중복 연결을 금지한다.
- ordinary 장면 부족분과 통신/Park/CollisionData 무수신의 안전 중단을 구분한다.
  실제 접촉은 증거와 안전한 종료가 검증된 장면만 실패·부족분으로 넘긴다. 불확실한 접촉을 우회하지 않는다.
- 정적은 원래 캡처의 QA를 유지하며 높이 확인만을 위한 추가 재취득을 남발하지 않는다.
- 사거리 실제 이동은 수동 높이 lookup의 출발→도착 두 waypoint·선형 이동을 따른다.
  음영의 이동은 독립 terrain/reference 준비를 유지한다. 자동 지면 추종 기능이 아니다.
  심각한 부유·실제 이동·충돌·Ego 정지 검사를 우회하지 않는다.

v150의 64+8장 QA 포함 가중 예상은 5K 약 8.14시간/31.25GiB다. 운영 예산은 8~12시간/45GiB이며
실제 본수집 rate로 갱신한다. v149와 표본이 달라 순수 속도 개선율로 해석하지 않는다. 10/15 여유 약 67GiB로 25K 전체 보관은 부족하다.
실제 상태·디스크·충돌 수신·안전 로그는 실행 중 다시 확인한다. 다른 agent가 MORAI를 쓰면 동시에 수집하지 않는다.

<a id="gt"></a>

## GT와 학습 입력

원본 PNG·native annotation·parent/component 관계는 보존한다. 카메라 RGB의 학습·분석·공유 입력은
원본을 무손실 보존한 상태에서 만든 Q90 JPEG다. mask/depth 등 보조 데이터는 JPEG로 변환하지 않는다.
다른 PC용 완료 묶음의 JPEG 전달·원격 검증 후 RGB PNG 정리는 후속 설계다. 현재 원본 해시 기반 resume 중 자동 삭제하지 않는다.
학습 GT는 native full RPY의 8모서리를 포함하는 upright 7D enclosure를 사용하며 중심·yaw·구성요소를 임의 합치지 않는다.

현재 native JPEG/upright 전달본은
`artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release`다.
512 입력의 유효 GT 11,052개와 이전 320 crop의 10,959개는 서로 다른 파생 버전이다.
가시성 미확인과 실제 비가시를 동일하게 취급하거나 낮은 LiDAR 점 수만으로 전부 버리지 않는다.

수집 원본의 vehicle/pedestrian/obstacle/animal 의미는 유지한다.
현재 학습은 Animal만 Pedestrian으로 매핑한 3클래스다. OBJ 차량은 Vehicle이고 복합 자산의 component별 의미·BBox를 보존한다.
자산 종류·family·Scenario container·모델 클래스는 같은 개념이 아니다.
원문 Object velocity와 trace는 진단 metadata이며 완전한 SaveSensorData 동기 속도 GT가 아니다.
현재 single-frame 7D OD 학습에 velocity supervision을 넣지 않는다. bag별 단위·기준점 정책은 별도로 확인한다.

[JPEG/GT 변환 근거](../../../docs/bevfusion/README.md#evaluation)와 [MR 자산 계약](MR_ASSET_COLLECTION_CURRENT.md)을 참고한다.
원본 5K와 새 pilot을 섞지 않으며 이미 끝난 학습 source를 문서 개편 때문에 재변환하지 않는다.

## 이력과 갱신

현재 상태는 이 문서의 기존 항목만 갱신한다. v105~v140 진행 이력은
[무손실 원문 스냅샷](../../../docs/perception/legacy/context_261012/README.md)에 보존했다.
과거 계획·ETA·중간 부족분을 현재 값으로 복구하지 않는다.
