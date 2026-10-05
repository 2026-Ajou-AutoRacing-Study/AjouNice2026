# Base5 일반 생성기 외관 정책·지면 높이 검증 — EVIDENCE v28

2026-10-06. MR / ROS / SaveSensorData. 최신 진입점은 `BASE5_V2_CURRENT.md`다.
Production E25·threshold·tracker·planning·localization은 변경하지 않았다.
10K 생성·본 수집·학습은 시작하지 않았다.

## 일반 생성기에 통합한 계약

- `base5_asset_appearance_policy_v1.json`의 **확인된 자산 이름**에만 정지 우선 정책을 적용한다.
  BabyStroller2/3, Bike3, Scooter1, NCAP_EPTa 및 확인된 rigid/vehicle 자산이다.
  Woman2, Dog1/2, Deer1, Ped_Pet1은 요청된 정지/동적 phase를 유지한다.
  미검증 자산은 동적 요청을 제거하지 않고 `unverified_preserve_request`로 추적한다.
  AI 보행자는 기존의 미검증 motion API 계약 때문에 standing을 유지한다.
- 기본 `natural-target-v1`: 캡처 목표 pivot 위치와 거리 quota를 먼저 정한 뒤,
  자연 속도 × 9초만큼 진행 반대쪽에서 출발한다. 사람 walking 1.1m/s,
  running 2.2m/s, Animal 1.2m/s다. 걷기/달리기 animation 보장은 아니다.
  끝점은 목표보다 3m 뒤에 두고, 실제 도착 XY 오차 ≤0.4m일 때 캡처한다.
  여러 moving actor는 동시에 도착해야 한다. 다중 moving actor의 동시 도착률은 아직 live 미검증이다.
- 거리 quota는 **캡처 목표**에 적용한다. 출발부터 끝까지 동일 거리 bin에 머무르게 하려고
  속도를 줄이지 않는다. 전체 경로의 Ego/타 객체 swept clearance·ROI는 별도로 검사한다.
- 실제 캡처의 raw annotation 거리·visibility·component·CollisionData 검사는 유지한다.
  `requested_motion`과 실제 적용 정책, 목표/출발 위치, policy hash를 남긴다.

## 소량 근접·혼합 검증

일반 `gen.build`로 9장(각 2 actor, primary 거리 3–5m 또는 5–10m)을 생성했다.
Woman2, Dog1/2, Deer1, Ped_Pet1, BabyStroller2/3, Bike3, Scooter1이 primary다.
secondary는 정적으로 제한하여 자연 도착 정책과 혼합 배치부터 검증했다.
실제 secondary에는 차량·바리케이드·스쿠터·유모차 등이 포함된다.
같은 자산/거리/방향 quota를 유지한 새 배치 4장도 추가 검증했다.

- 13장 모두 raw 저장. 오프라인 배치 audit 9/9 및 추가 4/4 통과.
- 동적 primary의 캡처 window Z 급변은 모두 0m였다. **안 떠 있다는 증명은 아니다.**
- 최초 9장 당시 ledger는 4 accepted/5 review였지만, 지면 평면의 공간적 지지 영역과
  제한된 외삽 조건을 추가한 **최종 코드 재검사에서는 2 accepted/7 review**다.
  추가 4장은 모두 review다. 기존 ledger를 성공으로 덮어쓰지 않았다.
- 최종 13장 중 2장만 전체 감사 통과: BabyStroller2+BabyStroller3, Bike3+Scooter_01.
  나머지는 주변 지면 미확인(7장), 지면 아래 배치 의심(3장), runtime/annotation 누락 등이다.
  reason들은 중복될 수 있다. 적은 point·복수 높이·부적절한 표면 평면 가능성이 있으므로
  지면 아래 판정은 물리 mesh 접촉에 대한 확정 진단이 아니라 **보류 근거**다.
- 원래 Scooter1 장면에서는 primary runtime/annotation이 누락됐다.
  재배치에서는 나타났지만 지면 검증이 부족해 그대로 review다. 카탈로그에서 제거하지 않았다.

## 새로 발견한 문제: 안정된 Z도 공중 배치일 수 있다

v27은 정지/이동 parent Z를 맞춘 **외관 비교**였다. 지면에 붙었음을 확인한 실험이 아니다.
기존 Dog1/2·Deer1·Ped_Pet1 정지 표본의 raw GT 하단과 주변 LiDAR 평면 간 간격은
약 +0.27–0.29m였다. 따라서 이전의 “Z 변화 없음”을 지면 정상으로 읽으면 안 된다.
MGeo에서 가져온 한 높이를 근처 모든 XY 배치에 쓰는 것은 렌더링 지형/경사와 다를 수 있다.
object waypoint mode의 추가 높이 차이와도 별개의 문제다.

현재 `base5_ground_support.py`는 actor OBB 영역을 제외한 주변 낮은 LiDAR return으로
로컬 평면을 적합한다. 최소 18 inlier, inlier 비율 ≥60%, RMS ≤2.5cm,
경사 ≤0.25, XY 2차원 지지 분산을 요구한다. 반경은 5m부터 실패 시 8m까지이며,
actor 위치가 inlier hull 밖으로 0.75m 넘게 떨어지면 외삽하지 않는다.
20m 이내 actor만 검사한다. 그 밖의 actor에는 **지면 확인을 주장하지 않는다.**
부모의 여러 component OBB 하단 중 최저점을 쓰므로 animation·label margin의 영향이 있다.

- 평면 대비 하단 >+0.15m: `actor_annotation_above_ground`.
- 하단 <−0.30m: `actor_annotation_deep_below_ground`.
- 안정된 평면을 못 구함: `actor_ground_plane_unverified`.
- 기존 캡처 window Z step >0.10m 제외 gate도 유지한다.
- 위 사유는 training candidate에서 제외하되 raw는 보존한다.

## 입력 높이 수정 후 재촬영: 효과 확인

`prepare_base5_ground_retry.py`는 신뢰도 조건을 통과한 **공중 배치**만 대상으로,
측정 gap에 따라 입력 scenario 및 waypoint Z를 낮춘 별도 retry deck을 만든다.
0.75m보다 큰 하향 보정·상향 보정·NPC vehicleList 변경은 하지 않는다.
raw point/RGB/annotation/GT를 사후 이동시키지 않는다. 새 촬영으로 다시 독립 검사한다.

| 조건 | 수정 전 GT 하단–지면 | 수정 후 | 결과 |
| --- | ---: | ---: | --- |
| Dog1 정지 | +0.287m | −0.029m | 통과 |
| Dog1 이동 | +0.252m | −0.036m | 통과 |
| Dog2 정지 | +0.286m | −0.029m | 통과 |
| Dog2 이동 | +0.263m | −0.035m | 통과 |

각 표본의 입력을 약 0.28–0.32m 낮췄다. −3cm 수준은 OBB label margin도 포함하는
기하학적 하단 지표이며 mesh가 지면 아래로 들어갔다는 확정은 아니다.
4조건 모두 visibility·실제 정지/이동·Z 급변·CollisionData 검사도 통과했다.
Dog2 이동은 최초 1회 목표 도착 미확인으로 저장 전에 차단됐고, resume 재시도에서 통과했다.

초기 retry manifest 작성 시 role 등 entry 필드 누락으로 raw 1장 저장 뒤 ledger 작성이
실패했다. raw를 삭제하지 않았고 종료Park를 확인했다. preparer의 필드 보완과 runner의
**ROS/control 이전 manifest 필수 필드 검사**를 추가했다. 해당 raw는 최종 4조건 표에서 제외했다.
이번 합계 19시도/18 raw 저장 중 정규 ledger 17장과 미등록 raw 1장을 보존했다.
최종 코드 재검사 합계는 정규 raw 17장 중 6 accepted/11 review다.
종료Park는 모든 세션에서 확인했으며 일부 세션은 EventCmd 첫 timeout 후 재시도 성공했다.
간헐 서비스 timeout 자체가 해결된 것은 아니다.

## 재현 경로와 남은 작업

`/home/luv/morai_cap/base5_v2_261006/` 아래:

- `appearance_integration_deck_v28/`, `appearance_integration_capture_v28/`: 9조건.
- `appearance_resample_deck_v28/`, `appearance_resample_capture_v28/`: 4조건 재배치.
- `ground_reference_deck_v28/`: 원래 raw를 수정하지 않은 지면 재검사 근거.
- `ground_retry_deck_v28b/`, `ground_retry_capture_v28b/`: 최종 높이 수정 4조건.
- `ground_retry_capture_v28/`: 필드 누락으로 중단한 raw/안전 종료 근거.
- `appearance_ground_report_v28_final.json`: 최종 코드 재검사, 원본 ledger와 구분.

Base5 unit tests 70개 및 수정 스크립트 compile 통과.
최종 안전 계약 v6에는 runner/감사/지면/안전 코드 hash를 포함한다. 코드 변경 뒤 기존 v5
촬영을 최신 합격으로 resume하지 않으며, 원본 계약을 고치지 않고 report 재검사를 사용한다.
다음은 **근접 배치의 지형 높이 산정과 sparse-ground fallback 개선**이다.
이번 방법은 소량 명시적 재촬영용이며 모든 10K 장면/이동 경로의 지면 높이를 자동 해결하지 않는다.
전체 자산 검증, 다중 moving actor 도착률, 장시간 안정성, GPU latency도 남는다.
