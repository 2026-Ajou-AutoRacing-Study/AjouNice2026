# Base5 최종 학습 후보 — 본수집 전 5개 보완 계획

2026-10-07. 상태: **PLAN / 미구현·미검증 항목 포함**.

후속 구현·실제 실패·보완은 [v105 근거](261007_base5_final_gates_v105_evidence.md),
현재 실행·잔여 조건은 [5K CURRENT](BASE5_V2_CURRENT.md)를 우선한다.
아래 항목의 "미구현"은 최초 계획 시점이며 현재 완료 판단으로 사용하지 않는다.

**후속 사용자 변경:** 이번수집은[Sunny11시직접5K](261007_base5_direct_5k_current_plan.md)다.
아래10K/A·B5K선별은변경전계획이다. 5번은직접5K의균형/희귀자산coverage감사로
대체하고나머지population·초근접·GT보완은유지한다.

## 목적과 현재 경계

이번 Sunny 11시 10K는 단순 폐기용 pilot이 아니라 최종 학습본 후보로 수집한다.
Base3 추가분이 아닌 독립 Base5 single-frame 학습용이며 temporal/velocity head는
학습하지 않는다. Ego는 Park·정지, 장면당 1프레임이다. 사람·동물 등 움직임에 따라
외관이 바뀌는 자산은 기존 실제 이동 캡처 정책을 유지한다.

최종 확장은 11/13/15시 × Sunny/Foggy, 환경당5K·총30K를 목표로 한다.
이번10K에서 사전 정의한 균형5K를 재사용할 수 있도록 설계한다. 대회까지 약3주인
일정을 고려해 아래 보완→대표 검증→설계 고정 순서로 진행하고 무제한 재설계를 피한다.

현재 v86의10K JSON 생성·오프라인 감사 및 소량 live 검증은 완료됐지만,
**아래5개 보완은 아직 완료되지 않았고 새10K 본캡처도 시작하지 않았다.**
기존 manifest를 이 계획의 구현 결과로 해석하지 않는다.
코드·config·manifest 변경 후에는 새 버전/새 output으로 검증하며 기존 수집에
다른 계약을 섞어 resume하지 않는다.

## 1. 장면당 객체 수 비율 및 실제 가시 객체 수

- **후속 v89:** 사용자와재합의하여아래50/30/15/5 전체공통개수안은채택하지않았다.
  범용만크기별예산,미션은실제상황별개수로분리했다.
  [구현·검증·제한](261007_base5_population_budget_v89.md)을우선읽는다.
- 현재 단일/2개/3개/4개는70/21/6/3%다. 제안 목표는 **50/30/15/5%**이며
  클래스/패밀리 비율을 바꾸는 뜻은 아니다. 비어 있는 배경500장에는 적용하지 않는다.
- primary와 secondary의 수를 센다. 지도에 원래 있는 Tree·표지판·가로등은 세지 않는다.
  배경 시설을 새로 무작위 배치하는 변경도 이번 계획에 포함하지 않는다.
- 복합 actor 하나가 여러 GT component를 반환하므로 **배치 actor 수와 positive GT 수를
  별도 집계**한다. 보이는 GT 수만으로 actor 수 quota를 대체하지 않는다.
- 실제 센서/positive 근거가 있는 actor 수, 가려진 actor 수, 클래스별 GT 수를 감사한다.
  명목상4개지만 실제1개만 보이는 장면을 혼합 coverage 성공으로 과장하지 않는다.
- 대표2/3/4개 장면에서 가림·접지·물리 충돌·가시성·재시도 비용을 확인하고,
  유효 혼합 비율이 지나치게 떨어지면 배치 간격/방향부터 수정한다.

## 2. 범용 및 정적 장애물 회피 초근접

- 현재 범용 차량 기본 중심 거리6m부터인 조건은 초근접을 보장하지 못한다.
  범용 장애물 측방2–5m도 중심 거리이므로 긴 자산의 실제 clearance와 다르다.
- 차량·장애물 **회전된 geometric BBox footprint 경계 간격** 기준의 근접 quota를
  별도로 설계한다. 전방·앞옆·좌우 나란히·통과 후 측후방을 포함한다.
- 정적 회피 mission의 approach/entry/alongside/clearing 및 좌우 균형은 유지·보강한다.
  요청 간격과 실제 annotation에서 계산한 간격을 모두 기록한다.
- 최소 간격/각 bin 비율은 자산별 크기와 실제 collision smoke로 결정한다.
  기존 요청0.1m를 실제 무충돌0.1m 확보로 해석하지 않는다.
- annotation BBox는 collider와 같지 않다. BBox 중첩만으로 물리 충돌이라고
  단정하지 않되 실제 CollisionData·배치 상태·접지 검증을 생략하지 않는다.

## 3. 고주로 추월 및 로터리 밀집 초근접

- 현재 고주로 중심 거리6m부터, 로터리5m부터인 기본 deck만으로는 부족하다.
- 고주로 mission: 앞→앞옆→측방→측후방의 추월 상대 위치를 좌우 모두 설계한다.
  정지 독립 배치로 순간 관측을 재현하며 실제 자동 추월 제어를 필수로 만들지 않는다.
- 로터리 mission: 진입·회전·출구 각각 앞차/뒤차 bumper 간격과 곡선에서 비스듬히
  인접한 차량을 포함한다. Ego와 NPC의 각 도로 접선 및 합법 경로를 유지한다.
- 초근접을 넣으면서 원거리45–99m 차량 coverage를 잠식하지 않는다.
  기존 family 내 근거리 bin 재배분을 우선 검토하고 숫자는 live 검증 후 확정한다.
- 다중 차량 가림/물리 충돌/ROI/실제 간격을 확인한다. 밀집 배치의 성공이
  detector 거리 오차 개선이나 안전한 실제 주행의 보장이라는 뜻은 아니다.

## 4. 가시성 및 GT 정책 최종 검증

- raw LiDAR Instance geometric annotation은 보존한다. 현재 positive 정책은
  `lidar1_camera0`이며 LiDAR≥1 또는 정책상 camera-visible 근거를 사용한다.
  이번 계획만으로 기준을 변경하거나 무조건5-point 제거로 되돌리지 않는다.
- 원거리camera-only, 초근접 일부 잘림, 낮은 point 수, 혼합 가림,
  복합 actor의 여러 component를 대표 조건으로 검사한다.
- 안 보이는 객체를 무조건positive로 넣지 않고, 점이 적다는 이유만으로
  camera-visible 객체를 모두 버리지 않는다. component별 근거/제외 이유를 남긴다.
- 근접 box가 ROI 일부를 벗어나는 경우와 center 기준 range의 기존 변환 계약을
  명시적으로 대조한다. 새 규칙은 raw 변경 없이 versioned 변환 정책으로 도입한다.
- 알려진 class/asset 정책 유지: vehicle/pedestrian/obstacle/animal;
  WoodBox·CargoBox 및 MR 복구 자산 포함, 열린G80 공식 제외,
  Tree/Fire/Speed_Bump/맨홀/일반 표지판·가로등은 추가positive 배치 제외.

## 5. 최종5K 선별을 수집 전에 고정

- 10K를 **A/B 각5K**로 사전 할당한다. 어느 한쪽도 family/subfamily·거리·방향·
  객체 수·근접 단계·asset coverage가 균형을 이루도록 층화한다.
- 기존 단일 quota deck를 무작위 반으로 나누는 것만으로 균형을 보장하지 않는다.
  홀수quota 및 희귀자산은 최대1장 차이 등 허용오차를 명시하고 결과표를 감사한다.
- A/B는 최종 환경 확장에 사용할 선택 묶음이지 train/val split이 아니다.
  train/val은 별도 scene 단위 계약을 유지한다. A만 선택해도 필요한 split이 있어야 한다.
- 같은 actor 배치의6환경 반복은 같은 scene group에 묶어 train/val 누수를 방지한다.
  Sunny11시10K 전체를 써 환경별 표본 수가 불균형하면 sampling 정책도 명시한다.
- 실패 대체 시 원래 A/B·family·거리·방향·근접 quota·split을 유지한다.
  평가가 잘 나온 scene만 골라5K를 만들지 않는다.

## 구현 및 채택 순서

1. 객체 수·초근접·미션 밀집·A/B selection을 generator/config/manifest에 구현한다.
2. 작은 deck으로 quota 전수감사와 geometry 단위검증을 수행한다.
3. MR에서 대표 근접·혼합·원거리camera-only·복합·동적 장면만 live smoke한다.
4. CPU 변환 GT 및 실제 가시 actor/간격/충돌/접지/재시도·resume 보존을 확인한다.
5. 실패 분포를 보고 제한적으로 수정 후 검증한다. 통과하면 새10K 생성·오프라인 감사,
   A/B각5K 통계표·운영 런북을 고정하고 사용자에게 결과/한계를 보고한다.
6. **본수집은 별도 사용자 승인 후 시작한다.** 이번 문서 커밋은 본캡처 승인이나
   위5개 구현 완료를 뜻하지 않는다. production 모델·threshold도 변경하지 않는다.

완료 보고에는 실제 구현 commit, config/manifest hash, 배치/가시 actor 분포,
실측 clearance, 제외/실패/부족분, GT 검사, capture wall-time을 남긴다.
현재 소량 표본 비용 약43.6시간/103.6GiB 외삽은 변경된 혼합/근접 정책의 ETA가
아니므로 새 대표 검증 후 재계측한다.

## 관련 진입점

- [현재 실행·검증 상태](BASE5_V2_CURRENT.md)
- [v86 통합검증 근거](261007_base5_prebulk_recovery_evidence.md)
- [운영 런북](261006_base5_bulk_resume_v45.md)
- [MR asset 정책](MR_ASSET_COLLECTION_CURRENT.md)
