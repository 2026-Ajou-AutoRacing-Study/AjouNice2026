# 검출 흔들림의 중복·분류 증폭 완화 후보

상태: EVIDENCE / opt-in. 2026-10-24. 운영 기본값은 두 후보 모두 **OFF**다.
실행 중인 ROS·MORAI·설치본을 변경하지 않았고, 판단·예측 3초/3초 정책도 유지한다.

## 문제와 범위

LAN Foggy15 `261024_035142_full_route.bag`의 두 중복 사례는 같은 프레임의 겹친
검출에 대한 NMS만으로 설명되지 않았다. 장애물은 서로 겹치지 않는 두 검출이
새 ID와 관측 경쟁을 만들었고, 트럭은 길이·중심 이상치로 기존 연결이 끊긴 뒤
새 후보가 다음 정상 관측을 가져가 확인됐다. 원본 raw 영상·LiDAR가 없어
검출기의 시각적 원인과 당시 정확한 내부 assignment는 확정하지 않는다.

분류 사례는 같은 ID 안에서 작은 장애물 박스와 승용차 크기의 검출이 교대로
연결된 것이다. 실제 차량 관측을 빼앗은 ID switch와 구분한다. 큰 차량 박스는
score 0.404~0.823이라 단순 저점수 제거·confidence 가중만으로 해결되지 않는다.

기록 commit의 실행본을 복제했다고 주장하지 않는다. 비교 기준은 submodule
`ea1429e13c7c99944607ad0aea18dc7ce14a1522`를 고정한 측정 시각 native 코어이며,
full-route는 기록 TF로 변환 가능한 2,334입력이다(3입력 TF 대응 불가 제외).

## 후보 1: 크기·점유 근거가 있는 연결 경쟁만 조정

`shape_consistent_association_ranking`은 다음 조건이 모두 있을 때만 동작한다.

- 기존 confirmed track과 새 tentative track 모두 원래 association gate를 통과한다.
- 관측 크기는 기존 track과 비슷하다(길이·너비 최대 치수비 1.35 이하).
- tentative와 관측은 크게 다르다(최대 치수비 1.50 이상, 유효 치수 0.25m 이상).
- 두 후보·관측의 방향이 약 45도 이내(180도 뒤집힘 허용)이고,
  tentative 중심과 관측 중심이 기존 track footprint 안에 있다(수치 여유 0.10m).

이때 tentative 연결의 순위를 해당 기존 후보 뒤로만 옮긴다. 허용 edge를 없애거나
gate·birth 거리·confirmation 횟수·coasting을 바꾸지 않는다. 기존 후보가 다른
관측을 받으면 tentative도 남은 관측을 받을 수 있다. 새 객체를 무기한 억제하지 않는다.

트럭 사례에서는 정상 박스를 기존 ID가 받아 잘못된 두 번째 confirmed ID가 사라졌다.
기존 트럭의 대응 위치 변화 최대는 약 6.8cm였다. 장애물 0134 사례는 그대로 남았다.
기존 14장면 GT 대응 평가에서는 obstacle false occupancy 감소가 있었지만,
모든 중복·오검출의 해결이나 새 LAN 주행의 개선으로 일반화하지 않는다.

차량·UNKNOWN 전체에 Gaussian 순위를 적용한 더 넓은 후보는 기각했다. 두 중복을
줄여도 1,236프레임을 바꾸고, 서로 겹치지 않는 실제 두 장애물 중 하나가 가려졌을 때
새 객체 관측을 안정적인 이웃 ID가 가져가는 native 반례가 재현됐다. 이 후보의
런타임 flag·구현은 제출 코드에 남기지 않았다.

## 후보 2: 큰 크기 변화와 함께 오는 분류 전환 확인

`guard_inconsistent_class_transitions`는 confirmed UNKNOWN↔CAR/TRUCK 전환에서
유효 치수비가 1.8 이상일 때만 class 표와 치수 갱신을 잠시 보류한다.
동일 family·비슷한 크기 관측이 3회 연속, 간격 0.5초 이내 이어지면 보류한 표를
원래 alpha 0.2 class EMA에 순서대로 반영한다. 위치 EKF 갱신·관측 수명은 계속된다.
미관측 입력·역행 시각·gap·track reset은 누적 근거를 초기화한다.

정지 차량도 올바른 차량 검출 4회로 원래 ID 안에서 UNKNOWN→CAR 복구가 가능하다.
첫 분류 영구 고정·정지 차량 승격 금지·차량/UNKNOWN 연결 전면 금지는 하지 않는다.
작거나 비정상인 치수는 근거로 사용하지 않고 원래 EMA로 복귀한다.

full-route의 대표 class 전환은 4→2회로 줄었다. 그러나 일관되게 반복되는 잘못된
차량 검출은 통과했다. 이는 진동 완화이지 정답 분류 복구 증명이 아니다.
14장면 pooled center AP의 class-only obstacle 변화는 약 −0.027%p였다.
따라서 기본 ON으로 올리지 않으며 새 raw 입력에서 오분류 원인을 대조한다.

## 최종 독립 장면 비교

14장면의 동일 5,822개 평가 입력에서 **후보 1만 ON**으로 비교했다.
기존 frozen-catalog center AP proxy이며 새 LAN 실주행의 tracking AP 인증은 아니다.

| 항목 | 기존 | 후보 1 |
|---|---:|---:|
| Obstacle center AP | 44.7746% | 45.7563% |
| Vehicle center AP | 77.0372% | 동일 |
| Pedestrian center AP | 92.5512% | 동일 |
| Obstacle GT 대응 표본(2m) | 3,047 | 동일 |
| Obstacle 예측 표본 중 catalog GT 미대응(2m) | 733 | 662 |
| GT 대응의 인접 ID 변화 횟수 | 72 | 57 |

GT 미대응은 해당 catalog·거리 규칙의 수치이며 모든 실제 오검출의 정답 판정이 아니다.
Vehicle GT 속도 오차 AVE 0.615917m/s는 그대로였다. Obstacle AP는 O02에서
약 +2.026%p, O01에서 약 −0.003%p로 소폭 악화도 있었다. 전체 수치가 좋아도
모든 장면이 좋아졌다고 표현하지 않는다. 후보 2까지 함께 켜면 obstacle AP는
45.7284%로, 후보 1 단독보다 조금 낮았다.

## 검증·한계

- 합성 native: 트럭 이상치→정상 복구, 가까운 얇은 장애물 두 개, 재취득,
  옆 차선/cut-in, 큰 차량 부분 가림·크기 회복, 보행자, 지속적인 새 큰 관측.
- class helper: 연속성·gap·잘못된 설정·치수·초기화·고정 크기 메모리와 원래 EMA.
- core 통합: 200프레임 OFF 동등성, 정지 차량·역전환·보행자·위치 갱신 보존.
- CMake에서 실제 ROS C++ 노드 빌드 및 native 테스트 91개 통과. class 통합 12개는
  ASan+UBSan도 오류 없이 통과했다. 분석기·launch·자동 계측 Python 테스트 28개 통과.
- 기존 14장면 5,986입력/5,822평가와 기록 TF 6 bag의 3,696입력으로 독립 회귀.
  native center AP proxy이며 AMOTA/IDF1·planner 안전 인증으로 부르지 않는다.
- full-route까지 총 12,016입력에서 옵션 모두 OFF는 기존 harness의 출력 필드와
  prior XY covariance 3원소가 일치했다. 모든 내부 행렬을 dump한 비교는 아니며,
  전체 state/covariance 검사는 위 native GTest의 합성 입력 범위와 구분한다.
- 부수 수정: inactive `TrackStruct.classification`도 UNKNOWN으로 초기화해,
  전체 slot 복사 시 sanitizer가 잡은 기존 미초기화 enum 읽기를 제거했다.
  대표 class 계산은 기존 class vote를 사용하며, 이 UB가 기록 문제의 원인이라는 뜻은 아니다.

추적 코어의 trace 출력·결과 복사 I/O를 제외한 `RunPrediction/RunUpdate`도 따로 측정했다.
매 장면 fresh process 첫 30입력만 timing warmup에서 빼고 3회 반복했다.
full-route의 평균/p95(ms)는 기존 `0.0247/0.0803`, 새 코드 옵션 OFF `0.0306/0.0841`,
후보 1만 ON `0.0310/0.0901`이었다. ROS callback·TF·메시지·GPU·planner 비용은 포함하지
않으며 이 결과를 live end-to-end 지연이나 최대 지연 보장으로 해석하지 않는다.
timing 사본의 마지막 primary 출력도 일반 trace 재생과 매 case/policy/repeat에서 일치했다.

문서 검증기는 새 연결의 파손 없이 기존 README/CURRENT 3개의 분량 초과를 보고했다.
이번 후보 작업과 무관한 해당 문서의 대규모 정리는 하지 않았다.

최종 source·header·binary hash, 반복 결과, 지표·비용 분포는 로컬의
`planner_logs/analysis/261024_tracker_consistency_v1/final_v2/`에 보존한다.
원인 역산 근거는 `planner_logs/analysis/261024_035142_full_route/{duplicate_rootcause,class_rootcause}/`다.

## 사용과 후속

대상 PC에서 변경된 submodule·launch를 함께 빌드한 **새 스택**에서만 선택한다.
기존 설치본과 새 source launch를 혼합하지 않는다. 운영 `tracking` alias는
`object_pipeline.launch`를 직접 사용하므로 인자는
`shape_consistent_association_ranking:=true`, `guard_inconsistent_class_transitions:=true`다.
별도 `tracking_prediction.launch` wrapper에서는 두 이름 앞에 `tracker_`를 붙인다.
두 경로 모두 기본값은 false다.
generic core/standalone launch도 OFF이며 기존 운영 alias를 몰래 교체하지 않는다.

2026-10-24 후속으로 [기록 입력의 실제 ROS·Cruise 대조](OBSERVATION_CONSISTENCY_ROS_VALIDATION.md)를
완료했다. 실제 LAN·LC/로터리 전체 판단·폐루프는 여전히 별도다.
후보 2는 반복되는 오분류의 영상·LiDAR 근거가
우선이다. 남은 장애물 중복을 없애려고 전역 거리·NMS·삭제시간을 강화하지 않는다.
