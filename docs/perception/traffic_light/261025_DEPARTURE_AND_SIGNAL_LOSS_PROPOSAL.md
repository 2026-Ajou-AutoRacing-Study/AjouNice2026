# 황색 새 출발·신호 상실 정책 제안

종류: PROPOSAL + IMPLEMENTED CANDIDATE / 2026-10-25.
**아래 본문은 10/25 기본 OFF 후보 당시 기록이다. 10/26 추가 시험과 운영 전환은
[후속 절](#production-261026)을 따른다. 과거 시험 결과·실패 분류는 소급 변경하지 않는다.**
현행 코드 사실은 [통합 감사](261025_SIGNAL_POLICY_INTEGRATION_AUDIT.md), 모델 상태는 [CURRENT](CURRENT.md)를 따른다.
MORAI 대회 시뮬레이션을 위한 제안이지 실제 도로용 안전 인증이나 법률 해석이 아니다.

## 권고

1. 정지선 전에 정차한 차량은 **Y·R+Y만 보고 새로 출발하지 않는다.**
   경로에 맞는 신선한 허가 신호를 다시 확인한다. D 좌회전은 R+좌화살표이고 G 단독이 아니다.
2. 신호·카메라 상실은 **GO를 새로 만드는 근거가 아니다.**
   이미 정지선을 정상 통과한 차량을 교차로 안에서 급정지시키는 것도 피한다.
3. '아직 정지선 전', '정차 중', '제동 가능/이미 진입 결정', '앞바퀴 통과 완료'를 나눈다.
   단순히 적색을 더 많이 발행하거나 timeout 하나를 바꾸는 것으로 끝내지 않는다.

정책 범위는 **A/B/C/D 전체**다. D는 R+Y 출발 문제가 처음 재현된 시험 지점일 뿐이다.
A/B/C에서도 허가 G로 접근하다 황색으로 바뀌는 제동 경계가 존재한다. 허가 판정은 해당 lanelet의
실제 진행 방향에 맞춰야 하며, B/C를 포함해 "어떤 녹색 전구든 보이면 GO"로 일반화하지 않는다.
현재 대회 경로의 A/B/C 직진과 D 좌회전, 다른 방향의 허가 신호는 구분한다.

[규정 v1.2 보관 원문](../../competition/261015_대회_규정_v1.2_원문.md)은 초록불/좌회전 신호 및
**앞바퀴가 공식 정지선을 넘는 순간**으로 판정하고, 신호 미준수에 **30초 패널티**를 부여한다.
네 교차로를 모두 미준수하면 해당 차시 실패이며, 동일 교차로 미준수는 한 번만 집계한다.
황색 또는 R+Y에서 새로 출발해도 된다는 명시는 없다. 따라서 "적색으로 바뀌기 전에만 건너면
무조건 허용"이라고 단정하지 않는다. 공식 추가 답변으로 해석이 바뀌면 갱신한다.

규정은 신호 변경 주기를 공개하지 않는다. 사용자는 황색 길이가 상황에 따라 달라질 수 있다는 점을
제기했다. 사용자의 2~3초일 가능성이 높다는 예상도 공식 최소/최대 시간이나 분포로 확정하지 않는다.
후속 주 시험은 2·3초를 중심으로 하고 1·5초는 off-nominal stress로 구분한다. 아래 과거 1·3·5초는
**당시 개발용 주입 시간**이지 실제 대회 주기나 출제 확률의 추정치가 아니다.
처음 황색을 본 시각은 점등 시작 시각이 아니다. 최근 G→Y 전환을 관측해도 황색 전체 길이의
하한을 모르면 남은 시간의 하한은 보장할 수 없다. 고정 황색 시간·카운트다운을 허가 근거로 사용하지 않는다.

## 상황별 계약 후보

| 상황 | 제안 | 피해야 하는 처리 |
|---|---|---|
| 선 전 정차 + Y 또는 R+Y | 정차 유지, 새 경로 호환 G/L 재관측 후 출발 | Y 포함만으로 즉시 GO |
| 선 전 이동 + 황색 전환 | 실제 속도·가감속·정지선 거리로 정지 가능성/진입 상태 판정 | 정지·이동을 같은 규칙으로 처리, 임의 잔여 시간 추정 |
| 유효 신호로 앞바퀴 통과 후 | 현재 교차로 통과를 이어감; 장애물 제동은 별도 | 이후 적색/카메라 상실만으로 교차로 안 급정지 |
| 선 전 + 새 이미지는 있지만 박스/색/ID가 불확실 | UNKNOWN 사유 노출, 짧은 관측 필터 후 허가 취소·정지 요구 | 빈 관측을 녹색으로 보완, 다른 방향 신호 사용 |
| 선 전 + 카메라/인지 publisher 단절 | 실제 관측 age를 기준으로 허가 만료·정지/감속 요구 및 진단 | timeout=GO, 마지막 G를 무기한 유지 |
| 정상 접근 후 등주가 화각 밖으로 나감 | 정지선 진행 상태와 마지막 유효 관측을 함께 판단 | 화각 밖이라는 사실만으로 새 허가 생성 |
| 장기간 상실로 재출발 불가 | 명확한 오류·수동 복구 안내; 새 유효 관측 후 재개 | 일정 시간 경과만으로 자동 GO |

움직이는 차량의 상실 대응은 '즉시 모든 속도 0'이 아니다. 실제 판단팀의 stop/commit/pass-judge와
속도 계획을 연결한다. 정지 불가능 영역을 처리하는 것과 신호 패널티가 없음을 보증하는 것은 다르다.

## 구현된 후보와 기본 동작의 구분

실제 구현은 [planning 가드](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/include/autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp),
[원본 모듈 연결](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/scene.cpp),
[실험 모듈 연결](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/experimental/scene.cpp),
[인지 관측 정책](../../../base_ws/src/perception/traffic_light_detection/scripts/signal_phase_policy.py)을 본다.

- `enable_signal_compliance_guard=false`가 기본이다. OFF의 기존 competition fail-open·황색 분기는
  별도 대조군이며, 후보 시험 결과를 기존 운영값의 안전성 인증으로 읽지 않는다.
- 후보 ON은 정지선 전 missing·stale·UNKNOWN을 새 GO 근거로 사용하지 않고 Y/R+Y의 정차 출발을 막는다.
  경로에 맞는 허가 G/좌화살표를 **서로 다른 신선한 source 관측 3개**로 확인한다.
  반복 planner callback이나 같은 stamp의 유지 관측은 새 확인으로 세지 않는다.
- 가드 자체는 네 구간에 공통이고, 허가 신호는 기존 lanelet 진행 방향별 판정을 재사용한다.
  D만 예외로 고친 전용 규칙이 아니며, A/B/C의 이동 중 G→Y도 같은 제동 경계 시험 대상이다.
- 이미 허가 G를 확인하고 이동하던 차량의 G→Y에만 앞바퀴 기준 제동 거리 예외를 검토한다.
  정차 상태의 황색 출발 허가는 아니다. **정지 거리 계산은 잔여 황색을 예측하지 않으며,
  이 예외도 앞바퀴 통과 순간의 허가 신호나 무패널티를 보장하지 않는다.**
- 앞바퀴가 선을 지난 뒤에는 GO_OUT을 유지하되 다른 장애물 제동은 별도다.
  rear-axle `base_link`→front axle 3.0m와 front bumper 위치는 구분한다.
- 현재 인지 소스는 채택한 관측의 원래 stamp를 유지한다. D bounded hold를 새 이미지의 재관측으로
  갱신하지 않으며, 정책 fallback R과 실제 램프 관측은 debug provenance에서 구분한다.
  `[무조건 새 stamp 유지]`라는 이전 문제 설명은 이 source 수정 전 상태에 해당한다.
- `planner_compliance_observations=false`가 기본이다. true는 상류 7/4 smoothing 대신 실제 관측을
  직접 전달하는 별도 시험 옵션이며, downstream 허가 확인을 없애는 기능이 아니다.
- D의 [기존 프로필](../../../base_ws/src/perception/traffic_light_detection/config/traffic_signal_profiles.yaml)에는
  `observation_guard.max_yellow_sec=4.0`이 남아 있다. `planner_compliance_observations=true`여도
  이 상류 장기 황색 가드는 적용되어 지속 R+Y를 fallback R로 바꿀 수 있다. **4초는 남은 황색의
  추정치나 실제 주기 보장이 아니라 기존 관측 유지 상한**이다. C++ 후보가 고정 황색 시간을
  통과 허가에 사용하지 않는다는 것과 전체 D 파이프라인에 시간 의존성이 없다는 것은 다르다.
  이 상한은 별도 시험 없이 변경하지 않았다.

### 3개 G 확인과 재출발 지연

3개 확인은 **3초 대기**가 아니다. 누락 없는 10Hz 입력이면 첫 허가 G 관측에서 세번째까지 약 0.2초다.
안정 G를 정지선 도착 전에 확인했다면 선 앞에서 그만큼을 다시 기다리는 것도 아니다.
기존 7/4 smoothing이 먼저 적용된 정상적인 R→G 전환에서는 이론적으로 약 0.3초가 더해져,
첫 G에서 약 0.5초가 될 수 있다. 이는 **균일 10Hz·연속 관측을 가정한 계산이며 실제 지연 측정값이 아니다.**
카메라·모델·큐·planner 주기, UNKNOWN에 따른 확인 초기화, 짧은 G를 놓쳐 다음 현시를 기다리는 비용을
별도로 측정해야 한다. 연속 프레임 오류는 상관되므로 "3개이면 오검출 확률이 세제곱"이라고 주장하지 않는다.
재출발 시간을 줄이는 목표는 유지하되, 시간 절약이나 순위 개선을 이 시험만으로 주장하지 않는다.

## 완료된 국소 개발 시험과 한계

[D 정차 황색 비교 JSON](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_yellow_comparison_v1.json)은
같은 D 접근 조건의 기존/후보 대조다. 실제 planner·controller·localization과 YOLO/CNN을 사용했으며,
native API는 **장면의 신호를 제어·독립 관측하는 개발 도구**일 뿐 planner 신호 입력으로 변환하지 않았다.

- 후보는 D의 R+Y 1·3·5초와 사이의 적색 2초 단계에서 정차를 유지했다.
  기존은 첫 R+Y에서 움직였고, 그 다음 적색 구간에서 앞바퀴 기준점이 유한 정지선을 넘었다.
  crossing 전후 exact D native readback은 모두 R=1이었다. 최초 R+Y가 인지에서 잘못 분류됐다는
  근거는 없으며, 동일한 R+Y wire를 받은 후보가 정차한 결과다.
  다만 5초 단계 후반에는 위의 기존 4초 가드가 영향을 줄 수 있으므로,
  "5초 내내 R+Y가 그대로 전달됐다"거나 장기 황색 처리 전체가 독립 검증됐다고 주장하지 않는다.
- 기존의 1·3초 단계는 완료됐지만 5초 단계는 목적지 정상 마감으로 약 2.61초만 관측됐다.
  이를 세 개의 독립적인 정차 재출발 시험이나 5초 전체 완료로 세지 않는다.
- [D 기존 missing 제어 기록](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_baseline_missing_v1/phase_controller/events.jsonl)과
  [원관측](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_baseline_missing_v1/observer/observations.jsonl)에서,
  R=1을 확인하며 신호를 drop한 약 3.03초 구간의 Ego XY 최대 이동은 첫 표본 대비 약 **5.11m**였다.
  [crossing 기록](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_baseline_missing_v1/observer/crossings.json)도
  전후 R readback이 확인된 선 통과를 남긴다. 같은 drop·잘못된 ID 주입에서
  [후보의 제어 기록](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_guarded_missing_v1/phase_controller/events.jsonl)과
  [원관측](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_guarded_missing_v1/observer/observations.jsonl)은
  정차 유지와 일치한다. 정확한 내부 planner 분기까지 기록한 인과 인증은 아니다.

이 결과는 상실·황색에서 불필요한 출발을 줄이는 후보의 근거이지, 공식 위반 집계·LAN 실시간성·
모든 미션/날씨·최신 3개 G 확인/직접 관측 옵션의 성능 인증이 아니다. 각 조건은 연속된 국소 시험이며,
주행 목표 SUCCESS·마지막 정지/P와 신호 준수 판정은 별도로 본다.

### 제어 도구의 −1 문제: 자연 복귀로 해석하지 않음

옛 개발 제어기는 마지막 `status=-1` 전송을 `natural_release`라고 기록했지만, 이 이름은 복원 증거가 아니다.
이후 exact-ID GET은 UNDEFINED를 반환했고,
[D missing 후보 crossing 직전 원본 JPEG](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_guarded_missing_v1/observer/crossing_jpegs/crossing_00017600_before.jpg)에는
R·Y·좌화살표·G의 동시 점등이 보인다. signed short의 −1은 `0xffff`이므로 모든 lamp bit 적용과
일치하지만 서버 내부 원인은 확정하지 않았다. **−1을 자연 프로그램 복원 명령으로 다시 사용하지 않는다.**
자연 복귀 뒤 crossing은 허가 여부 미확인으로 남긴다. 반면 −1 이전에 **양수 R=1과 전후 관측으로
확인된 기존 대조군의 적색 crossing 근거는 이 복귀 오류 때문에 소급 무효가 되지 않는다.**

재시작·원본 장면 재로드 후 제어 도구 없이 실제 프로그램의 시간 변화를 확인하기 전에는 자연 복귀를
인증하지 않는다. 양수 G/좌화살표를 주입한 controlled GO도 재출발 기능 시험일 뿐 자연 현시 인증이 아니다.
native/GT와 Ego는 계속 개발 평가 전용이며 production 인지·판단 입력에 넣지 않는다.

### 재시작 후 자연 A/B/C/D 폐루프 1회

`compliance_full_guarded_natural_v1`은 `enable_signal_compliance_guard=true`,
`planner_compliance_observations=true`, `route_progress_guard=true`, 관측 timeout 0.5초의 별도 시험이다.
Sunny13·무객체·od_gt·실제 localization/planner/controller 조건이며 BEVFusion 풀스택 부하 시험은 아니다.
신호 제어 SET 없이 진행했고 native GET과 원본 JPEG만 평가에 사용했다.
[확정 요약](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_full_guarded_natural_v1_summary.json)과
[정지·P receipt](../../../planner_logs/validation/traffic_light_compliance_full_regression_261025_234245/success/full_regression_20261025_234251_01/safety_receipt.json)를 분리해 보존했다.

- A/B는 G=16, C 두 head는 G+L=48, D는 R+L=33으로 정지선 교차 전후 0.25초·0.5초 native bracket이 모두 일치했다.
  이는 개발용 앞바퀴 중심 기준점과 유한 정지선의 교차 proxy이며 공식 판정 프로그램의 PASS가 아니다.
- route ID는 `0→A→B→C→D→0`으로 진행했고, 이번 주행에서 지난 미션의 역방향 재선택은 없었다.
- goal SUCCESS와 마지막 정지·P 확인을 마감했다. 원본 JPEG 전후 쌍 4개(8장)는 별도 보존했다.
- callback p50/p95는 24.83/43.86ms였다. MORAI 동시 실행 로컬의 기술 통계이지 다른 후보 대비 인과적 가속 결과가 아니다.
- 이 자연 주행에서 A/B/C의 이동 중 황색 제동 경계를 충분히 밟았다는 근거는 없다.
  따라서 정차 황색 2·3초, 이동 중 황색, 신호 상실·큰 측위 오차 시험을 대신하지 않는다.

### A/B/C/D 황색 2·3초 후속 시험: 완료 구간과 제어 실패 구분

모두 최신 3개 허가 관측·직접 관측 전달·0.5초 timeout 후보이며, 정차 중 황색 새 출발 시험이다.
**이동 중 G→Y의 딜레마 시험은 아니다.** 기존 1·3·5초 기록을 덮어쓰지 않았다.

| 미션 | 2초·3초 황색 구간 | 시험 전체 마감 |
| --- | --- | --- |
| A | 두 구간 완료·정차, wire에도 황색 수신 | 이후 controlled GO 확인 후 실제 상태가 달라져 중단·정지/P 확인 |
| B | 두 구간 완료·정차, wire에도 황색 수신 | 이후 controlled GO 확인 후 실제 상태가 달라져 중단·정지/P 확인 |
| C | 미수행 | 초기 적색 요청에 두 head가 48/1로 달라 준비 중 중단·정지/P 확인 |
| D | 두 구간 완료·정차 | 이후 controlled GO 확인 후 실제 상태가 달라져 중단·정지/P 확인 |

분석 근거는 [A 수정 경로 요약](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_A_guarded_yellow_2_3_v1_summary_v2.json),
[B 요약](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_B_guarded_yellow_2_3_v1_summary.json),
[C 미수행 요약](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_C_guarded_yellow_2_3_v1_summary.json),
[D 요약](../../../artifacts/evaluation/traffic_light_robustness_261025/compliance_D_guarded_yellow_2_3_v1_summary.json)이다.
A의 최초 파생 요약은 분석자가 존재하지 않는 validator 경로를 입력해 P를 미확인으로 남긴 자료다.
원본은 보존하고 올바른 `error/` 경로로 `_summary_v2`를 새로 만들었다. 이후 요약기는 존재하지 않는
명시 실행 경로를 출력 생성 전에 거부한다. 시험 자체의 P 실패와 분석 경로 오류를 혼동하지 않는다.

A/B/D의 황색 구간 raw Ego 최대 속도는 모두 0.0015m/s 미만이고 출발 proxy는 false였다.
다만 전체 실행은 MANUAL_ABORT/시스템 오류 분류이며 PASS로 승격하지 않는다. 이 표의 중단은
사용자 수동 중지라는 뜻이 아니라 평가 helper 오류에 대한 소유자 watchdog의 정상 안전 종료다.
C의 route/trajectory 누락 경고도 controller 시작 전 종료와 구분하며, 색 인식 실패로 집계하지 않는다.
강제 GO 유지 실패는 제어 API/신호 프로그램 경합 등의 별도 조사 대상이고 후보의 황색 오출발 증거가 아니다.
재시도하면서 native 불일치를 허용하거나 C 한 head를 생략해 통과 판정을 만들지 않는다.

## 필요한 시험

- 자연 신호 프로그램 복원 확인을 먼저 마감하고, 이전 −1 이후 구간을 자연 현시 근거에 섞지 않는다.
- A/B/C 직진 및 D 좌회전에서 G/L→Y/R+Y→R, 정차 중 Y/R+Y 최초 관측, 주기 중간 시작.
  이동 접근의 제동 가능/불가능 경계를 구분한다. 가변 주입 황색은 2·3초 중심 주 시험과
  1·5초 off-nominal stress로 나누며 어느 값도 운영 countdown이나 실제 대회 주기 보장으로 쓰지 않는다.
- G 관측 직후 카메라 단절 / 프레임만 계속되고 검출 없음 / route·TF 상실 / 잘못된 ID.
- D 가드가 유지한 옛 R+L의 실제 나이, 복구 직후 한 프레임 G/L로 새 출발하지 않는지.
- 상류 smoothing OFF/ON과 downstream 3개 확인의 조합에서, 첫 실제 G→허가→제어→앞바퀴 통과 지연과
  UNKNOWN 초기화·G 놓침·추가 대기를 측정한다. 이전 후보 시험을 새 temporal 옵션의 증거로 재사용하지 않는다.
- D의 4초 장기 황색 가드 전후에서 fresh R+Y·원관측 stamp·fallback R를 구분하고,
  가드를 유지/변경할 근거를 별도 비교한다. 황색 잔여시간을 아는 것으로 해석하지 않는다.
- 정지선 전 먼 곳/제동 경계/이미 넘어간 뒤 각각 상실, 정상 화각 이탈과 통신 단절 구분.
- 원시 관측·유지 사유·미션 ID·공식 선까지 거리·front-wheel 통과·최종 제동/GO를 함께 기록.
- 공식 LAN·다른 미션·다른 환경에서 회귀 후 운영 승격 여부를 별도 결정한다. 기본 OFF는 이 문서로 바뀌지 않는다.

평가는 위험 오출발, 신호 위반, 교차로 내부 불필요 정지, 허가 후 재출발 지연을 함께 본다.
"정지율이 높음"이나 단일 이미지 색 정확도만으로 채택하지 않는다.

<a id="production-261026"></a>

## 2026-10-26 추가 검증과 운영 전환

사용자 요청에 따라 v5 모델은 그대로 두고 **준수 가드·직접 관측 전달·route 진행 보호**를
새 실행 기본으로 전환한다. 전체 대회 주행의 무위반 인증은 아니며 팀 4090 LAN 검증은 별도다.
기존 본문의 `기본 OFF`는 당시 상태이고, 현재 실행·복귀는 [CURRENT](CURRENT.md)와
[팀 설치 안내](V5_CANDIDATE_HANDOFF.md)를 따른다.

### 시험 방식과 증거 경계

양수 native override도 C 두 head 불일치·GO 유지 실패가 있어, 이를 허용하며 시험 PASS를 만들지 않았다.
대신 `synth_traffic_light_planner_trial.py`가 **개발 전용 별도 토픽**으로 합성 신호를 내고,
소유 wrapper가 시험 구간에만 표준 신호로 중계했다. native GET은 독립 기록만 하고 native SET은 하지 않는다.
이때 실제 YOLO/CNN은 계속 실행하지만 합성 구간에 planner로 전달하지 않는다. 따라서 아래 결과는
**실제 localization/planner/controller의 정책 시험이지 YOLO 정확도·실제 램프 준수 시험이 아니다.**
완료된 실제 이동/정차 구간만 후속 카메라 관측에 handoff한다. helper 오류·heartbeat 상실·미수행은
조용히 실관측으로 바꾸지 않고 해당 시험을 중단·정지/P로 마감한다. 종료 정리 뒤 heartbeat 만료는
시험 중 단절과 구분한다. 원본·실패 시도·native 상태·source stamp는 그대로 보존한다.

공통 준비 자료는 `artifacts/evaluation/traffic_light_robustness_261025/production_trials_abcd_261026_v4/`다.
유한 OSM 정지선, 실제 route, raw Ego heading −1.347° 한 번 보정 + 기록된 TF + 앞축 3m를 사용한다.
평가는 앞바퀴 중심 proxy이며 공식 판정 프로그램을 재구현한 것은 아니다.

| 시험 | 관찰 | 해석 |
| --- | --- | --- |
| A 이동 중 제동 가능, Y 2초 | 선앞 14.20m·1.03m/s에서 전환. Y 구간에는 가속했지만 선을 넘지 않았고, 후속 R에서 선앞 약 0.86m에 정차 | 황색 즉시 제동으로 설명하지 않음. 실제 정지 계획과 물리 응답을 구분 |
| A 근접 Y 3초 | 선앞 0.882m·9.03m/s, wire Y 30/30, 약 64ms 뒤 선 통과 | 가까워 제동이 어려운 조건의 통과. 공식 황색 허가를 보증하지 않음 |
| A 선 전 신호 단절 2초 | 실제 표준 메시지 공백 2.097초. 약 0.5초 뒤 정지 궤적, 약 0.89초 뒤 첫 제동. 선앞 약 0.883m에 정지 | timeout 뒤에도 계속 GO라는 증거가 아님. 감속 전환·제어 추종 지연 존재 |
| D 근접 R+Y 2초 | 선앞 0.871m·7.87m/s, wire R+Y 20/20. DEBUG 정지거리 13.026m·통과 결정, 약 81ms 뒤 선 통과 | 최근 허가 좌회전→황색의 이동 중 예외가 실제로 적용됨 |
| C 선 통과 후 신호 단절 2초 | 원시 선 통과 35ms 뒤 단절, wire 공백 2.238초. 속도 7.90~8.32m/s, 제동 0 | DEBUG GO_OUT 확인. 해당 신호 상실만으로 교차로 내부 급정지하지 않음 |
| B 정차 중 Y 3초 | wire Y 30/30, raw 최대 속도 0.000347m/s·최대 이동 약 0.016mm, 선앞 약 0.875m 유지 | 황색 새 출발 없음. 실제 G 재관측 뒤 정상 진행 |
| D 선 통과 후 R 2초 | 실제 선 통과 약 48ms 뒤 R, wire R 20/20. DEBUG GO_OUT, 구간 끝에도 7.16m/s | 이후 적색만으로 교차로 안 급정지하지 않음. 다른 감속은 해제하지 않음 |

위 일곱 시험은 각각 목적지 SUCCESS와 별도로 최종 정지/P VERIFIED를 확인했다.
원본 wrapper 경로는 순서대로 `production_fixture_A_yellow_stoppable2_261026_v3`,
`production_fixture_A_yellow_close3_261026_v1`, `production_fixture_A_loss_prefront_261026_v1`,
`production_fixture_D_yellow_close2_261026_v1`, `production_fixture_C_loss_postfront_261026_v1`이다.
추가 둘은 `production_fixture_B_stopped_yellow3_261026_v1`, `production_fixture_D_postfront_red_261026_v1`이다.
원본 입력 70개 SHA·원시 Ego exact-header 조인·유한 선 교차·표준 메시지 원stamp·정지/P를
[독립 파생 검토](../../../artifacts/evaluation/traffic_light_robustness_261025/production_fixture_independent_review_261026_v1.json)에 보존했다.
이전 A stoppable v1/v2는 각각 debug argv 오류·helper snapshot clock race로 중단됐고 성공 표본에 섞지 않는다.

### 이번에 바로잡은 코드·설치 문제

- 선 통과 GO_OUT 뒤 큰 재배치가 있으면 같은 callback에서 접근을 다시 평가한다.
  이전에는 다음 callback까지 한 번 GO로 남을 수 있었다. 가드 ON만 수정하며 OFF는 유지한다.
- 저속 triangular jerk 제동 분기에서 기존 공통 함수가 음수 종료 시간을 쓸 수 있음을 독립 적분과
  설치 라이브러리 호출로 확인했다. 1m/s·초기가속도 0·최대감속 −6·jerk −10·지연 0.5초에서
  기존 0.353m 대비 보정 0.798m, 1.8m/s 경계에는 기존 약 0.72m 불연속이 있었다.
  **신호등 가드 ON의 해당 분기만** 초기가속도로 구한 양의 종료 시간으로 보정한다.
  공통 라이브러리·다른 판단 consumer·가드 OFF·기존 trapezoid 값은 바꾸지 않는다.
  지연 구간의 기존 `v×delay` 근사까지 완전한 차량 동역학 모델로 바꾼 것은 아니다.
- 개발 DEBUG는 실제 planner 하위 launch까지 전달하되 운영 기본 logging은 기존대로 둔다.
  내부 분기 로그가 없던 앞선 시험에 새 로그를 소급 적용하지 않는다.
- 딸깍 설치기는 Python 모델뿐 아니라 신호등 C++ plugin을 재빌드·설치하고 source/build/install을
  대조한다. CMake RPATH 제거에 따른 정당한 ELF 차이만 정규화하며 다른 section 변경은 거부한다.
  이 검사는 설치 단계이고, 대회 기동 중 새 refusal/정지 gate를 추가한 것이 아니다.

### 운영 기본값 그대로의 자연 신호 확인

wrapper의 `--mode production`은 신호 준수/관측/route 인자를 덮어쓰지 않는다.
`--compliance-debug`는 개발용 내부 판단 로그만 켜며 정책 옵션은 아니다. native SET과 합성 proxy는 없다.
실제 parameter snapshot에서 가드 ON·직접 관측 ON·route 보호 ON·timeout 0.5초를 확인했다.

Sunny13 전체 경로 `production_full_natural_sunny13_261026_v1`은 SUCCESS·정지/P VERIFIED였다.
4회 통과는 A/B G16, C 두 head G+L48, D R+L33으로 전후 0.25/0.5초 bracket에서 모두 허가였다.
route는 `0→A→B→C→D→0`, 지난 미션 matched 재선택은 없었다.
matched 562프레임 callback p50/p95는 34.02/50.44ms이며 전체 접근/미가시 포함 값과 구분한다.
[독립 검토](../../../artifacts/evaluation/traffic_light_robustness_261025/production_full_natural_sunny13_independent_review_261026_v1.json)는
raw Ego header·TF 재계산·원본 10개/JPEG 8장의 SHA와 실제 설치 plugin SHA를 보존한다.

Foggy13 첫 전체 주행 `production_full_natural_foggy13_261026_v1`은 **외부 종료 신호에 의한
MANUAL_ABORT/ERROR**로 남긴다. 신호를 누가 보냈는지는 미확인이다. SUCCESS로 소급 변경하지 않는다.
KST 01:21:42에 시험이 중단됐고 01:21:47.335에 정지/P를 확인했다.
D는 그 전까지 native16=직진 G였으므로 좌회전 정지 판단이 맞았다. 안정 정차 마지막 3.61초의
30프레임은 current G27 + bounded hold3으로 표준 출력도 G30/30이었다.
실제 R+L33 최초 GET는 01:21:47.847, 최초 원본 CNN은 47.924, 표준 발행은 47.958,
판단 3개 확인·정지 해제는 48.321이었다. **이때는 이미 controller를 내리고 P 정리 중이므로
R+L가 켜져도 차가 출발하지 않는 장면을 신호 인식 실패로 집계하지 않는다.**
별도 `production_D_natural_foggy13_recheck_261026_v1`은 실제 출발·목적지 SUCCESS·정지/P VERIFIED였다.
첫 전체 ERROR·D 재시험·후속 전체 시험을 구분하고 중단 전후 기록을 합쳐 가짜 완주를 만들지 않는다.

Foggy13 전체 재시험 `production_full_natural_foggy13_261026_v2`도 SUCCESS·정지/P VERIFIED였다.
4회 통과 A16/B16/C48(두 head)/D33 모두 전후 0.25/0.5초 native 허가를 확인했고,
route는 `0→A→B→C→D→0`, 지난 미션 재선택은 없었다. matched 603프레임 callback p50/p95는
30.34/44.18ms였다. [독립 검토](../../../artifacts/evaluation/traffic_light_robustness_261025/production_full_natural_foggy13_v2_independent_review_261026_v1.json)는
원본 9개/JPEG 8장의 SHA와 정확한 source stamp를 보존한다.
D 통과 전후 유지 관측 age 0.241/0.370초는 원래 stamp 그대로이며 0.5초 이내였다.
B의 fallback R은 교차 추정 시각과 약 58μs 차이여서 선 전/후의 정밀 인과는 단정하지 않는다.
이 pipeline fallback과 실제 native 허가 상태는 별개다. 로컬 주행 결과만으로 LAN 부하 여유를 보증하지 않는다.
실험을 마친 뒤 API에서 Sunny13 복원 readback을 확인했고 차량은 P·정지 상태로 남겼다.

### 최종 코드·설치 검사

- Python 관련 310개(신호등 167개 포함), 실제 C++ 모듈/독립 oracle/기존 황색 회귀 42개 통과.
- 과거 정책을 기대하던 테스트는 근거 커밋에 맞춰 exact 조합으로 교정했으며 xfail/검사 제거는 하지 않았다.
  통합 launch의 옛 고정 3초 기대값도 현재 인자 전달 계약으로 교정했고 prediction runtime은 바꾸지 않았다.
- 딸깍 설치와 `--check` 모두 완료. 최종 설치 plugin SHA256은
  `9799944cde843f566dc45a5317db0a6adb9c4b353d85008451bc69393bd895f3`이다.
  [설치 receipt 사본](../../../artifacts/evaluation/traffic_light_robustness_261025/production_runtime_install_receipt_261026.json)은
  build/install 일치 근거이며 주행·공식 신호 준수 인증이 아니다.
  테스트 소스 수정 직후의 이전 receipt 불일치는 정상 재빌드로 해소했고 검사를 우회하지 않았다.

### 운영 계약과 남은 범위

정차 중 Y/R+Y 새 출발 금지, 신선한 경로 허가 관측 3개 확인, 원래 관측 stamp 기준 0.5초 만료가 기본이다.
앞축 통과 뒤에는 현재 교차로를 이어가며, 최근 허가→황색에서 실제 제동 불가인 경우만 통과 예외를 둔다.
D의 기존 4초 장기 황색 관측 상한은 유지하며 잔여 황색 예측에 사용하지 않는다.
단절을 GO로 덮지 않으므로 장기 인지/TF/route 상실 시 정차가 계속될 수 있다. 원인을 해결해야 한다.
프로필 이름 `v5_candidate`·가중치 metadata는 호환을 위해 보존하고 모델 재학습/재업로드는 하지 않는다.
합성 정책 시험·로컬 무객체 자연 주행·공식 LAN/BEVFusion 부하 검증을 서로 대체하지 않는다.
