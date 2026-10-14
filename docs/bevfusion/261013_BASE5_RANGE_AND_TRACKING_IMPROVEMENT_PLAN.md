# Base5 원거리 차량과 근거리 객체 개선 계획

문서 상태: PLAN. 기준일: 2026-10-14. 2026-10-13 계획을 최신 로컬 근거로 갱신했다.
현재 실행값은 [모델 CURRENT](CURRENT.md), 배치 release는
[데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md), 후단 실행값은
[Tracking CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)가 관리한다.
이 계획은 E24 풀스택 검증, 원거리 차량 전달 안정화, 다음 데이터 수집과 최종 학습을 연결한다.
검증 후보를 production에 자동 적용하지 않으며 각 단계의 결과에 따라 계획을 수정한다.

## 진행 순서와 현재 상태

| 단계 | 담당과 현재 상태 | 다음 단계로 넘어가는 기준 |
|---|---|---|
| 1 새 bag의 E24 단계별 지연 | 최신 완주·무충돌 확인과 기존 bag 분석 완료. 당시 237ms의 단계 분해는 계측 부족 | 새 계측 bag 도착 시 우선 분석. 계산·큐·전달·tail·drop을 분리하고 실제 consumed UUID/path 대조 |
| 2 저장 검출의 원거리 score 분석 | 로컬 CPU 비교 완료 | 동일 입력의 거리별 누락·미대응·위치 오차를 구분하고 후단 후보 선정 |
| 3 낮은 score로 기존 차량 track 갱신 | 독립 실제 EKF 코어 원형과 12프로필 비교 완료. production OFF | 이동·클래스 안정·XY 갱신 후보의 실제 ROS 및 새 초기 접근 구간 검증 |
| 4 지도 보조와 예측 경로 안정화 | 순위 후보는 보류. 출력 동일 CPU 최적화·보행자 Gaussian 순위·E24 중심 보완 반영 | 새 bag의 live tail/age/drop·근접 2인·복합체와 실제 planner 소비를 확인 |
| 5 다음 생성 분포 조정 | 후보 방향 정리. v150은 그대로 유지 | primary·asset·거리/방향·split 변경을 명시하고 CPU 감사와 소량 캡처 검증 |
| 6 다른 PC의 병렬 수집 | 전달 실행기 미구현 | 독립 배치·resume·유한 보충·JPEG 전달본·업로드 검증의 한 묶음 시험 |
| 7 짧은 학습 비교와 최종 학습 | 미실행 | 입력·GT·평가 split을 고정하고 후보 효과와 근접 회귀를 확인 |

실제 작업 우선순위는 [활성 목록](../ACTIVE_PRIORITIES.md#perception-priorities)을 따른다.
새 bag 분석에 필요한 입력은 [단계별 지연 확인 목록](../ACTIVE_PRIORITIES.md#perception-latency)에 둔다.
LiDAR 50은 사용자 승인으로 운영 기본값·로컬 설치·실제 수신 검증을 완료했고 팀 planning 통합은 남아 있다.
새 bag을 기다리는 동안 생성기 보완·소량 검증을 진행할 수 있다. 실패 원인을 확증하기 전에
후단과 planner를 동시에 바꾸지 않는다. 본수집·학습·임대 인스턴스 조작·원본 삭제는
별도 실행 요청과 해당 release 계약을 따른다.

2026-10-14 [센서·추적·모델·CPU 검토](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md)에서
완료 5K/native GT를 대조했다. Query peak3/5 사후 전환은 현재 E24의 장애물 recall 회귀로
기각했다. 후속 검증을 거친 보행자 Gaussian association 순위는 운영에 반영했다.
20~30m 장애물은 camera 지원이 있는데 LiDAR 형상이 희소한 경우를 별도로 보강한다.
전방 근접·얇은 barrier·희귀 passive OBJ support를 후보로 삼되 v150/기존 5K는 보존한다.
같은 검토에서 output bottom-center→ROS geometric-center 계약 문제도 확인했다.
후속 실제 ROS 검증 뒤 E24는 tagged bottom→gravity adapter를 사용한다. 기존 tracker·planner는
fresh stack으로 시작한다. raw export·학습 GT 변경이나 재수집으로 해결할 문제가 아니다.
nearest depth hint의 연속 A/A는 통과했지만 compile의 association·coasting 예외는 남아
추론 compile OFF를 유지한다. 새 학습 전에 이 출력 계약과 평가 입력 조건을 먼저 고정한다.

## 유지할 정책과 보류할 제안

- E24 production의 클래스·threshold·NMS·추론 옵션과 현재 bounded coasting을 유지한다.
- 완료 Sunny13 5K와 원래 native 의미·component 관계·가중치·학습 source는 보존한다.
- 50m 밖 객체의 클래스를 모두 Vehicle로 바꾸지 않는다. 저신뢰도 검출도 원래 클래스와 score를 보존한다.
- 센서에 남아 있는 원거리 사람·장애물의 GT만 지우지 않는다. 학습에서 무시하려면 heatmap·query loss 등을 별도로 검증해야 한다.
- 객체를 한 번 봤다는 이유만으로 확정하거나 기억 시간을 두 배로 늘리지 않는다.
- AP·detector score·track confidence·경로 confidence를 독립 검출 확률로 해석하지 않는다.
- 고주로·로터리 차량 전용 배치와 음영 구간의 별도 예외를 유지한다.

50m 이상 사람·장애물은 기존 train GT의 약 0.98%다. 이들이 원거리 차량 학습을 크게
방해했다고 확정하지 않는다. 공유 feature의 학습 영향과 새 데이터 수집 예산의 우선순위는
별개로 판단한다. 기존 노출과 첫 score 비교의 수치·해시는
[47절 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-range-tracking-pivot-261013)에 둔다.

## 원거리 score 분석

저장된 E24 AMP·병렬화 ON·compile OFF 출력에 같은 서버 floor 0.1/top100과
class-aware rotated NMS 0.5를 적용한다. 동일한 1,570개 입력 중 정확도 집계에는
기존의 full-ROI GT 적격 1,413장을 사용한다. 새 추론이나 새 GT를 만들지 않는다.

첫 비교는 Vehicle 0.40/0.35/0.30/0.20/0.10이다. Pedestrian과 Obstacle은 0.10으로 유지한다.
ROI 전체에서 먼저 대응한 뒤 GT 거리로 recall, 예측 거리로 미대응 수를 집계한다.
거리 경계를 넘는 정상 대응을 FP로 잘못 세지 않는다. 0~30m 회귀와 30~50/50~80/80~100m를 함께 본다.

각 case에서 원래 LiDAR index·GT 제외·0.25초 초과 시간 공백을 기준으로 누락 구간을 나눈다.
catalog에 없는 배경, 다른 클래스 주변 후보, 같은 객체의 중복/위치 오차를 분리한다.
미대응 후보가 실제 오검출인지는 대표 이미지·point·연속 이력을 확인한 뒤 판정한다.
현재 score를 낮추는 것만으로 80~100m 검출이 안정화된다는 근거는 없다.

재현 명령은 저장된 개발용 입력에만 적용한다. 출력 경로는 매번 새 이름을 사용한다.

```bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 \
  perception/detection3d/bevfusion/tools/analyze_base5_far_vehicle_scores.py \
  --inputs artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1 \
  --predictions artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1/predictions_0.json.gz \
  --reference-report artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/report_validated_v6.json \
  --output artifacts/evaluation/bevfusion/base5_range_tracking_improvement_261013/score_sweep_v2.json
```

## 기존 track 갱신과 지도 보조

우선 후보는 높은 score와 반복 관측으로 새 track을 확인하고, 낮은 score 후보는 이미 확인된
차량과 association될 때만 갱신에 활용하는 방식이다. Pedestrian/Obstacle, duplicate-birth guard,
UNKNOWN stationary policy와 coasting은 첫 A/B에서 바꾸지 않는다. GT는 평가에만 사용한다.

현재 남긴 독립 시험 후보는 `weak010_moving_class090_xy`다. 30m 이상에서 0.10~0.40
Vehicle 후보를 기존 confirmed 이동 차량에만 연결한다. 높은 관측을 먼저 대응하고, 거리 2m·
pi 대칭 yaw 60도·길이/너비 비율 2 이내를 요구한다. 클래스 평활 score 0.90과 현재 정지
판정 경계인 world speed 2m/s 초과를 보조 조건으로 사용한다. 두 값은 검증 후보이며 실제 확률이나 정답 판정이 아니다.
정지·저속 차량은 삭제하지 않고 원래 정책을 유지한다. 약한 관측으로 새 ID를 만들지 않는다.

약한 관측은 XY 정보만 보충하고 크기·Z·클래스 score를 유지한다. 마지막 높은 Vehicle
관측 이후 0.5초까지만 허용한다. 이것은 원래 삭제 타이머와 다른 제한이며, 약한 관측이
last measurement를 갱신하므로 총 유지 시간이 늘 수 있다. 실제 잔류 episode를 함께 평가한다.
일반 저신뢰도 갱신은 장애물로 클래스가 바뀐 뒤 최대 27cm의 위치 차이가 남았고,
클래스 안정도만으로는 사라지지 않았다. 이동 조건 추가 뒤 이번 네 bag의 confirmed
비차량 출력 상태는 baseline과 같았다. [48절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-low-score-core-probe-261013)을 따른다.

이 원형은 별도 생성·컴파일한 코어이며 설치된 tracker/node/launch나 하위 submodule을 바꾸지 않았다.
기존 동일 bag에서 확인된 이득은 대부분 이미 보던 차량이 멀어질 때의 갱신이다.
처음부터 80~100m에서 접근하는 차량의 bootstrap 효과나 실제 ROS 안전성을 인증하지 않는다.

후보 갱신에는 위치·크기·시간·motion 일관성을 확인한다. 낮은 score를 confidence가 높은
측정처럼 취급하거나 갱신되지 않은 예측을 새 센서 관측처럼 표시하지 않는다. 새 낮은 후보를
여러 개 받아 기존 서로 다른 차량을 합치는 회귀도 검사한다. 검출 recall 증가만으로 채택하지 않는다.

필수 보고 항목은 거리별 1m/2m recall, 누락 구간, ID 변화, 미대응·중복·잔류 episode,
중심/BBox 경계/속도 오차의 평균·p50·p90·p95와 대표 실패다. 실제 ROS replay와
팀장님 주행의 planner 반응을 별도로 확인한다. 현재 추적기에 바로 이식할 완성 알고리즘은 아니다.
[ByteTrackV2](https://arxiv.org/abs/2303.15334)는 낮은 score 후보의 계층적 association 참고다.

지도는 차량 후보의 신뢰도를 보조하는 opt-in 후보로 시작한다. 도로 polygon과 BBox,
방향의 모호성, 합류·차선 변경, 지도/localization 불확실성을 고려한다. centerline 거리 하나로
삭제하지 않으며 관측 시각의 좌표 변환을 사용한다. 지도 매칭 실패 시 기존 경로로 fallback한다.
현재 Map-CV는 검출 삭제 필터가 아니며 매칭 실패를 Cartesian CV로 처리한다.

## 예측 경로와 planner 검사

현재 저장소의 Lane Change는 `use_all_predicted_paths=false`이지만 최고 confidence가
정확히 동점인 경로는 모두 남긴다. Obstacle Cruise는 inside 검사에서 하나, outside에서
설정 개수를 선택하며 통합 설정의 outside 값도 1이다. 로터리 전용 모듈은 confidence
0.05 이상 경로를 순회하며 위험 충돌 시 중단한다. RViz에 Top-2가 보인다는 사실만으로
차선 변경이 그 경로까지 사용했다고 판단하지 않는다. 실제 팀장님 실행 parameter와
소비 module을 확인하기 전에는 Top-K 때문에 추월이 막혔다고 확정하지 않는다.

먼저 발행 path 수와 각 confidence, planner가 고른 path, 현재 차선 및 미래 경로와 Ego 후보
경로의 충돌을 기록한다. 동점, 잘못된 Top-1, lane association 흔들림, 속도/BBox 오차와
관측 age, planner 안전거리 조건을 구분한다. 단순 Top-1으로 바꾸고 해결됐다고 결론내지 않는다.

2026-10-14 [49절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014)의
고주로/로터리 E24 상태·현재 지도 비교에서, 3초 Top-1/minFDE-K 평균은 각각 2.33/2.30m와
4.95/2.44m였다. 정답에 현재 대응하고 미래 GT가 지원되는 조건부 표본이며 minK는 미래를
사용한 평가 상한이다. 따라서 고주로 후보 전체 삭제의 이득은 미확인이고 로터리의 실제 분기는 유지한다.
로터리 시작 lanelet의 일반 successor가 아닌 변화는 이력 진단에서 39→15회였지만,
그 진단만으로 실제 Top-1 정확도 향상으로 해석하지 않는다. 별도 연구 복사본에서 이력
시작 차선의 score를 1.25배로 반영하자 R01 3초 FDE 평균/p95는 4.953/17.106→
5.079/18.419m로 악화했다. 단순 이력 유지의 production 적용은 기각한다.

같은 날 [50절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-cv-candidates-261014)의
후속 후보를 먼저 동결하고 H03/R01에서 비교한 뒤 9/15 세 주행과 9/17 주행에 재적용했다.
관측 적합도 순위는 일부 3초 평균을 약 2~4.5% 줄였지만 소수 잘못된 분기 승격의 큰
새 오차가 남았다. 전체 평균·p95 통과만으로 production에 적용하지 않는다. 회귀 결과를
보고 후보 수치를 다시 맞추지 않았다. 횡방향/중심선 복귀 후보도 기각 또는 보류했다.
원래 예측 정책을 보존하는 arc index·고정 geometry cache·독립 메시지 복사·DFS 계산은
정확한 출력 비교 후 소스와 로컬 설치본에 반영했다. 30Hz private ROS 600개 입력의
진단 OFF/ON wire·전달 수가 같았으며 팀 배포와 실제 주행 tail은 별도다.

후속은 실제 관측의 경로 적합성과 lateral motion을 사용하는 이력이다. 단순 Top-1
hysteresis를 해결책으로 고정하지 않는다. 구역별 정책이 필요하면 Ego geofence
하나가 아니라 대상 차량의 도로 구간·진입/탈출·이력도 사용한다. 로터리의 실제 분기 후보를
임의로 삭제하거나 남긴 경로의 confidence를 자동으로 1.0으로 만들지 않는다.
이력은 관측 gap·클래스 변경·새 ID·지면/좌표 미정합에서 초기화하고, 현재 거리/yaw gate를
벗어난 이전 lanelet을 강제로 복원하지 않는다. 명확한 차선 변경은 새 후보로 전환하며
실제 분기의 확률은 시작 차선 안정화와 별도로 다룬다. 지도 위라는 사실로 새 차량을 확정하지 않는다.
[Autoware map prediction](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_map_based_prediction/)의
과거 lanelet 연결성·maneuver 이력은 참고이며 현재 커스텀 Map-CV에 구현됐다는 뜻은 아니다.

다음 replay에는 높은 관측 이후 age와 모든 실제 관측 이후 age, 약한 association 여부와
측정/추적 confidence를 구분해 기록한다. 낮은 갱신을 무관측 coasting이나 확실한 새 검출로 표시하지 않는다.
새 초기 접근 검증에는 앞/뒤 80~100m에서 30m로 접근하는 차량 구간이 필요하다.
0.40으로 걸러진 detection만 저장하면 낮은 후보를 복원할 수 없으므로 짧은 raw 센서 구간이나
별도 이름으로 저장한 0.10 후보가 필요하다. canonical planner 입력을 몰래 낮춰 녹화하지 않는다.

## 다음 데이터 수집과 학습

새 후보는 장애물·보행자를 주로 0~40m에 배치하고, 40~50m에서 선행 회피에 유용하고
센서 지원이 있는 사례를 소량 유지하는 방향이다. 50m 이상 작은 비차량의 의도적 quota를
줄이고 차량의 50~80m 전후방·인접 차선·부분 가림과 위치/형상 사례를 우선 보강한다.
80~100m 차량은 어려운 사례로 유지하되 안정적인 실전 인지 범위로 인증하지 않는다.

이 방향은 v150의 실행 정책 변경이 아니다. 다음 manifest에서 바뀐 primary/secondary,
family·asset·거리·방향·scene/group split을 명시한다. 보강 전후 실제 GT와 센서 지원을
확인하며 희귀 자산·초근접 통과·음영 예외·배경 negative를 잃지 않는다. 새로운 Sunny13는
기존 30K에 자동 합산하거나 기존 5K를 자동 교체하지 않는다.

기존 근거리 보강은 생성기와 소량 native 검증까지 완료한 상태다. 새 5K 본수집·재학습은
완료하지 않았다. 다음 수집은 [보강 런북](../../dataset/morai_od/docs/261013_base5_obstacle_support_runbook.md)과
새 release의 검증 경계를 따른다. 최종 학습에서 근접 장애물 AP·경계 정밀도의 회귀를 반드시 확인한다.

같은 초기화·입력·평가 split을 사용한 짧은 비교로 주요 회귀를 먼저 거른다. 1~2 epoch는
조기 문제 검사용이며 최종 성능 증명이 아니다. 최종 학습 기회가 적으므로 공간 격자·depth·
이미지와 후단을 한 번에 바꾸지 않는다. 개발용 bag이나 Base3 val을 train에 넣지 않는다.

## 병렬 수집 전달본

PC별 독립 seed/shard와 환경을 할당하고 수집 source·release·센서·환경을 기록한다.
같은 geometry의 파생 장면은 물리 group split을 공유해 train/val 누출을 방지한다.
다른 PC에서도 부족분 보충 상한과 성공분의 중복 없는 resume를 유지한다.

완료 묶음 검증 → RGB Q90 JPEG 변환 → GT/보정/metadata를 포함한 상대 경로 전달본 확정
→ Drive 업로드와 원격 checksum 검증 → 허용된 RGB PNG만 정리하는 순서다.
Instance·depth·annotation·LiDAR는 JPEG로 바꾸거나 삭제하지 않는다. Collector의 원본
해시를 유지하는 resume와 JPEG 전달본의 완료 receipt를 구분한다. 업로드·변환 실패 시 원본을 보존한다.
현재는 이 자동 전달·삭제 경로를 구현한 상태가 아니다. 공용 PC의 무인/원격 사용은 관리자의 허용을 전제로 한다.

## 갱신과 종료 기준

각 후보는 baseline → 한 요인 변경 → 회귀/효과 비교 → 유지·수정·기각으로 기록한다.
수치·실패·hash는 EVIDENCE에, 실행값은 각 CURRENT에, 단계와 다음 선택은 이 계획에 둔다.
팀장님 검증 결과가 들어오면 새 bag을 기존 개발용 분석과 구분하고 해당 단계 상태를 갱신한다.
Production 채택은 실제 입력·후단·planner 회귀와 사용자 선택을 확인한 뒤 수행한다.
