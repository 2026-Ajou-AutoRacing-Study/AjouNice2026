# AjouNice2026 활성 최우선 계획

문서 상태: PRIORITIES. 인지 항목 갱신: 2026-10-15, Asia/Seoul.
이 문서는 다음에 수행할 미완료 작업과 착수 조건만 관리한다. 실행값·실측·명령은
각 CURRENT, EVIDENCE, RUNBOOK을 따른다. 완료된 실험을 날짜별로 다시 쌓지 않는다.
Localization P0는 이번 인지 문서 정리에서 변경하지 않았다.

## 현재 실행 기준의 소유 문서

- Detector·LiDAR: [모델 CURRENT](bevfusion/CURRENT.md#runtime)와
  [E24 production handoff](bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md).
  E25는 복귀 후보이며 운영 모델로 자동 선택하지 않는다.
- Tracker·motion policy·Map-CV: [후단 CURRENT](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md).
- 생성기·수집·GT·resume: [데이터 CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md).
- Localization·제어 입력: [측위 CURRENT](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
- 영역별 진입점: [인지 작업 안내](perception/README.md). 다른 영역의 운영 설정을 이 문서로 덮어쓰지 않는다.

## P0. 대회 측위 종·횡 안정화와 제어 채터링 제거

- 현장점검에서 fusion localization의 steering-rate absolute p95가 약 `1.59 rad/s`,
  GT localization은 약 `0.34 rad/s`였다. 시각화 문제가 아니라 제어 안정성 문제다.
- 정상 구간 횡 채터링은 약 `14.8/18.5 cm`로 양자화된 GNSS update를 Global EKF가
  강하게 따라가고 controller가 Global state를 직접 소비하는 구조가 주원인이다.
- 로컬 한-PC full-stack의 velocity 과적분은 production 속도 bias로 취급하지 않는다.
  현장 bag의 이동거리/속도 적분 비율은 정상이며, 로컬 MORAI 실시간율과 receipt-time
  artifact를 분리해야 한다.
- 경량 GT-object 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump
  p95 약 `1.31 cm`, steering-rate p95 약 `0.54 rad/s`를 유지했다. 이 test bed 확보는
  완료됐고 `sigma=0.05, Qxy=0.05` preset은 기각한다.
- Global/Local state 분리와 Smooth controller 입력은 경량 전체 경로 폐루프 A/B를
  통과해 production에 반영했다. Quantization-aware 후보는 동일 기록 controller
  A/B에서 Smooth보다 우위가 없었고, 폐루프 승격 전이므로 기본 OFF다.
- 공식 START 고정 위치 + 보정된 Competition heading 초기화는 MR 로컬 대조를
  통과했다. 대회 LAN 출발·GPS 음영 복귀·checkpoint 순간이동 검증은 남아 있다.
  공식 START는 그때까지 opt-in이며 개발 기본 초기 위치는 `/Ego_topic`이다.

현재 production 계약과 개선 순서:
[`localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
누적 문제 근거:
[`localization/260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).
최신 폐루프 근거:
[`localization/260920_lightweight_closed_loop_evidence.md`](localization/260920_lightweight_closed_loop_evidence.md).

## P1. 인지의 다음 작업

<a id="perception-priorities"></a>

| 우선 항목 | 착수 조건과 상태 | 종료 기준 |
|---|---|---|
| A 새 팀장님 bag의 단계별 지연 분석 | 필요한 계측을 포함한 새 bag이 도착하면 인지 분석에서 먼저 수행한다 | 센서→검출→후단의 계산·큐·전달 비용, p50/p95/p99·누락을 분리하고 병목 개선 전후 회귀를 비교 |
| B LiDAR 50 팀 배포 마무리 | 로컬 기본값·설치·실제 수신 검증 완료. planning·develop·main 전체 병합 및 최신 ZIP 재동결 반영. 팀 PC 설치·LAN 확인 필요 | 최신 코드·설치본·실제 packet count 일치, LAN 입력·카메라 정합·연속 출력 확인 |
| C 다음 생성기 분포 보완 | 새 bag을 기다리지 않고 소량 실험을 진행할 수 있다 | 근접 장애물·얇은 barrier·희귀 자산과 원거리 차량의 실제 GT·센서 지원·거리/방향 편중을 비교하고 새 release 확정 |
| D 병렬 수집 전달본 준비 | 생성기 분포 확정 뒤 수행한다 | 독립 환경/seed/shard, resume·유한 보충·JPEG 변환·metadata·checksum·업로드를 소량 end-to-end 검증 |
| E 짧은 학습 비교와 최종 학습 | compile OFF·nearest 공통 설정·준비 전용 CPU 경로 완료. 데이터·GT·split 동결 및 GPU 자격 검증·별도 실행 승인 후 학습한다 | 근접 장애물 회귀와 원거리 차량·보행자·복합체를 동일 평가 입력으로 비교하고 checkpoint 백업 확인 |

A는 새 자료 도착 시 우선하는 작업이지 C를 막는 선행 조건이 아니다.
현재 바로 진행할 순서는 B의 팀 PC 설치·LAN 확인과 C의 생성기 소량 검증이다.
각 후보의 상세 실험은 [개선 계획](bevfusion/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)을 따른다.
본수집·학습·임대 인스턴스 조작·원본 삭제는 이 목록만으로 자동 실행하지 않는다.

<a id="perception-latency"></a>

## 새 bag을 받으면 먼저 확인할 지연 분석

기존 팀장님 bag의 검출 header→bag 수신 age 약 237ms는 전체 구간의 관측값이다.
원시 센서와 단계 timing이 부족하므로 이를 GPU 추론시간이나 CPU 과부하로 단정하지 않는다.
기존 bag의 발행 주기·공백·age 및 후단 재생 비용 분석은 활용하고, 당시 풀스택 부하의
원인 분해와 개별 함수 재생의 계산 비용을 구분한다.

새 bag 수령 시 다음 기록의 실제 존재와 동일 frame 연결부터 검사한다.

- 짧은 동시 raw 구간: `/sensor/lidar1/packets`, `/sensor/lidar1`, 네 카메라 compressed 입력,
  Ego·Competition·Object 상태와 필요한 TF. 서로 다른 녹화의 평균을 이어 붙이지 않는다.
- 실행한 detector namespace의 `frame_timing`·`status`·`latency_ms`.
  Production은 `/perception/bevfusion/...`, trial은 `/perception/base5_trial/...`다.
  토픽을 요청한 것과 실제 메시지를 저장한 것을 구분한다.
- detection·tracking measurement·tracking·prediction, Map-CV와 motion policy의 처리 진단.
  Tracker 30Hz 출력 stamp와 마지막 실제 검출 관측 시각을 구분한다.
- 공유 묶음의 노드 로그·모델/소스/설치본·resolved config·센서 설정과 가능한 CPU/GPU 부하 기록.
  Planner processing time과 실제 소비 경로 기록은 판단 팀과 함께 대조한다.

LiDAR header→callback, 입력 선택·raw/prepared queue 대기, 전처리, TCP/server,
검출 발행, 후단 callback 계산·발행·전달을 연결 가능한 범위에서 분리한다.
ROS source stamp와 monotonic/host clock의 기준을 확인하고 서로 다른 clock을 그대로 빼지 않는다.
없는 단계는 미측정으로 남긴다. 계측 ON/OFF 비용과 출력 동등성도 고려한다.

준비된 기능·토픽과 측정 범위는
[개별 노드 진단](bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#diagnostics),
[보조 recorder](../scripts/rosbag/record/rosbag_for_perception_latency_eval.sh),
[validator 런북](planning/SCENARIO_VALIDATOR_RUNBOOK.md#5-결과-확인공유)을 따른다.
새 녹화 요청과 GUI/공유 설정 변경은 해당 담당자와 조율한다.

## C++ 포팅과 보류 후보

전면 C++ 포팅은 현재 최우선이 아니다. LiDAR driver·pointcloud·EKF tracker·주요 planner는
이미 C++이고 모델의 무거운 연산은 C++/CUDA다. Map-CV 등의 출력 동일 CPU 최적화도 반영했다.
언어와 무관하게 큰 계산·큐 대기·직렬화·스케줄링·p95/p99 병목부터 개선한다.
포팅은 측정한 병목에 한정해 출력 동등성·수치·ID·stamp·폐루프 회귀를 확인한다.
[노드별 포팅 판단](bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#porting)을 따른다.

초기 차량 방향 가드·재취득 공분산/reset·낮은 score 갱신·Map-CV 순위 변경은
운영 미적용 후보다. 새 bag에 관련 실패가 있으면 별도로 재검토하되 생성기의 필수 선행 조건으로 삼지 않는다.
추론 compile 기본 OFF, 현재 bounded coasting과 planner 정책은 유지한다.
Temporal/OD velocity 연구, 전역 기억 확대, 사후 head peak 변경은 자동 재개하지 않는다.
기존 5K·native GT·가중치는 보존하며 센서에 보이는 원거리 GT를 임의로 삭제하지 않는다.

## 기록과 갱신 규칙

완료·기각 항목은 이 목록에서 제거하고 해당 CURRENT/EVIDENCE의 링크로 보존한다.
새 bag 분석, 생성기 검증, 운영 승격은 서로 다른 완료 상태로 기록한다.
이번 정리 이전의 날짜별 계획은
[2026-10-14 정리 전 원문](perception/legacy/active_priorities_261014_before_refresh.md)에 보존했다.
