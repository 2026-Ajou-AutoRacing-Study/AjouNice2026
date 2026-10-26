# 신호 구간 움찔: 실제 입력 재생과 로컬 주행 보완

2026-10-26 작업 근거. 현재 운영 기본값은 [CURRENT](CURRENT.md)가 관리한다.
이 문서는 특정 소스·입력·설정에서 확인한 사실이며, 이후 실험이나 기본값 승격을 자동으로 뜻하지 않는다.
이전 [저확신 완충·원본 판정선 기록](261026_SIGNAL_SMOOTHING_AND_ORIGINAL_JUDGING_LINE.md)의
단위 시험과 표본 분석에서 멈추지 않고, **연속 실제 입력과 실제 MORAI 폐루프 주행**을 추가 검증했다.

## 먼저 구분한 세 가지 사실

1. 팀장님 bag에는 실제 반복 재제동이 있었다. 지도 v87은 확인했으나 당시 모델·실행 설정은 미상이다.
2. 현재 모델로 같은 원본 입력을 연속 처리해도 지도 불확실성 거부로 신호 공급이 끊겼다.
   CNN 저확신 완충만으로 해결하지 못하는 실제 구현 경로였다.
3. 최신 v94 지도와 실제 planner/controller를 사용한 새 로컬 A 주행에서도 재출발 직후 재제동을 확인했다.
   따라서 구버전 bag의 문제라고만 넘기지 않고 새로운 후보를 구현·재생·실주행했다.

## 팀장님 v87 bag에서 확인한 현상

대상은 `rosbags/traffic_light/261026_teamlead_154234/planning_rviz_20261026_154234.bag`다.
녹화된 vector map의 버전은87이며, 전방 원본 JPEG·측위·TF·계획·제어가 있다.
traffic 결과/debug·모델 SHA·실행 설정 사본은 없어 당시 신호 출력 전체를 복원하지는 못한다.

- A/C/D 재출발 근방에서 명확한 저속 재감속8개를 확인했다. 전체 주행의 brake burst36개를
  모두 신호 인식 오류로 분류한 것은 아니다.
- B80.48초와 후반160.56초에는 path optimizer 실패 후 이전 경로로 제동하는 별도 로그가 있었다.
- 늦은 B/D 접근에서는 원본 신호 housing이 전방 화각 위로 잘리는 실제 사진도 확인했다.
  이 정상적인 시야 한계를 오래된 녹색 유지로 숨기지 않는다.
- 당시 실행 인자를 모르므로 새 후보를 당시 모델의 정확한 재현이라고 부르지 않는다.

정의·시각·원본은 `planner_logs/analysis/261026_154234_signal_hesitation/REPORT.md`와
`requested_bag_report_v1.json`에 보존했다. 이8개 실제 재감속과 아래 관측 Guard 전환 수는 다른 지표다.

## 원인과 보완: 옛 색을 유지하지 않고 현재 색을 계속 읽기

기존 CNN 저확신 완충은 현재 지도 선택이 유효한 `low_confidence_bits`에만 적용된다.
실제 문제 프레임에는 YOLO 박스가 있어도 지도 ROI가 탈락해 **CNN crop 자체가 없었다.**

새 `visual_continuation_sec` 후보는 최근 정상 지도 연결의 identity를 짧게 이용하되,
**현재 YOLO 박스의 현재 crop을 CNN으로 다시 분류하고 현재 원본 시각으로 발행한다.**

- 기본 정상 지도 경로를 우선한다. global/per-head uncertainty 거부에만 후보를 시도한다.
- 이미지 시각의 TF·측위·외부 보정·현재 미션·nominal housing의 전체 화각이 유효해야 한다.
- 실제 covariance는 그대로 보존하며, 정상 ROI의 X90/Y200 및3도 기준을 작게 조작하지 않는다.
  추가 identity 경로에도 실제 XY/Z sigma1.5m·yaw sigma5도의 별도 상한을 둔다.
- 최근 **정상 map-match**에서 최대0.3초, 해당 원본 source age 최대0.5초 이중 기한이다.
  visual fallback이 정상 anchor의 기한을 갱신하지 않는다.
- 같은 physical head의 이전 검출과 높은 겹침·제한된 중심 이동·크기비·유일한 검출·다른 head 경쟁 배제를
  확인한다. 실제 픽셀 근거 없이 head를 바꾸거나 색상으로 허가 head를 고르지 않는다.
- detector 경계 흔들림은 identity IoU 계산에만 최대2px 및 짧은 변10% 이내로 허용한다.
  원본 detector 좌표와 실제 CNN crop은 변경하지 않는다.
- 실제 R/Y는 즉시 현재 관측으로 전달한다. 현재 CNN UNKNOWN은 그대로 UNKNOWN이다.
  TF/route/화각 실패·다른 head 모호성·기한 만료는 오래된 녹색으로 덮지 않는다.

이는 불확실성이 큰 지도 ROI를 정상인 척 복원하는 기능이나 허가 신호를 무기한 기억하는 기능이 아니다.
기본값 승격 여부는 CURRENT와 실제 launch/설치본으로 확인한다.

## 같은 원본 입력·같은 GPU의 연속 비교

A70~77초, C100~109초, D130~142초의 전방 원본635개에 실제 ROI/inference callback을 적용했다.
10Hz 상한·이전 head·캐시·원본 source stamp를 연속 유지했다. 같은 고정 v5 YOLO/CNN·GPU에서
visual OFF/ON을 비교했으며, 과거 모델을 추측해 대체하지 않았다.

| 구간 | 미션 유효 출력 | visual OFF UNKNOWN | 후보 UNKNOWN | 추가 현재 crop |
|---|---:|---:|---:|---:|
| A |48|19|7|12|
| C |66|13|0|13|
| D |95|44|27|17|

추가42개의 현재 crop·직전 정상 crop·원본 전체 화면을 모두 직접 대조했다. 관찰상 잘못된 head나
이미지와 모순되는 램프는 보이지 않았으나 독립 인간/native GT 정확도 인증은 아니다.
42개 모두 현재 JPEG의 원본 시각으로 발행됐고, 과거 램프 hold는0개였다.
이중 A5개의 적색과 D4개의 황색도 현재 관측으로 전달됐다. A71.825초에는 이전 crop은 적색,
현재 crop은 녹색이었고 후보는 현재 녹색을 즉시 보냈다.

실제 생산 C++ `Guard`에도 위 연속 관측을 공급했다.

| 구간 | GO→STOP, OFF→후보 | STOP 요청 출력 수, OFF→후보 |
|---|---:|---:|
| A |3→2|38→25 / 48|
| C |4→0|44→17 / 66|
| D |6→4|81→55 / 95|

이것은 **정지선 전 관측 허가 proxy**이며 GO_OUT·경로 속도 삽입·제어·실제 제동을 재생한 값은 아니다.
잔여 A/D 전환을 실제 불필요 제동 횟수라고 단정하지 않는다. 일부 화각 이탈·투영 거부는 확인했지만
잔여 전환 전체의 실제 제동 인과 분류는 미완료다. 실제 측위 앞축과 로컬v87 선을 대조하면 A 두 전환은 이미 앞축 통과 후에 가깝고, D 네 전환 중 하나는 현재 CNN 적색이다. 당시 guard 설정·기록된 지도 binary 좌표까지 확정한 것은 아니다(`residual_timing_v1.json`). D 끝 구간까지0으로 해결했다고 보고하지 않는다.
원본 bag에는 실제 traffic 출력/native 현시가 없고, offline TF는 미리 적재했으므로 그 한계도 유지한다.

근거는 `planner_logs/analysis/261026_154234_signal_continuous_visual_candidate_v1/`의
`REVIEW.md`, `inference_gpu_v2/replay.json`, `actual_guard_v2/summary.json`, `visual_review_v2/`다.

## 실제 MORAI 폐루프 주행은 별도로 확인

Sunny13·v94·무객체 OD_GT·실제 localization/ROS1 planning/controller·황색 OFF 조건이다.
목표 도착 PASS만으로 움찔이 해결됐다고 세지 않고, 실제 정차 종료부터 원본 MGeo 앞차축 통과까지의
brake·속도·계획속도·C++ STOP trace를 함께 대조했다. 제동 기준은 `brake>0.05 & speed>0.3m/s`다.

| 실행 | 신호 앞 실제 정차 | 재출발→원본선 | 그 구간 재제동 | 허가 후 STOP 재발 |
|---|---:|---:|---:|---:|
| A 기존 |46.380초|1.890초|1회|1회|
| A visual 후보 자연 반복 |47.520초|1.249초|0회|0회|
| B 기존 |48.000초|2.679초|0회|0회|
| C 기존 |78.020초|1.183초|0회|0회|
| C visual 후보 자연 반복 |77.780초|1.172초|0회|0회|
| D 기존 자연 실행 |정차 없음|N/A|N/A|N/A|

기존 A에서는 padding97.529px가 수평 상한90px를 넘어 현재 A077 ROI가 사라졌고,
허가→UNKNOWN/STOP→허가와 함께 실제 brake0.400, 속도0.629→0.316m/s, 계획속도0.0이 발생했다.
후보 A는 실제 재출발부터 원본선까지 재제동·STOP 재발이 없었고, 출발0.3초 후 계획속도 최저1.108m/s였다.
두 A 실행은 모두 목표 도착·최종 정지/P를 확인했다.

**이 A 비교는 같은 입력의 통제 A/B 시험이 아니라 자연 신호의 반복 주행이다.** 후보 실행에서 예전과
정확히 같은 이동 중 GREEN 지도 dropout이 재현되지는 않았다. 실제 사용된 visual4개는 정차 RED 구간이었다.
따라서 반복 주행의 관찰 결과1→0은 보고하되, 동일한 원인 사건을 실주행에서 수정했다고 확정하지 않는다.
B/C의 기존0회는 회귀 기준이며 새 후보의 개선율이 아니다. D 무정차 실행을 정차·재출발 통과로 세지 않는다.

실주행 근거는 `planner_logs/analysis/local_signal_smooth_v94_261026/summary_ABCD_v2.md`와
`summary_A_visual_comparison_v1.md`다. 구간·신호 위상·자료 범위가 다른 실행을 합산하지 않는다.

## 별도 planning 보완: MPT 탐색 cursor

신호와 별개인 optimizer 제동도 놓치지 않기 위해 root가 MPT 탐색 cursor 후보를 수정·빌드·설치했다.
새로 검증된 incumbent가 없는데도 이전 seed 방향으로 탐색을 되돌리면 매 cycle 같은 후보를 반복할 수 있어,
이전 seed 선호를 **이번에 다시 검증한 incumbent가 있을 때만** 적용하도록 바꿨다.
모의 scheduler/cursor 회귀7개가 통과했지만, 실제 QP 수렴·장애물 회피·80.48초 또는160.56초의
실제 원인이 해결됐다고 인증한 것은 아니다. 모든 신호 재제동을 이 수정의 효과로 합산하지 않는다.

근거: `scripts/evaluation/audit_mpt_search_cursor.py`, `test_audit_mpt_search_cursor.py`,
`planner_logs/analysis/261026_154234_signal_hesitation/cursor_counterexample_v2_after_candidate_patch.json`.

## 추가 시험 진행 상태

- **후보 C 실제 반복 주행:** PASS·최종 P 확인. 원본선 전 UNKNOWN·재제동·STOP 재발0, 출발0.3초 뒤 계획속도 최저1.092m/s였다. 현재 녹색+좌화살표 visual 판독1개는 원본선 뒤0.930초이므로 선 전 개선의 인과 증거로 세지 않는다.
- **A 이동 중 uncertainty 통제 stress:** OFF/ON 모두 yaw sigma3.2도를 신호등용 평가 odometry에만0.2초 부여했다. 실제 pose·TF·planner 입력은 보존했다. ON 첫 실행은 Ego 입력1.249초 단절로 안전 중단·P 확인, pulse0회라 비교에서 제외했다. ON 재시험은 완주·P 확인했고 아래와 같이 보완 효과를 확인했다.
- **D 후보 자연 주행:** 완주 PASS·native R+좌화살표 허가·최종 P 확인. 이번도 정차 없이 통과했으므로 정차 후 재출발은 N/A다. 약한 양의 목표속도 감속과 목표 정차를 신호 재제동으로 세지 않는다.
- **반영:** 독립 검토·로컬 실주행 이후 두 v5 launch 기본을 `visual_continuation_sec=0.3`으로 적용했다. raw Python 기본0과 명시적 `:=0` 복귀는 유지한다. 다른 PC의 기존 설치본이 자동 변경된 것은 아니며 [팀 업데이트](../../runbooks/261026_TRAFFIC_V94_TEAM_UPDATE.md)로 설치한다.

| A stress 동일 정의 | OFF | ON |
|---|---:|---:|
| pulse 직후1초 UNKNOWN |3프레임|0프레임|
| pulse 직후1초 STOP episode·실제 재제동 |각1회|각0회|
| 실제 재출발→원본선 전체 재제동 |2회|0회|
| 같은 전체 구간 계획속도 최저(출발0.3초 뒤) |0.000m/s|1.294m/s|
| 원본선 native 허가·종료 P |확인|확인|

ON은 pulse 중 현재 GREEN crop2개, 후속 자연 ROI gap에서1개를 새로 읽었고 세 출력 모두 현재 JPEG 시각·held=false였다. 적색 정차 중 새 RED도34개 전달했다. 서로 다른 자연 주행의 동일 pulse 사양 시험이지 byte-identical 영상 A/B는 아니다. 원본선 직전 UNKNOWN1개는 map GO_OUT 이후여서 재제동이 없었으며, 모든 시야 실패를 해결한 것은 아니다.
상세: `planner_logs/analysis/local_signal_smooth_v94_261026/summary_A_covariance_stress_comparison_v1.md`·`summary_C_visual_comparison_v1.md`·`summary_D_visual_v1.md`.

현재까지 인지316개 및 평가 도구를 합친350개 회귀, actual bag 연속 재생, 추가42개 전수 시각 검토와 실제 로컬 주행을 수행했다.
독립 코드 검토의 낮은 위험 보강 두 가지도 반영했고, GPU 저장 검출에 최종 helper를 재공급해
42개 head/crop 선택이 동일함을 확인했다. 검토 및 최종 source SHA는 같은 분석 폴더의
`independent_review.md`와 `hardening_saved_predictions_v1/report.json`을 본다.
새 학습은 필요하지 않으며, 새 runtime helper가 있으므로 팀 배포에는 기존 설치기의 source/install 확인과
스택 재시작이 필요하다. 이 기록은 새 LAN·대회 4090 풀스택 또는 모든 날씨의 무움찔 인증은 아니다.

## 추가 수령한 실제 v94 bag 대조 (2026-10-27 정리)

`planning_rviz_20261026_222146.bag`은 7,692,098,685바이트이며 Drive MD5와 일치한다.
녹화된 vector map의 실제 버전도94다. 원본은
`rosbags/traffic_light/261026_teamlead_v94_222146/`에 보존했다.
구형 July bag16개만 정리해17,189,191,330바이트를 확보했고 새 bag·기존 신호 비교 bag은 보존했다.

| 구간 | 이번 bag에서 실제 확인한 행동 |
|---|---|
| A | 정차 후 한 번 출발했고 이후 ZERO 목표로 돌아오지 않았다. |
| B | 출발→정지 목표 복귀5회. 그중 이동 중 명확한 재제동3회, 나머지2회는0.3m/s 미만 출발 취소다. |
| C | 신호 앞 정차·재출발이 아닌 연속 감속이며, 직후 별도 급제동에는 run_out/crosswalk 증거가 있다. |
| D | 정차 없이 통과했다. 약8.83→8.34m/s의 작은 속도 dip은 있으나 신호 STOP pulse는 확인되지 않았다. |

B의 같은 창에서 BPP 최근접 목표속도는60개 모두16.3889m/s였지만 후속 behavior path에는
ZERO가 반복됐다. 제어기 단독 떨림은 아니며, 이번 bag에는 이전 optimizer fallback 제동 로그도 없다.
다만 실제 신호 출력·신호 모듈 상태·실행 SHA가 빠져 어느 BVP 모듈이 ZERO를 넣었는지 최종 확정할 수는 없다.

전방 원본1,083장을 현재 고정 v5로 연속 재처리했다. B의 STOP5개 시각 모두±0.25초 안에
visual OFF의 uncertainty 거부 UNKNOWN을 ON의 **현재 GREEN crop 판독**으로 회복하는 원본이 있었다
(고유6프레임). B 추가 GREEN7개 전수 화면 검토에서 같은 head와 실제 녹색을 확인했고,
추가 RED5개도 그대로 적색으로 전달됐다. 오래된 녹색 유지가 아니다.
이는 구체적인 반사실 대조이며 당시 실제 신호 입력의 인과 확정이나 실차 제동5/5 해결 인증은 아니다.
UNKNOWN 총량에는 초기 미관측·통과 후 프레임도 있어 제동 오류율로 환산하지 않는다.

상세 시각·분모·표·이미지는 `planner_logs/analysis/261026_v94_latest_bag/`의
`BRAKE_CLASSIFICATION.md`, `B_stop_replay_overlay_v1.md`, `visual_review_v1/`에 있다.
이전8회→이번3회를 개선율로 쓰지 않는다. 이번 C/D는 정차·재출발 조건 자체가 달랐다.

## 최신 planning 통합·설치 뒤의 추가 A 시험

`traffic_light_compliance_A_map_candidate_261026_233610`은 실제 planner/controller로 완주했고
최종 정지/P를 확인했다. 첫 GREEN148.733초, 최종 계획 출발149.083초,
원본 MGeo선 통과150.479초이며, 출발→원본선의 목표 ZERO 복귀0/15·이동 중 재제동0회였다.
정차 hold 해제 때 속도가 사실상0인 단일 brake 표본은 이동 중 재제동으로 세지 않았다.

그러나 **실행 전체가 무움찔은 아니다.** 앞선 적색 접근·대기97~120초의 callback은 평균241ms,
p95 615ms·최대813ms였고, 출력 source age 최대888ms와 중간 미수신이 있었다.
기존 timeout1초 fail-open GO 로그와118.187초 MOVE→118.553초 ZERO,
속도0.321m/s에서 실제 재제동1표본을 확인했다. 최종 GREEN 구간은 callback 평균28ms·p95 34.9ms였다.
실제 추론549건은cuda:0이며 startup의 model.device CPU 표기1개를 CPU 추론 증거로 읽지 않는다.
부하가 CPU·GPU 중 어디서 생겼는지는 확정하지 못했다. 팀과 합의한 timeout1초 fail-open은
이 보완에서 바꾸지 않았으며, 이 별도 지연 사례를 지도 연결 보완의 해결 성과로 포함하지 않는다.
