# v5 설치기·실제 경로·폐루프 연결 시험

종류: EVIDENCE / 2026-10-25, 작업 기준 `3ecfb82f` 이후 변경.
현재 선택은 [CURRENT](CURRENT.md), 설치법은 [팀 시험 안내](V5_CANDIDATE_HANDOFF.md)를 따른다.
**운영 기본 승격·전체 주행 PASS·공식 신호 준수 인증이 아니다.**

## 설치·GUI

- 고정 모델 압축본을 별도 version 폴더에 설치한다. 압축 SHA/크기·경로·regular file·
  개별 가중치 SHA를 확인하고 원자적으로 게시한다. 기존 변경/미관리 파일은 덮어쓰지 않는다.
- 기존 환경 설치기를 재사용하고 후보 입력용 Pillow 버전을 고정한다. 시스템 Python을 변경하지 않는다.
- 새 wrapper는 YOLO26n960/map-association/ResNet18 v5를 명시한다. 기존 launch 기본은 640/HSV다.
- Validator GUI의 후보 선택, 실제 operator save, runtime preflight, actual sensor profile 전달을 연결했다.
  옛 YAML을 열고 저장하기만 해도 후보 키가 추가되는 방식은 쓰지 않았다.
- 후보를 시스템 Python YAML에서 명시적으로 선택할 때만 전용 환경을 선택한다.
  기존 가상환경 설정·새 실행 snapshot·고정 계약 및 동시 편집 충돌 검사는 유지한다.
- 개인 Drive 백업을 공개로 바꾸지 않았다. 해당 팀원의 읽기 권한 또는 모델 압축본 전달이 필요하다.

실제 두 모델을 설치된 후보 경로에서 읽고 RTX5060Ti/CUDA 및 ROS import·forward를 통과했다.
기존 전용 Conda 환경을 `--existing-python`으로 사용했으므로 **새 PC의 빈 환경 전체 설치 인증은 아니다.**
팀 4090과 BEVFusion 동시 부하 검증도 남아 있다.

## 실제 경로 bag 재생

입력: `rosbags/eval/perception/261024_teammate_lan_foggy15_full_route/261024_035142_full_route.bag`.
`/planning/mission_planning/route`, `/planning/route_state`, `/localization/kinematic_state`만 읽었다.
현재 source matcher와 v87 지도로 receipt-time을 재생했으며 Ego/native 정답은 넣지 않았다.

- 초기 선택은 A→B→C→D였다. 옛 D-1은 allowlist 밖이다.
- D 통과 이후에도 D가 재선택되는 구간이 있었다. matcher는 매번 route 앞부터 전구 기준
  half-plane을 검사하며 route 진행 이력을 누적하지 않는다. 먼 다른 구간에서도 같은 면을 다시 가로지를 수 있다.
- 당시 route의 primitive 중 현행 지도에 없는 lane ID가 10개였다. 원래 지도와 동등한 재생은 아니다.
  이 결과만으로 실제 운영 중 잘못된 신호를 사용했다고 단정하거나 matcher를 일괄 수정하지 않았다.
- 근거: `artifacts/evaluation/traffic_light_robustness_261025/actual_route_bag_replay_v1.json`.
  새 `audit_traffic_light_route_replay.py`는 source/map/bag SHA와 누락 lane ID를 기록한다.

## 실제 MORAI 주행 — 도착 전 정상 중단

Sunny13 로컬 MORAI, 실제 localization + 실제 ROS1 planner/controller, BEVFusion OFF,
OD_GT를 켜되 차량/보행자/장애물/spawn을 비운 장면이다. Ego/native는 개발 감시·초기화용이며
모델 미션 선택은 actual route와 localization/TF다. 기존 풀코스 시작점·goal·waypoints는 보존했다.
수집/주행 소유권·P·freshness 검사를 켜고 원본 front JPEG·신호 관측·경로·제어·TF를 녹화했다.

첫 실행은 출발 전에 중단됐다. 새 launch의 `$(eval)`이 숫자로 변환한 GPU 번호를 문자열과
연결하는 오류를 고쳤고 GPU/CPU 양쪽 launch expansion을 재검사했다. 별도로 지도 로더 설치본이
10/23 빌드, 최신 source가 10/24 수정인 불일치를 확인해 **해당 패키지만 재빌드**했다.
지도 단독 실행에서 v87 발행을 확인했다. planning 알고리즘·신호 YAML은 수정하지 않았다.

두 번째 실행 `planner_logs/validation/traffic_light_v5_no_objects_261025_160214/`은 실제로 주행했다.
bag은 약 335.5초, 2.50GB이며 초기 준비 시간도 포함한다. 연결된 관측 수는 다음과 같다.

| Route 미션 | 해당 head로 지도 연결된 관측 |
|---|---:|
| A / 077 | 319 |
| B / 091 | 31 |
| C / 016, 018 | 117, 18 |
| D / 052 | 45 |

관측 당시 selection이 선언한 head 수다. 독립 identity GT가 없으므로 정확도 100%가 아니다.
먼 접근·아직 화면에 없는 시점·정차를 포함한 모든 callback 수를 분모로 recall을 계산하지 않는다.

### D 근접에서 드러난 두 문제

Ego가 정지선 앞 `map≈(107.46, 644.68)`에 정차한 뒤 관측이 복구되지 않았다.
원본 JPEG에서는 화면 상단에 신호등이 보인다.

1. 동일 새 YOLO 가중치·동일 JPEG에서 960 추론의 정답 위치 후보 score는 **0.0669**였다.
   현행 기준 0.25를 못 넘는다. 같은 사진은 640에서 **0.8193**이었다. 기존 모델 640은0.9711이었다.
   입력 해상도를 키우는 것이 모든 객체 크기/화면 위치에서 단조롭게 유리하지 않다는 반례다.
   단일 개발 사진으로 전체 실행을 640으로 교체하거나 threshold를 전역으로 낮추지는 않았다.
2. D head 깊이 약8.76m에서 공분산 기반 필요 padding은 XY≈70.83/184.21px였다.
   수평 상한90px는 통과하지만 수직 상한180px를 넘어서 `projection_uncertainty_exceeds_bound`였다.
   이는 "실제로 신호가 보이지 않음"이 아니라 현행 선택 gate의 거부다. 라벨·YOLO와 별개 문제다.

두 조건이 각각 해결되지 않으면 score만 완화하거나 색 CNN만 개선해도 출발이 막힐 수 있다.
검출이 불가능하니 timeout으로 GO를 허용하는 것은 이 실패의 해결책으로 채택하지 않았다.
근접/상단 신호 자료·다중 크기 대조와 bounded 지도 검색을 먼저 검증해야 한다.

추가 출발 없이 정상 interrupt로 시험을 마감했다. stop/P 확인은 events에 남았으며 최종
`MANUAL_ABORT`, `recording_warning=false`다. 이후 본인 시험 노드는 모두 종료했고 rosout만 남았다.
신호 native 상태와 앞바퀴 공식 선 통과를 동시 대조하지 않아 신호 위반률은 미측정이다.

## 비용·검사 결과

- 2,758개 runtime 관측에서 callback→출력 평균22.54ms, p50 21.25ms, p95 32.34ms,
  최대51.81ms, 실제 처리 약8.51Hz(상한10Hz). local MORAI 동시, BEVFusion 제외 조건이다.
- 위 전체 callback에는 crop이 없는 경우가 많다. 전체 평균 classifier 시간을 실제 CNN 비용으로
  해석하지 않도록 summary에 `matched_frame_timing_ms`를 따로 추가했다.
  연결된530개에서는 CNN/crop p50 4.09ms/p95 5.80ms, 전체 callback p50 26.94ms/p95 36.75ms다.
- `frame_timing`은 monotonic callback wall이다. debug JPEG encoding 및 LAN end-to-end 비용은 제외한다.
- 설치기/프로필/실제 GUI 영향 범위29개, trial 생성/자산13개 통과(공통11개는 중복이므로 합산 금지).
  현행 인지 계약84개 통과. 옛 `test_signal_policy.py`는 v1.0 기대값과 현행 YAML이 달라 6개 실패하며
  별도 current contract test가 존재한다. 이번에 운영값을 바꿔 테스트를 맞추지 않았다.
  이 범위를 합친 최종 Xvfb 실행은 **115개 통과/생략0**이었다.
- validator 전체 비GUI: 388개 통과/34개 생략/1개 실패. 실패는 기존 mission manifest의
  `morai_lanelet_static_tf.yaml` SHA 불일치다. 이번 작업에서 해당 TF·manifest는 변경하지 않았다.
  임시 HOME/X11 제한으로 실패한 초기 실행은 통과 근거에 넣지 않았다.
- source/install의 후보 launch·GUI SHA 일치, context checker 및 `git diff --check` 확인.

후속 순서: D 근접 검출·지도 gate → 새 전체 주행 → 팀4090 동시 부하 → 정책 별도 승인·회귀 → 운영 승격.
정차 중 Y/R+Y 새 출발과 카메라/신호 단절은 [정책 제안](261025_DEPARTURE_AND_SIGNAL_LOSS_PROPOSAL.md)을 따른다.
