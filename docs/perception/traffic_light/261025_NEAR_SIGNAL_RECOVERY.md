# D 근접 신호 검출·지도 연결 보완

종류: EVIDENCE / 2026-10-25, `cb42ebb0` 이후 구현·로컬 시험.
운영 선택은 [CURRENT](CURRENT.md), 팀 설치는 [배포 안내](V5_CANDIDATE_HANDOFF.md)를 따른다.
가중치·정책은 바꾸지 않았다. 이전 실패 bag과 보고서는 당시 결과 그대로 보존한다.

## 무엇이 문제였나

[이전 폐루프 시험](261025_INSTALLER_ROUTE_AND_DRIVING_TRIAL.md)은 D 정지선 앞에서 멈췄다.
화면에 housing이 완전히 보였지만 두 독립 조건이 관측을 차단했다.

1. 약 8.76m 위치에서 YOLO960의 score가 기준 0.25에 미달했다. 같은 원본은 640에서 검출됐다.
2. 실제 측위 공분산으로 계산한 수직 검색 폭 184.21px가 상한 180px를 넘었다.

따라서 색 CNN을 바꾸거나 timeout으로 GO를 허용하는 것은 직접 해결책이 아니다.
근접 누락을 카메라 스트림 단절과 구분하며, 화면 밖 정상 화각 이탈도 같은 실패로 세지 않는다.

## 변경 범위

- 기본 YOLO960·confidence0.25·CNN v5·crop margin12·bit confidence0.6·smoother7/4는 유지.
- 지도 수직 padding 상한만 180→200px. 수평90px·측위/TF/공분산 검사·다른 head 모호성
  거부·housing 종횡비·nominal 전체 화각 검사는 유지한다. 필요한 불확실성 자체를 작게 조작하지 않는다.
- 첫 검출이 현재 미션 head에 연결되지 않고, 같은 JPEG stamp의 유효한 지도 투영이
  깊이3~15m·폭60px 이상일 때만 전체 화면640을 한 번 추가 추론한다.
- 이미 연결된 관측, 먼/작은 신호, map missing/stale/invalid, 다른 head 모호성,
  CNN UNKNOWN, legacy에서는 재시도하지 않는다. 고정 D ID나 픽셀 위치로 선택하지 않는다.
- 두 번째 검출도 기존 전체 identity association을 통과하고 원래 허용한 현재 미션 head여야 한다.
  분류 결과나 native phase로 신호등을 선택하지 않는다. 실패 시 missing을 유지한다.
- 원본 JPEG를 다시 decode/압축하지 않고 동일 BGR 배열을 쓴다. 추가 비용은 기존
  `selection_including_map_wait_sec`의 **부분집합**인
  `near_rescue_detector_and_association_sec`다. 둘을 더해 총시간을 이중 계산하지 않는다.

`traffic_light_node.launch`와 호환 v5 wrapper는 `near_rescue_enabled:=true`,
`map_max_vertical_padding_px:=200`이다. 원래 경로 A/B 대조는 각각 false/180으로 지정한다.
raw Python node의 재시도 기본은 OFF, 명시적 `traffic_legacy`도 OFF다.
모델 SHA·배포 압축본은 그대로이므로 가중치를 새로 업로드하거나 재학습할 필요는 없다.

## 판단 근거와 한계

자료 루트: `artifacts/evaluation/traffic_light_robustness_261025/`.

| 자료 | YOLO960 | YOLO640 | 해석 |
|---|---:|---:|---|
| 기존 개발 test 미션 recall | 93.0% | 80.2% | 전체640 전환은 기각 |
| 이전 bag의 D 근접 연속79표본 검출 | 3/79 | 79/79 | 같은 주행의 상관된 표본, 독립 정확도 아님 |
| 알려진 실패 위치·6환경×5현시 | 0/30 | 30/30 | 개발용 재현, 640 crop→CNN도30/30 일치 |
| 새 위치2곳·6환경×5현시 | 60/60 | 60/60 | 정상 근접 장면 유지; 두 pose만의 구성요소 holdout |

새 위치는 `near_holdout_plan_v1.json`에 예측 확인 전 고정했다. 모델·임계값을 새 결과에
맞춰 재조정하지 않았다. 모든 JPEG의 SHA와 전후 exact native ID·상태를 확인하고 원본
전체/확대 contact sheet를 직접 검토해 housing 대응과 램프를 확인했다. 박스는 검출 결과나
지도 투영을 GT로 재사용하지 않은 수동 시각 기준이다. 독립된 사람의 전수 라벨은 아니다.
새 두 pose에서는 이미960이 정상이므로 재시도의 **독립 실패 복구 성공**을 증명한 것은 아니다.

지도 counterfactual 재계산은 D의 기존176/6431 eligible에5380개를 추가한다.
Y210/240은 Y200보다 이득이 없었다. 정지 구간의 최장 map-missing은200.607→0.126초로
계산되지만 검출/CNN/주행 성공률은 아니다. 기존348장 개발 replay의 선택339·오선택0,
실제 다른 head 음성18개·합성 보행자474개 결과는 기존 개발 회귀 한계 안에서만 해석한다.

재현 도구: `ablate_traffic_light_near_detector_scales.py`, `audit_traffic_light_roi_caps.py`,
`evaluate_traffic_light_near_development_matrix.py`, `review_traffic_light_near_matrix.py`.
결과는 `near_detector_scales_261025_v1/`, `near_roi_caps_audit_v1/`,
`near_D_holdout_{lateral,back}_review_v1/`에 보관한다.

## 실제 주행·검증 경계

실제 ROS1 planner/controller·localization/TF·v87 전체 경로를 사용하며, 차량/객체는 비우고
OD_GT·BEVFusion OFF 조건으로 시험한다. 원래 goal/waypoint를 줄이지 않는다.
native 신호 제어는 정지 데이터 시험에서만 사용하고 모두 자연 프로그램으로 복귀한 후 주행한다.
개발 Ego는 초기화·감시용이며 운영 선택 입력은 actual route와 localization/TF다.

LAN·팀4090·BEVFusion 동시 부하, 공식 정지선 통과 시점과 native 상태 대조,
황색 새 출발·신호 상실 정책은 별도 검증이다.

### 마감 검증

| 실제 무객체 풀코스 | Sunny13 v3 | Foggy13 v4 |
|---|---:|---:|
| Validator 결과 | SUCCESS | SUCCESS |
| D 재검출 시도 / 지도 연결 승인 | 93 / 93 | 92 / 92 |
| D 정지 구간 내 재검출 승인 | 84 / 84 | 82 / 82 |
| D 정지 시간(localization) | 10.84초 | 10.72초 |
| 일반 연결 프레임 callback p50 / p95 | 26.23 / 34.80ms | 25.87 / 33.82ms |
| 재검출 프레임 callback p50 / p95 | 35.44 / 41.31ms | 35.54 / 43.07ms |
| 재검출 자체 p50 / p95(위 시간에 포함) | 8.83 / 17.08ms | 9.47 / 15.90ms |
| 전체 처리율(기존 최대10Hz 정책 유지) | 8.53Hz | 8.50Hz |

v3는 첫 통합본, v4는 재검출 종료 시 현재 미션·packet age를 재확인하는 보호까지 포함한
최종 실행이다. 같은 코드 payload가 source/install에 일치함을 확인했다. 각 실행의
`runtime_receipt.json`·파라미터 사본으로 당시 실행을 구분한다.
처리시간은 로컬 RTX5060Ti + MORAI·ROS 실제 실행의 callback wall이며 debug JPEG encoding,
LAN 전달·BEVFusion 동시 비용은 제외한다. 첫 shape warm-up 등의 tail도 숨기지 않았다:
v4 전체 callback 최대403.04ms(초기 검출), 재검출 프레임 최대96.13ms다.

두 실행 모두 신호 관측을 복구해 정차 뒤 재출발했고, goal 도달 후 실제 정지·P를 확인했다.
v4 환경 API 전후 Foggy13을 확인하고 추출한 원본 JPEG에서 안개 렌더링과 D housing을
직접 확인했다. 종료 뒤 Sunny13으로 복원했으며 본인 시험 노드는 모두 종료, rosout만 남겼다.
모든 시도는 같은 주행의 상관된 프레임이다. 93/93은 **선택 규칙 통과 수**이며 독립된
native/공식 정지선 GT에 대한 정확도100% 또는 위험 오통과0%라는 뜻이 아니다.

- 실행: `planner_logs/validation/traffic_light_v5_no_objects_261025_181557/`,
  `planner_logs/validation/traffic_light_v5_no_objects_261025_182258/`.
- 분석: 자료 루트 `fullroute_near_fix_v3/`, `fullroute_near_fix_foggy13_v4/`의
  `observations_summary.json`, `near_retry_summary.json`. 종료 bag·원본 사진·실패 실행을 보존했다.
- 재현: `summarize_traffic_light_near_retry_bag.py BAG --confirm-closed --camera-topic
  /sensor/camera/front/compressed --output artifacts/evaluation/NEW.json`.
- 테스트: 현행 인지110개 + 설치/평가32개 = 중복 없는142개 통과. 옛 `test_signal_policy.py`는
  기존 v1.0 기대값과 현행 YAML이 다른 별도 레거시 검사라 이 집계에서 제외했다.
  정책 소스를 바꿔 검사를 맞추지 않았다. context checker·diff 검사·설치 payload 대조도 통과했다.

### 남은 것

통과한 신호의 전구 기준 half-plane을 나중에 다시 가로지르면 route helper가 D/C/B/A를
다시 선택하는 기존 문제가 v3/v4에도 남았다. 대부분 해당 housing이 화면 밖인 구간이며,
그 missing을 D 정지선 앞 검출 실패와 합쳐 recall로 계산하면 안 된다. v4의 미션 변경
보호는 **한 번의 재검출 도중** 변경만 막으며 route 진행 이력 문제를 해결한 것은 아니다.
전체 주행 SUCCESS를 모든 미션 identity/신호 위반률 인증으로 확대하지 않는다.
