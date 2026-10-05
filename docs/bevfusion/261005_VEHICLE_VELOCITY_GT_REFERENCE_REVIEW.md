# 차량 tracker 속도 GT 기준 재검토

상태: EVIDENCE / 보행자 메시지 우선 opt-in 및 차량 세 기준 비교 구현·검증 완료. Production runtime 미변경. 2026-10-05.

## 현재 구현과 변경 경계

현재 `pipeline_evaluator.py`는 차량 `/Object_topic.velocity`가 0인지 아닌지로
분기하지 않는다. 카탈로그로 보정한 geometric BBox 중심의 최근 0.5초 위치를
causal polynomial로 fitting하여 속도를 계산한다. 표본 3개 이상이며 시간 폭은
최소 0.001초만 요구한다. 5개 이상이면 2차, 그 미만이면 1차 fitting이다.

- Vehicle Speed MAE/p50: 객체 heading 방향의 signed longitudinal speed 오차.
- 현재 Vehicle 2D AVE: tracker XY 벡터와 **heading 방향으로 투영한 GT 속도 벡터**의
  평균 L2 오차. GT 위치 미분의 전체 XY 벡터를 그대로 사용하는 기준과 다르다.
- Pedestrian: 위치 이력의 전체 XY 미분을 사용한다. Speed 오차는 벡터 크기 차이,
  AVE는 XY 벡터 차이이다. mesh heading과 이동 방향이 다르거나 메시지 velocity가
  0이어도 이동을 평가한다.

기본 실행은 과거 결과 재현을 위해 위치 기반을 유지한다. 아래 opt-in으로 보행자
메시지 우선 정책을 선택할 수 있다. Production detector, tracker, prediction은 변경하지 않았다.

## 단위 문제 설명의 정정

과거 저장된 bag의 속도값에 m/s와 km/h 해석 차이가 있다는 기록은 있지만,
**MORAI 자체가 임의로 단위를 섞어 출력했다는 증거는 아니다.** 당시 UDP→ROS
어댑터의 변환 여부 등 녹화 코드 이력과 구분해야 한다. 현재
`morai_ego_udp_receiver.cpp`는 Object 속도에 km/h→m/s 변환을 적용한다.
과거 bag은 당시 recorder/adapter revision과 위치 변화 대조로 단위를 확인한다.

움직이는 보행자의 velocity=0 사례는 차량 메시지 속도를 사용하지 않을 이유가
아니다. 클래스별 GT 정책을 분리할 수 있다.

## 새 H01: 동일 E25 tracker 출력으로 GT 기준 A/B

대상: `260923_161315_H01_ajounice_eval_highway_balanced_0.bag`.
기존 saved E25 `pipeline_outputs.bag`를 사용하며 새 GPU 추론/추적은 하지 않았다.
기존 평가와 같은 asset-name map, status-size-v2 catalog, MR heading -1.347°,
map transform, Ego 30m ROI, association 2m, unresolved-GT frame exclusion을 유지했다.
양쪽 모두 같은 matched observation **856건**이며 기존 위치 기반 결과를 정확히 재현했다.

| 지표 (m/s) | 기존 위치 미분 GT | 메시지 종방향 속도 GT | 메시지 기준 − 기존 |
|---|---:|---:|---:|
| Speed error p50 | 0.319735 | 0.179214 | -0.140520 |
| Speed MAE | 0.431420 | 0.305677 | -0.125743 |
| 2D AVE (평균) | 0.605111 | 0.510607 | -0.094504 |

이 A/B는 기존 heading-projection 계약을 유지하고 `state.speed`만 메시지 body-x로
바꿨다. 메시지 body-y 및 차축→BBox 중심 회전 속도 보정은 포함하지 않았다.
**오차가 낮아졌다는 사실만으로 메시지 GT의 물리적 정확도가 증명되는 것은 아니다.**
다만 기존 GT 선택이 tracker 지표에 무시할 수 없는 영향을 줬음은 확인됐다.
7-bag macro 또는 과거 보고서 결과를 이 단일 bag 숫자로 대체하지 않는다.

## 위치 미분 기준의 취약점

별도 GT끼리 대조한 audit는 모델 matching이 아닌 native GT timestamp 관측을
사용했으므로 위 tracker A/B의 표본/지표와 혼동하지 않는다.

- 카탈로그 식별 차량의 Ego 30m 이내 1,346관측에서 메시지 body-x와 BBox 중심
  위치 미분의 차이는 평균 0.441m/s, p50 0.194m/s, p95 1.378m/s였다.
- 최악 사례는 3표본이 0.032초에 몰렸고, 같은 위치가 약 0.0004초 간격으로 반복됐다.
  다음 위치의 이동과 짧은 시간 폭 때문에 메시지 속력 약 20.77m/s에 대해 위치 기반
  속력 약 38.00m/s가 계산됐다. 표본 개수만으로 충분한 시간 이력을 보장하지 못한다.
- 이력 폭 ≥0.4초인 관측만 집계하면 종방향 차이 평균 0.237m/s, p95 0.696m/s였다.
  진단용 subset이며 기존 평가 결과를 이 조건으로 덮어쓰지는 않았다.
- 메시지를 map XY로 그대로 해석하거나 다시 /3.6 하면 차이가 크게 증가했다.
  이 H01에서는 body-frame m/s 해석이 위치 변화와 가장 잘 맞았다.

## 차축 속도와 BBox 중심 속도

MORAI 답변은 차량 `/Object_topic.position`이 후륜 차축 중심이며 센서 캡처 GT가
geometric BBox 중심이라고 확인했다. **velocity 역시 그 차축 점의 속도인지까지
답변으로 확인된 것은 아니다.** position 기준으로 velocity 기준을 자동 확정하지 않는다.

velocity가 차축 속도임이 검증되면, 동일 map 좌표계에서:

```text
v_bbox = v_axle + omega × r
r = R(yaw) × catalog.pivot_to_bbox_center_local
```

평면에서는 `omega × r = [-omega*r_y, omega*r_x]`이다. 직진은 거의 동일하지만
회전에서는 tracker가 중심을 정확히 추적해도 기준점 차이로 평가 오차가 생긴다.
Tracker를 바꾸기보다 평가 GT의 기준점을 맞추는 방향을 우선 검토한다.
메시지가 이미 중심 속도라면 이 보정은 이중 적용이므로 금지한다.

## 고주로·로터리 교차 검증 완료

`scripts/compare_vehicle_velocity_gt.py`와 `velocity_reference.py`로 opt-in 후보를
구현했다. 기본 평가기를 변경하지 않고 기존 saved output/summary를 읽어 동일
매칭·속도 표본에서 기준만 비교한다. 회전 변환 단위 테스트 3개를 통과했다.

H01 고주로 856건, R01 로터리(`260923_163034`) 853건의 기존 baseline을
Speed p50/MAE와 AVE 모두 1e-8 이내로 재현했다.

| 속도 GT 기준 | H01 Speed p50 / MAE / AVE | R01 Speed p50 / MAE / AVE |
|---|---|---|
| 기존 위치 미분 + heading 투영 | 0.320 / 0.431 / 0.605 | 0.292 / 0.405 / 0.598 |
| 위치 미분 전체 XY (투영 제거) | 0.320 / 0.431 / 0.618 | 0.292 / 0.405 / 0.677 |
| 메시지 body-x + heading 투영 | 0.179 / 0.306 / 0.511 | 0.291 / 0.388 / 0.588 |
| 메시지 body-x/y 전체 XY, 회전 보정 없음 | 0.179 / 0.306 / 0.522 | 0.291 / 0.388 / 0.735 |
| 메시지 body-x/y + 차축→중심 회전 보정 | 0.179 / 0.306 / 0.542 | 0.291 / 0.388 / 0.980 |

단위 m/s. 투영 제거와 기준점 보정 효과를 섞지 않도록 5개 변형을 별도 집계했다.
종방향 offset은 주로 횡속도를 바꾸므로 Speed MAE가 거의 같아도 AVE는 달라진다.
R01 매칭은 모두 yaw-rate >3°/s 회전 구간이다. H01 회전 141건의 AVE는
기존 투영 0.996, 위치 전체 XY 1.070, 메시지 전체 XY 0.955, 회전 보정 1.055였다.

### 기준점 가정에 반대되는 근거

Tracker 오차만으로 어떤 GT가 정확한지 판단하지 않고, 시간 이력 ≥0.4초의
matched GT 중심 위치 미분과 메시지 속도를 추가 대조했다.

| R01 속도 벡터 일치도 (m/s) | 평균 | p50 | p95 |
|---|---:|---:|---:|
| 메시지 ↔ 원본 position(차축)의 위치 미분 | 0.500 | 0.509 | 0.652 |
| 메시지 ↔ 보정 BBox 중심 위치 미분 | **0.170** | **0.160** | **0.283** |
| 회전 보정 메시지 ↔ 보정 BBox 중심 위치 미분 | 0.503 | 0.507 | 0.646 |

R01 메시지 body-y 평균 0.443m/s, BBox 중심 위치 미분 body-y 평균 0.314m/s,
추가 회전 보정 body-y 평균 0.359m/s다. **메시지 속도는 차축보다 이미 중심
인근 기준 속도에 가깝고, omega×r를 더하면 이중 보정일 가능성**이 있다.
이것은 물리 기준점 확정이 아니며 정확한 CoM/다른 기준점·heading 계약·메시지
횡속도 정의를 분리하기 위한 live 검증이 필요하다. Position 후륜 차축이라는
공식 답변을 velocity 후륜 차축이라는 답변으로 확대하지 않는다.

또한 앞 절 native GT audit의 짧은 이력 outlier와 달리, 이번 실제 matched 속도
표본에서는 두 bag 모두 0.1초 미만 이력이 **0건**이었다. 그 최대 17m/s 사례가
기존 tracker MAE에 그대로 들어갔다고 해석하면 안 된다. H01에서 메시지 기준이
좋아지는 현상은 matched subset에서도 존재하지만 짧은 이력만으로 설명되지는 않는다.

### 판정 및 다음 live 단계

- 메시지 속도 + 차축→BBox 회전 보정은 **기본값 승격 보류**한다.
- 메시지 전체 XY, 위치 전체 XY, legacy projection을 모두 보존한다.
  로터리에서 낮은 projection AVE만 선택해 개선으로 보고하지 않는다.
- 다음은 통제된 직진과 좌/우 일정 회전에서 메시지 body-x/y, pivot/center 위치,
  heading과 yaw rate를 기록하고 실제 velocity 기준점/좌표계 가정을 판별하는 live 단계다.
- 이번 작업에서는 MORAI 제어/연결 및 새 녹화를 시작하지 않았다.

결과(ignored): `planner_logs/analysis/new_highway_velocity_gt_audit/`
`H01_161315_three_way.json`, `R01_163034_three_way.json`.

재현(ROS 환경 source 후, GPU/ROS master 불필요):

```bash
PYTHONPATH="$PWD/base_ws/src/tracking_prediction_evaluation/src:$PYTHONPATH" \
  /usr/bin/python3 base_ws/src/tracking_prediction_evaluation/scripts/compare_vehicle_velocity_gt.py \
  artifacts/evaluation/bevfusion/teammate_manual_261004/e25/R01_163034/pipeline/summary_status_size_v2.json \
  --output planner_logs/analysis/new_highway_velocity_gt_audit/R01_163034_three_way.json
```

## 이동 보행자 UDP/ROS 경로 추가 확인

2026-10-05 live: 같은 Man1 공식 waypoint 경로를 activeDistance=1000으로
로드 직후 활성화하여 UDP Ego/Object 및 UDP ScenarioLoad9095로 두 번 반복했다.
기존 validator Network의 포트와 sidecar를 사용했으며 production 설정은 변경하지 않았다.

| UDP 반복 | 이동량 | velocity.x/y/z nonzero 표본 | 로드 후 Ego 위치 오차 최대 |
|---|---:|---:|---:|
| 1 | 13.8286m | 0 / 533 | 0.00278m |
| 2 | 13.8286m | 0 / 539 | 0.00255m |

같은 즉시 활성 조건의 앞선 direct ROS rosbridge 반복에서는 이동13.8286m,
nonzero229/538 및230/535, 이동 중 raw 속력 약10이 관측됐다. 다만 그 ROS 반복은
Ego 정지에 실패했으므로 모든 조건이 동일한 엄격한 A/B라고 주장하지 않는다.
GUI의 별도 '바로 이동' enum을 선택한 검증도 아니며 activation 거리를 확장한 방식이다.

따라서 **UDP velocity=0은 기존 15m proximity trigger 조건에만 국한되지 않는다.**
현재 UDP→ROS 관측에서는 즉시 활성 조건에도 위치만 이동하고 속도는0이었다.
UDP 어댑터는 Object velocity 성분에 단위 변환만 적용하며 보행자에0을 강제하는
분기는 없다. 그러나 UDP 원본 byte를 별도 캡처·검증하지 않았으므로 MORAI 원본
출력 문제인지 packet 해석/경로 문제인지는 공식 확인 전 확정하지 않는다.
Vehicle에 같은 현상이 있다고 확대하지 않는다.

MORAI 추가 질문 후보:

> 같은 Man1 waypoint 이동에서, UDP로 받은 보행자 상태는 위치가 약13.8m
> 변하는 동안에도 velocity.x/y/z가 계속0이었습니다. ROS 직접 출력에서는
> 이동 중 nonzero 속도가 관측됐습니다. 보행자의 UDP Object Info에는 이동
> 속도가 제공되지 않는 것이 정상인지 확인 부탁드립니다.

원본/재현(ignored): `planner_logs/analysis/pedestrian_udp_velocity_261005/`
(`probe.py`, `summary.json`, `zz_ped_vel_udp_261005_0.bag`, `zz_ped_vel_udp_261005_1.bag`).
MR 임시 scenario/sidecar는 테스트 후 제거했다. Network 원본은 변경하지 않았다.

## 후속 계획

### 합의한 평가 정책 — opt-in 구현 완료

Base5 수집 연결은
[동기 캡처 런북](../../dataset/morai_od/docs/BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)을 따른다.
수집 방식과 평가 GT 선택을 분리하며, 아래 합의만으로 기존 결과를 덮어쓰지 않는다.

- Pedestrian: **직접 ROS 경로의 유효한 메시지 속도를 우선** 사용한다. bag별 단위와
  좌표계를 확인해 m/s의 동일 map XY로 변환하고 평가 시각에 정렬한다. 앞선 ROS
  실측 raw 속력 약10을 곧바로10m/s로 해석하지 않는다.
- 정지 보행자의 메시지0은 유효할 수 있다. 하지만 위치가 이동하는데 velocity가
  계속0인 UDP 기록은 유효한 정지 GT로 쓰지 않고 BBox 중심 위치 미분으로 fallback한다.
  직접 ROS에서도 같은 누락이 발견되면 동일하게 처리한다. bag/class/구간별 GT 출처와
  fallback 표본 수를 기록하며, 메시지가 nonzero인지 하나만으로 매 프레임 방식을 바꾸지 않는다.
- Vehicle은 동일 tracker 출력·ROI·매칭 표본으로 **세 가지 주요 기준을 모두 유지**한다:
  (1) 카탈로그 보정 BBox 중심 위치 미분 전체 XY,
  (2) 메시지 body-x/y를 map XY로 변환, 추가 회전 보정 없음,
  (3) 메시지를 차축 속도로 가정하고 `omega × r`를 추가한 BBox 중심 속도.
  세 번째는 기준점 미확정인 가설 비교이며 공식 GT 또는 기본값으로 승격하지 않는다.
- 과거 결과 재현용 heading-projection 변형 두 개도 보존하되, 위 세 가지 주요
  기준과 이름을 구분한다. 성능이 가장 좋아 보이는 GT만 사후 선택하지 않는다.
- Speed MAE/p50와 2D AVE를 함께 보고하고 차량의 Speed는 기존 signed 종방향,
  Pedestrian의 Speed는 벡터 크기 차이로 명시한다. 비교 표본·GT 출처가 다른 지표를
  같은 평가 계약처럼 합치지 않는다.

추가 실측: `active=false`, `activeDistance=0` 복사본도 로드 후 약1.38초에 이동을
시작해13.8286m 이동했다. UDP538개 표본의 velocity는 모두0, Ego 이동은 약2.55mm였다.
이는 큰 활성 거리 없이도 같은 현상이 재현됐다는 증거다. GUI '바로 이동' 모드와의
정확한 대응이나 모든 pedestrian 자산의 UDP 속도 미제공을 확정하지 않는다.
근거: `planner_logs/analysis/pedestrian_udp_velocity_261005/ACTIVE_OFF_RESULT.md`.
추가 MORAI 문의는 당장 진행 조건으로 요구하지 않으며 원본 속도·위치 이력을 보존한다.

### 실행 설정과 검증 결과

`evaluate_pipeline_prediction.py`에 다음 옵션을 추가했다. 기존 위치 기반 호출은
그대로 재현되며 새 정책은 명시적으로 선택한다. 메시지 모드에 단위/좌표계가 없으면
실패하므로 ROS/UDP라는 이름만으로 단위를 추정하지 않는다.

```bash
# 직접 ROS 보행자 bag의 단위/좌표계를 확인한 뒤 기존 평가 명령에 추가
--pedestrian-velocity-reference message-preferred \
--pedestrian-message-unit kmh \
--pedestrian-message-frame body
```

`body`는 각 객체의 보정·map 변환된 heading을 사용한다. `morai-map`은 메시지
XY를 MORAI map 벡터로 해석하고 static map TF yaw만 적용한다. 좌표 회전을 중복
적용하지 않는다. 이번 직접 ROS Man1의 kmh/body는 현재 관측에 근거한 입력 계약이며
다른 자산/버전에 무조건 적용하지 않는다.

메시지 가용성은 segment 내 finite/nonzero 표본3개 이상으로 판단하며 offline GT
품질 판정이다. 매칭 시점의 최근0.5초 이력이 모두0이고, 시간 폭≥0.1초이며 위치
미분 속력>0.2m/s이면 moving-zero fallback한다. 단일 zero 표본 때문에 전환하지 않는다.
메시지 가용성이 확인된 segment의 실제 정지0은 그대로 사용한다. 이 기준은
인지 threshold나 tracker 속도 threshold가 아니라 평가용 기록 품질 정책이다.
JSON에 `velocity_gt_source_counts`와 단위/좌표계/정책을 기록한다.

Saved output 비교는 GPU와 ROS master 없이 실행한다:

```bash
ajou
export PYTHONPATH="$AJOUNICE_ROOT/base_ws/src/tracking_prediction_evaluation/src:$PYTHONPATH"
/usr/bin/python3 base_ws/src/tracking_prediction_evaluation/scripts/compare_pedestrian_velocity_gt.py \
  artifacts/evaluation/bevfusion/teammate_manual_261004/e25/P01_163434/pipeline/summary_status_size_v2.json \
  --message-unit mps --message-frame body \
  --output planner_logs/analysis/new_highway_velocity_gt_audit/P01_velocity_policy.json
```

위 P01은 UDP bag이므로 단위는 현재 UDP 어댑터 m/s 계약이며, velocity 전부0으로
229/229 표본이 unavailable fallback했다. 기존 speed p50/MAE는0.170237/0.402942m/s,
AVE는0.586339m/s이며 새 정책에서도 동일하다. 새 ROS 메시지 기반 실주행 tracker
성능 검증 완료를 의미하지 않는다. ROS 경로는 kmh/body 단위·회전 및 메시지 우선
단위 테스트로 검증했으며 해당 실주행 saved output A/B는 후속이다.

차량 비교 helper는 `primary_velocity_references`에 세 주요 기준과 기존 JSON key의
대응을 기록한다. `message_axle_full_xy`는 역사적 key이며 회전 보정 없는 메시지
기준이다. 이름 때문에 물리 기준점이 차축으로 확정됐다고 해석하지 않는다.
H01/R01 재평가에서 기존 TP856/853과 baseline Speed/AVE를1e-8 이내로 재현했다.
평가 패키지 전체 pytest55개 통과. 새 추론/학습/실주행은 실행하지 않았다.

1. 차량 bag별 메시지 속도 단위/body-vs-world 좌표계와 recorder revision을 확인한다.
2. 회전 구간에서 velocity 기준점 및 yaw-rate 추정 신뢰도를 검증한다.
3. 비교용 opt-in 구현은 완료했다. live 검증 후 Vehicle의 기본 평가 계약을 선택한다.
   전체 XY 벡터와 기준점 보정을 명시하고 legacy 지표도 별도 보존한다.
4. Pedestrian은 위 ROS 메시지 우선/fallback 정책을 opt-in 구현·검증했다. 현재 기본
   평가기는 여전히 위치 기반이다. 짧은 이력,
   중복성 표본, timestamp 이상은 evaluator만에서 품질 처리한다.
   live UDP/localization 입력 timestamp 정책을 이번 작업 때문에 변경하지 않는다.
5. 같은 tracker 출력/매칭으로 미션별 재평가 후 CURRENT와 보고서 기준을 갱신한다.
   이번 증거만으로 production 또는 과거 수치를 자동 교체하지 않는다.

재현 코드/수치(ignored, Git 전달본 아님):
`planner_logs/analysis/new_highway_velocity_gt_audit/`
(`audit.py`, `summary.json`, `worst.json`, `compare_tracker.py`, `tracker_comparison.json`).

이번 결론은 temporal/velocity-head 학습 재개가 아니다. Base5는 single-frame OD이며
속도는 tracker에서 추정한다. 동기 캡처 BBox GT 학습과 tracker 속도 평가 계약을 분리한다.
