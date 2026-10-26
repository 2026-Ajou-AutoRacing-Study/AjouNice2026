# 시간제한 황색 통과 후보와 A/B/C/D 검증

상태: 2026-10-26, **EVIDENCE / 명시적 opt-in 후보**. 운영 기본값을 승격한 문서가 아니다.
현재 권위는 [CURRENT](CURRENT.md), 기존 배포는 [handoff](V5_CANDIDATE_HANDOFF.md)다.
후보 사용법은 [Validator runbook](../../planning/SCENARIO_VALIDATOR_RUNBOOK.md#시간제한-황색-후보-설정-2026-10-26)을 따른다.

## 범위와 변경하지 않은 것

신호등 판단의 허가 관측 이력·황색 통과 시간·진단과 launch/Validator 전달만 변경한다.
YOLO/CNN 가중치·threshold, 미션 지도 연결, 차선 변경·추월·로터리 판단, PP/PI 제어기와
공통 제동 함수는 이 후보에서 바꾸지 않는다. 장애물 정지 요구를 신호등 GO로 덮지 않는다.
앞차축 기준은 rear-axle `base_link`에서 +3.0m이며, 공식 front-wheel 판정 인증은 별도다.

## 기본값·명시적 후보·복귀

| 항목 | 운영 기본 | 이번 명시적 시험 후보 |
|---|---|---|
| `traffic_light_yellow_passthrough` | false | true |
| `signal_compliance_enable_timed_yellow` | false | true |
| `signal_compliance_required_green_observations` | 3 | 2 |
| `signal_compliance_min_yellow_sec` | 2.0초(비활성 가정값) | 2.0초 가정 |
| `signal_compliance_yellow_time_margin_sec` | 0.2초(비활성 여유값) | 0.2초 |

```yaml
stack:
  traffic_light_yellow_passthrough: true
  signal_compliance_enable_timed_yellow: true
  signal_compliance_required_green_observations: 2
  signal_compliance_min_yellow_sec: 2.0
  signal_compliance_yellow_time_margin_sec: 0.2
```

GUI의 기존 `황색 통과 허용`은 master switch다. 그것만 켜면 시간제한 후보가 자동 선택되지 않는다.
새 4개 고급 YAML 값은 GUI 저장·빠른 시험에서 보존된다. GUI 기본 체크 상태는 바꾸지 않았다.
현재 기본인 `enable_signal_compliance_guard=true`를 전제로 하며, 가드 OFF의 이전 정책은 이 후보가 아니다.
둘 다 true일 때만 시간제한 경로를 사용한다. master ON/timed OFF는 이전 제동거리 예외 경로다.
복귀는 편집용 YAML에서 **두 flag 모두 false**, 확인 횟수 3으로 설정하고 새 실행을 만든다.
새 snapshot은 네 고급 값을 명시 동결하며, 과거 snapshot·원본·hash/resume 보호는 수정하지 않는다.

## 전환 시각과 도달 시간

코어는 [준수 가드](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/include/autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp)의
`Guard`와 `timedYellowDecision`이다. 일반/experimental 모듈 모두 같은 순수 판단 함수를 쓴다.

1. 설정 횟수만큼 **서로 다른 신선한 영상 stamp의 해당 방향 허가**를 확인한다.
2. 허가→황색 전환을 확인하면 마지막 허가 영상 시각 `tG`를 황색 시작의 보수적인 하한으로 고정한다.
3. 판단 시각 `now`에서 `remaining = minimum_yellow - (now - tG)`로 계산한다.
4. 새 황색 프레임마다 `tG`를 앞으로 밀거나, 처음 황색을 본 때부터 2초를 새로 세지 않는다.
5. `arrival + margin <= remaining`일 때만 이동 중 시간제한 통과 후보가 된다.

도달 시간은 양의 가속을 기대하지 않고 `a=min(0,current_acceleration)`를 사용한다.
`a≈0`이면 `arrival=d/v`; 음의 가속이면 `v²+2ad`를 검사한 뒤 양의 도달 해를 계산한다.
현재 제동을 유지하면 선 전에 멈추는 입력은 통과로 승인하지 않는다. 이는 모델이지 물리적 도달 상한의 증명이 아니다.
시간제한 후보는 옛 제동거리 예외와 달리, 정지가 가능하더라도 시간 여유가 있으면 통과를 검토할 수 있다.
단, 이전 STOP 상태·정차·저속·허가 전환 이력 없음은 계속 통과 근거에서 제외한다.

## 실패 시 동작과 한계

- 최초 관측이 황색이면 전환 시작을 모르므로 STOP 쪽이다. D의 직진 G는 좌회전 허가가 아니다.
- 적색·비허가·UNKNOWN·missing·stale은 선 전의 통과 근거를 무효화한다. 원래 source stamp TTL을 유지한다.
- 남은 시간이 소진되거나 도달 여유가 부족해지면 정지 판단이다. 무조건 GO를 잠그는 latch가 아니다.
- 이미 앞차축이 선을 넘은 뒤는 기존 GO_OUT으로 빠져나가며 다른 객체 충돌 방지는 유지한다.
- 적색이 예상보다 일찍 오거나 앞차 때문에 늦어지면 여전히 딜레마가 생길 수 있다.
- **최소 황색 2초는 공식 보장이 아니다. 0.2초도 검증된 종단 지연 상한이 아니다.**
- 영상 timestamp가 실제 점등 순간과 일치한다는 보장도 없다. 렌더링·카메라·분류·제어 오차가 남는다.
- 적색 전 도달·실험 PASS·native 황색 통과가 공식 규정의 무패널티를 의미하지 않는다.

`[signal_timed_yellow]` 로그는 실제 `allow/reason/remaining/arrival/front`를 남긴다.
디버그 영상의 planner 설정 표시는 **CONFIG ONLY**이며 실제 GO 결정이나 잔여 시간 표시가 아니다.
2회 확인은 후보 설정일 뿐 운영 3회를 바꾼 결과가 아니고, 누락 없는 10Hz의 이론상 0.1초 이득을
실측 평균 재출발 개선이나 독립 오인식 확률 개선으로 주장하지 않는다.

## 실제 입력과 개발용 신호 제어의 분리

실험은 실제 카메라→YOLO/CNN→ROS1 planner/controller를 사용한다. native 정답을 판단에 넣지 않는다.
신호 조절은 개발용 **UDP SET을 physical head별로 전송**하고 독립 native GET으로 대조한다.
특히 C는 `C1256W000016`과 `C1256W000018`을 각각 조절·조회한다. 단일 head만 조절한 불일치를
자연 미션 성능으로 집계하지 않는다. GET은 native 상태이지 독립 픽셀 정답이나 연속 점등 인증이 아니다.
자연 주기, 강제 신호 시험, 합성 planner 신호 fixture는 서로 다른 근거다.

## 완료 검증

- Native 관련 시험 **84개 통과**: 순수 가드·원본/experimental 연결·기존 모듈 회귀를 포함한다.
  같은 시험 재실행 XML을 중복 합산하지 않았다.
  [XML 근거 폴더](../../../artifacts/evaluation/traffic_light_robustness_261025/timed_yellow_tests_261026_v1/).
- Validator+디버그 회귀 **425 통과 / GUI 37 제외 / 기존 manifest 생성 일치 시험 1개 제외**.
  GUI 포함 신호등 관련 별도 묶음 **36개 통과**이며 두 숫자는 별도 실행으로 합산하지 않는다.
- launch dump에서 master/timed bool, 확인 횟수 int, 최소 시간/여유 float 전달을 확인했다.
  문서 검사 오류 0. 단위 시험과 정적 검사는 실제 LAN·4090 동시 부하 인증이 아니다.

### 자연 풀주행: 후보 ON이지만 황색 통과 효과 시험은 아님

로컬 실제 ROS1 planner/controller + `od_gt` 자연 풀주행 1회가 PASS했다.
전후 native bracket에서 A=16, B=16, C 두 head=48, D=33으로 네 통과가 방향 허가 상태였다.
**시간제한 `allow=true`는 0회**여서, 기본 주행 회귀 근거이지 황색 통과 성공률 인증은 아니다.
B/C의 정확한 2회 확인 해제 순간은 throttled 로그만으로 단정하지 않는다.
[독립 대조 v3](../../../artifacts/evaluation/traffic_light_robustness_261025/timed_full_natural_independent_audit_261026_v3.json).

### A: 실제 native 황색 ON/OFF 국소 대조

| 조건 | 황색 전환 trigger→앞차축 통과 | 관찰 |
|---|---:|---|
| timed ON | 약 0.959초 | 황색 native=4 통과, 정차 표본 0 |
| master OFF | 약 6.866초 | 선 약 1.14m 전 정차 후 강제 허가에서 출발 |

차이는 약 **5.906초**지만 동일 운동 상태의 인과 효과나 자연 주기 절약 시간이 아니다.
ON trigger는 약 6.54m/6.52m·s⁻¹, OFF는 약 7.99m/6.21m·s⁻¹로 달랐다.
OFF 대조에는 비자연적 **적색 3초 후 허가**를 사용했다. ON의 native 황색 통과는 공식 허가 인증이 아니다.
ON 기록에서 통과 전 timed allow→deny 반전은 없었다. 전체 조건에서 반전이 없다는 보장은 아니다.
[ON/OFF 독립 비교](../../../artifacts/evaluation/traffic_light_robustness_261025/timed_A_native2_ON_OFF_independent_comparison_261026_v1.json),
[ON 원본 대조](../../../artifacts/evaluation/traffic_light_robustness_261025/timed_A_native2_ON_261026_v1_derived_review.json),
[OFF 원본 대조](../../../artifacts/evaluation/traffic_light_robustness_261025/timed_A_native2_OFF_261026_v2_derived_review.json).

## B/C/D 국소 ON/OFF 대조

양쪽 허가 확인은 2회이며, 실제 카메라·planner·PP/PI를 사용했다.
요청 황색 2초→시험용 적색 3초→허가의 조절 시험이며, 도달 시간은 실제 trigger 기준이다.

| 미션 | timed ON 통과 | master OFF 통과 | 관찰 |
|---|---:|---:|---|
| B | 1.021초, native Y4 | 6.798초, G16 | ON 무정차, OFF 선 약 1.28m 전 정차 |
| C | 0.934초, 두 head Y4 | 7.051초, 두 head G+좌48 | ON 무정차, OFF 선 약 1.12m 전 정차 |
| D | 1.090초, R+Y5 | 6.828초, R+좌33 | ON 무정차, OFF 선 약 1.01m 전 정차 |

각 ON/OFF의 trigger·속도·목적지가 완전히 같지는 않다. 약 5.7~6.2초 차이를
자연 신호의 평균 절약 시간이나 인과 효과로 일반화하지 않는다.
주 비교 8개 실행은 모두 목적지 PASS·충돌 0·종료 정지/P VERIFIED였다.
이번 국소 trigger 속도는 약 6m/s대이므로 대회 고속·LAN·BEVFusion 동시 부하를 인증하지 않는다.
ON 통과 전 기록된 allow→deny 반전은 없었지만, 모든 장면의 반전 방지를 증명한 것은 아니다.
각 통과는 원본 GET의 양측 0.25초 bracket으로 대조했다. native 황색/ R+Y 통과는 공식 허가 인증이 아니다.

### 시험 실행기 실패와 재시험

- A OFF 첫 시도는 장면 Load로 native 상태가 초기화된 뒤 적용 재확인 timeout을 전체 준비 시간에
  잘못 연결해 중단됐다. 연속 불일치 기간으로 수정하고 새 폴더에서 재시험했다. 실패 원본은 보존했다.
- B 첫 시도는 낮은 초기 속도에서 시간 부족으로 정상 STOP했다. 목표 B 이후의 다른 C 미션까지
  포함된 목적지에서 terminal native 유지가 깨져 전체는 ERROR였다. B 전용 목적지를 C 전으로 좁혔다.
- C 첫 시도는 연속 UDP 두 패킷에서 C018만 override가 유지되고 C016은 자연 Y/R로 돌아갔다.
  두 head SET 사이 각 100ms를 두고 두 GET을 별도 확인한 재시험에서 Y/R 제어와 ON 통과를 확인했다.
  특정 Unity 수신 구현의 원인을 확정하거나 모든 패킷 손실을 해결했다고 주장하지 않는다.
- C OFF 다음 시도는 목표 C를 정차 후 G+좌로 통과했지만, 그 약 15초 뒤 terminal G 유지가 깨져
  전체는 ERROR였다. C 이전 입력·통과 근거는 보존하고 C 이후·D 이전의 짧은 목적지로 재시험해 PASS했다.
  native 불일치 검사나 차량 정지/P 검사는 완화하지 않았다.

시험 자료는 `artifacts/evaluation/traffic_light_robustness_261025/timed_matrix_261026_v*`에 보존한다.
완료된 input과 분리된 `_derived_review.json`, `_independent_integration_v1.json`을 근거로 사용한다.
### 1초·3초와 허가 이력 없는 D 황색

- B 요청 3초는 전체 PASS, trigger→통과 약 1.048초이며 native Y4 양측 bracket을 확인했다.
- B 요청 1초 스트레스도 목적지는 도착했으나, 통과의 native bracket은 Y→R 전환으로 **색 미확정**이다.
  trigger→통과 약 1.068초, 첫 RED SET은 통과 약 47ms 뒤였다. 직전 원본 JPEG는 황색으로 보이지만
  native 미확정 결과를 덮어쓰지 않는다. 실제 요청/응답 기반 Y 구간도 약 1.014~1.217초라서
  정확히 1.000초의 안전성이나 1초에서도 2초 가정이 성립한다는 증거로 쓰면 안 된다.
- D 정차 황색 재시험은 전체 PASS. 허가 관측 없이 R→R+Y 2초·3초에서 양의 출발 명령 0회였다.
  native 확인 후 실제 R+Y 구간은 약 2.022/3.033초, raw Ego 최대 속도는 약 0.00107/0.000051m/s였다.
  실제 raw CNN도 R+Y를 각각 18/18, 26/28 target 프레임에서 읽었고, 뒤의 2개는 missing이었다.
  두 프레임을 허가로 보정하지 않았다. 이후 실제 좌회전 허가에서 출발했다.
  이것은 R→황색의 무허가 이력·정차 시험이며, 새 노드가 최초 프레임부터 Y를 본 실제 우회전 장면과
  동일한 시험은 아니다. 최초 Y의 순수/scene 계약은 단위 시험으로 따로 확인했다.
- D 정차 시험 첫 시도는 초기 red 적용의 준비 시간 timer 문제로 Y 시험 전 중단됐다.
  moving과 같은 연속 불일치 timeout으로 수정해 새 폴더에서 재시험했고 원본은 보존했다.

독립 대조: `timed_matrix_261026_v4/D_stopped_yellow_independent_review_v2.json` 및 각 실행의
`_independent_integration_v1.json`. 실행기에서 moving 요청 시간이 누락된 옛 receipt는 수정하지 않았고,
해당 실행의 불변 controller metadata와 실제 stage 기록으로 보완한다. 후속 실행기는 요청 시간도 기록한다.

## 설치·마감

딸깍 설치기에 Validator Python 26개도 source/install exact-byte 검산으로 추가했다.
새 live startup 거부 조건이 아니라 기존 설치 시점 감사의 보완이다. 설치기 mock 회귀 29개가 통과했다.
최종 설치 및 `--check`를 주행 정지/P 정리 뒤 완료했다. 가중치는 바꾸거나 다시 업로드하지 않았다.
설치 후 Native 84개도 다시 통과했고 source/install 검산 불일치는 0개였다.
최종 시험 XML은 `timed_yellow_tests_final_261026_v1/`, 독립 통합 요약은
`timed_yellow_compact_independent_stats_261026_v3.json`이며 위 근거 artifact 루트에 있다.
시험한 plugin과 최종 설치 plugin SHA256은 모두
`421859c533ef98745e2ba02436710fd9088488c0b42ea7ef0f77c0343016baef`로 같다.
시험 중 native UDP를 사용했으므로 자연 주기 재시험 전에는 MORAI를 새로 시작해 현시를 재확인해야 한다.
이 문서만으로 후보 기본 ON이나 PR 병합을 승인하지 않는다.

<a id="stopline-followup"></a>
## 정차 목표와 도색 통과선 분리 후속

이 절은 위 황색 실험 이후의 변경이다. 이전 입력·결과를 소급 수정하지 않는다.
v87 ref_line과 MGeo type530을 비교한 상류 차이는 A0.75/B6.73/C0.75/D7.75m였다.
MGeo 도색선도 공식 판정 프로그램의 인증된 판정선은 아니며 명시적 비교 proxy다.
기존 황색 ON D의 지도선 통과는 R+Y였지만, 원본선 proxy 통과는 약0.924초 뒤이며
native R+Y→R 전환 bracket에 걸렸다. 적색 위반 확정도 황색 통과 확정도 아니다.

### 구현과 기본 선택

- 공유 설정: `base_ws/src/perception/traffic_light_detection/config/traffic_signal_stop_lines.yaml`.
  v87 예상 ref_line·원본 도색 polyline·출처 SHA를 가진다. 지도 자체는 변경하지 않는다.
- 현재 지도 anchor가 정합되면 B/D `additional_stop_margin_m: 0.75`를 적용한다.
  A/C는0이며, 틀린 지도·잘못된 profile은 경고 후 기존 지도 동작으로 돌아간다.
- `signal_stop_line_crossing_reference_enabled:=true`는 **별도 시험 후보**다.
  planner GO_OUT·황색 ETA와 route helper의 미션 전환을 원본 도색선 기준으로 바꾼다.
  동일 인자를 planner와 traffic 양쪽에 사용해야 하며 `morai_planning_stack.launch`는 함께 전달한다.
- 후보는 기본 OFF다. 유효 profile이 있지만 물리 통과선이 OFF/불일치이면 잘못된
  virtual ETA를 쓰지 않도록 황색 override를 허용하지 않는다. GUI의 황색 체크만으로 우회하지 않는다.
- YOLO/CNN 가중치는 그대로다. 운영 direct 경로는 색·화살표를 추론해 추가하지 않는다.
  미관측·모순은 UNKNOWN이고, 기존 색 보정·1.5초 유지·4초 황색 cap은 명시적 legacy에만 남는다.
- 같은 head의 detector 누락만 원본 source age0.2초까지 보존한다. 실제 R/Y는 즉시 갱신하고,
  CNN UNKNOWN/all-OFF·지도/TF 실패·identity 모호성·기한 만료는 보존하지 않는다.
  held stamp로 허가3회 확인을 늘리거나 GO 유효기간을 새로 시작하지 않는다.

### 실제 D 시험에서 확인한 미완료 경계

실제 localization·YOLO/CNN·ROS1 planner/controller·OD_GT 조건으로 시험했다.
첫 실행은 평가용 원본선을 정차 준비 gate에도 사용해 시작 조건을 충족하지 못했고,
정지/P를 확인하여 마감했다. 준비 gate와 관측 crossing geometry를 별도 지정하도록 고쳤다.

다음 실행은 R+Y 정차 유지 후 실제 좌회전 허가에서 출발했지만, 앞축 map X≈111.38m에서
재정지했다. 원본 도색선 X≈118.86m까지 약7.48m 남았으며, 이 구간에 실제 허가가 유지되는
동안에도 관측이 끊겼다. 기록된 지도 사유는 `housing_not_fully_in_image`였다.
종료 후 같은 위치에서 정지 상태의 원본 JPEG를 다시 받아 신호등이 화면 상단에 잘린 것을 확인했다.
후반 native override도 불일치하여 전체 실행은 ERROR이며, 이 실행을 주행 PASS로 세지 않는다.
종료 정지/P는 VERIFIED다. 단발 누락 완충을 늘려 영구 사각을 숨기거나 timeout GO를 넣지 않았다.

따라서 **물리 통과선 분리의 코드·단위 시험 완료와 운영 적용 완료는 다르다.**
현재는 추가 정차여유와 관측 보존/짧은 detector 완충만 운영 경로에 두고,
물리선 후보는 OFF로 남긴다. 기존 virtual GO_OUT의 조기 통과 판정 한계는 아직 남아 있다.
물리선까지 관측이 없는 이동 구간의 승인/취소 정책을 별도로 결정·검증해야 한다.

근거: `artifacts/evaluation/traffic_light_robustness_261025/`의
`stopline_map_visibility_review_261026_v1/`, `stopline_separation_D_261026_v1/`와 `v2/`,
`stopline_separation_trials_261026_v1/D_after_abort_original.jpg`.
새 frozen 시험 준비는 `prepare_signal_stop_line_trials.py`, 실행기의
`--preparation-geometry`는 정차 준비용이고 `--geometry`는 관측 통과선용이다.

### 기본 stop-only 경로의 재검증

물리선 후보를 OFF로 명시하고, B/D 추가 정차여유0.75m·관측 보존·허가3회·황색OFF로
각 구간을 다시 실행했다. B/D 모두 목적지 PASS·충돌0·종료 정지/P VERIFIED였다.
각 약2.025초의 Y/R+Y 정차 구간에 양의 출발 요청은 B0/101, D0/102였다.
정차 중 검출 박스 상단 여유는 B약45px/D약42px였고, 원본 도색선 통과는 각각
native G16/R+좌화살표33의 전후0.25/0.5초 bracket이 일치했다.
허가 확인→양의 제어는 B0.630/D0.691초, 실속도0.2m/s 초과는 B0.806/D0.809초였다.
이는 로컬·무객체·OD_GT 조건의 기술 통계이며 LAN/BEVFusion 부하나 반복 신뢰도 인증이 아니다.

기본 stop-only PASS는 기존 virtual GO_OUT을 유지한 결과이며, 사각 구간의 신호 전환 문제를
해결했다는 뜻이 아니다. 이 한계 때문에 물리선 후보와 황색 override를 기본 승격하지 않았다.
전체 인지 Python276개·Native107개 회귀가 통과했고 source/install 일치 검사도 마감한다.
근거: `stopline_stop_only_{B,D}_261026_v1/`, `stopline_BD_final_safety_review_261026_v2.json`,
`stopline_separation_native_tests_final_261026_v1/`. 이전 실패 원본은 보존했다.

사용자 승인 후 Drive `kcity_map/lanelet_map`과 최신 planning ref도 확인했지만
현재와 같은 v87(size/MD5 일치)이 최신이었으므로 지도 재다운로드·수정·업로드는 하지 않았다.
정차여유는 작은 공유 설정으로 적용하며, 대규모 지도 자산 갱신으로 사각 문제를 덮지 않는다.

<a id="standard-stop-margin"></a>
## 팀 관리 설정으로 통일: PR54 전용 계약 철회

사용자와 팀장님 결정에 따라 정차 위치는 기존 Lanelet 지도와 Autoware의 `traffic_light.stop_margin`으로
통일했다. 위 절의 전용 YAML·MGeo 통과선 분리·anchor 자격 검사·구간별 추가 margin은 이후 제거했다.
해당 소스와 시험은 Git `ef2a7384` 및 위 frozen 실행에 보존되며 현재 운영 코드가 아니다.
이전 시험 결과를 바꾸거나 현재 설정의 실주행 인증으로 재사용하지 않는다.

운영 설정은 `planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml`이다.
기존 값0.0을0.75m로 바꾸었고 normal profile과 독립 traffic-light 모듈 기본도 동기화했다.
A/B/C/D 모두 같은0.75m이며, 정차 목표는 지도선에서 앞범퍼 여유0.75m를 뺀 위치다.
PR54 정상 v87 조건과 비교하면 B/D의 정차 여유는 같고 A/C는0.75m 더 일찍 정차한다.
지도 ref_line의 기하 좌표·신호등 관측·센서·모델 가중치는 바꾸지 않는다.

인지의 관측 보존·detector-only 원본age0.2초 완충·허가3회 확인·황색 기본OFF는 유지했다.
전용 계약에 의존하던 추가 황색 차단도 해당 코드와 함께 제거되므로, 명시적 황색ON 시험은
기존 후보 로직이 된다. 지도선과 공식 판정선 차이·화각 이탈을 해결하기 전 운영ON을 권하지 않는다.
원본 도색선으로 GO_OUT을 바꾸어 교착된 실험도 현재 기능으로 남기지 않는다.

전용 계약 시험을 제외한 기존 Native84개와 신규 공통 margin 시험2개(각 A/B/C/D 루프),
인지 Python269개, 설치기30개가 통과했다. 실제 launch 해석에서도 `stop_margin=0.75`,
허가3회·황색OFF 및 전용 계약 parameter 부재를 확인했다. 새 MORAI 주행 인증은 아니며,
이 변경에서는 기존 실행을 시작하거나 팀장님 PC의 설치 상태를 추정하지 않았다.
설치기는 CMake install에 남을 수 있는 PR54 전용 YAML도 과거 원본 SHA가 맞을 때만 지운다.
사용자가 별도로 수정한 설치 파일은 보존하고 새 코드는 그 파일을 읽지 않는다.
Native 근거는 `artifacts/evaluation/traffic_light_robustness_261025/standard_stop_margin_tests_261026_v1/`다.

<a id="map-stopline-candidate"></a>
## 후속 결정: 공통 margin 원복·지도 후보 검증 대기

위 공통0.75m는 Git `d86fd564`/PR55 초안의 당시 설정이며 최종 운영 결정이 아니다.
사용자는 팀장님이 관리하던 방식인 **stop_margin=0.0 + Lanelet ref_line 위치 조정**을 선택했다.
4개 기존 모듈 설정을0.0으로 복원하며 PR54 전용 YAML/분리 코드 제거는 유지한다.

v87 원본 SHA `70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575`는 보존한다.
v88 후보 SHA는 `3cfb4dde4dbbbb44fc86f4e45c747576565793db48b04a631a61ab34af265d44`다.
A/C만 각0.75m 상류 이동하고 B/D는 변경하지 않는다. 변경 노드는92992/92993/93034/93035뿐이며,
이 노드의 좌표 이외 바이트·도로 경계·route topology·신호 ID·모델·센서는 그대로다.
기존 MGeo 도색선 대비 상류 차이 A약0.75→1.50m/C약0.75→1.50m, B약6.73m/D약7.75m 유지다.
이는 검증할 초기 후보이지 B/D만큼6~8m 일괄 이동하는 확정값이 아니다.

생성 도구는 `scripts/map/shift_signal_stop_lines.py`이며 배포 지도를 덮어쓰지 않는다.
검증 실행기는 `--map-file`로 실제 planning/map projector와 신호 route geometry에 같은 후보를
명시 전달하고 SHA를 기록한다. 인자를 생략하면 기존 v87이며 production 기본 지도를 몰래 바꾸지 않는다.
새 시험 입력은 `scripts/evaluation/prepare_signal_map_candidate_trials.py`로 만들고 전방 JPEG도 기록한다.
원본 frozen 실행과 증거는 수정하지 않는다.

현재는 **정적 검사·회귀 시험 단계**다. 자연 신호 복원을 위한 MORAI 재시작 뒤 실제 localization+
OD_GT+ROS1 planner/PP·PI로 정차·재출발·최종정지/P·신호 가시성을 확인하기 전 배포/업로드하지 않는다.
기존 강제 현시가 남은 프로세스의 결과를 자연 신호 검증으로 간주하지 않는다.
통과 판정/GO_OUT은 여전히 지도 ref_line을 사용하므로 지도선을 당기면 해당 시점도 당겨진다.
공식 도색선/벌점 판정선과 같은 의미가 아니며 황색 통과는 계속 기본OFF다.

후속 준비 검증: Python318개·Native86개(8개 XML, 실패/오류0) 통과, 재빌드·설치기`--check` 통과.
Autoware Lanelet 확장 등록 후 후보를 실제 파서로 읽어 lanelet1302개/regulatory78개를 확인했다.
확장을 등록하지 않은 순수 Python 파서는 기존 roundabout/crosswalk 타입을 몰라 실패하므로 이를
지도 자체의 새 오류로 해석하지 않는다. launch 해석에서도 후보 경로/SHA와 stop_margin0.0을 확인했다.
넓은 `check_runtime_map_version.py --installed` 검사는 기존 평가 도구·사용자 workspace·오래된 설치본의
v85 참조 때문에 미통과다. 이 작업에서 해당 frozen/별도 범위를 일괄 수정하지 않았고,
전체 설치공간이 최신이라는 주장 대신 위 시험 대상 launch의 실제 해석과 설치 일치만 확인했다.
실주행은 아직0회이며 지도 후보는 로컬 생성본일 뿐 Git/Drive 운영 배포 자산으로 승격하지 않았다.

<a id="map-stopline-live-validation"></a>
## 후속 실주행: 자연 신호 개별4회 + 연속 풀코스1회

위 '실주행0회'는 준비 완료 시점의 상태다. 사용자 MORAI 재시작 뒤 새 프로세스에서
v88/stop_margin0.0으로 아래5회를 실시했다. native는 GET만 사용했고 강제 현시·가짜 lamp 입력은 없었다.
실제 localization, OD_GT, YOLO/CNN v5, ROS1 planning 및 PP·PI 제어기를 사용했다.
BEVFusion 동시 부하·LAN·팀장님4090 조건을 시험한 것은 아니다.

| 실행 | 주행 시간 | 정차 검증 | 결과 |
|---|---:|---|---|
| A 개별 | 63.60초 | 약47.38초 R 대기 후 G 출발 | PASS·충돌0·최종정지/P 확인 |
| B 개별 | 67.77초 | 약47.28초 R 대기 후 G 출발 | 동일 |
| C 개별 | 94.77초 | 약76.90초 R 대기 후 G+좌화살표 출발 | 동일 |
| D 개별 | 11.90초 | 처음부터 허가여서 무정차; 정차 검증으로 세지 않음 | 동일 |
| 연속 풀코스 | 208.62초 | D에서11.00초 정차, 직진G/Y 유지→R+좌화살표 출발 | 동일 |

5회 모두 지도선 앞차축 통과는 exact native 양쪽 bracket0.25초 조건으로 허가였다.
별도 MGeo 원래 도색선 proxy에서도 개별4회 및 연속A/B/C/D 모두 허가였다.
C는 두 physical head 모두 같은 상태를 확인했다. native 샘플 사이 전환·공식 벌점 판정까지 보증하지 않는다.

정차 중 원본 JPEG를 직접 확인했으며 A/C는 신호등이 화면 안에 충분히 남았다.
B/D도 보였지만 화면 상단 여유가 작다. 서로 다른 기록 runtime frame 중 matched/전체는
A381/392, B352/390, C629/631, 연속D86/92였다. 이는 정차 중 기록 프레임 비율이지
독립 test 정확도나 영상 전체 recall이 아니다. 누락을 GO로 풀지 않았고 각 허가 재관측 후 출발했다.
정차 중 localization 앞범퍼의 지도선 기준 median gap은 A−0.042/B−0.043/C−0.141/D−0.037m였다.
0 margin 목표의 작은 초과이며 앞차축·공식 도색선 초과와 같은 뜻이 아니다. 차량 자세·측위가 달라지면
B/D 가시성 여유를 다시 확인해야 한다. 이 결과만으로 추가6~8m 이동이나 긴 관측유지를 도입하지 않는다.

근거는 `artifacts/evaluation/traffic_light_robustness_261025/map_v88_{A,B,C,D,full}_natural_261026_v1/`,
각 실행의 `planner_logs/validation/traffic_light_compliance_*_map_candidate*` 및
`map_v88_*_stop_analysis_261026_v*`다. 원본 bag·JPEG·native 관측·안전 receipt를 보존했다.
`scripts/evaluation/analyze_signal_map_stop_trial.py`는 닫힌 bag에서 정차를 재고 원본 JPEG를 추출하며
`--paint-geometry`로 런타임과 독립된 원래 도색선 통과를 추가 대조한다. 원본 입력은 변경하지 않는다.

v88 파일은 승인된 팀 Drive의 `kcity_map/lanelet_map/lanelet2_map_v88.osm`에 별도 공유한다.
v87을 덮어쓰지 않으며 Git/launch 운영 기본은 아직v87이다. PR55는 이 결과를 반영한 draft로 유지한다.
팀 기본 전환에는 route/map SHA 계약과 각 launch의 지도 선택을 함께 갱신해야 하며,
지도 파일 하나만 바꾼 채 팀 환경에 이미 적용됐다고 설명하지 않는다.
업로드 후 원격 MD5 `8e74ca03cc6b90452e23e38f48b6a68e`와 로컬이 일치함을 확인했다.
후속 분석기의 원본 보존/active bag 거부 시험을 포함한 최종 Python321개가 통과했다.
후보 manifest는 `configs/map/kcity_v88_stopline_candidate.yaml`이며 기본 전환 미실시를 명시한다.

<a id="map-stopline-visibility-v89"></a>
## 후속 선정: A/C 유지·B/D 추가2m, 화면 상단 여유 확보

사용자가 v88 정차 사진을 보고 B/D 여유 부족을 지적했고, 임의0.75m 추가로 고정하지 말고
A/B/C/D 전체를 다시 판단하도록 요청했다. v88은 '재출발 성공' 근거였지 충분한 화각 여유의 보장이 아니었다.
v87에서 **A/C+0.75m 유지·B/D+2m**인 v89를 별도로 생성했다. `stop_margin=0.0`, 신호 정책·모델은 그대로다.
8개 exclusive ref_line 끝점 좌표 외에는 원본 바이트를 보존하며 차로·연결·고도·속도·신호ID를 바꾸지 않았다.
원래 MGeo 도색선 대비 지도선 상류 차이는 A약1.50/B약8.73/C약1.50/D약9.75m다.

이번에는 1280×720 원본의 actual detector housing bbox 상단 여유를 source stamp로 localization에
대조했다. 지도선 상류20m부터 선 후0.5m까지·대상 차로 부근에서 감속/정차 표본을 검사했다.
같은 날 이전v88 B/D bag도 같은 분석기로 다시 읽었고, 아래 수치는 검출 박스가 있는 기록 표본 기준이다.
없는 박스는 화면 밖이었다고 단정하지 않으며 누락 건수와 원본 사진을 따로 보존한다.

| 구간 | v87 대비 이동 | 감속·정차 표본 최소 상단 여유 | 정차 중 중앙값 | 선정 |
|---|---:|---:|---:|---|
| A | +0.75m | 80.2px | 91.6px | v88 위치 유지 |
| B | +2.00m | 73.4px | 77.5px | v88의 최소9.4px/정차17.9px에서 개선 |
| C | +0.75m | 84.9px | 96.4px | v88 위치 유지 |
| D | +2.00m | 64.0px | 73.4px | v88의 최소6.0px/정차16.7px에서 개선 |

개발상 최소60px 수준의 여유를 목표로 삼았고, 이는 공식 기준이나 모든 차량 자세에 대한 보장이 아니다.
B 개별은67.47초 PASS, R에서46.86초 정차 후 G 출발, 충돌0·최종정지/P 확인이었다.
새 지도 연속 풀코스도209.46초 PASS·충돌0·최종정지/P 확인이었다. 연속B는 G 통과라 정차 검증은 개별 결과를 쓴다.
연속A/C/D는 각각28.70/7.80/10.76초 실제 정차했다. D는 직진G/Y에 대기하다 R+좌화살표로 출발했다.
새 지도선과 독립 대조한 원래 MGeo 도색선의 앞차축 통과 모두 네 구간에서 native 양쪽0.25초 bracket 허가였다.
신호 강제 SET 없이 자연 현시·실 localization+OD_GT+YOLO/CNN+ROS1 planner/PP·PI로 시험했다.

정차 원본과 최저 여유 시점 원본 JPEG를 모두 직접 확인했다. B/D를 더 크게 당길 근거는 이번 표본에선 없었고,
A/C 역시 추가 이동하지 않았다. 유일한 최적값을 탐색한 시험이 아니라 충분한 가시성과 불필요한 후퇴를 절충한 선정이다.
박스 경계는 픽셀 GT가 아니며 누락도 남는다: B 정차 matched368/390, 연속D87/90 기록 프레임이었다.
무관측을 GO로 풀거나 긴 GO hold를 추가하지 않았다. 더 위로 움직이는 차체·측위 편차·LAN/4090/BEVFusion 동시 부하는 미검증이다.
지도선을 당기면 기존 GO_OUT도 당겨진다는 경계는 그대로이며, stop 위치 보강이 공식 도색선 판단 분리를 해결한 것은 아니다.
원래 도색선까지 출발 거리도2m 늘므로 황색은 계속 기본OFF이고, 임의의 추가 후퇴를 계속 누적하지 않는다.

근거: `map_v89_B_natural_261026_v1`, `map_v89_full_natural_261026_v1` 및
`map_v89_{B,full_A,full_B,full_C,full_D}_clearance_analysis_261026_v1`(기존 artifacts root 아래).
분석기는 검출 박스 여유와 정차/접근 원본 JPEG를 구분해 저장하고 active bag·증거 덮어쓰기를 거부한다.
지도/분석 관련 회귀를 포함한 Python323개가 통과했다. 실제 파서도 lanelet1302/regulatory78개를 읽었다.
선정 manifest는 `configs/map/kcity_v89_stopline_candidate.yaml`이다. **배포 기본은 아직v87**이며
기본 전환과 route/map SHA 계약의 일괄 전환·PR 병합은 이번 선정과 구별한다. 이전v88 및 원본 증거는 변경하지 않는다.
팀 Drive `kcity_map/lanelet_map/lanelet2_map_v89.osm`에 별도 업로드했다. v87/v88은 덮어쓰지 않았다.

<a id="map-stopline-visibility-v91"></a>
## 최종 미세 조정: A/C 유지·B+3m·D+2.75m 후보

사용자는 네 구간 모두를 다시 판단하되80px 초중반에 억지로 일치시킬 필요는 없다고 했다.
v90(v87 대비 A/C+0.75m, B+2.5m, D+2.75m)을 먼저 시험했다. B 단독은 최소86.2px였지만
같은 위치의A→D 연속 제동 표본은72.3px까지 내려갔다. 단독 수치만 인용하지 않고 B만0.5m 더 당긴
**v91: A/C+0.75m·B+3.0m·D+2.75m**를 선정했다. D는 v90에서 최소90.1px로 여유가 있어 유지했다.
A/C도 이미 충분하여 유지했다. 원본v87 대비8개 끝점 좌표 외 바이트는 그대로이며 원본/이전 후보는 수정하지 않았다.
MGeo 원래 도색선 대비 상류 거리는 A약1.50/B약9.73/C약1.50/D약10.50m다.

| v91 검증 표본 | 감속·정차 관측 최소 top | 정차 중 median | 실제 정차 |
|---|---:|---:|---:|
| A, 연속 시험 | 85.5px | 93.4px | 26.86초 |
| B, 별도 개별 시험 | 98.1px | 103.3px | 41.40초 |
| C, 연속 시험 | 82.6px | 98.0px | 7.92초 |
| D, 연속 시험 | 92.9px | 97.2px | 10.34초 |

이는 각 표본의 detector bbox가 있는 기록 프레임 기준이며 보편적인 최소값/픽셀GT가 아니다.
최종 연속 시험에서B는 녹색 통과였으므로 B 정차 값은 개별 실행에서 가져왔다. 그 조건 차이를 숨기지 않는다.
B98px/D93px가80보다 크다고 다시 앞으로 옮기지는 않았다. 정차 원본을 직접 보고 가시성이 확보됨을 확인했다.
누락은 여전히 남는다: B 정차174/200, A212/224, C63/63, D87/87 기록 프레임이 matched였다.
이 비율은 새 독립 분류정확도나 전체 이미지 recall이 아니며 원본age0.2초 hold/허가3회/황색OFF는 유지한다.

v91 B 개별61.14초, A→B→C→D 연속121.98초 모두 PASS·충돌0·최종 정지/P VERIFIED였다.
실 localization+OD_GT+YOLO/CNN+실제 ROS1 planner/PP·PI, 자연 현시/native GET만 사용했다.
새 지도선 및 별도 MGeo 도색선 앞차축 통과는 네 미션 모두 native 양쪽0.25초 bracket에서 허가였다.
D는 직진G/Y에는 정차하고 R+좌화살표에 출발했다. C의 두 head는 같은 상태로 확인했다.
기본 정차 margin은0.0이며 정책·모델·카메라·센서 설정은 바꾸지 않았다.

### 중단을 제외하지 않고 구분한 범위

- v90 첫B 실행은 Ego 수신 age1.168초 초과 후 종료 정지도 검증하지 못해 ERROR였다.
  `map_v90_B_natural_261026_v1`을 보존한다. 이후 정지 장면·네트워크 재로드로 fresh Ego/영상과 P를
  확인한 기록은 `map_v90_recovery_261026_v1`이다. 이 실패를 통과로 바꾸거나 safety 임계값을 완화하지 않았다.
- v90 B 재시험70.31초와 A→D 연속128.36초는 PASS했다.
- v90 전체 코스는 네 신호를 통과한 뒤 남쪽 복귀 구간에서 `Ego is out of route`와
  `planning trajectory is too far from ego`를 반복하며 정지했다. 에이전트가 정상 중단하고 최종 P를 확인했다.
  `map_v90_full_natural_261026_v1`의 MANUAL_ABORT와 원본 bag를 보존한다. 전체 코스 PASS가 아니다.
- 당시 로컬 load20대·swap 포화와 수신 지연이 관찰됐지만 제어기 latency 단독 원인은 확정하지 않았다.
  사용자도 MORAI 동시 부하를 의심했다. 지도 변경이 원인이라고도, 무관하다고도 인증하지 않는다.
- 신호 검증은 기존 full 경로의 D 출구 waypoint(133.972899406,668.836010783)를 그대로 종료 goal로
  쓰고 그 전 waypoint를 보존한 별도 corridor로 분리했다. 원래 full config는 수정하지 않았다.
  새 준비 함수는 원래 waypoint에서 해당 goal을 유일하게 찾지 못하면 거부하며 실제 planner/controller를 유지한다.
  따라서 v91 결과는 **4구간 연속 검증**이지 전체 코스/실대회 LAN/4090/BEVFusion 동시 부하 인증이 아니다.

최종 근거는 `map_v91_B_natural_261026_v1`, `map_v91_corridor_natural_261026_v1`,
`map_v91_B_clearance_analysis_261026_v1` 및 `map_v91_corridor_{A,B,C,D}_clearance_analysis_261026_v1`이다.
모두 기존 traffic_light_robustness artifacts root 아래이며 각 실행의 안전 receipt·원본 bag/JPEG를 보존했다.
선정 manifest는 `configs/map/kcity_v91_stopline_candidate.yaml`이다.
지도선 상류 이동은 여전히 기존 GO_OUT 시점도 앞당긴다. 정차 시야 보강이 공식 도색선 판정 분리를 해결한 것은 아니다.
**운영 기본v87과 PR55 draft는 유지**한다. 기본 전환에는 route/map SHA 및 launch 선택 동기화가 필요하다.
최종 Python327개·문서 검사·설치기`--check`가 통과했다. 회귀는 주행 종료 후 수행했다.
v91은 팀 Drive에 별도 업로드했고 원격/로컬 MD5 `09832c1fdcde2cf770937d3c4d34bbc5` 일치를 확인했다.

<a id="v91-latency-retest"></a>
## A/B/C/D 독립 재시험: 지연과 구간 고유 현상 구분

사용자는 CPU 부하라고 단정하기 전에 B/D에서만 반복되는지 확인하도록 요청했다.
선정한v91/모델/정책/goal/안전 기준을 그대로 두고 A→B→C→D 각1회 실제 validator로 다시 실행했다.
운영 기본인 C++ 상세DEBUG OFF, 모니터1Hz(no GPU polling), 원본 JPEG 전체 bag 제외 조건이다.
카메라 입력·YOLO/CNN·observer 교차 원본 사진·native GET은 유지했다. 각 trial의 설정/수신/안전 결과와
같은 PC의 CPU·MORAI process CPU·메모리·UDP counter를 같은 시각으로 기록했다.
무거운 테스트/빌드/업로드를 주행과 겹치지 않았다. 이 기록은 과거DEBUG ON과의 통제된A/B는 아니다.

| 구간 | 결과/시간 | 전체CPU 평균 | trajectory header→bag p95 | ctrl_cmd 최대 bag 수신 간격 |
|---|---|---:|---:|---:|
| A | PASS / 61.99초 | 72.3% | 46.9ms | 28.0ms |
| B | PASS / 67.04초 | 71.0% | 41.3ms | 27.7ms |
| C | PASS / 95.71초 | 71.8% | 42.9ms | 26.6ms |
| D | PASS / 12.30초 | 69.3% | 40.7ms | 22.3ms |

모두 충돌0·최종정지/P VERIFIED·선정 지도선의 앞차축 허가 통과 native0.25초 bracket 확인이었다.
bag 수신 기준 Ego 약45.2~46.9Hz, 측위/제어용Smooth 약50Hz, 최종trajectory 약10Hz, ctrl_cmd 약50Hz였다.
Ego·두 측위·trajectory·ctrl_cmd의0.5초 초과 수신 공백은0개였다. host UDP InErrors/RcvbufErrors/
SndbufErrors 증분도 각 주행 창에서0이었다. 이 counter는 전체 namespace이며 센서 packet 손실 보장이 아니다.
동작 중 motion_velocity_planner 공개 처리시간 Total의 p95는 A2.77/B3.42/C2.73/D3.31ms였다.
초기 map/RouteHandler 준비 표본의 약2.5초를 주행 중 처리시간으로 섞지 않았다.
신호등 callback→출력 p95는 A32.2/B30.7/C31.9/D29.5ms였다. 다른 planner/controller의 순수 계산시간은
별도 공개 계측이 없어 이 수치로 대신하지 않는다. `/control/command/control_cmd`는 메시지0이며,
이번 실제 제어 발행은 `/ctrl_cmd`로 확인했다.

**이번에는 B/D만의 멈춤·재출발 실패·큰 지연은 미재현**이었다. 그러나 구간당1회이며 D는 처음부터
좌회전 허가에 통과해 대기 없는 짧은 시험이었다. 이전D 정차 성공은 앞 절의 별도 연속 시험 근거다.
DEBUG/녹화 조건·시각·신호 phase·시스템 부하도 과거와 같지 않으므로 CPU 원인 확정이나
지도/정책 문제의 완전 배제를 주장하지 않는다. 전체 코스 남쪽 복귀는 이번에 다시 달리지 않았다.

header age는 송신 stamp 의미/전달/queue를 포함하며 순수 계산시간이 아니다. Ego stamp는 UDP receiver
측이므로 MORAI 물리 갱신 지연은 직접 재지 못한다. `/ctrl_cmd`에는 header가 없어 발행 cadence만
계산했으며 실제 UDP 송신→시뮬레이터 조향/가감속 적용 latency는 미측정이다.
기록된 controller 설정은 receipt trajectory timeout0.5초, source trajectory timeout10초,
state source/receipt timeout0.5초였다. 분석 JSON의 header age>500ms 집계를 곧바로 controller timeout으로
해석하지 않는다. 설정을 바꾸거나 안전 guard를 완화하지 않았다.

근거: `artifacts/evaluation/traffic_light_robustness_261025/v91_abcd_latency_retest_261026_v1/`의
`host.jsonl`, `events.jsonl`, `receipt.json`, 구간별 observer/원본 case bag 및 `comparison_v2.json`이다.
같은 폴더의v1 비교 JSON도 보존하며 v2는 공개 processing 진단의 실제 숫자를 추가했다.
실행 도구는 `scripts/evaluation/retest_signal_latency.py`, 읽기 전용 분석기는
`scripts/evaluation/analyze_signal_latency_retest.py`다. 시간 비교는 controller 시작~종료 창만 사용한다.
추가 도구·기존 host sampler 대상17개 단위 테스트 통과. 지도/정책/운영 기본값/팀 배포 상태는 바꾸지 않았다.

<a id="continuous-braking-comparison"></a>
## 연속 주행 제동 대조: B/D 고유 현상이 아니었음

사용자가 '연속 주행 제동에서 여유 감소/재정지가 B/D에만 반복됐는지' 확인하라고 지적했다.
새 주행·지도 변경 없이 v89 full, v90 corridor, v91 corridor 원본 bag와 진단을 같은 기준으로 다시 읽었다.
각 버전은 정차 위치·부하·신호 위상이 달라 통제된 인과 시험은 아니다. 특히 B는 세 연속 기록 중
정차한 것이 v90 한 번뿐이고 나머지는 녹색 통과였다. 이를 B에서 계속 반복된 증거로 쓸 수 없다.

최종 정차 위치0.3m 이내·속도0.3m/s 미만에서 상단 여유의 일시적인 감소를 비교했다.
기준은 정차2초 이후~끝0.5초 전의 median이며 표본이 없으면 N/A로 남겼다.

| 구간 | 같은 위치 부근의 median 대비 일시 감소 | v90 정차→짧은 이동→재정지 |
|---|---|---|
| A | 약8.0~11.5px | 있음, native Ego 순이동 약16.8cm |
| B | v90 대기 창이2.24/2.46초로 짧아 안정 기준 N/A | 있음, 약17.5cm |
| C | 약11.5~15.5px | 해당v90은 녹색 통과; 다른 두 기록은 한 번씩 정차 |
| D | 약2.3~9.4px | 있음, 약7.9cm |

따라서 화면 여유 감소는 A/C에도 있었고, 짧은 재정지는 A에도 있었다. v90 이동 구간의 측위 기록 간격은
약20~21ms, 최대 관측 속도는 A0.77/B0.74/D0.43m/s였으며 native 위치 변화도 확인했다.
단순 로그 공백을 반복 제동으로 오인한 경우는 아니었다. 다만 8초 창의 brake>0.05 pulse 개수는
가속/곡선/제어 잔진동을 포함하므로 그 자체를 고장 횟수로 해석하지 않는다.

더 직접적인 단서는 같은 시각의 신호 공급과 목표속도였다. v90 B/D 재정지 주변의 정확한 native ID
조회 샘플은 각각 G(16)23개, R+좌화살표(33)22개로 허가 상태였지만, 영상 처리 출력은
허가→missing/UNKNOWN→허가를 반복했다. 선택 사유에 `no_visible_route_head`,
`localization_uncertainty_exceeds_bound`, `map_packet_missing_or_stale`가 함께 있었다.
이는 실제 영상에서 신호등이 잘렸다는 증거가 아니며 CNN 오분류와도 구분해야 한다.
A도 같은 종류의 선택 사유와 판독 중단이 있었다(관측 창 앞쪽은 R→G 자연 전환 포함).
planner의 가까운 trajectory 목표속도는 A0→0.25→0, B0→약0.47→0, D0→0.25→0m/s로 바뀌었다.
이 값은 posterior nearest-point 진단이며 controller가 실제 소비한 index의 직접 계측은 아니다.

D 실제 planner 로그에서도 UNKNOWN 후 허가 관측 횟수가1로 다시 시작하고,
이후 source_age0.539~0.748초 stale로 정지 요구가 유지되는 것을 확인했다. 일부 변화는 source stamp와
planner receipt 시점이 달라 일대일 원인 시간을 엄밀히 확정할 수 없지만, 단순 제동거리 증가보다
**유효 허가 신호의 연속성 상실→정지 요구 재발**이 더 직접적인 확인 경로다.
왜 지도/측위 자격과 freshness가 흔들렸는지는 추가 점검 대상이며 CPU·물리 피치·정지선 위치 중
하나가 단독 원인이라고 확정하지 않는다. IMU sensor pitch 변동도 A/C에 있었으나 camera pitch와 동일시하지 않는다.

따라서 B의72px 짧은 재정지 표본을 안정 정차의 최적 위치 근거처럼 취급한 것은 불충분했다.
정지선을 더 당기는 작업보다 선택 중단/freshness/측위 불확실성 경로를 먼저 확인해야 한다.
이번 분석에서는 지도·모델·판단/제어·허가3회·UNKNOWN 정지 정책을 수정하지 않았다.
현재 v91은 시야 확보 후보이지 이 연속성 문제까지 해결한 버전이 아니다.

분석 도구: `scripts/evaluation/analyze_signal_braking_comparison.py`.
근거: `planner_logs/analysis/signal_braking_comparison_261026_v1`, `_v2`, `_v3`의 JSON.
v3은 native 위치·신호 조회·선택 변경·목표속도를 추가하며 이전 분석/원본은 보존한다.

<a id="common-stopline-standard"></a>
## A/B/C/D 공통 정차 시야 기준으로 조정 마감

사용자 목적은 한 구간씩 계속 당기는 것이 아니라 **동일 기준으로 네 정차선을 맞추는 것**이다.
추가 주행·지도 이동 대신 기존 v91 기록을 같은 규칙으로 재검산했다.
각 정지선에서 같은 거리만큼 이동하는 방식은 신호등 설치 위치·높이가 달라 같은 시야를 만들지 못한다.
따라서 차량 앞범퍼 기준과 영상 내 실제 선택 housing의 여유를 공통으로 사용한다.

- 전방 원본1280×720, 앞범퍼 `base_link +3.845m`, `stop_margin: 0.0`을 공통 사용.
- 기존 분석기의 속도0.15m/s 미만·지도 정차선 부근 정차창 중3초 이상인 창만 비교.
- 정차창에서 검출된 housing의 상단 여유 median90~105px, 관측 최소80px를 실용적 수용 범위로 정함.
  주행 자세 변화로 이전 표본에서 약8~15.5px 감소한 점을 고려해, 안정 화면을80px에 딱 맞추지 않는다.
- 접근 중 검출된 표본의 최소 여유도80px 이상인지 확인. 정차창의 앞범퍼→지도선 오차는±0.5m 이내인지 확인.
  이는 이번 후보용 수용 기준이지 대회 공식 허용 오차나 미래 모든 제동의 보장값은 아니다.

| 구간 | 정차창 median / 최소 | 접근 관측 최소 | 공통 기하 기준 |
|---|---:|---:|---|
| A |93.4 /85.5px|85.5px|충족|
| B |103.3 /98.1px|98.1px|충족|
| C |98.0 /82.6px|82.6px|충족|
| D |97.2 /95.0px|92.9px|충족|

**v91의 좌표를 더 바꾸지 않고 정차선 시야 조정은 여기서 마감한다.** 실제 도로의 설치 형상이 달라
모든 순간의 픽셀 수가 같아야 한다는 뜻은 아니다. A/C/D는 동일 corridor, B는 개별 주행의 정차 기록이다.
같은 측정 규칙을 썼지만 같은 부하·시각·신호 위상의 통제 시험은 아니며, 이미 본 표본을 이용한
사후 engineering 수용 판단이다. 미검출 프레임·다른 날씨·모든 자세까지80px를 보장하지 않는다.
선택 housing은 C016/C018 간 바뀔 수 있어 고정 단일 physical head의 픽셀 GT와도 구분한다.

### 정지선과 분리한 신호 선택 끊김

v90 재정지 주변(첫 정차창 끝2초 전~다음 정차창 시작2초 후)의 실제 map ROI 패킷을 추가 확인했다.
`no_visible_route_head`의 구체 사유는 A11/B7/D7개 모두 해당 mission head의
`projection_uncertainty_exceeds_bound`였다. 화면 밖으로 잘린 것으로 단정했던 해석과 다르다.
대표 패킷은 필요한 수평 탐색 여유가 A101.1/B95.8/D96.1px로 상한90px를 넘었고,
수직 여유는143.4/146.0/139.6px로 상한200px 이하였다. 같은 창에 별도의 측위 불확실성 초과도
A9/B6/D3개 있었다. 양상은 B/D에만 한정되지 않는다.

이것은 **지도 연결 불확실성 guard가 선택을 거부했다는 직접 증거**이지, 실제 측위 오차나 CPU 지연이
그만큼 컸다는 증거가 아니다. 공분산이 순간적으로 커지는 원인과 최신 패킷 지연은 아직 해결하지 않았다.
이번에는 지도/안전 기준을 더 벌리거나 UNKNOWN을 GO로 바꾸지 않았다.
기하 수용과 신호 공급 연속성 해결을 별개로 기록한다. 운영 기본v87·PR55 draft 상태도 그대로다.

공통 기준과 수치는 `configs/map/kcity_v91_stopline_candidate.yaml`에 둔다.
재검산 도구 `scripts/evaluation/audit_signal_stopline_standard.py`와 회귀 테스트를 추가했다.
결과는 `planner_logs/analysis/signal_stopline_common_standard_261026_v2/measurements.json`이다.
선택 거부 세부 내역은 기존 분석기에 추가했으며
`planner_logs/analysis/signal_braking_comparison_261026_v4/measurements.json`에 보존했다.
원본 bag·JPEG·이전 결과·지도 좌표·운영 프로세스는 변경하지 않았다.

<a id="pixel-alignment-v93"></a>
## 사용자 요청의 약5px 이내 정렬: v93 선정

앞 절의90~105px 사후 수용 범위 대신, 사용자가 **네 구간 안정 정차 중앙값을 약5px 이내**로
맞추도록 명시했다. B의 마지막0.5m 추가 후퇴를 필수로 보았던 근거가 약했던 점도 함께 반영했다.
기존 신호 판단·허가3회·freshness·UNKNOWN 정지·제어기·카메라는 바꾸지 않고 지도 후보만 수정했다.

동일1280×720 입력, 앞범퍼`base_link +3.845m`, `stop_margin: 0.0`이다.
각3초 이상 정차창에서 시작2초~끝0.5초를 제외하고 최소5개 원본 검출 관측의 상단 중앙값을 계산한다.
고정 physical head A077/B091/C016/D052만 비교하며 C018과 섞지 않는다.
짧은 재정지·녹색 무정차·검출 없는 창은 통과가 아니라 자료 부족이다. 반복 유효 정차창이 있으면
그 중앙값 전체의 범위를 검사해, 서로 다른 정차를 평균하여 불일치를 감추지 않는다.

v92는 v87 대비 A0.75/B2.60/C0.55/D2.60m로 시작했다. 원본 안정창의 깊이와 핀홀 근사로
초기 이동량을 계산했지만 이를 실측으로 간주하지 않았다. 실제 개별 정차는 A94.62/B93.03/C98.69px로
차이5.66px여서 C만0.20m 더 하류 이동한 **v93: A0.75/B2.60/C0.35/D2.60m**로 재시험했다.
v91 대비 A유지·B0.40m·C0.40m·D0.15m 덜 일찍 서는 위치다. Exclusive stop-line8개 node만 바뀌며
도로 경계·연결·ID·고도·속도 속성과 원본v87은 그대로다.

| 구간 | v87 대비 상류 이동 | 안정 정차 중앙값 | 안정 관측 수 | 접근 관측 최소 | 정차 시간 |
|---|---:|---:|---:|---:|---:|
| A |0.75m|94.62px|374|88.18px|47.76초|
| B |2.60m|93.03px|371|86.65px|47.88초|
| C |0.35m|94.85px|630|87.94px|77.38초|
| D |2.60m|92.52px|66|89.30px|10.34초|

**안정 중앙값 최대−최소=2.337px로 요청한5px 이내다. 이 위치에서 조정을 마감한다.**
A/B는v92 개별 시험이며 v93에서도 해당 미션의 전체 stop-line geometry와frame이 정확히 같음을
원본 geometry SHA와 group 전체 비교로 확인해 재사용했다. C는v93 개별, D는v93 A→D corridor다.
따라서 이 표를 '한 번의 같은 지도 연속 주행에서 네 구간 모두 정차했다'고 읽으면 안 된다.
다른 환경·접근 자세·제어 오차·제동 순간마다5px를 보장하거나 별도 pixel GT를 얻은 것도 아니다.

v92 A/B/C/D, v93 C/D, v93 corridor 총7회는 모두 PASS·충돌0·최종 정지/P VERIFIED였다.
두 D 집중 시험은 허가 도착으로 정차하지 않아 정차 검증에서 제외했다. 두 번째는 시험 준비의
P 확인 hold만1→6초로 늘려 자연 현시 도착을 달리했으며, 신호 프로그램을 변경하거나 native SET을
쓰지 않았다. 최종 D는 정상1초 hold의 기존 실제 A→B→C→D 경로에서 정차를 확보했다.
마지막 corridor는 기존 D exit waypoint에서 끝난 것이지 남쪽 복귀까지 full-course PASS가 아니다.
시험 후 ROS graph는`/rosout`만 남고, MORAI는 최종 정지/P 확인 후 그대로 두었다.

기존4구간 경량 기록이 각각20~89MiB였으므로, 새 실행 사본에는 최소1GiB 디스크 reserve를
명시하고 원본 카메라 전체 bag는 제외했다. 실제 controller/안전 guard를 완화한 것이 아니며,
원본 detector 진단·측위·신호·제어·native GET·교차 시점 JPEG 및 종료 안전 증거는 기록했다.
정차 중 원본 JPEG 전체는 없으므로 이번 표는 detector box 진단이고 새 정차 이미지 검사는 아니다.
원본 모델과 SHA 동일한 미사용 임시 checkpoint 두 복사본만 약1GiB 정리했고 원본 가중치는 보존했다.

현재 후보 manifest는 `configs/map/kcity_v93_stopline_candidate.yaml`이다.
OSM SHA256은 `363f107d6a1460af6a71c1dc0c129f73f7a1da049686f5df159bf557cd3133e3`,
MD5는 `1b605c43f0b2225649106c3ee6a5b9e0`이다. 운영 기본v87·PR55 draft는 유지한다.
신호 선택 불확실성에 따른 끊김은 여전히 별도 문제이며, 시야 정렬로 해결했다고 주장하지 않는다.

근거: `artifacts/evaluation/traffic_light_robustness_261025/` 아래
`v92_abcd_alignment_261026_v1`, `v93_cd_alignment_261026_v1`,
`v93_corridor_alignment_261026_v1` 및 각 `map_v92_*`/`map_v93_*_clearance_analysis_261026_v1`이다.
최종 비교는 `planner_logs/analysis/signal_stopline_alignment_261026_v2/measurements.json`이다.
이전v1 결과의 D 자료 부족도 보존했다. 재검산 도구는
`scripts/evaluation/audit_signal_stopline_alignment.py`, 실행기는
`scripts/evaluation/retest_signal_latency.py`이며 source trial을 변경하지 않고 새 사본을 만든다.

로컬 v93 생성·주행·비교와 관련30개 회귀 테스트·문서 검사·diff 검사는 통과했다.
Drive 업로드는 `ajou_drive`의403 rateLimitExceeded로 실패했고, 전용 `ajou_drive_private` 조회도
invalid_grant(인증 만료/취소)여서 재시도 루프 없이 중단했다. **새 v93의 Drive 업로드는 미완료**이며
이전v91 원격 자산을 덮어쓰지 않았다. Git의 builder와 manifest 이동량으로 동일 SHA를 재생성할 수 있다.

<a id="merged-map-v94"></a>
## 팀장님 v88-2 도로 수정과 정지선의 통합: v94 자산

사용자가 팀장님이 별도로 올린 `lanelet2_map_v88-2.osm`과 우리 정지선 수정을 합쳐 새 버전으로
업로드하도록 요청했다. 팀 Drive의 기존 파일 metadata MD5와 다운로드한 원본이 일치함을 확인했다.
v88-2의 SHA256은 `dcddeee7defb0ba169d1ec14882f0d8cd98d520839c788c745f22a9d6156be37`이다.

팀장님 수정은 v87 대비 node793개 추가/763개 삭제, way16개 추가/34개 삭제, road lanelet relation7개
삭제·기존 relation8개 수정이다. 유지된 node/way의 내용 변경은 없고, 도로 경계·centerline 재구성과
lanelet 연결·속도 제한 변경이다. `201451/201452`의 speed_limit은150→70,
`201448/201479`는150→80이다. 삭제 relation은 traffic regulatory가 아닌 road lanelet이다.
우리 대상8개 정지선 node·4개 way·4개 traffic relation은 v87과 같고 다른 도로 way와 공유되지 않았다.

따라서 **v88-2를 바탕으로 v93의 검증 좌표를 그대로 이식**했다. `.75/2.6/.35/2.6m`를 incoming에
다시 더하지 않았으며, 팀장님 노드에 새 metadata/다른 좌표가 있었다면 거부하는3-way 검사를 적용했다.
실제 추가 수정은 node92992/92993/92998/92999/93020/93021/93034/93035의lat/lon·local_x/y와
`MetaInfo map_version=94`뿐이다. 그 부분을 원복하면 v88-2와 바이트 단위로 완전히 일치한다.
팀장님 수정과 원본v87/v93/v88-2는 모두 보존했다.

통합파일은 `kcity_map/lanelet_map/lanelet2_map_v94.osm`이며,
SHA256 `3d11cd820b2f4d755960ae565dbc267e4cf5826307ef2df1149e10922c138cbf`,
MD5 `ce5047b8abc12f5f26a68a42b6e49a50`이다. node151258/way4177/relation1373,
중복ID·깨진참조0이다. 운영의`libautoware_lanelet2_extension_lib.so`를 등록한 실제 Lanelet parser에서
v88-2/v94 모두 lanelet1295/regulatory78개·오류0으로 읽혔다. 확장을 등록하지 않은 system Python은
기존 roundabout/crosswalk 두 타입을 모르는 오류가 있으므로 이를 새 지도 결함으로 해석하지 않는다.
VMB validation 및 통합 후 폐루프 주행은 이번에 수행하지 않았다.

통합 도구 `scripts/map/merge_signal_stop_lines.py`와 작은 합성자료18개 회귀 테스트를 추가했다.
병합 영수증은 `planner_logs/analysis/map_v94_merge_261026/merge_receipt.json`이다.
Drive 업로드·후속 hash 확인 상태는 `configs/map/kcity_v94_merged_asset.yaml`을 현재 기준으로 본다.
이번 요청은 자산 통합·업로드이며 launch/route/SHA 기본값·설치본·실행 프로세스를 v94로 바꾸지 않았다.

동일 팀 Drive root-folder-id를 명시한 단일 파일 immutable 업로드 명령은 정상종료(code0)했다.
다만 업로드 후 별도 metadata/hash 조회는 간격을 둔 재확인에서도403 rateLimitExceeded였다.
따라서 업로드 명령 완료와 독립 원격 checksum 확인을 구분하며, 후자는 아직 미완료다.
원격의 기존 파일·공유 권한을 바꾸거나 대량 자산 업로드/삭제를 수행하지 않았다.

사용자는 이 공용 연결 우회를 지적했다. 설정 확인 결과 `ajou_drive_private`의client_id는
사용자가 발급한 값과 일치하지만 인증 갱신이invalid_grant로 실패한 상태였고, `ajou_drive`는
명시적client_id 없이 공용client를 사용했다. API 발급이 없던 문제가 아니라 전용 인증 갱신을
먼저 요청했어야 한 문제다. 공용 호출을 중단했고 사용자에게 전용 remote reconnect를 요청했다.
후속 확인에는 전용 연결만 사용하며 공용client로 조용히 fallback하지 않는다.
