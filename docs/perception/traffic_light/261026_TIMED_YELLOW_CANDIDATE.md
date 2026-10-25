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
