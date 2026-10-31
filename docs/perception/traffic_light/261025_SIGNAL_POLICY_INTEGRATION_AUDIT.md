# 신호등 관측·정책·Planning 통합 감사

종류: EVIDENCE / 2026-10-25 정적 source·설정 대조. 현재 선택은 [CURRENT](CURRENT.md)를 따른다.
이 문서는 정책 변경·규정 승인·실차/대회 주행 인증이 아니다. 정책 source/YAML과 planning은 수정하지 않았다.

## 핵심 결론

1. 현재 인지의 적색 fallback과 planning의 `competition_fail_open=true`는 **다른 층의 정책**이다.
   올바른 ID로 인지가 계속 발행하면 미검출은 적색으로 보완되지만, 해당 ID가 없거나 발행이 끊기면
   planning은 missing/UNKNOWN/stale에서 GO를 반환한다. 따라서 시스템 전체가 fail-closed인 것은 아니다.
2. D(옛 D-2)의 `Y`는 인지에서 `R`로 바뀌어 좌회전 STOP이고, `R+Y`는 그대로 남아 planning의
   `yellow_light_is_go` 조기 반환으로 GO다. **이미 좌회전 중인지, 정지 상태에서 새로 출발하는지 확인하지 않는다.**
3. CNN은 잘못된 색·화살표 관측을 줄이는 후보이지, 황색 통과 규칙·정지선 통과 시점·정보 상실 정책을 해결하지 않는다.
   미션은 A/B/C/D 네 곳이며 옛 D-1/201397은 판단 대상이 아니다.

## 근거와 범위

- 사용자 제공 [규정 v1.2 원문](../../competition/261015_대회_규정_v1.2_원문.md) L598–610:
  녹색/좌회전 신호에 맞춰 주행하고 앞바퀴가 정지선을 넘는 순간의 신호로 판정한다.
  위반은 30초이며 네 판정 교차로 모두 위반하면 차시 실패다. 현시 변경 주기는 공개하지 않는다.
  이 원문에는 `R+Y`에서 새 출발을 허용한다는 명시 조항이 없다.
- 팀 원문 `artifacts/reference/traffic_light_team_notes_261025/source.md`의 **v1.1, L716–981**과
  `team_chat_261025.txt`를 직접 대조했다. 앞부분 v1.0·옛 제안을 현재 계약으로 사용하지 않았다.
  v1.1은 A/B 적색 fallback, C 단일 녹색 보완, D의 두 황색 분리를 설명한다.
  대화에서는 이미 진행 중인 통과와 처음 `R+Y`를 본 뒤의 출발을 구분해 우려했고 공식 문의를 남겼다.
  2.5초 카운트는 현시 중간부터 관측할 수 있어 배제했으며, 주기 단축·정지선 이동 예상은 공식 확정이 아니다.
  원문 보존·차이는 [팀 기록 대조](261025_TEAM_NOTES_AND_ROBUSTNESS_AUDIT.md)를 따른다.
- 이하 표는 현재 source의 순서를 읽고 `SignalPhasePolicy.apply()`를 4구간×16조합으로 실행해 대조한 것이다.
  C++ planner를 실행하거나 ROS parameter server를 조회하지 않았다. source와 설치된 planning YAML 3종은 SHA가 같았지만,
  실행 인자·실제 로드 바이너리·live override까지 동일하다는 인증은 아니다.

## 연결 순서와 미션 ID

`camera → YOLO → 대상 선택 → HSV/CNN 원시 관측 → profile 보정 → smoother/관측 가드 → 표준 신호 메시지 → BVP`

`debug_observed_label`은 보정 전이고 `debug_raw_label`은 이름과 달리 보정 후·smoother 전이다.
CNN의 `known=false`와 확신 있는 `no_lamps`는 `debug_classifier`에서는 다르지만, 현재 정책 입력은 둘 다 빈 램프 배열이다.
BVP에는 CNN의 abstention 사유나 박스/head identity 진단이 전달되지 않는다.
연결은 [node](../../../base_ws/src/perception/traffic_light_detection/scripts/traffic_light_node.py)의
`image_callback`에서 observed → `apply` → `process_normalized` → `elements` 발행 순서다.

| 구간 | regulatory ID | 의도 경로 | profile / 대상 |
|---|---:|---|---|
| A | 201351 | 직진 | `simple_3color`, 차량 head C077 |
| B | 201365 | 직진 | `simple_3color`, 차량 head C091 |
| C | 201290 | 직진 | `straight_left_combined`, 동기 차량 head C016/C018 모두 허용 |
| D, 옛 D-2 | 201326 | 좌회전 | `straight_or_protected_left`, 차량 head C052 |
| 옛 D-1 | 201397 | 미션 아님 | 인지 legacy profile은 passthrough, BVP 등록에서 제외. 지도 모드도 미션 allowlist에서 제외 |

BVP 방향은 ID별 상수로 결정하지 않는다.
[manager.cpp](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/manager.cpp) L133–160과
[utils.cpp](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/utils.cpp) L217–242는
경로에 실제 포함된 다음 분기 lane의 `turn_direction`을 선택해 모듈에 저장한다. 기존 모듈에는 정지선만 갱신한다.
따라서 표의 D=left 등은 실제 route/decision-lane 태그와 함께 검증해야 한다.
태그가 없거나 잘못된 경우 [traffic_light_utils.cpp](../../../planning_ws/src/autoware_traffic_light_utils/src/traffic_light_utils.cpp)
L66–99의 일반 녹색 규칙이 적용될 수 있다. 구간 ID만 맞는다고 방향까지 검증된 것은 아니다.

## 원시 관측 → 보정 → 신호 predicate

R=적색 원형, Y=황색 원형, G=녹색 원형, L=녹색 좌화살표다. `보정값 / GO·STOP`으로 적었다.
GO는 **이 신호 모듈이 정지를 요구하지 않는다는 뜻**이며 다른 장애물·경로 모듈의 정지까지 해제한다는 뜻이 아니다.
이 표는 올바른 ID·의도 경로·신선한 메시지와 안정된 출력 기준이다. 뒤의 시간 필터·pass judge 예외를 별도로 읽어야 한다.
빈 관측은 즉시 보정 단계에서는 R이지만 D 관측 가드 등은 직전 출력을 잠시 유지할 수 있다.

| 원시 관측 | A/B, straight | C, straight | D(옛 D-2), left |
|---|---|---|---|
| ∅ / raw UNKNOWN | R / STOP | R / STOP | R / STOP |
| R | R / STOP | R / STOP | R / STOP |
| Y | Y / GO | Y / GO | R / STOP |
| R+Y | R / STOP | R / STOP | R+Y / GO |
| G | G / GO | G+L / GO | G / STOP |
| R+G | R / STOP | R / STOP | R / STOP |
| Y+G | R / STOP | R / STOP | R / STOP |
| R+Y+G | R / STOP | R / STOP | R / STOP |
| L | R / STOP | G+L / GO | R+L / GO |
| R+L | R / STOP | R / STOP | R+L / GO |
| Y+L | Y / GO | R / STOP | R / STOP |
| R+Y+L | R / STOP | R / STOP | R / STOP |
| G+L | G / GO | G+L / GO | G / STOP |
| R+G+L | R / STOP | R / STOP | R / STOP |
| Y+G+L | R / STOP | R / STOP | R / STOP |
| R+Y+G+L | R / STOP | R / STOP | R / STOP |

근거: [profile YAML](../../../base_ws/src/perception/traffic_light_detection/config/traffic_signal_profiles.yaml) L19–69와
[policy](../../../base_ws/src/perception/traffic_light_detection/scripts/signal_phase_policy.py) L92–127.
정확한 correction → valid phase → unsupported-lamp 제거 후 재검사 → fallback 순이다.
따라서 모든 R 포함 조합을 단일 규칙으로 처리하는 것은 아니다. 위의 비자연 조합은 stress 동작이지 자연 현시 목록이 아니다.
옛 D-1은 `passthrough=true`가 먼저 실행되므로 빈 관측도 그대로 빈 배열이며, 설정의 녹색 fallback을 실행하지 않는다.
`ignored_regulatory_ids: [201397]`와 manager L121–123 때문에 이 ID의 판단 모듈은 생성하지 않는다.

정책이 오류를 보완하거나 가리는 대표 예는 다음과 같다.

- A/B/C의 잘못된 R+G는 R로 보완된다. 최종 정지가 맞아도 원시 분류 정답은 아니다.
- C의 G 단독 또는 L 단독은 G+L로 보완된다. 누락 화살표를 정책이 붙인 결과를 CNN/HSV arrow recall에 넣지 않는다.
- D의 L 단독은 R+L로 보완돼 GO다. 실제 G를 L로 오분류하면 이 보완이 잘못된 좌회전 허가로 이어질 수 있다.
  반대로 D의 G+L은 G로 보완돼 좌회전 STOP이다. 단순히 녹색이 포함됐다는 이유로 D를 GO로 세면 안 된다.
- 함수의 `:fail_open` reason 및 GO를 설명한 옛 주석과 달리 현재 A/B/C/D의 YAML fallback은 R이다.
  이름이 아니라 실제 램프 배열과 downstream 판단을 확인한다.

## BVP의 실제 검사 순서와 황색

[scene.cpp](../../../planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/scene.cpp) L193–249는 다음 순서다.

1. 신호 stamp 없음 → `competition_fail_open=true`이면 GO.
2. 신호가 1.0초보다 오래됨 → 같은 설정이면 GO.
3. 알려진 color+shape 램프가 하나도 없음 → 같은 설정이면 GO.
4. Y가 있고 `yellow_light_is_go=true` → **즉시 GO**.
5. 그 뒤에야 arrow-aware 황색 로직과 경로 방향별 녹색/화살표 검사를 한다.

[현재 신호 parameter YAML](../../../planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml)
L1–17은 timeout=1.0, `yellow_light_is_go=true`, `competition_fail_open=true`,
`enable_arrow_aware_yellow_passing=false`, `v2i/use_remaining_time=false`다.
일반/normal [통합 YAML](../../../planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml) L360–379도 같은 값이다.
보통 녹색 원형은 straight를 허용하지만 left는 L을 요구한다(traffic_light_utils L79–94).

그러므로 D의 R+Y가 통과하는 직접 원인은 현재 단계 4다. 앞에 R이 있거나 차량이 정지해 있어도 이 분기에는
직전 보호좌회전 현시·이미 교차로 진입 여부·공식 정지선 도달 시간·남은 황색 시간 조건이 없다.
Y 단독은 인지에서 R로 제거돼 단계 4에 도달하지 않는다. A/B/C의 Y는 제거하지 않아 GO다.
Y를 늦게 처음 관측한 것을 점등 시작으로 간주할 수 없고, 인지의 4초 가드는 물리 황색 잔여 시간을 보장하지 않는다.

## 신호 정보가 없을 때: 적색 보완과 GO를 구분

| 실제 상황 | 인지/전송 | 현행 BVP 결과 |
|---|---|---|
| 올바른 미션 ID를 알며 새 이미지가 계속 들어오지만 박스/색이 없음, CNN UNKNOWN·소등 | 원시 ∅ → profile R. 일시적으로 smoother/관측 가드 hold 가능 | 결국 신선한 해당-ID R이면 STOP predicate |
| route는 유효하지만 지도 TF/측위/association이 missing | 해당 route ID에 원시 ∅ → 같은 R 경로 | 위와 같음. TF missing 그 자체가 신호 모듈을 중단시키지는 않음 |
| route ID가 없거나 stale, 기본 fallback ID=0 | R을 내더라도 **ID 0**. 현재 경로의 실제 regulatory ID 데이터가 아님 | 해당 ID missing → GO 가능. 'R 발행 중'만으로 fail-closed라 할 수 없음 |
| 다른 ID만 발행하거나 새 배열에 대상 ID가 빠짐 | planner의 raw-ID map에서 대상 항목 제거 | 대상 missing/빈 looking state → fail-open GO |
| 이미지/인지 발행 자체가 멎음 | fallback timer가 따로 발행하지 않음 | 마지막 신호 stamp가 1초를 넘으면 fail-open GO |
| 다른 publisher가 신선한 대상-ID의 빈 elements / 전부 UNKNOWN을 보냄 | 알려진 lamp 없음 | fail-open GO |
| 옛 D-1/201397 또는 경로상 신호 모듈 자체가 없음 | 미션 대상 아님 / 모듈 미등록 | 이 신호 모듈의 정지 요구 없음. '미션 녹색 판정 성공'으로 세지 않음 |

[PlannerData](../../../planning_ws/src/autoware_behavior_velocity_planner_common/src/planner_data.cpp) L44–68은
매 배열마다 raw-ID map을 비우고 새 ID들을 채운다. scene L396–407은 `getTrafficSignal(id, false)`를 호출하므로
last-observed cache를 사용하지 않는다. 정확한 ID가 빠졌을 때 옛 적색이 안전하게 유지된다고 가정하면 안 된다.
BVP의 known-lamp 검사는 confidence threshold 검사가 아니다. 인지 fallback R의 confidence가 0이어도 R이라는 의미는 남는다.
판단 메시지 stamp는 node가 쓰는 원본 JPEG stamp이며 처리 지연도 stale 판정에 영향을 준다.

## 시간 필터와 'STOP=즉시 정지'가 아닌 이유

- 기본 smoother 7/4는 램프 **집합 전체**를 투표한다. 큐가 차기 전 첫 6개는 즉시 전달하고, 이후 4표 이상이면
  그 집합의 최신 payload를 반환한다. A/B/C의 미검출 R도 투표에 들어가므로 직전 G/Y가 잠시 유지될 수 있다.
- D는 `:fail_open` 원시 누락을 투표·유효 관측 갱신에서 제외하고 마지막 정상 출력의 관측 나이를 검사한다.
  유효 관측 공백 1.5초 또는 연속 관측된 황색 4.0초를 넘으면 R이다. 새로 확인된 다른 phase가 들어오면 상태가 바뀐다.
  이 검사는 이미지 callback 때 수행된다. callback 자체가 멎으면 별도 가드 만료 메시지가 나오지 않는다.
  held 출력은 새 JPEG stamp와 함께 발행될 수 있어 BVP의 message freshness와 실제 램프 재관측 여부는 다르다.
- BVP는 STOP 신호에도 0.1초 hysteresis, 정지선 교차·모듈 상태·restart suppression·pass judge를 적용한다.
  scene L351–393의 pass judge는 속도/가속도/jerk와 `yellow_lamp_period=2.75`로 정지 가능성·도달 거리를 계산하며,
  호출 조건에 현재 신호가 황색이라는 검사가 없다. 따라서 위 표의 STOP을 실제 제동·정지선 준수 완료로 해석하지 않는다.
  module은 계산 정지점을 2m 넘으면 GO_OUT으로 넘어가며, 정지점 계산이 안 되는 경우도 이 모듈이 정지를 만들지 못한다.

근거: [smoother](../../../base_ws/src/perception/traffic_light_detection/scripts/signal_smoother.py) L26–42,
policy L159–206, scene L64–187. `stop_margin=0`과 차량 전방 offset으로 계산한 지도 정지점이
공식 판정 정지선/앞바퀴 통과 시점과 일치하는지는 별도 기하·주행 검증 대상이다.

## 해결한 범위와 남은 결정

| 구분 | 현재 코드/후보가 다루는 것 | 아직 보장하지 않는 것 |
|---|---|---|
| CNN | 선택 crop의 R/Y/G/L 원시 분리, 저확신 UNKNOWN 기록 | 올바른 대상·미검출 회복·화각 밖 신호·황색 진입 허가·규정 준수 |
| opt-in 지도 선택 | 네 미션 차량 head 연결, 동기 C016/C018 허용, 다른 head·불확실 입력 거부 | 실제 측위/TF 오차와 지연에서의 완전한 identity·가시성·연속 수신률 |
| 현행 profile | 불가능 조합 적색 우선, C 화살표 보완, D의 Y와 R+Y 분리 | 픽셀 정답, R+Y에서 새 출발과 이미 진행 중인 통과의 구별 |
| smoother/관측 가드 | 일부 단발 오류·관측 공백 완충 | 실제 현시 시작·잔여 시간·공식 정지선 도달 전 적색 전환 예측 |

필요한 후속 결정은 인지 모델 교체와 별개다.

1. 운영측에 황색/적+황의 진입·통과 판정과 공식 정지선 기준을 확인한다. 이미 진행 중인 통과와 새 출발을 구별해 질문한다.
2. 팀이 missing/UNKNOWN/stale의 전역 정책과 장기 적색 fallback의 대응을 명시적으로 결정한다.
   현재 fail-open GO와 적색 fallback의 어느 한쪽만 보고 안전·완성이라고 결론 내리지 않는다.
3. 실제 경로 decision-lane 방향, 메시지 ID/stamp, 원시 관측/보정/held 이유, 공식 앞바퀴 정지선 통과 시점을 함께 검증한다.
   D의 G에서 좌회전했다는 팀 보고는 당시 동기 로그가 없어 classifier·방향 태그·누락 fail-open·pass judge 중 원인을 단정하지 않는다.
4. CNN raw exact/arrow 지표와 정책 후 GO/STOP, 실제 주행 위반을 별도로 보고한다.
   규정상 금지된 native 신호 조회·Ego GT는 개발 평가에만 쓰며 대회 runtime 입력/모델 feature로 사용하지 않는다.

`test_signal_policy_current_contract.py` 4개 테스트(4구간×16조합 포함)를 재실행해 통과했다.
이는 현재 구현 계약을 재현한 것이지 위 정책의 규정 적합성을 승인한 시험이 아니다.

## 대조 revision·SHA256

대조 HEAD는 `898598f4ed4ccbbf94172b1bad3a65debaee0c4a`다. 작업 중 후보 node 변경은 존재하지만
아래 정책/planning source·규정 원문은 이 문서 작업으로 변경하지 않았다.

```text
traffic_signal_profiles.yaml  5000dd5434dfd97aeae8e207e85142a10a22c5ba99b9d14a97d9cd550f31ac80
signal_phase_policy.py        a4d913004f54c343d0858be88b1538f1805d320a46534c593a9918501aa3a30a
signal_smoother.py            0076f4cffd83f7ef0bfaa61333b26756c1c31cc79daeb71d662b504452574d2a
BVP scene.cpp                0e32b21c34ed846ea8f7c8bf8a2903a8f991f4be9285d015476fbcba5eacb15b
BVP manager.cpp              b8542ea5f91c75a5270fc58ca514df1f78a88c204d675f6b210b2b0dbd8e65cc
traffic_light_utils.cpp      03ff62642221c735dd51413e9bb6bc0e944705822812dd3d0851c8796f5a1d81
planner_data.cpp             dbd713f0347ad98e16501f4166f6f11c134e39941c69c3e0e97afbcb2dd33c16
traffic_light.param.yaml     9fd7c59849a1e5e80b75ef0b70e1c05de587230eabb2128239cb79cf098b59bd
팀 source.md                 029a7feb52dcda4bb10a549c7930c51b09024b40066c8d11c25883683a3b9297
팀 chat                      a93f42f0b58dfbb7529ee7309f76e053c27c491d1dcaf09a8b10f03e7bd65aca
규정 v1.2 원문               fb22f75cafd443f2ff3ea1b06b423d0ca3801e1850ee157842e0346e899e68aa
```
