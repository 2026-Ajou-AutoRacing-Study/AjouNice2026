# 황색 판단 교체: Lanelet 정차 목표 + 감속도8 단순 근사

종류: IMPLEMENTATION / 2026-10-27. 현재 기본값은 [CURRENT](CURRENT.md)를 따른다.

후속 변경(2026-10-28): 사용자 요청으로 두 planning profile의 공통 `delay_response_time`을
0.5→0.0초로 변경했다. `system_delay=0.5`·감속도·jerk는 유지했다. 아래 구현 당시 수치와
이전 주행 결과는0.5초 조건의 근거이며, 새0초 조건의 실주행 검증은 아직 하지 않았다.

## 후속: 판단선 1회 확정 v2 (2026-10-28)

사용자/팀 합의로 아래의 기존 황색 전환 전용 식 대신 **jerk 판단선 상태기계**를 구현했다.
기존 `enable_yellow_passthrough` ON + 준수 guard ON일 때 v2를 사용하며 master 기본 OFF는 유지한다.
OFF는 기존 색 판단을 유지하되, STOP 이력 때문에 정차한 차량의 가상선 초과 자동 GO는 막는다.
ON에서는 더 이상 예전 황색 전환 자격 helper나 `enable_pass_judge`로 2차 판단하지 않는다.
이 문서의 아래 v1 테스트/영상은 v2 검증으로 읽지 않는다.

```text
b = signal_compliance_yellow_deceleration_mps2 = 8
J = signal_compliance_yellow_jerk_mps3 = 16
tau = delay_response_time = 0 (현재 로컬 설정)
T = min(b/J, sqrt(2*v/J))
D = v*tau + v*T - J*T^3/6 + (v-J*T^2/2)^2/(2*b)
```

- 현재 가속도0인 근사다. jerk는 센서/제어 응답지연의 대체물이 아니다.
- `d`는 이미 앞범퍼·stop_margin이 반영된 Lanelet 정차 목표까지 거리다. 앞차축3m를 또 빼지 않는다.
- `d<=D` 안에서 신선한 방향 허가·주행 중이면 GO를1회 확정한다. 부동소수점 등호 프레임을 기다리지 않는다.
- GO 확정 전 Y/R은 STOP. STOP은 목표보다 앞/뒤 정차·경로 crop·순수 UNKNOWN으로 풀리지 않고
  새로운 신선한 방향 허가로 해제한다. D 원형G는 허가가 아니며 기존 R+좌화살표 허가는 그대로다.
- 확정 GO는 후속 Y/R에도 유지한다는 사용자의 시험 정책이다. 적색 채점선 통과 위험을 없애는 보장은 아니다.
  실제 정차·시간 역전·미션 재설정 시 재판단한다. 다른 객체/안전 모듈의 STOP을 무시하지 않는다.
- STOP 목표가 경로 뒤로 사라지면 현재 ego에 STOP을 삽입한다. 가짜 crop 거리0으로 GO 확정하지 않는다.
- 기존 경쟁용 미수신/1초 timeout fail-open GO는 보존한 예외다. 이 예외 동안 내부 STOP 이력은 보존한다.
- C016/C018은201290 하나이며 원본 MGeo 기준 통과 후 미션 종료한다. 두 번째 head를 새 미션으로 만들지 않는다.
  원본 MGeo선까지 크게 넘어 route provider가 미션을 퇴역시킨 경우는 단순 Lanelet 오버슈트 보장의 범위 밖이다.
- 영상 하단에 미션·속도·관측 신호·실제 GO/STOP·확정 여부·Lanelet 목표/MGeo 채점선/계산 제동거리를 한글로 표시한다.
  10/28 요청 정정: 추가했던 `[signal_decision]` 콘솔 INFO는 제거했다. 기존 planner 로그는 유지한다.
  영상 전용 `planner_decision_debug` 토픽은 구독자가 있을 때만 직렬화하며 판단 로직의 입력이 아니다.
  과거 run02 재편집은 당시 남은 0.5초 표본 로그를 사용하며, 정확한 카메라 프레임별 판단 기록으로 해석하지 않는다.
- A/B/C40m·D30m 인지 게이트, 기존0.2/0.3초 완충, 허가1회, 지도와 인지 가중치는 변경하지 않는다.
- 설정 전달은 planning/full-stack launch와 validator `stack.signal_compliance_yellow_jerk_mps3`이며
  새 실행 계약 이름은 `lanelet_decision_line_jerk_v2`다. 과거 실행 snapshot은 수정하지 않았다.

### v2 간단 검증 결과

- 관련 native 테스트117개(Guard36 / normal 실제 모듈40 / experimental 실제 모듈24 / 새 순수식·상태17) 통과.
  앞/뒤 정차 유지·첫Y STOP·fresh허가 재출발·경계 건너뛴 GO확정·새R에도 확정GO유지·
  실제정차 재무장·crop 가짜0·명시적 masterOFF·미수신GO 등을 검사했다.
- 실제 source/installed launch 해석4개, jerk 전달/validator 집중27개 통과(생략1).
  광역validator498개 통과·기존 waypoint/TF manifest불일치2개 실패는 새 변경과 구분한다.
- 첫 실제 주행은 정차 중 미세 음수 속도 때문에 불필요한 STOP이 기록되는 버그를 찾아
  **MANUAL_ABORT 후 정지·P 확인**했다. 실패/중단 기록을 PASS로 바꾸지 않았다.
  −0.1~0m/s를0으로 정규화해 수정했고, 두 번째 주행에서 invalid_velocity0·처음UNKNOWN GO를 확인했다.
- `artifacts/evaluation/traffic_decision_line_261028/run_02/`는 **Cloudy13 자연 신호(noSET)**,
  localization+od_gt+실제 ROS1 planning/PP·PI, A/B/C/D 전체 통과·목표 PASS·실제 정지/P VERIFIED다.
  응답지연0 / b8·J16 / ABC40·D30 / G1·UNKNOWN0.3 / 공유지도v97의 실제 snapshot·설치plugin을 대조했다.

| 미션 | 실제 정차/확정 관찰 |
|---|---|
| A | Lanelet 목표0.52~0.56m 전 STOP 유지, 신선한 허가에 재출발 |
| B | 허가 접근 중 v14.99m/s·목표16.08m·D17.70m에서 GO확정, 이후UNKNOWN에도 진행 |
| C | 목표0.07~0.16m 전 STOP 유지 후 허가 재출발, 원본선 통과 뒤 D로1회전환·C재선택0 |
| D | 목표0.06~0.12m 초과 정차해도 STOP 유지, 신선한 R+좌화살표에 재출발 |

D 정차 중 raw영상은 **원형G(좌회전비허가)→Y→R+좌화살표**다.
로그의 `Observation::Stop`을 원시 적색으로 잘못 설명하지 않는다.
실제 적색/황색 오버슈트 유지의 일반성은 native모듈 테스트로 확인했고,
이 자연 주행은 D의 비허가G/Y 정차 유지·새 좌회전 허가 출발을 직접 확인한 것이다.

- 원본 MGeo선 통과 전후0.25초 exact native 조회에서 A16/B16/C양쪽48/D33 허가를 확인했다.
  공식 패널티 인증은 아니며, GO확정 뒤 실제 새Y/R로 바뀌는 도전 상황은 이번 자연 주행에 없었다.
- 제동을8/16 한계로 요청하는 별도 actuator 채널은 추가하지 않았다. 이 주행은 근접경계에서의
  최대 급제동 거리 실측/최악 지연 검증이 아니므로 이후 실제 정지거리로 파라미터를 튜닝해야 한다.
- 마지막 수정은 아직 보지 않은 미래 미션의 missing INFO만 숨기는 로그 정리다. 정책은 그대로이며
  다시 빌드/설치/계약 검사했다. 마지막 로그 정리 후 추가 실주행은 하지 않았다.
- 전체 자료: 같은 상위 `driving_audit_02.json`, `supervisor_02/summary.json`,
  실제 캠페인 `planner_logs/validation/decision_line_v2_261028_261028_042825`.
  영상은 `run_02/front_video/front_evidence.mp4`이며 D통과+5초에 끝난다.

## 목적과 범위

사용자는 원본 MORAI 채점선 전에는 멈출 수 있어도, 당겨둔 Lanelet 정차 목표를 지나
신호등을 못 보는 위치에 멈추는 상황을 피하고자 기존 황색 판단의 교체를 요청했다.
따라서 기존 jerk 기반/황색 최소시간·ETA 판단을 활성 준수 가드 경로에서 대체한다.
인지 모델·미션 매칭·0.3초 UNKNOWN 유예·일반 제어 감속도는 바꾸지 않는다.

## 식과 허가 조건

```text
d_stop = v * delay_response_time + v² / (2 * yellow_deceleration)
yellow_deceleration = 8.0 m/s² (기본)

이전 방향 허가 확인 + 주행 중 + 실제 황색 전환 + master ON:
  d_stop > Lanelet 정차 목표까지 남은 거리  => 기존 GO 유지
  d_stop <= 남은 거리                     => STOP
그 외                                    => 황색 예외 불허
```

- 첫 관측이 황색이거나 이미 정차했거나, 이전 STOP 판단 뒤 황색이 계속돼도 새 출발하지 않는다.
- D의 직진 G는 좌회전 허가가 아니다. 실제 허가 이력이 없는 G→Y는 예외가 아니다.
- 허가1회라는 기존 설정은 유지하지만 중복 프레임이 새 허가 이력을 만들지는 않는다.
- 새 RED·색 모순·stale·시간 역전·다른 미션의 기존 처리는 유지한다.
- 무조건 GO를 고정하는 새 latch는 넣지 않는다. 황색 중에도 속도/거리를 다시 계산하며
  다른 이유로 감속해 정차 가능해지면 STOP을 선택할 수 있다.
- 미래 황색 지속시간은 가정하지 않는다. 적색 전 통과·무패널티 보장은 아니다.

## 좌표와 거리

`base_link`는 후륜 중심, 휠베이스3.0m, 앞 오버행0.845m다.
플래너의 기존 `signed_arc_length_to_stop_point`는 **후륜 중심이 실제 정차 pose까지
이동해야 할 거리**이며, 이미 앞범퍼3.845m와 stop_margin이 반영돼 있다.
이를 제동거리와 직접 비교해 기존 카메라용 정차 위치를 유지한다.
여기에 앞차축3m나 앞범퍼 길이를 또 빼지 않는다.

황색 ON의 원본 MORAI선 통과 완료는 후륜 위치에서 차량 방향으로3m 옮긴 앞차축으로
기존대로 확인한다. 이는 제동거리 판단선과 다르며, Lanelet 목표를 지났다는 이유만으로
공식선 전 새 적색을 무조건 무시하는 변경은 하지 않는다.
원본선 ETA 계산 실패는 새 Lanelet 제동 판단 자체의 거부 사유가 아니다.

## 설정과 근사 한계

- 전체 스택/단독 planning: `signal_compliance_yellow_deceleration_mps2:=8.0`.
- Validator: `stack.signal_compliance_yellow_deceleration_mps2: 8.0`.
- master `enable_yellow_passthrough` / `stack.traffic_light_yellow_passthrough` 기본 OFF 유지.
- 기존 공통 `delay_response_time` 사용(현재 소스0.5초). 추가 임의 거리 마진은 넣지 않았다.
- 전역 `max_accel=-6`, `max_jerk=-10` 및 제어기 설정은 변경하지 않는다.
- 8은 **황색 의사결정용 설정**이지 MORAI에서 즉시 실현되는 감속의 실측 보장이 아니다.
  실제 반응/제동이 근사보다 느리면 정지 가능성을 과대평가하여 늦게 STOP할 위험이 남는다.
- 기존 timed flag/minimum/margin 인자는 호환용으로 수용하지만 새 분기를 바꾸지 않는다.
  새 실행은 정책 구현명·decel값을 보관한다. 옛 snapshot bytes를 수정하지 않더라도
  바뀐 코드로 재실행하는 것은 옛 시간/jerk 모델의 동일 재현을 뜻하지 않는다.
- guard OFF는 명시적 legacy 복귀이며 이 교체의 대상은 production 준수 가드 ON 경로다.

예시(응답지연0.5초, 속도 일정 근사): 36km/h에서11.25m, 50km/h에서약19.00m다.
같은 속도라도 목표까지 거리가 이 값 이하인지 초과인지에 따라 정지/진행이 달라진다.
본 문서는 구현 계약이며 실제 A/B/C/D 주행·제동/시야 검증 결과는 별도로 구분한다.

## 이번 검증

- Native C++ 9개 테스트 바이너리 총144개 통과. 일반 Path와 experimental Trajectory
  양쪽에서 A/B/C/D 첫 황색·정차·이전 STOP·D 비허가 직진G 이후 황색을 차단했다.
- Lanelet 목표와 더 먼 원본선을 분리한 입력, 중복 앞차축 감산 방지, 동일 거리 STOP,
  응답지연/8값 적용, 새 RED·timeout·이전 UNKNOWN 이력, 옛 timed flag 무효를 검사했다.
- 인지·평가·녹화 Python522개 통과, launch 실제 source/installed 설정 해석 각각16개 통과.
- Validator 전체471개 통과·기존 자산 불일치2건 실패(waypoint/manifest); 관련 자산은 고치지 않았다.
- 설치기 scoped rebuild/install·설치본 일치 검사 완료. 실행 중 ROS/MORAI는 재시작하거나
  주행하지 않았으며, 새 정책의 실제 제동/시야·공식 신호 판정은 아직 검증하지 않았다.
- 문서 검사는 새 링크·신호등 CURRENT100줄이 유효하며 기존 지도 README 길이 초과만 남는다.

## 후속 풀주행 녹화와 시험 제어 감사

근거 루트는 `artifacts/evaluation/traffic_light_lanelet_braking_261027_live/`다.
공통 시작 위치에서 실제 localization·ROS1 planner·PP/PI와 od_gt를 사용했고,
YOLO/CNN은 현재 카메라 영상을 판독했다. BEVFusion·LAN 풀스택 인증은 아니다.
공통 RViz의 차량 추종·지도·표시 토픽은 유지하고 view distance만75.25→150으로 넓혔다.

| 실행 | 명시 지도 | 결과와 영상 |
|---|---|---|
| run_01c | v88 | A/B/C 통과, D 앞 UNKNOWN 정체로 정상 중단·P 확인 |
| run_02_v94 | v94 | 전체 goal 도달·P 확인, D 통과+5초에 화면/전방 영상 종료 |
| run_03_v94 | v94 | A 통과 뒤 B 앞 UNKNOWN 정체로 정상 중단·P 확인 |

세 주행은 요청 황색2/3/2초의 **UDP 강제 제어 stress**다. 이후 감사에서
run03 B의 요청 적색 구간에 독립 native 조회가 `R→G→R`로 바뀐 사실을 확인했다.
인지도 실제 녹색을 읽었다. 반복 UDP SET는 자연 프로그램을 영구 고정하지 않으므로
이 구간의 출발·재제동을 인지 오판으로 분류하거나 요청 주기를 실제 GT로 쓰면 안 된다.
UDP 전송 직후 GET 일치와 2초 지속 mismatch 감시는 짧은 중간 점등을 배제하지 못했다.
지속 모드를 별도 자격 검증하기 전에는 안정된 단축 주기 비교로 인정하지 않는다.

별개로 run03 B는 최초 황색 STOP 때 목표 약5cm 초과에서 정차한 뒤,
녹색에 다시 출발하여 가상 정차 목표를4.494m 지난 순간 UNKNOWN 새 STOP이 들어갔다.
최종7.645m는 최초 정지 요청의 오버슈트가 아니다. run01c D도 최초 정차 오차
약0.1~0.2m와 재출발 뒤 UNKNOWN 재정지를 구분해야 한다. 황색ON의 원본선 GO_OUT과
그 앞쪽 Lanelet 정차 목표 사이에서 새 UNKNOWN 정지점이 생기는 경로가 남아 있다.
따라서 황색ON 후보의 무정체·완치 검증은 통과하지 않았으며 운영 기본OFF를 유지한다.

run01c/02의 제어 명령50Hz(최대 수신 간격29/30ms), 측위50Hz,
신호 출력8.39/8.36Hz, 신호 header→수신 p95 45.5/40.6ms를 확인했다.
전방 기록기의 고유 궤적 표본 header→수신 p95는209/254ms다. 현재의 지연을 무시할 수는
없지만 `/ctrl_cmd`에는 header가 없어 발행→액추에이터 지연을 직접 측정한 것은 아니다.
당시 두 주행의 CPU 시계열은 없으며 뒤의 run03 CPU87~88%를 소급 적용하지 않는다.
run02 bag은 디스크1GiB 미만으로 시작 직후 기록이 중단됐고, 주행·두 영상·observer는
보존됐다. 이를 온전한 full-route bag이라고 쓰지 않는다. 이후 공간을 확보했다.

원격 planning 최신은 v97이었으며, 로컬 시험의 v88/v94 override가 production 지도나
원격 파일을 덮어쓰지는 않았다. 전용 Drive에서 받은 v97 SHA는 원격 manifest의
`da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9`와 일치한다.
v94↔v97의 A/B/C/D 정지선 차이는 최대0.056mm지만 다른 지도 속성·코드까지 동일한 것은
아니다. 최신 원격의 공통 response delay0.0과 이번 로컬0.5초 조건도 구분한다.

다음 wrapper 실행부터는 `observe_traffic_light_runtime_timing.py`를 자동으로 붙인다.
실제 `/planning/scenario_planning/trajectory`의 주기·age, 카메라·측위·제어 주기와
1Hz CPU·메모리·디스크를 수동 판독과 별개로 보관하며 발행·제어·영상 decode는 하지 않는다.
현재 시험은 정지/P로 마감했다. 강제 제어를 끝냈다는 사실은 자연 현시 복원 인증이 아니다.

## 근접 판독 종료 뒤 GO: 사용자 요청 보완과 v97 재주행

사용자는 신호등을 보다가 화면 위로 지나간 뒤 다시 정차시키지 말고 진행하도록 요청했다.
판단 입력에는 화면 위 이탈 여부의 전용 비트가 없으므로 **같은 접근의 실제 색 관측 이력 +
앞차축이 가상 Lanelet 선을 지남 + 신선한 순수 UNKNOWN**을 근접 관측 종료의 대용 조건으로
사용한다. 이때 새 STOP을 넣지 않으며 이미 정차한 상태도 포함한다. 이전 녹색·최소 속도·
STOP 이력 제한은 추가하지 않았다. 첫 원거리 미관측·경계 거리0·stale·색 근거가 있는 모순은
이 예외가 아니고, 새 적색·황색은 계속 기존 판단을 따른다. 모듈을 조기 GO_OUT으로 고정하지
않아 아직 읽을 수 있는 새 적색을 무시하지 않는다. normal/experimental에 같은 조건을 적용했다.
이는 상단 이탈을 직접 판정하는 영상 알고리즘이 아니므로 모든 UNKNOWN의 원인을 증명하지는 않는다.

`run_04_persistent_v97` 수정 전과 `run_05_terminal_exit_v97` 수정 후에 동일한 시작 장면·
v97 SHA·계획02·황색ON/8m/s²·실제 planner/controller로 풀경로를 주행했다.
두 실행 모두 goal·정지/P 확인, D 원본선 통과+5초 영상 종료를 완료했다.
비교 창은 각 주행의 마지막 허가 판독부터 원본선 통과+2초까지다.

| 구간 | 수정 전 최저 km/h | 수정 후 최저 km/h | 최대 brake 전→후 | 수정 후 계획속도0 |
|---|---:|---:|---|---|
| B | 6.77 | 28.65 | 1.00→0.295 | 없음 |
| D | 1.77 | 24.50 | 1.00→0.132 | 없음 |

종료 후 flush까지 포함해 수정 후 B에서 새 분기 로그2행, D에서3행을 확인했다.
초기 관제 중 D0행 집계는 미완료 로그를 읽은 값이었다. throttle 로그행 수는 작동 프레임 수가 아니다.
폐루프 도착 시점·속도도 달라 완전히 동일한
입력 재생 대조는 아니다. 두 실행의 원본선 통과 ±0.25초 native 조회는 모두
A=G/B=G/C=G+좌화살표/D=R+좌화살표였다. 황색 GO 분기는 발동하지 않았으므로
이 영상을 황색 통과 성공 인증으로 쓰지 않는다. A의 단발 황색 판독은 주변 native 적색과
불일치해 실제 황색 검증에서 제외한다.
B 궤적 header→수신 p95는178→194ms로 오히려 증가했는데 급제동이 없어져,
단순한 CPU 개선만으로 생긴 차이는 아니었다.

driver는 공식 gRPC `SetTrafficLightState`의 `is_impulse=false,set_sibling=false`로 전환했다.
5개 head가 반복 명령 없는5초 동안 각50회 GET에서 적색을 유지한 뒤 주행을 시작했다.
그러나 독립 조회에는 A/B의 자연30초 경계 부근에 요청과 다른 순간 상태가 여전히 남아,
이 방식도 자연 controller 주기를 완전히 고정했다는 인증은 아니다. 평가에는 요청 색이
아닌 현재 영상과 독립 native 조회를 사용했다. C의 두 physical head는 각각 제어했다.

새 helper/실제 모듈 시험을 포함한 native72개, 관련 Python44개 통과.
로컬 C++ 재빌드·설치 및 기존 설치기의 source/install/ELF receipt 검사를 완료했다.
황색 master 기본OFF·적색 정책·미수신 timeout·인지 가중치는 변경하지 않았다.
변경분은 아직 PR/팀 배포하지 않았다. 영상·bag·Hz/age/CPU 자료는 두 실행 폴더에 보존했다.
