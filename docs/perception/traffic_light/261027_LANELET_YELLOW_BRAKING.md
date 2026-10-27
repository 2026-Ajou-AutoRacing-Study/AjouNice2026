# 황색 판단 교체: Lanelet 정차 목표 + 감속도8 단순 근사

종류: IMPLEMENTATION / 2026-10-27. 현재 기본값은 [CURRENT](CURRENT.md)를 따른다.

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

수정 후 B에서 새 분기 로그2회를 확인했다. D는 불필요한 정지점·급제동이 사라졌지만 분기
로그0회라 직접 발동 효과를 단정하지 않는다. 폐루프 도착 시점·속도도 달라 완전히 동일한
입력 재생 대조는 아니다. 두 실행의 원본선 통과 ±0.25초 native 조회는 모두
A=G/B=G/C=G+좌화살표/D=R+좌화살표였다. 황색 GO 분기는 발동하지 않았으므로
이 영상을 황색 통과 성공 인증으로 쓰지 않는다. A의 첫 황색 정지 처리는 유지됐다.
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
