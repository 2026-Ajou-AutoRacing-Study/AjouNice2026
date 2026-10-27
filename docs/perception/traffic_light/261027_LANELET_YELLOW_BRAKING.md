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
