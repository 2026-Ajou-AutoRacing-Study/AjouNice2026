# 고정 AUTONOMOUS operation mode 계약 위반 기록

## 결정

AjouNice MORAI 대회 실행 환경은 코드 기동 시 항상 자율주행하며 운전 모드를
전환하지 않는다는 프로젝트 운영 가정을 적용한다. 이에 따라
`ajounice_planning_integration`이 `/system/operation_mode/state`에 다음 상태를
10 Hz로 고정 발행한다.

```text
mode: AUTONOMOUS
is_autoware_control_enabled: true
is_in_transition: false
is_stop_mode_available: false
is_autonomous_mode_available: true
is_local_mode_available: false
is_remote_mode_available: false
```

각 메시지의 `stamp`만 현재 ROS 시간으로 갱신한다. 구현은
`scripts/fixed_autonomous_operation_mode_publisher`, 실행 진입점은
`launch/fixed_autonomous_operation_mode.launch`다. `planning.launch`와
`morai_planning_stack.launch`의 `publish_fixed_autonomous_mode` 기본값은
`true`다.

## 기존 계약과의 차이

기존 통합 계약은 System/차량 인터페이스가 실제 모드, 실제 Autoware 제어권,
전환 여부를 관측해 사실대로 발행하고 상시 autonomous fixture를 사용하지
않도록 요구했다. 이번 구현은 사용자 결정에 따라 이 요구를 의도적으로
위반한다. 이 publisher는 실제 상태 adapter가 아니며 controller 프로세스,
`/ctrl_cmd`, MORAI UDP 연결, 차량 응답 또는 제어권을 관측하지 않는다.

## 영향과 검증 경계

- controller가 꺼지거나 죽어도 `is_autoware_control_enabled=true`가 계속 발행된다.
- MORAI UDP 또는 명령 전달이 끊겨도 operation mode만으로 장애를 감지할 수 없다.
- requester의 AUTONOMOUS/control-enabled 조건은 실제 제어 상태와 무관하게
  만족하므로 Route A→B 재요청이 진행될 수 있다.
- Planning 모듈은 수동/정지/전환 상태를 관측할 수 없다.
- 이 토픽 수신과 이를 이용한 route/trajectory 출력은 실제 제어권, controller
  건전성 또는 안전한 폐루프 동작의 검증 증거로 사용할 수 없다.

따라서 controller·UDP·차량 응답의 장애 검출과 안전 정지는 별도 경로에서
검증해야 한다. 향후 실제 상태 adapter를 도입하면 고정 publisher를 끄고 단일
publisher만 유지한다.

```bash
roslaunch ajounice_planning_integration morai_planning_stack.launch \
  publish_fixed_autonomous_mode:=false
```

외부 System stack이 `/system/operation_mode/state`를 발행할 때도 위 옵션을
반드시 `false`로 설정해 중복 publisher를 방지한다.
