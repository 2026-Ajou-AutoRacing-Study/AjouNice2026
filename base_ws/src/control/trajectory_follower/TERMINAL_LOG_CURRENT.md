# 제어기 터미널 로그 CURRENT

기준일: 2026-10-29. `traj_fol_a` / `pp_pi_autoware.launch`의 표시 계약.

```text
00.0 CTRL  23.45kph  6.55mps2 +12.3°
01.0 CTRL 134.55kph  0.25mps2  +2.1°
02.0 CTRL   0.00kph -6.55mps2 -40.0°
```

## 입력과 단위

- 첫 필드는 현지 시각의 초·소수 첫째 자리(`SS.s`)다. 매 분 `00.0`부터 반복한다.
- 속도는 `/Competition_topic.velocity.x`만 사용한다. UDP bridge가 이미 m/s로
  변환하므로 표시에서 3.6을 곱한다. `/Ego_topic`·localization fallback은 없다.
- Ay는 `/sensor/imu.linear_acceleration.y`를 그대로 m/s²로 표시한다.
  위치 미분·횡가속도 추정·중력 보정·차량 축 재변환은 추가하지 않는다.
- 조향은 `/ctrl_cmd`로 publish하는 최종 타이어 조향 명령이다. 측정 바퀴각이 아니다.
  `/control/terminal_state`에 동일한 명령과 기존 제어 분기의 상태를 함께 전달한다.
- 숫자는 각각 `%6.2f`, `%5.2f`, `%+5.1f`로 오른쪽 정렬하며 항목 사이는 한 칸이다.
  기대 범위는 속도 134.55kph까지, Ay 절댓값 6.55mps2까지, 조향 ±40도다.
  예상 범위를 초과한 관측값을 숨기기 위한 clipping은 하지 않는다.

## 색상과 전환

- 정상: 밝은 초록, 1초마다.
- 최종 조향 명령 절댓값 40도 이상: 밝은 노랑, 첫 진입 즉시 이후 0.2초마다.
  반올림 전 값으로 판정하고 1e-6도 이내 수치 변환 오차만 허용한다.
  포화 경고에는 별도 WARN 문구를 붙이지 않는다.
- 시작 입력 대기: 노랑 `CTRL WAIT <기존 제어기가 보고한 원인>`.
- 정상 제어 이후 입력 단절·stamp 이상·frame 불일치·PP 실패: 빨강
  `CTRL ERROR <기존 제어기가 보고한 원인>`.
- node 시작 설정 오류는 기존 ROS FATAL 로그에 보존하며, 표시 노드가 수신하면
  빨강 FATAL로 즉시 표시한다. required node 종료는 roslaunch도 보고한다.
- 위치 점프 제동 중에는 숫자 형식을 유지하며 노랑이다.
- Competition/IMU가 아직 없거나 수신 후 0.5초를 넘었거나 NaN/Inf면 해당 숫자를
  고정 폭 `--`로 표시하고 노랑으로 바꾼다. 이전 정상값·0으로 대체하지 않는다.
- 제어 상태 보고가 0.5초 이상 끊기면 빨강 `CTRL ERROR controller timeout`이다.
  이는 MORAI 수신 실패의 판정이 아니다.
- 경고·오류 중 반복 주기는 0.2초다. 첫 정상 복구는 1초 타이머를 기다리지 않고
  즉시 초록으로 출력하며, 그 복구 시점부터 정상 1초 주기로 돌아간다.
- 색상은 TTY에서 적용하고 `NO_COLOR`, `TERM=dumb`, 파일/pipe 출력에서는 생략한다.
  주기는 wall/monotonic clock으로 관리한다. ROS simulation clock 정지와 독립적이다.

## 구조와 경계

`longi_pi_autoware`가 기존 안전/정상 분기와 최종 조향을 관측용 `TerminalState`로
publish하고 `control_terminal.py`가 표시만 담당한다. 제어기 알고리즘·출력 명령·기존
safety timeout은 바꾸지 않는다. alias도 바꾸지 않는다.

기존 `[CTRL][LAT]`, `[CTRL][LONG]` 상세 로그는 ROS 로그 파일·`/rosout`에 남긴다.
launch의 두 제어기 출력은 `log`, 표시 노드 출력만 `screen`이다. 부팅 시 roslaunch의
기본 프로세스 안내는 남는다. `traj_fol_global`도 같은 launch의 표시 형식을 사용한다.
`pp_pi.launch`의 기존 제어기는 대상이 아니다.

MORAI UDP 송신기는 별도 프로세스다. UDP 송신 성공이 MORAI 수신·적용 성공을
보증하지 않으므로 이 화면에서 `Ctrl connected`를 추정하지 않는다.

## 검증

- 고정 폭·출력 주기·포화·즉시 복구·미수신·NaN/Inf: Python 단위검사.
- 실제 ROS subscriber와 PTY: Competition Vx와 IMU Ay, ANSI 초록/노랑/빨강,
  포화 0.2초·정상 1초, 첫 복구 즉시 출력, Ego 미구독 검사.
- 기존 C++ 단위검사와 freshness/pose-jump ROS transport 회귀 검사.
- 설치된 binary·메시지·Python script·launch의 source/build parity 확인.

위 검사는 격리 master에서 수행한다. MORAI 폐루프 주행 검증을 대신하지 않는다.
