# 제어기 터미널 로그 CURRENT

기준일: 2026-10-28
대상 브랜치: `planning/autoware-universe-to-ros1`
대상 런타임: `base_ws/install_aw149`의 Autoware PP-PI controller

이 문서는 `trajectory_follower`의 사람이 읽는 터미널 상태 로그에 대한 현재 구현과
후속 작업 경계를 기록한다. 제어 알고리즘·출력 명령·안전 임계값의 기준 문서는 아니며,
그 값은 기존 YAML과 controller 구현을 따른다.

## 현재 상태

- `purepursuit_autoware`와 `longi_pi_autoware`가 각각 1초에 한 번 상태를 한 줄로 출력한다.
- 정상 상태는 ROS `INFO`, 기동 중 입력 대기는 `WARN`, 정상 제어 이후 안전 정지는
  `ERROR`로 구분한다.
- `WARN` 메시지 본문은 ANSI 밝은 노랑, `ERROR`·`FATAL` 본문은 밝은 빨강이다.
- 상태 출력 주기는 공통 설정의 `logging/status_period_s`로 조절하며 현재값은 `1.0`초다.
- 로그 추가 과정에서 steering, accel, brake 계산과 publish 주기는 변경하지 않았다.
- 수정된 source와 config는 `base_ws/install_aw149`에 선택 설치했다. 다음
  `traj_fol_a` 실행부터 적용된다.

대표 출력 형식은 다음과 같다.

```text
[INFO]  [CTRL][LAT][TRACK] v=... | steer=... | lateral=... | lookahead=... | curvature=... | age(T/S)=...
[INFO]  [CTRL][LONG][ACCEL] v=...->... | accel=... brake=... | steer=... | a(plan/meas)=... | path=... | age(T/S)=...
[WARN]  [CTRL][LAT][WAIT] stale or unavailable trajectory/state | steering=0deg
[ERROR] [CTRL][LONG][SAFE_STOP] ... | accel=0.00 brake=1.00 steer=0deg
```

## 비정상 상황과 색상

색상은 단순 장식이 아니라 운전자가 취해야 할 행동의 긴급도를 나타낸다. 정상 제어를 한
번도 시작하지 못한 입력 대기는 `WARN`과 **밝은 노랑**, 정상 제어 시작 후 제어를 계속할
수 없는 상황은 `ERROR`와 **밝은 빨강**, node를 시작할 수 없는 설정 오류는 `FATAL`과
**밝은 빨강**을 사용한다. 실제 메시지의 `[WARN]`, `[ERROR]`, `[FATAL]` 접두사는
rosconsole 출력 형식이 붙이는 것이며 controller 메시지 본문은 `[CTRL]`부터 시작한다.

| 상황 | 출력 상태 | Severity·색상 | 실제 안전 출력 | 운영자 확인 |
| --- | --- | --- | --- | --- |
| 시작 직후 trajectory/state 대기 | `LAT/WAIT` | `WARN`·밝은 노랑 | `steering=0deg` | 두 입력 topic의 publish 여부와 frame 확인 |
| 시작 직후 trajectory/state/steering/lateral 대기 | `LONG/WAIT` | `WARN`·밝은 노랑 | `accel=0`, `brake=stop_brake_cmd`, `steer=0deg` | 누락 또는 stale로 표시된 입력 확인 |
| 정상 추종 후 입력 수신 중단·source stamp stale | `LAT/SAFE_STOP`, `LONG/SAFE_STOP` | `ERROR`·밝은 빨강 | 횡 `steering=0deg`, 종 `accel=0`, full brake, `steer=0deg` | 실패 이유와 trajectory/state 송신 node 확인 |
| trajectory와 state frame 불일치 | `LAT/SAFE_STOP`, `LONG/SAFE_STOP` | `ERROR`·밝은 빨강 | 위와 같은 안전 정지 | 두 메시지의 `header.frame_id` 확인 |
| 횡제어 target 계산 실패 | `LAT/SAFE_STOP` | `ERROR`·밝은 빨강 | `steering=0deg`; 종제어도 lateral invalid로 안전 정지 | trajectory 형상·차량 pose·횡제어 실패 이유 확인 |
| 시작 파라미터가 유효하지 않음 | `LAT/FATAL`, `LONG/FATAL` | `FATAL`·밝은 빨강 | node가 종료되어 제어 명령을 만들지 못함 | YAML 값과 launch에서 실제 로드된 파라미터 확인 |

### 색상이 적용되는 대표 예시

아래 코드 블록은 저장 로그에서도 읽을 수 있도록 ANSI 문자를 제외한 모습이다. 실제
터미널에서는 `WAIT` 메시지 본문이 밝은 노랑, `SAFE_STOP`과 `FATAL` 메시지 본문이
밝은 빨강으로 보인다. 숫자는 예시이며 실제 값은 설정과 입력 상태를 따른다.

```text
[WARN]  [CTRL][LAT][WAIT] stale or unavailable trajectory/state | steering=0deg
[WARN]  [CTRL][LONG][WAIT] trajectory unavailable; state unavailable; steering unavailable; lateral unavailable | accel=0.00 brake=1.00 steer=0deg

[ERROR] [CTRL][LAT][SAFE_STOP] trajectory and state frame mismatch | steering=0deg
[ERROR] [CTRL][LONG][SAFE_STOP] state rx stale 0.73s | accel=0.00 brake=1.00 steer=0deg
[ERROR] [CTRL][LONG][SAFE_STOP] lateral unavailable; lateral: pure pursuit target unavailable | accel=0.00 brake=1.00 steer=0deg

[FATAL] [CTRL][LAT][FATAL] invalid purepursuit_autoware parameter
[FATAL] [CTRL][LONG][FATAL] invalid longi_pi_autoware parameter
```

색상만 직접 확인하려면 controller를 실행하지 않고 다음 명령을 사용할 수 있다.

```bash
printf '\033[1;33m%s\033[0m\n' '[WARN]  [CTRL][LAT][WAIT] input unavailable | steering=0deg'
printf '\033[1;31m%s\033[0m\n' '[ERROR] [CTRL][LONG][SAFE_STOP] state stale | accel=0.00 brake=1.00 steer=0deg'
printf '\033[1;31m%s\033[0m\n' '[FATAL] [CTRL][LONG][FATAL] invalid parameter'
```

### 색상 해석 시 주의사항

- `WAIT`는 시작 전 입력 준비 상태이므로 노랑이다. 차량이 이미 움직이는 상황에서 계속
  나타난다면 정상 상태로 간주하지 말고 입력 구성을 확인한다.
- `SAFE_STOP`은 정상 제어를 한 번 시작한 뒤 입력이 끊기거나 계산이 실패한 상태이므로
  빨강이다. 색상과 별개로 실제 안전 여부는 같은 줄의 `accel`, `brake`, `steer`와
  `/ctrl_cmd`를 함께 확인한다.
- `HOLD`와 `PATH_END`는 의도된 정지 상태라서 `INFO`이며 빨강으로 표시하지 않는다.
- 위치 점프 감지는 현재 별도 `ROS_WARN("Pose jump ...")`를 한 번 출력하고 1초 제동을
  수행한다. 이 메시지에는 controller가 넣는 사용자 정의 ANSI 노랑이 적용되지 않는다.
- ANSI escape는 메시지 본문에 포함되므로 rosbag, `/rosout`, 파일 로그에도 남을 수 있다.
  `less -R`처럼 ANSI를 해석하는 뷰어를 사용하거나 파싱 전에 escape를 제거한다.
- `NO_COLOR` 환경 변수나 터미널 종류를 자동 감지하는 구현은 현재 없다. 색을 지원하지
  않는 터미널에서도 severity와 `[CTRL][영역][상태]` 문자열로 같은 판단이 가능해야 한다.

## 상태 이름과 판정

| 구분 | 상태 | 의미 |
| --- | --- | --- |
| 횡제어 | `START` | node 시작과 입출력 topic·로그 주기 표시 |
| 횡제어 | `TRACK` | 유효한 trajectory/state로 조향 계산 중 |
| 횡제어 | `WAIT` | 아직 정상 출력을 만든 적이 없고 입력을 기다리는 중 |
| 횡제어 | `SAFE_STOP` | 정상 제어 이후 입력 stale/invalid 또는 frame 불일치 |
| 종제어 | `START` | node 시작과 입출력 topic·로그 주기 표시 |
| 종제어 | `ACCEL`, `BRAKE`, `DECEL`, `SMOOTH_STOP`, `COAST` | 현재 종방향 제어 모드 |
| 종제어 | `HOLD`, `PATH_END` | 정지 유지 또는 trajectory 끝 도달 |
| 종제어 | `WAIT` | 아직 정상 입력 조합이 없어서 full brake 대기 중 |
| 종제어 | `SAFE_STOP` | 정상 제어 이후 입력 freshness/frame/lateral validity 실패 |

종제어는 `/control/autoware/failure_reason`을 구독한다. 따라서 lateral validity가
false일 때 가능한 경우 횡제어 실패 이유를 `SAFE_STOP` 원인에 함께 표시한다.

## 변경 파일

- `config/trajectory_follower_autoware.yaml`: `logging/status_period_s: 1.0`
- `src/purepursuit_autoware.cpp`: `[CTRL][LAT]` 상태, severity, ANSI 색상
- `src/longi_pi_autoware.cpp`: `[CTRL][LONG]` 상태, 입력 실패 원인, ANSI 색상
- `test/autoware_freshness.test`: stale-source 시험만 `0.5`초 override
- `README.md`: 운영자용 로그 해석 안내

시험의 `trajectory_source_timeout_s: 0.5`는 production의 cached trajectory 허용값
`10.0`초와 독립적으로 stale-source 거부를 검증하기 위한 test-only 설정이다.
production timeout을 바꾼 것이 아니다.

## 완료된 검증

- `git diff --check`: 통과
- `catkin_make --only-pkg-with-deps trajectory_follower`: 통과
- C++ 단위검사: 38/38 통과
- `autoware_freshness.test` ROS transport 검사: 1/1 통과
- 설치 대상 `base_ws/install_aw149`: source/config 반영 및 ELF dependency 누락 없음
- 통합 검사에서 노랑 `WAIT`, 빨강 `SAFE_STOP`, 정상 `TRACK`과 종제어 모드 출력을 확인

ROS integration test는 격리된 ROS master에서 수행했다. 실제 MORAI 폐루프 주행이나
현장 터미널 가독성 승인을 의미하지 않는다.

## 후속 확인

실제 실행 시 다음 순서로 확인한다.

```bash
source scripts/workspace/source_aw149.sh
traj_fol_a
```

1. controller 시작 전에 노란 `WAIT`가 약 1초 주기로 보이는지 확인한다.
2. trajectory/state 입력 후 `LAT/TRACK`과 `LONG/<mode>`가 각각 약 1 Hz인지 확인한다.
3. 안전한 조건에서 입력을 끊었을 때 빨간 `SAFE_STOP`과 full brake가 함께 보이는지 확인한다.
4. `/ctrl_cmd` publisher가 `longi_pi_autoware` 하나뿐인지 확인한다.
5. 파일로 저장한 로그를 파싱할 때는 ANSI escape를 제거한다.

로그 문구나 필드를 바꿀 때는 제어 계산과 safety timeout을 함께 손대지 않는다. 상태 판정
변경이 필요하면 startup `WAIT`와 운행 중 `SAFE_STOP`의 경계를 유지하고 freshness transport
검사를 다시 실행한다.

## 작업 트리 주의

이 로그 작업과 무관한 `planning_scenario_validator`의 campaign script와 test에는 기존 사용자
수정이 있다. 제어 로그 후속 작업에서 해당 파일을 정리하거나 되돌리지 않는다.
