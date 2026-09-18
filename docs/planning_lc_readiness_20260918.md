# BPP LC 후보 준비 상태 수정 및 검증

> 후속 조사: [최신 18:41 백 및 경계 교차 원인 분석](planning_lc_boundary_audit_20260918.md).
> 아래의 경계 탈락은 14:24 백의 제한적 재생 결과이며, 최신 주행 실패를 뜻하지 않는다.

## 범위

현재 preferred를 주행 중이지만 전방에서 LC가 필요한 경우의 거리 계산 누락과,
승인 대기 중 목표가 사라져도 이전 차선 정보를 유지하던 문제를 수정했다.
코스 lanelet ID를 구현에 추가하지 않았다. 지도와 requester YAML은 이번 작업에서 변경하지 않았다.

최종 요구는 201086 → 201456 진입부터 필수 통과점 201439까지 상대 차량 예측,
충돌 검사, 에고 가감속 제약을 고려해 좌우 차선으로 주행 시간을 줄이는 것이다.
이는 이번 변경에 포함하지 않는다. 앞으로 차선/추월 선택 정책과 LC 궤적 생성·안전검사를
분리해 다루며, 이번 수정은 후자의 기본 동작을 대상으로 한다.

## 변경

- `calc_shift_intervals`: 현재가 preferred인 연속 prefix에서 탐색을 중단하지 않고,
  전방에서 필요한 횡이동 간격을 찾는다. 끊어진 successor 연결을 건너뛰지 않는다.
  횡이동을 찾은 뒤에는 기존 구간 종료/최대 차선 이동 수 보존 동작을 유지한다.
- `update_lanes`: 현재/목표 차선 탐색 실패 시 대기 중인 geometry, 거리, terminal 후보를
  무효화한다. 승인된 실행 중 LC는 기존 geometry를 유지한다.
- LC 판단 로그에 ego lane, 각 lane sequence 길이, 거리 계산 유효성을 추가했다.
- `Insufficient data`를 planner 포인터/현재 차선/목표 차선/인접 차선 없음으로 구분했다.
- Frenet 후보 생성에서 준비 경로 거부, 참조 경로 없음, 샘플링 거부, 생성 수,
  중복 제거, 경계 교차 검사 거부를 각각 기록한다.

## 검증

- 합성 3차로 지도에서 현재 preferred → 전방 좌/우 preferred 변화 회귀 시험은
  수정 전 양방향 실패, 수정 후 통과했다.
- 유틸리티 시험 9개, scene 시험 9개 통과. 기존 disabled 시험 1개는 실행하지 않았다.
- 설치된 shared library를 실제로 로딩하는 상태에서도 18개 통과했다.
- 설치 Build ID: `3412e61e7ffc782d04a6631ba8a871130b2481d2`.
- 설치 SHA256: `dfd7ef709fd708a50d0e90c66b515e210a86df6195a6785978430d6f74a6d239`.
- 개발/설치 ELF Build ID는 동일하다. 설치 시 RPATH 변경 때문에 파일 SHA는 다르다.

## 마지막 백을 사용한 제한된 재생

입력: `planner_logs/bags/planning_rviz_20260918_142413.bag`, 경과 95~116초.
별도 ROS master(11439)에서 BPP만 실행하고 지도/route/odom/accel/objects를 재생했다.
scenario와 autonomous mode는 시험에서 제공했으며 BPP 파라미터는 당시 실행 중
노드에서 읽었다. 이전 전체 실행 상태와 폐루프 차량 반응은 재현하지 않았다.
따라서 두 실행의 LC 시작 시각 차이를 성능 향상 수치로 사용하지 않는다.

수정 후 현재 ego lane이 201470/200130인 상태에서도 `distance_valid=1`이고,
예를 들어 경과 112.498초에 `prepare_samples=7 generated=7 boundary_rejected=7 retained=0`이
기록됐다. 후보 생성 자체를 건너뛰던 문제는 해소됐지만 이 후보들은 여전히 탈락했다.

여기서 boundary rejection은 `check_out_of_bound_paths()`의 판정이다.
pose가 너무 적거나, 차폭을 반영한 궤적 선이 현재 차선 경계와 교차하지 않으면 거부한다.
이는 그 자체로 장애물 충돌 판정도, 도로 밖 침범 판정도 아니다.
따라서 이번 결과를 고주로 LC 전체 해결이나 실제 주행 성공으로 해석하지 않는다.
다음 확인 대상은 준비 구간 이후의 LC 시작/끝과 실제 경계 교차 가능 구간의 일치 여부다.

로그는 `planner_logs/lc_readiness_20260918/`에 보관했다(생성물, Git 제외).
라이브러리는 `planning_ws/install_isolated/lib/`에 설치했다. 플래닝을 자동 재시작하지 않았다.
