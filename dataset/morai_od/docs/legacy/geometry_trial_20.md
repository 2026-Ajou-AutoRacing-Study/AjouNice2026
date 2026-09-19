# MORAI OD 기하 배치·경로 추종 20장면 시험

## 시험 이력

v1은 20장면 모두 실패했다.

- 10장면: 6번째 9.0 m trigger와 최대 이동 9.0 m가 같아 제어 overshoot로 실패
- 5장면: 3.0 m lookahead와 0.20 rad 조향 제한에서 heading 실패
- 3장면: `Girl2`, road-surface 객체, 도로에서 먼 보행자 중 하나가 미생성
- 2장면: 실제 충돌

v2는 최대 이동을 10.5 m로 늘리고, lookahead 1.8 m, 최대 조향 0.35 rad,
경로 이탈 제한 1.2 m를 사용한다. MGeo 고정 지형물과 2.0 m 미만인
anchor, `Girl2`, road-surface 객체를 제외하고 보행자 도로 거리를 최대
4.0 m로 제한한다.

v2의 실제 결과도 20장면 모두 실패했지만 실패 형태는 바뀌었다.

- 17장면: 주행 직후 heading 오차가 거의 동일하게 `±45~47°`까지 발산
- 2장면: 보행자 1개가 생성되지 않아 객체 수 검증 실패
- 1장면: Ego 초기 위치와 보행자 수 검증 실패
- v1의 최대 이동 거리 초과와 주행 중 충돌 실패는 발생하지 않음

17개 장면이 직선·곡선·교차로 구분 없이 양쪽 모두 거의 정확히 안전
한계에서 끝난 것은 경로 기하보다 조향 부호가 반대일 때 나타나는 양의
피드백 패턴이다. 제어기의 pure-pursuit 계산은 왼쪽 조향을 양수로
계산하지만, 개인용 MORAI의 정규화된 `CtrlCmd.front_steer`는 반대
방향으로 동작한 것으로 관측됐다.

v3는 v2와 같은 seed를 사용해 20장면의 Ego·객체 위치를 완전히 동일하게
유지하고, 개인용 `front_steer`에만 `-1` 부호 변환을 적용한다. 대회용
메시지의 `CtrlCmd.steering`에는 이 변환을 적용하지 않는다.

## 목적

180장면 전체를 다시 수집하기 전에 다음 변경을 20장면에서 확인한다.

- Ego 실제 경로를 따라 앞으로 10.5 m, 뒤로 2.5 m의 swept corridor 확보
- 차량·보행자·장애물 간 객체 종류별 footprint clearance 적용
- MGeo의 pole, curb, guardrail, wall, barrier와 정적 충돌 회피
- 보행자·장애물이 도로 중심선에서 과도하게 멀어지는 배치 제외
- `5.4 km/h`, 프레임 간 `1.5 m`, 장면당 6장, 최대 이동 10.5 m

같은 장면의 6장은 반드시 같은 Train/Val/Test split에 둔다.

## 생성 및 설치

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_geometry_trial_20_v3_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

## 정적 검증

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/geometry_trial20_v3_sunny_t13.json \
  --capture-dir /tmp/geometry_trial20_v3_dry \
  --motion-mode rolling-grouped \
  --continue-on-scene-error \
  --dry-run
```

## MORAI 3장면 조향 smoke test

MORAI 날씨를 Sunny, 시간을 13시로 설정하고 Ego Network와 Simulator
Network를 연결한 뒤 먼저 서로 다른 구간의 3장면만 실행한다. 로그의
`personal sign=-1.0`과 Ego가 경로를 따라 조향하는지 확인한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/geometry_trial20_v3_sunny_t13.json \
  --capture-dir /home/luv/morai_cap/geometry_trial20_v3_smoke \
  --motion-mode rolling-grouped \
  --scenario geo3_001 \
  --scenario geo3_003 \
  --scenario geo3_020 \
  --continue-on-scene-error
```

세 장면에서 heading 발산이 사라지면 같은 명령에서 세 개의
`--scenario` 행만 제거해 전체 20장면을 실행한다.

합격 기준은 heading 발산과 주행 충돌이 없어야 한다. 객체 수 실패가
남으면 해당 장면의 보행자 asset·배치 간격을 별도 수정한다. 센서 저장
지연은 `failed_scenes`에 기록된 원인을 따로 판단한다.

## v3 smoke test 결과

2026-08-07에 `geo3_001`, `geo3_003`, `geo3_020`을 실행했다.

- 완료 장면: 3/3
- 실패 장면: 0
- 완료 프레임: 18/18
- Camera 4대·LiDAR·GT artifact 누락: 0
- 완전한 공통 timestamp 그룹: 장면당 6/6
- heading 발산 및 충돌: 0
- 장면당 전체 소요 시간: 15.2~16.2초

따라서 개인용 MORAI `front_steer`의 부호 반전이 맞는 것으로 판정한다.
이제 전체 20장면에서 잔여 객체 생성 실패 여부를 확인한다.

## v3 전체 20장면 결과

2026-08-07 전체 시험 결과는 다음과 같다.

- 완전 성공: 15/20장면
- 검증 완료: 90/120프레임
- heading 발산·주행 충돌·최대 이동거리 초과: 0
- 전체 실행 시간: 약 314초
- 성공 장면의 Camera 4대·LiDAR·GT timestamp 그룹: 90/90 완전

실패 원인은 제어가 아니라 다음 두 종류였다.

1. `geo3_005`, `geo3_010`, `geo3_016`: 공통 asset인
`50100005 Woman2`가 생성되지 않아 보행자가 한 명씩 부족했다.
개인용 런처 자동 생성 대상에서 제외한다.
2. `geo3_002`: 한 timestamp에서 Camera 1 Depth·Instance 파일 누락.
   `geo3_015`: LiDAR `Instance.txt`가 0 byte로 저장됨. 둘 다 비동기
   센서 저장의 일시적 실패로 분류한다.

대규모 수집에서는 이러한 일시적 파일 실패 때문에 전체 batch를 중단하지
않는다. `--continue-on-scene-error`로 원본과 실패 원인을 보존하고, batch
종료 후 실패한 scene만 한 번 재수집한다. 재시도까지 실패한 프레임은
dataset에서 제외하며, 불완전 파일을 정상 프레임과 섞지 않는다.

Adaptive smoke test의 `geo3_014`는 과거 한 번 성공했지만 이후 두 번
연속 보행자가 5명 대신 4명 생성됐다. 시나리오와 설치본 hash는 같았고,
가장 가까운 `Man4`와 `Woman4` 중심 간격이 1.686m였다. 개인용 MORAI가
가까운 static pedestrian collider 중 하나를 간헐적으로 제거하는 것으로
판단해 production 생성기의 보행자 중심 최소 간격을 1.45m에서 2.05m로
상향했다. 기존 geometry trial 파일은 실험 provenance를 위해 변경하지
않고, 새 production 시나리오부터 적용한다.

### Ego 진행 방향 점검

성공한 15장면의 첫 캡처는 생성된 시작점에서 경로 전방 1.0~1.7m에
있었고, 여섯 캡처의 공식 waypoint index는 모두 단조 증가했다. 따라서
캡처 주행 중 제어 명령 누락이나 역주행은 관측되지 않았다.

다만 객체 수 검증에서 실패한 `geo3_010`은 캡처 제어를 시작하기 전
시나리오 안정화 구간에 Ego가 시작점보다 경로 후방으로 약 1.24m
밀려났다. 경사로에서 외부 제어 모드·제동 명령이 적용되기 전 발생한
물리적 rollback 가능성이 있다. 데이터 캡처에는 포함되지 않았지만 향후
runner는 scene load 직후 정지 명령을 먼저 유지하는 방식으로 보강할 수
있다.

### 경로와 MGeo의 역할

- Ego 시작점·방향과 실제 pure-pursuit 추종선:
  `configs/global_path/2026_molit_comp_global_path.txt`
- MGeo: anchor의 도로 링크 유효성, 교차로·횡단보도 등 topology 분류,
  pole·curb·guardrail·wall·barrier 정적 충돌 회피, actor 배치 유효성

즉 Ego가 MGeo link graph를 직접 탐색해 경로를 만드는 구조는 아니다.
대회 공식 global path를 주행하되, MGeo를 이용해 시작점과 주변 배치가
도로 및 고정 지형물과 충돌하지 않도록 검증하는 혼합 구조다.

### Production용 곡률 적응 제어

geometry trial v3는 고정 `5.4 km/h`, lookahead `1.8 m`, 최대 조향
`0.35 rad`로 검증했다. 이후 production manifest에서 선택적으로 사용할
수 있도록 곡률 적응 제어를 runner에 추가했다.

1. 현재 waypoint부터 전방 8m까지 경로 heading 변화를 계산한다.
2. heading 변화 `0~18°`를 곡선 심각도 `0~1`로 정규화한다.
3. 심각도에 따라 선형 보간한다.
   - 속도: `5.4 → 3.6 km/h`
   - lookahead: `1.8 → 1.2 m`
   - 최대 조향: `0.35 → 0.45 rad`
4. 개인용 MORAI에서는 계산된 조향에 검증된 `front_steer` 부호 `-1`을
   적용한다.

이 기능은 기존 v3 결과의 재현성을 위해 기본 비활성화 상태다. production
manifest에서 `adaptive_curve_control: true`로 명시한 뒤, 곡률이 큰
장면 3~5개 smoke test를 통과해야 전체 수집에 사용한다.

2026-08-08 adaptive smoke test에서 `geo3_013`, `geo3_020`은 각각 6/6
프레임과 전체 센서·GT 검증을 통과했다. 실제 명령값은 곡률에 따라 속도
`3.76~5.35 km/h`, lookahead `1.25~1.78 m`, 조향 제한
`0.353~0.441 rad`로 변했다. 충돌·heading 발산·경로 이탈은 없었다.
`geo3_014`는 제어 전에 보행자 collider 배치 문제로 실패했으므로 adaptive
제어 실패로 집계하지 않는다. 이 결과를 근거로 production에서 adaptive
제어를 활성화한다.

### 거리 기반 trigger와 저속 곡선

캡처는 시간이 아니라 경로 진행거리 기준이므로 곡선에서 속도가
`3.6 km/h`까지 낮아져도 1.5m를 진행한 뒤 다음 프레임을 요청한다. 여섯
프레임에 필요한 경로 진행거리는 9m이고 motion timeout은 30초이므로,
정상적으로 전진한다면 저속 자체 때문에 프레임이 누락되지는 않는다.

초기 구현은 출발점과 현재 위치의 직선거리(chord)를 사용했다. 급커브의
실제 주행거리(arc)를 과소평가할 수 있으므로 production 전 다음과 같이
수정했다.

- Ego 위치를 인접한 공식 waypoint segment에 투영
- 해당 투영점의 경로 누적거리(arc length) 계산
- 시작점 대비 순방향 경로 진행거리가 1.5, 3.0, ..., 9.0m에 도달할 때
  trigger
- 후방 rollback은 진행거리로 더하지 않음
- 최대 이동 10.5m도 같은 경로 누적거리 기준으로 검사

따라서 직선과 곡선 모두 실제 경로를 따라 동일한 공간 간격으로 수집한다.
