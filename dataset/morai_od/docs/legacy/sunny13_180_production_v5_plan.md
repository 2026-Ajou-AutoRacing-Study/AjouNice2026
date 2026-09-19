# MORAI OD production v5 개선 계획과 검증 순서

## 범위

v5는 PointPainting 전용 데이터가 아니라 BEVFusion·Fast-BEV 등 향후 다중 센서
BEV 학습에도 재사용할 수 있는 정적 객체 배치를 목표로 한다. 기존 v3/v4
시나리오와 수집 결과는 재현 자료로 보존하고 덮어쓰지 않는다.

이번 버전에는 split의 공간 중복 제한을 넣지 않는다. split은 시나리오 단위
70/15/15만 유지한다. 데이터 규모가 커진 뒤 실제 검증 편향이 확인될 때만
공간 분리를 다시 검토한다.

## 확정한 배치 정책

### 1. 4.5 m 기준점과 곡선

6프레임은 약 9 m의 주행 구간에서 촬영되므로 객체 배치 기준은 최초 Ego가
아니라 공식 경로를 따라 4.5 m 전진한 중간점이다. 기준점의 yaw도 최초 yaw를
복사하지 않고 그 4.5 m 지점의 경로 접선으로 계산한다.

따라서 우회전 장면에서는 기준점이 우회전 경로 위로 이동하고 전·좌·우·후
좌표축도 회전한다. 여기서 전방은 차량의 최초 직선 방향이 아니라 촬영 구간
중앙에서 앞으로 진행할 방향이다.

### 2. 카메라 방향 구역

기준점의 경로 좌표계에서 방위각으로 `front`, `left`, `right`, `rear`를 나눈다.
목표 비율은 후보 선택의 가중치이며 강제 quota가 아니다. MGeo 도로 존재 여부,
횡단보도, Ego swept corridor 및 충돌 안전 검사가 항상 우선한다.

| 객체 | Front | Left | Right | Rear | 의도 |
|---|---:|---:|---:|---:|---|
| 차량 | 50% | 17% | 13% | 20% | 전방 회피와 후방 끼어들기 모두 학습 |
| 보행자 | 65% | 17% | 13% | 5% | 전방·측방 보행자 미션 중심 |
| 장애물 | 70% | 16% | 12% | 2% | 이미 지나간 후방 장애물의 과표집 방지 |

좌우를 50:50으로 만들지 않는다. K-City 우측통행에서 왼쪽 차선과 대향차가 더
자주 보이는 자연스러운 구조를 인정해 왼쪽 목표를 조금 높게 두었다.

### 3. 차량 진행 방향

차량은 같은 방향 62%, 대향 18%, 교차·합류 20%를 목표로 한다. 대향차를
완전히 제거하면 실제 도로 배경과 카메라·LiDAR 표현이 빈약해지므로 남긴다.
다만 v4 정적 결과의 대향차 34.5%는 대회 미션 대비 과하다고 판단해 줄인다.

### 4. 보행자·장애물과 도로

일반 보행자는 MGeo 도로 표본으로부터 4 m 이내를 유지한다. 횡단보도 topology는
목표 방향에 실제 crosswalk polygon이 있을 때 그 polygon을 우선한다. 장애물도 주행 회랑을
막지 않으며 전방·측방 위주로 둔다. Speed_Bump는 대회 OD 대상과 미션에
해당하지 않아 자동 생성 풀에서 계속 제외한다.

6프레임 촬영의 중심 이동 거리는 약 9 m지만, actor 및 MGeo 정적 구조물
충돌 검사는 시작점 전방 15 m까지 수행한다. 이는 차체 전방 오버행과 마지막
trigger 처리 중 이동량을 포함하기 위한 값이다. 기존 10.5 m 회랑에서는
경로 전방 13.50 m, 중심선 거리 1.03 m의 보행자가 검사 범위 밖으로 빠져
5번째 trigger 뒤 Ego와 충돌한 사례가 확인됐다.

버전 wrapper가 회랑 길이를 바꿀 때 실제 검사에도 적용되도록
`route_corridor_points()`의 기본값은 호출 시점에 해석한다. 생성 metadata의
`geometry_policy.ego_route_ahead_m`와 실제 검사 길이는 항상 같아야 한다.

### 5. 반복 실패 위치

`2020_Kia_Stinger` 자체는 정상 asset이므로 제거하지 않는다. 다만
`A2256W000131`, `(-42.420, -531.344)` 주변 6 m는 GPS 음영 돔 옆에서 동일한
객체 누락이 반복됐으므로 차량 후보 위치에서만 제외한다. 음영 구간 전체는
최종 OD 데이터에 소량 포함할 가치가 있으므로 제외하지 않는다.

최초 live smoke에서 `s13p5_005`의 Stinger와 DEFAULT_MPV가 세 번 연속
동일하게 누락됐다. 네트워크와 나머지 actor는 정상이라 일시 오류가 아닌 배치
실패로 판정했다. 두 asset은 다른 장면에서 정상 로드된 이력이 있으므로 자동
풀에서 제거하지 않고 다음 두 위치 주변 6 m만 추가 제외한다.

- Stinger: `A2256W000075`, `(-86.211, -358.250)`
- DEFAULT_MPV: `A2256W000324`, `(-80.533, -361.721)`

두 위치를 옮긴 뒤에도 설치본 해시가 갱신된 상태에서 동일 두 asset만 다시
누락됐다. 두 asset은 과거 다른 scene에서 함께 정상 로드됐으므로 asset 자체를
격리하지 않는다. 대신 곡률 27.4도의 복잡한 교차로 anchor index `154`를 v5
후보에서 제외하고 다른 intersection anchor로 교체한다.

교체한 anchor index `986`에서도 동일 두 asset만 누락됐다. 당시에는 위치와
anchor 가설을 기각하고 두 template를 조건부 불안정으로 판정했다.

2026-08-22에는 알려진 정상 차량 control을 매 probe 직전에 넣고 두 위치에서
다시 검증했다. Stinger와 DEFAULT_MPV 모두 2/2 생성됐으므로 asset 고유
quarantine은 해제한다. 다만 과거 장면 실패가 있었던 anchor index `154`와
기록된 exclusion zone은 위치·장면 위험을 보수적으로 피하기 위해 유지한다.

## 구현·검증 순서

1. 공통 생성기에 경로 접선 기준의 4방향 sector와 metadata 기록을 추가한다.
2. 차량 방향·진행관계, 보행자와 장애물의 클래스별 목표 가중치를 적용한다.
3. 반복 실패 좌표만 경험적 exclusion zone으로 적용한다.
4. 180개를 정적 생성해 JSON 수, actor 수, 거리·충돌 제약과 실제 분포를
   자동 집계한다.
5. 여섯 scene type smoke test를 개인용 MORAI에서 수행한다.
6. 실패가 없으면 전체 180개를 수집한다. 일부 센서 파일 누락은 runner report에
   남기고 `--continue-on-scene-error`로 다음 장면을 계속한다.

공간 기반 split 중복 제한은 위 순서에 포함하지 않는다.

## 정적 생성·검증 결과

180개 생성과 직렬화 후 재검증을 완료했다. split은 126/27/27이고, 총 actor는
차량 538, 보행자 629, 장애물 280개다.

| 객체 | Front | Left | Right | Rear |
|---|---:|---:|---:|---:|
| 차량 | 50.8% | 15.2% | 7.8% | 26.2% |
| 보행자 | 65.5% | 19.0% | 10.1% | 5.4% |
| 장애물 | 71.4% | 17.5% | 10.4% | 0.7% |

차량 진행관계는 같은 방향 65.3%, 대향 25.6%, 교차·합류 9.1%다. 목표값과
완전히 같지 않은 것은 quota보다 실제 MGeo 후보와 충돌 안전을 우선하기
때문이다. 대향차는 v4의 34.5%에서 25.6%로 줄었고 삭제되지는 않았다.

곡선 장면에서 최초 Ego yaw와 4.5 m 기준점의 경로 접선 yaw 차이는 최대
22.99도였다. 따라서 우회전·좌회전에서 기준 좌표계가 실제 경로와 함께
회전함을 확인했다. GPS 음영 돔 반복 실패 exclusion 안에 배치된 차량은 0대다.

정적 검증 재실행:

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/validate_scenario_distribution.py \
  dataset/morai_od/metadata/sunny13_180_production_v5
```

## 다음 단계: MORAI live smoke

먼저 v5를 개인용 MORAI 시나리오 폴더에 설치한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_sunny13_180_production_v5_set.py \
  --install-only \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

`--install-only`는 정적 검증이 끝난 기존 180개를 복사하므로 actor layout을
다시 생성하지 않는다. 이 옵션 없이 실행하면 180개를 재생성한 다음 설치하며,
진행 상황은 10개 장면마다 출력된다.

MORAI를 Sunny 13시로 설정하고 Ego Network·Capture Mode를 연결한 뒤 여섯
scene type을 한 개씩 수집한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v5_sunny_t13.json \
  --capture-dir /home/luv/morai_cap/sunny13_v5_smoke6 \
  --motion-mode rolling-grouped \
  --scenario s13p5_001 \
  --scenario s13p5_002 \
  --scenario s13p5_003 \
  --scenario s13p5_004 \
  --scenario s13p5_005 \
  --scenario s13p5_006 \
  --scenario-load-attempts 1 \
  --continue-on-scene-error
```

완료 조건은 여섯 장면, 장면당 6프레임, 각 프레임 Camera 1~4와 LiDAR 및
활성화된 GT 파일 존재다. live smoke 통과 전에는 v5 전체 180개 수집으로
넘어가지 않는다.
