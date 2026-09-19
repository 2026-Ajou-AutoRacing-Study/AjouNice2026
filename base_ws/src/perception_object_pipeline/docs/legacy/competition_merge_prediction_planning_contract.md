# 대회 합류 미션의 Prediction–Planning 계약

## 목적과 기준

이 문서는 대회 규정에서 말하는 `끼어들기`, K-City v60 Lanelet2 경로,
현재 ROS1 Planning 포팅의 `PredictedObjects` 소비 방식, 그리고 향후 다중 경로
Prediction을 Planning에 연결하는 기준을 정리한다.

분석 기준은 다음과 같다.

- 규정: `docs/jsw/competition_rules/규정집 v1.0 : 국토부 KATRI 대학생
  AISW 모빌리티 경진대회.md`.
- 지도: `kcity_map/lanelet_map/lanelet2_map_v60.osm`.
- 공식 Route 참고 구현:
  `base_ws/src/bringup/validation/scripts/kcity_competition_live_route_publisher.py`.
- Planning 최신 코드 기준: `origin/planning/sub/morai_validation_hj`.
- `origin/AW`는 위 Planning tree에 2026-08-17 Autoware 원본 검증 자료를 한
  커밋 추가한 브랜치다.
- `origin/planning/autoware-universe-to-ros1`은 이름과 달리 최신 Planning
  구현 브랜치가 아니다.

## 용어를 먼저 구분한다

### 대회 규정의 끼어들기

규정에는 다음과 같이 적혀 있다.

> 회전 교차로, 합류 구간 등에서 NPC 차량 충돌에 유의하며 끼어들기를 진행한다.

여기서 끼어들기는 **Ego가 NPC 교통 흐름 사이로 진입하는 동작**이다.

- 로터리 차량 사이로 Ego가 진입한다.
- 고주로 차량 사이로 Ego가 합류한다.
- 필요하면 Ego가 주행 차로를 변경하며 추월한다.

### 일반 자율주행 코드의 cut-in

Autoware와 현재 Dynamic Avoidance 코드에서 `cut-in`은 주로 반대 의미다.

- 다른 차량이 Ego 차로 또는 Ego 계획 경로로 들어온다.
- Ego가 상대 차량 사이로 들어가는 merge/lane change와 구분한다.

따라서 `other-vehicle cut-in` 모듈 하나로 대회의 Ego 합류 미션을 해결할 수
없다.

## Planner는 미션 이름을 직접 선택하지 않는다

현재 Planner는 `고주로 합류`, `로터리 끼어들기`, `추월` 같은 문자열 모드를
수동으로 선택하지 않는다.

- `/planning/scenario_planning/scenario`는 주로 `LANEDRIVING`과 `PARKING`을
  구분한다.
- `/planning/mission_planning/route`의 `LaneletRoute`가 주행할 Lanelet과 선호
  Lanelet을 제공한다.
- Lanelet2의 `following`, `left`, `right`, regulatory element가 어떤 행동
  모듈이 필요한지 결정한다.
- 객체의 현재 위치·속도·예측 경로가 해당 행동의 안전성을 결정한다.

정상적인 고정 Route라면 사용자가 주행 중 합류 모드를 수동으로 바꿀 필요는
없다. Route가 목표 차로를 표현하지 못하거나 GPS 음영 구간의 랜덤 미션처럼
외부 결정이 필요한 경우에만 Route 갱신 또는 외부 행동 요청이 필요하다.

## 경로 confidence와 객체 confidence는 다르다

- 객체 classification confidence는 `이 객체가 차량인가`를 나타낸다.
- predicted path confidence는 `이 차량이 이 미래 경로를 택할 가능성이
  얼마인가`를 나타낸다.

Motion Velocity Planner의 기본 `0.5` threshold는 객체 classification
confidence다. Intersection의 `0.05` threshold는 predicted path confidence다.
두 값을 같은 정책으로 해석하면 안 된다.

## 현재 Planning의 다중 경로 소비 방식

모듈별로 정책이 제각각이다.

- 교차로.
  - path confidence `0.05` 미만을 제거한다.
  - 남은 경로를 모두 시간·공간 충돌 검사한다.
  - 하나라도 Ego 통과 시간과 겹치면 위험으로 판단한다.
- 횡단보도·사각지대.
  - 사실상 모든 predicted path를 검사한다.
- Dynamic Avoidance의 상대 차량 cut-in.
  - 상대 객체의 confidence가 가장 높은 Top-1 path만 검사한다.
- Run-out.
  - confidence가 가장 높은 Top-1 path만 사용한다.
  - 대상은 주로 보행자·자전거·오토바이다.
- Motion Velocity Planner.
  - predicted path를 본격적으로 비교하지 않는다.
  - 객체의 현재 위치·속도를 Ego trajectory에 투영해 정지·감속·순항을
    결정한다.
- 현재 ROS1 Lane Change 안전 검사.
  - 목표 차로 주변 객체의 현재 위치를 검사한다.
  - 상대 차량의 여러 미래 경로를 충돌 검사하지 않는다.

Top-1 선택은 Planning 전체에서 메시지의 나머지 path를 삭제한다는 뜻이 아니다.
해당 모듈 내부에서만 가장 높은 path를 사용하며, 다른 모듈은 같은 메시지의
다른 path를 다시 검사할 수 있다.

## v60 로터리 Route의 실제 관계

공식 Ego Route의 로터리 구간은 다음과 같다.

```text
200968
  -> 201095
       -> 201098
            -> 201096
                 -> 201087
                      -> 201086
```

Routing graph의 갈림길은 다음과 같다.

```text
201095
  |- 201097
  `- 201098  # 공식 Route.

201096
  |- 200480
  `- 201087  # 공식 Route.
```

확인 결과는 다음과 같다.

- 위 전이는 모두 Lanelet2 `following` 관계다.
- `left` 또는 `right` lateral relation이 아니다.
- 로터리 진입·순환·이탈에서 Lane Change 모듈은 활성화되지 않는다.
- 해당 v60 Lanelet에는 `turn_direction`이 없어 현재 Intersection 모듈의
  로터리 충돌 검사 대상으로도 들어가지 않는다.
- 현재 competition launch에는 Autoware 전용 Roundabout 모듈이 없다.

따라서 현재 로터리에서 Ego path 자체는 Route centerline으로 생성되지만, NPC
차량의 여러 미래 경로를 이용해 진입 가능한 gap을 계산하는 전용 로직은 없다.

`sensor_cam_raw_roundabout1_roundabout_only.bag`의 Ego는 lanelet `200968`에
정지해 있었다. 이 bag은 NPC prediction 평가에는 유효하지만 Ego의 실제 로터리
진입 동작을 검증하는 자료는 아니다. 위 관계는 bag이 아니라 v60 routing graph와
공식 Route를 직접 분석한 결과다.

## v60 고주로 합류의 실제 관계

공식 Ego Route의 진입부는 다음과 같다.

```text
201087 -> 201086 -> 200101
```

`201086`에는 다른 접근 Lanelet도 합쳐진다.

```text
201087 --\
201089 ----> 201086 -> 200101
201100 --/
```

확인 결과는 다음과 같다.

- Ego의 `201087 -> 201086 -> 200101`은 모두 `following` 관계다.
- 초기 고주로 합류는 lateral Lane Change가 아니다.
- 현재 Lane Change 안전 모듈은 이 합류를 담당하지 않는다.
- 현재 Merge gap acceptance 모듈도 없다.
- 현재 Planner는 이 구간을 기본적으로 Route를 따르는 Lane Following으로
  처리한다.

따라서 현재 고주로 초기 합류에서는 NPC의 1~3초 미래 경로를 이용해 안전한
진입 gap을 계산하지 않는다. Motion Velocity Planner의 현재 위치·속도 기반
처리가 일부 위험을 막을 수 있지만, 합류 시공간 점유를 직접 비교하는 방식은
아니다.

## 고주로 bag에서 확인한 Lane Change

`sensor_cam_raw_highway1.bag`의 Ego trajectory를 v60에 map matching한 결과,
초기 합류 이후 실제 lateral 이동이 확인됐다.

```text
201086 -> 200101
200101 --left--> 201434
201434 --left--> 200152
```

- 약 11~12초에 `200101`에서 왼쪽 `201434`로 이동했다.
- 이후 `201434`에서 다시 왼쪽 `200152`로 이동했다.
- 이는 bag의 Ego가 실제 차선을 바꿨다는 뜻이다.
- bag에는 당시 Planning Route와 Planning 출력이 없으므로, 이 동작이 현재
  Planner가 자동 생성한 결과라고 단정할 수는 없다.

현재 Planner가 추월 Lane Change를 실행하려면 다음 중 하나가 필요하다.

- Route의 preferred Lanelet이 현재 Lanelet과 다르다.
- 외부 Lane Change 요청이 들어온다.

느린 앞차만 보고 스스로 추월 차로를 고르는 완전한 overtaking policy는 현재
구현의 범위가 아니다. Lane Change가 요청돼도 안전 검사는 상대 차량의 현재
위치를 중심으로 하며, 뒤에서 빠르게 접근하거나 곧 목표 차로에 들어올 차량의
미래 path를 사용하지 않는다.

## 원본 Autoware의 정책

원본 Autoware도 모든 모듈이 동일한 다중 경로 정책을 사용하지 않는다.

- Obstacle Cruise의 상대 차량 cut-in.
  - `num_of_predicted_paths: 1`이 기본값이다.
  - confidence가 높은 path부터 설정 개수만큼 사용한다.
- Lane Change safety.
  - `collision_check.use_all_predicted_paths: true`가 기본값이다.
  - Ego가 차선을 변경할 때 주변 객체의 모든 path를 검사한다.
- Roundabout과 Intersection.
  - `min_predicted_path_confidence: 0.05` 이상인 path를 충돌 검사한다.
- Run-out.
  - `confidence_filtering.only_use_highest: true`가 기본값이다.

즉 Autoware도 무조건 Top-1 또는 무조건 Top-N인 것이 아니다. 행동별 위험 특성에
맞게 정책을 다르게 둔다.

## 다중 경로 Prediction의 목적

Prediction의 목적은 Planning에 상대 차량의 미래 정답 하나를 고르게 하는 것이
아니다.

```text
상대 차량 A.
  |- 로터리 순환: confidence 0.48.
  `- 출구 진출: confidence 0.52.

Planning.
  |- 두 경로가 Ego 계획과 만드는 충돌 위험을 각각 계산한다.
  `- 통과·감속·양보·정지 중 Ego 행동을 선택한다.
```

Planning이 선택해야 하는 것은 `상대 차량이 실제로 어느 길로 갈지`가 아니라,
불확실한 미래들을 고려한 **Ego 행동**이다.

다중 경로를 발행만 하고 현재 Planning을 그대로 두면 효과는 제한적이다.

- Top-1만 보는 모듈에서는 추가 path가 무시된다.
- 미래 path를 안 보는 Lane Change와 Motion Velocity Planner에는 영향이 없다.
- 교차로처럼 모든 path를 보는 모듈에서만 즉시 활용된다.

그러나 다중 경로 interface는 로터리의 `순환 vs 출구`, 고주로의 `차로 유지 vs
차선 변경` 불확실성을 표현하고 향후 tail error를 줄이기 위한 필수 기반이다.

## 권장 구현 순서

### 1. Prediction Top-N과 confidence를 먼저 신뢰 가능하게 만든다

- routing graph상 연결 가능한 path만 생성한다.
- confidence 합을 1로 정규화한다.
- top-1 ADE/FDE, minADE-K/minFDE-K, path recall, calibration을 함께 평가한다.
- 후보 수를 늘려 정답을 우연히 포함하는 것만으로 성공으로 보지 않는다.

### 2. 고주로 Lane Change safety를 Autoware 방식으로 보강한다

- 현재 위치뿐 아니라 모든 plausible predicted path를 검사한다.
- Ego lane-change trajectory와 상대 path의 시간별 footprint 충돌을 검사한다.
- 뒤에서 빠르게 접근하는 차량의 TTC를 포함한다.
- confidence가 극히 낮은 path만 제거한다.

### 3. Roundabout gap acceptance를 포팅한다

- v60 로터리 conflict area를 명시한다.
- path confidence `0.05` 이상인 NPC path를 모두 검사한다.
- Ego의 로터리 진입·통과 시간과 NPC 점유 시간을 비교한다.
- 충돌 가능성이 있으면 감속·양보·정지를 구분한다.

### 4. 고주로 successor merge gap acceptance를 추가한다

- 여러 predecessor가 하나의 successor로 합쳐지는 Lanelet topology를 감지한다.
- Lane Change가 아니어도 합류 conflict zone을 생성한다.
- target-lane NPC의 predicted path와 Ego 진입 시간을 비교한다.

### 5. 확률 기반 위험 정책을 단계적으로 적용한다

- 낮은 확률 path 하나 때문에 항상 정지하는 구조를 피한다.
- path confidence, TTC, 충돌 심각도, 제동 여유를 함께 사용한다.
- 첫 버전에서는 모든 plausible path에 대한 보수적 충돌 검사로 시작하고,
  실제 bag에서 과도한 정지율을 측정한 뒤 비용 정책을 추가한다.

## 검증 기준

- Prediction 단독.
  - top-1 FDE와 minFDE-K를 분리한다.
  - branch별 path recall과 confidence calibration을 확인한다.
- Planning 단독.
  - 동일한 객체 path 입력에서 Ego 통과·감속·정지 결정이 재현되는지 확인한다.
  - Top-N 적용 전후 false-safe와 unnecessary-stop을 함께 측정한다.
- End-to-end.
  - 로터리 진입 성공률과 최소 NPC 간격을 측정한다.
  - 고주로 합류 성공률, TTC, 합류 대기 시간을 측정한다.
  - 고주로 추월 중 앞·뒤 차량과의 최소 간격을 측정한다.
  - 충돌 회피뿐 아니라 대회 총 주행 시간 악화 여부도 확인한다.

## 현재 결론

- 대회의 끼어들기는 Ego merge/gap acceptance 문제다.
- 로터리와 고주로 초기 합류는 v60에서 Lane Change가 아니라 `following`
  관계의 successor 전이이다.
- 현재 Planner는 두 합류에서 상대 차량의 미래 path를 충분히 사용하지 않는다.
- 고주로 추월은 실제 lateral Lane Change지만 현재 안전 검사는 미래 path를
  사용하지 않는다.
- 따라서 Prediction의 다중 경로 발행과 Planning의 Top-N 소비를 한 단계로
  연결해야 실제 주행 성능이 개선된다.
