# 동적장애물 자동화 인수인계

작성일: 2026-08-10
목적: MORAI에 저장한 `dynamic_*.json` 시나리오를 AJOUNICE 스택으로 반복 검증하고,
조건별 결과표를 남기는 작업을 다음 담당자가 안전하게 이어갈 수 있게 한다.

## 범위와 안전 경계

이 작업은 **동적장애물 검증 자동화 인프라**만 다룬다. 행동 플래너의 동적장애물 판단
로직, MVP 로직, 속도 정책을 이 작업에서 변경하지 않는다.

- 변경된 플래너 런타임은 [morai_scenario_validation.py](../scripts/runtime/morai_scenario_validation.py)뿐이다.
- 필수 경로 발행 보정은 [kcity_competition_route.launch](../base_ws/src/bringup/launch/kcity_competition_route.launch)에만 있다. `v60` 맵과 그에 대응하는 route definition을 명시해, ScenarioLoad 뒤의 kroute가 빈 경로 정의로 종료되지 않게 한다.
- 사용자 데이터인 `planner_logs/log_data_06.csv`, `log_data_07.csv`, `log_data_08.csv`와 `artifacts/`는 수정·삭제·커밋하지 않는다.
- 아직 커밋하거나 푸시하지 않았다. 다음 담당자는 자동화 관련 파일만 별도 커밋으로 묶고, 다른 브랜치 변경과 섞지 않는다.

## 구성

```text
MORAI 시나리오 JSON
  └─ /ScenarioLoad (rosbridge WebSocket)
       └─ MORAI ROS 상태·객체·충돌 토픽
            └─ UDP sensing/localization/planner/kroute/follower
                 └─ trajectory_validator + 결과 JSON/CSV
```

주요 파일은 아래와 같다.

| 구분 | 파일 | 역할 |
| --- | --- | --- |
| 단일 실행 | [run_morai_scenario_validation.sh](../scripts/runtime/run_morai_scenario_validation.sh) | ROS 환경, 시스템 Python, `AJOUNICE_ROOT`를 준비하고 Python runner를 실행 |
| 검증 본체 | [morai_scenario_validation.py](../scripts/runtime/morai_scenario_validation.py) | ROS 스택 기동, ScenarioLoad, kroute/follower/validator 실행, 결과 저장 |
| 속도별 배치 | [run_morai_scenario_batch_validation.sh](../scripts/runtime/run_morai_scenario_batch_validation.sh) | 파일명의 `egoXX` 별로 스택을 재기동하고 결과를 합침 |
| 결과 집계 | [summarize_morai_scenario_validation.py](../scripts/runtime/summarize_morai_scenario_validation.py) | 여러 세션의 `summary.json`을 결과 CSV/Markdown으로 합침 |
| 네트워크 프로필 생성 | [create_morai_scenario_load_network.py](../scripts/runtime/create_morai_scenario_load_network.py) | MORAI 원본 프로필을 덮어쓰지 않고 `/ScenarioLoad`용 프로필 생성 |
| 단위 테스트 | [test_morai_scenario_validation.py](../scripts/runtime/test_morai_scenario_validation.py) | ROS 없이 파일명 파싱·동적 반응·재연결·궤적 유효성 검증 |
| 시나리오 배포본 | [morai_kcity_dynamic_validation](../configs/scenario/morai_kcity_dynamic_validation/README.md) | 동적 시나리오 압축본, 해시, 설치 방법 |

## MORAI 설정

MORAI Network File Manager에서 다음 전용 프로필을 사용한다.

```text
ajounice_full_ros_scenario_load.json
```

프로필 위치는 다음이다.

```text
$HOME/Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/25.S4.MolitComp03/
```

ROS Bridge 주소는 이 PC 기준 `192.168.0.5:9090`이다.

- **Ego Network > Cmd Control**은 기존 UDP `MoraiCmdController` (`9315` / `9314`)를 유지한다.
- **Ego Network > Publisher, Subscriber, Service > ROS**는 상태·객체·충돌 및 `/ScenarioLoad`용이다.
- **Simulator Network > ROS**도 같은 Bridge 주소를 쓴다.
- Ego 제어 모드는 `AV-ExternalCtrl`이어야 한다.

현재 프로필은 Publish-Subscribe 방식의 `/ScenarioLoad`를 사용한다. Service Model의
`/Service_MoraiSL`은 아직 구성·검증하지 않았다.

## 단일 시나리오 실행 절차

기존 AJOUNICE ROS launch를 모두 종료하고 MORAI만 연다. 다음 명령으로 한 건부터
확인한다.

```bash
cd ~/Desktop/AjouNice2026

MORAI_ROSBRIDGE_HOST=192.168.0.5 \
scripts/runtime/run_morai_scenario_validation.sh \
  --execute \
  --scenario dynamic_adult_loc01_obj02_front_crosswalk_ego20_act07_pos001.json \
  --velocity-kph 20 \
  --no-rosbag
```

명령 실행 중 MORAI 조작은 두 번 필요하다.

1. 첫 `[action required]`에서 ROS Bridge 연결을 연다.
2. `[ready] MORAI ScenarioLoad transport: topic (/ScenarioLoad)` 뒤 ScenarioLoad가 적용되면 MORAI가 ROS 클라이언트를 끊는다.
3. 두 번째 `[action required] MORAI applied ScenarioLoad ...`에서 다음 두 곳을 **Connect**한다.
   - `Ego Network > Publisher, Subscriber, Service > ROS`
   - `Simulator Network > ROS`
4. `[ready] MORAI ROS transport reconnected after ScenarioLoad`가 나온 뒤 kroute, validator, follower가 시작되는지 확인한다.

Network File Manager의 `Load`는 실행 도중 다시 누르지 않는다. 이미 불러온 프로필에서
`Connect`만 누른다.

## 결과 확인

세션마다 다음 경로가 만들어진다.

```text
artifacts/morai_scenario_validation/<세션시각>/summary.json
artifacts/morai_scenario_validation/<세션시각>/results.csv
artifacts/morai_scenario_validation/<세션시각>/<시나리오이름>/logs/
```

`results.csv`의 핵심 열은 `passed`, `end_reason`, `collision_detected`,
`dynamic_object_seen`, `dynamic_encounter_detected`, `dynamic_stopped`,
`dynamic_resumed`, `dynamic_failure_reasons`다.

속도별 배치는 다음 명령이다. 단, 현재 `/ScenarioLoad` 방식은 매 케이스마다 위의
수동 재연결이 필요하므로 336개 같은 대규모 무인 실행에는 사용하면 안 된다.

```bash
MORAI_ROSBRIDGE_HOST=192.168.0.5 \
scripts/runtime/run_morai_scenario_batch_validation.sh --execute --speed 20
```

## 현재 검증 상태

확인된 사실은 다음과 같다.

- rosbridge 기동, `/ScenarioLoad` subscriber 감지, 시나리오 요청 전달까지 성공했다.
- ScenarioLoad 직후 MORAI가 rosbridge 클라이언트를 모두 끊는 동작을 로그로 확인했다.
- 재연결에 성공한 20260810_035138 세션에서는 kroute, validator, follower까지 기동했다.
- `kcity_competition_route.launch`의 v60 맵/route definition 누락으로 kroute가 즉시 종료하던 문제는 보정했다.
- 재연결 성공 세션에서 ego 이동거리는 `0.0 m`였고 follower 로그에 `rejected invalid trajectory`가 반복됐다.
- 이에 runner가 `/control/trajectory`를 기다릴 때 헤더 frame, 점 개수(최소 3), pose/velocity 유한값을 검사하고 정확한 거절 원인을 내도록 보강했다.
- 가장 최근 세션(20260810_041447)은 ScenarioLoad 뒤 재연결을 하지 않아 kroute 이전에 종료됐다. 따라서 새 궤적 진단은 아직 실기 MORAI에서 확인하지 못했다.

## 다음 담당자의 우선순위

1. 단일 시나리오에서 두 번째 재연결까지 완료해 `[ready] ... reconnected` 이후의 결과를 확보한다.
2. `planner did not publish a valid /control/trajectory ...`가 나오면 그 상세 사유를 기준으로 velocity smoother 또는 trajectory publisher를 조사한다. 이 단계 전에는 동적장애물 판단 로직을 수정하지 않는다.
3. 차가 실제 주행하고 `/Object_topic`에서 객체가 보이면, `results.csv`의 정지·재출발 판정을 검증한다.
4. **완전 무인 배치가 목표라면** MORAI의 Service Model에서 `/Service_MoraiSL`을 제공하도록 네트워크를 구성하고, ScenarioLoad 뒤에도 연결이 유지되는지 한 건으로 검증한다. 이것이 확인되기 전에는 336/전체 배치를 실행하지 않는다.

## 변경 검증

현재 자동화 관련 변경을 수정한 뒤에는 아래만 실행하면 된다. Python/launch 스크립트만
바뀐 경우 catkin 재빌드는 필요 없다.

```bash
cd ~/Desktop/AjouNice2026
/usr/bin/python3 -m py_compile \
  scripts/runtime/morai_scenario_validation.py \
  scripts/runtime/test_morai_scenario_validation.py \
  scripts/runtime/create_morai_scenario_load_network.py
/usr/bin/python3 scripts/runtime/test_morai_scenario_validation.py
bash -n scripts/runtime/run_morai_scenario_validation.sh \
  scripts/runtime/run_morai_scenario_batch_validation.sh
git diff --check
```

동적장애물 **판단 코드**나 C++ controller를 수정한 경우에만 해당 catkin 패키지를 다시
빌드한다. 결과물·로그는 검증 증거이므로 임의 삭제하지 않는다.
