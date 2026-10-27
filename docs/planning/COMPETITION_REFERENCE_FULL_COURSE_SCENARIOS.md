# 대회 참고 전체 코스 시나리오 세트

상태: **로컬 생성 완료 / MORAI 실제 주행 전**

## 목적과 범위

사용자가 내려받은 MORAI 시나리오 묶음의 장애물·보행자·NPC·spawn 위치를 이용하되, 실행 장면의 뼈대는 대회 제공 전체 코스 장면으로 유지한다. 따라서 각 변형은 대회 전체 코스의 Ego 시작점, 지도, 신호등, GPS 음영, 경유점·종료 목표를 유지하고, 상황별 actor 위치만 다운로드 장면에서 가져온다.

다운로드 묶음의 출처는 사용자가 제공한 로컬 파일이며, 이 문서는 그것을 대회 운영측이 공식 확정한 출제 분포라고 주장하지 않는다.

## 입력과 보존 계약

| 항목 | 값 |
| --- | --- |
| 위치 입력 | `/home/ajou/Downloads/scenarios-20261027T135947Z-1-001.zip` |
| 입력 SHA-256 | `b449357cd2e18d667fa9155cae096d8cf78852049c1940ae724f0c4b345e45df` |
| 전체 코스 베이스 | `configs/scenario/2026_molit_comp_sample_scene.json` |
| 베이스 SHA-256 | `e7d01898dc28e1a548f26ca4b2812409b0ae159dd2218029fa2d6b1ac7013467` |
| 생성 원칙 | 베이스의 코스 계약 유지 + source 장면의 actor 목록 교체 |

각 생성 JSON에서 아래 항목은 대회 전체 코스 베이스에서 그대로 보존한다.

- `egoVehicle`, `mapInfo`, `tlList`, `shadedAreaList`
- `frictionChangeMaterialList`, `v2IInfo`, `waypointDataList`

아래 actor 목록만 다운로드 JSON에서 가져온다. 위치·회전·asset·link·spawn 관련 값은 변경하지 않는다.

- `objectList`
- `pedestrianList`
- `vehicleList`
- `spawnPointList`
- `pedspawnPointList`

## 만들어진 세트

| 상황 | 장면 수 | 전체 코스 실행 YAML |
| --- | ---: | --- |
| 정적 장애물 | 17 | `contest_reference_20261027_static_bev.yaml` |
| 동적 장애물 | 301 | `contest_reference_20261027_dynamic_bev.yaml` |
| 고속도로 | 125 | `contest_reference_20261027_highway_bev.yaml` |
| 회전교차로 | 168 | `contest_reference_20261027_roundabout_bev.yaml` |
| GPS 음영 정적 | 200 | `contest_reference_20261027_shadow_static_bev.yaml` |
| GPS 음영 동적 | 303 | `contest_reference_20261027_shadow_dynamic_bev.yaml` |
| **합계** | **1,114** | 6개 YAML |

로컬 생성 루트는 아래다. `workspace/`는 팀원 개인 작업 영역이라 Git에서 의도적으로 제외된다.

```text
base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/workspace/
  contest_reference_20261027/
    generated/scenarios/<상황>/*.json
    generated/manifest.json
  contest_reference_20261027_<상황>_bev.yaml
```

`manifest.json`에는 각 출력의 source 이름·SHA-256·actor 수·회전교차로 속도 변경 이력이 기록된다.

## 회전교차로 25 km/h 상한

회전교차로의 **다운로드 JSON 속 NPC 차량** 속도를 25 km/h 이하로 제한했다. `vehicleList` 내부의 `desiredVelocity`, `currentVelocity`, `viewCurrentVelocity`, `linkVelocity`, 저장된 수치형 `velocity`만 상한 처리했다. 위치·회전·차종·링크·spawn은 그대로다.

- 대상: 168개 회전교차로 장면
- 변경된 속도 필드: 11,256개
- 생성 후 25 km/h 초과 속도 필드: 0개

이는 NPC 속도 상한이다. Ego 속도는 planning/control 스택의 별도 정책이므로 이 시나리오 생성 작업에서는 바꾸지 않았다.

## 검증과 사용 전 조건

- 1,114개 출력 JSON 파싱, source actor 일치(회전교차로는 속도 필드 제외), 대회 베이스 필드 보존을 확인했다.
- 6개 YAML의 경로·구문·대상 JSON 수를 확인했다.
- `planning_validator --check`는 MORAI를 시작하지 않았고, 공통 설정의 신호등 가상환경 `~/.local/share/ajounice/traffic_light_gpu/bin/python`이 현재 PC에 없어 그 사전 검사 단계에서 멈췄다. 대회 유사성을 위해 신호등은 off로 바꾸지 않았다.
- 실제 MORAI 배우 동작·경로·신호 연동·폐루프 주행은 아직 검증하지 않았다. 전체를 바로 실행하지 말고 GUI의 **planning 빠른 테스트** 또는 한 장면 선택으로 먼저 smoke test 한다.

검사 예시는 다음과 같다.

```bash
./planning_validator \
  base_ws/src/planning_addons/planning_scenario_validator/config/missions/full_course/workspace/contest_reference_20261027_roundabout_bev.yaml \
  --check
```

신호등 Python 환경을 준비한 뒤 위 검사가 통과하면, 한 장면 smoke test의 녹화·route·actor 동작을 확인하고 상황군 단위 실행으로 확대한다.

## 재생성 범위

생성기는 개인 workspace의
`contest_reference_20261027/import_competition_reference_scenarios.py`에 보존되어 있다. 같은 zip과 전체 코스 베이스를 입력으로 사용하면 actor 위치 계약과 회전교차로 25 km/h 상한을 다시 적용한다. 원본 zip·생성 JSON·캠페인 YAML은 개인 작업 산출물이므로 이 Git 문서에는 포함하지 않는다.

## 복합 전체 코스 커버리지 세트

앞 절의 1,114개는 각 상황군을 **단독으로 바꾼 후보 풀**이다. 실제 전체 코스 검증에는 한 JSON 안에서 정적·동적·고속도로·회전교차로·GPS 음영 정적·GPS 음영 동적 조건이 모두 바뀌어야 한다. 이를 위해 별도의 복합 세트를 생성했다.

| 항목 | 값 |
| --- | ---: |
| 복합 전체 코스 JSON | 300개 |
| 각 JSON의 component | 6개 상황군에서 active source 1개씩 |
| 사용 active source | 정적 17 / 동적 300 / 고속 125 / 회전 168 / 음영 정적 200 / 음영 동적 300 |
| 제외 source | 동적 `dq_base0`, 음영 동적 `sq_base0~2`: 해당 상황의 실제 동적 actor가 없는 baseline |

300개는 6개 상황군 중 가장 큰 active 풀 크기다. 각 상황군의 선택 순서는 서로 다른 stride를 사용한 순환 배치이므로, 모든 active source가 적어도 한 번은 복합 장면에 포함되고 연속 장면에서 같은 source를 반복하지 않는다. 이는 전수 Cartesian product가 아니라 실행 가능한 bounded coverage다.

```text
정적 17 × 동적 300 × 고속 125 × 회전 168 × 음영 정적 200 × 음영 동적 300
```

위 전수 조합은 현실적으로 실행할 수 없으므로 생성하지 않았다.

복합 장면은 대회 전체 코스의 Ego·지도·신호·음영·경유점은 유지한다. actor 목록은 빈 상태에서 여섯 component를 병합하고, component 간 `UNIQUEID` 충돌은 재할당한다. source의 위치·회전·asset·link·spawn 값은 보존한다. 회전교차로의 병합 NPC 속도는 계속 25 km/h 이하로 제한한다.

로컬 실행 설정:

```text
.../full_course/workspace/contest_reference_20261027/
  composite_coverage_300/scenarios/full_mix_001.json ~ full_mix_300.json
  contest_reference_20261027_composite_coverage_bev.yaml
  contest_reference_20261027_composite_coverage_smoke_bev.yaml
```

`smoke` YAML은 `full_mix_001.json` 한 장면만 선택한다. 전체 300개를 실행하기 전에는 이 한 장면의 MORAI actor 동작·route·신호·녹화를 먼저 확인한다.

생성 후 300개 JSON 파싱, 모든 active source 포함, actor `UNIQUEID` 중복 0건, 회전교차로 25 km/h 초과 0건, 두 YAML의 대상 파일 수(300/1)를 검증했다. 실제 MORAI 폐루프 주행은 아직 수행하지 않았다.
