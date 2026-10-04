# Localization evaluation scenarios

## `localization_full_route_no_actors.json`

Localization과 controller의 폐루프 A/B를 위한 MR 전용 전체 코스다.

- 기준: `configs/scenario/2026_molit_comp_sample_scene.json`
- 유지: Ego 시작 상태, K-City map 정보, 신호 상태, GPS 음영 영역
- 제거: 초기 NPC, 보행자, 장애물, 차량/보행자 spawn, actor waypoint
- 실행: detector 대신 `od_gt`, 신호등 인지와 planner plugin은 모두 비활성화

신호 상태를 시나리오에서 삭제하지 않은 이유는 원본 코스 정의를 가능한 한 그대로
보존하기 위해서다. 실제 정지는 runtime에서 `traffic`을 실행하지 않고
`planning enable_traffic_light_planner:=false`로 막는다.

이 구성은 측위와 제어기의 연속성만 격리해서 보는 1차 시험용이다. 객체·신호등을
포함한 production full-stack 회귀 시험을 대체하지 않는다.

재생성:

```bash
cd ~/AjouNice2026
python3 scripts/scenario/build_localization_full_route_scenario.py
```

## `localization_shadow_entry_{clear,cargobox}.json`

2026-09-29 Planning RViz bag의 GPS 음영 횡드리프트를 경량 폐루프로 재현하기 위한
동일 조건 A/B다. 위 actor-free 장면을 기반으로 두 장면 모두 Ego를 음영 진입 약 35 m
전의 동일 MGeo link로 옮겼다. `clear`에는 객체가 없고 `cargobox`에는 공식 샘플의
`CargoBox`(DataID `40100001`, sample scale `2 x 3 x 2`) 하나만 음영 초입 차로
우측에 배치했다. 차량·보행자·spawn은 두 장면 모두 없으며 GNSS shaded area,
신호 상태와 지도는 동일하다. 신호등 노드와 planner plugin은 실행 시 끈다.

MORAI MR에서는 각각 `0_loc_shadow_clear`, `0_loc_shadow_cargobox`로 로드한다.
Ego Network는 `ego_network_competition_local.json`, Sensor는
`Sensor_localization_gt_lightweight.json`을 사용한다. `localization`의
`gt_source=competition`을 유지하고 `/Ego_topic`은 평가 oracle로만 기록한다.
출발 전 정상 GNSS와 초기 state를 확인하고, 음영 중 정지하면 최소 25초 더 녹화한다.
이 장면은 계획된 정지·회피가 반드시 성공한다는 보장이 아니라 측위 실패 재현용이다.

`localization_shadow_full_start_cargobox.json`은 동일한 CargoBox 및 나머지
시나리오 내용을 유지하고 Ego만 원본 전체 코스 시작점에 둔 원인 분리용 장면이다.
MR에서는 `0_loc_shadow_full_start_cargobox`로 연다. 중간 진입 장면은 객체 없는
Route B에서 정상 동작했지만 CargoBox가 있으면 정적 장애물 회피 모듈의
`behavior_path_planner_full_node`가 SIGSEGV를 냈다(2026-09-30). 이 변형으로
Route A부터 자연스럽게 진입해도 충돌하는지 확인한다. 결과 확인 전에는
CargoBox 자체 또는 Route B 자체가 원인이라고 단정하지 않는다.

재생성:

```bash
python3 scripts/scenario/build_localization_shadow_repro.py
```
