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
