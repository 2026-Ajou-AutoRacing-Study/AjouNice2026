# K-City v87: 고주로 경계 정렬 및 lanelet 병합

v86을 바탕으로 진입부·병목의 경계를 GIS 원본에 맞춰 직선으로 정리하고, 같은 차로의 불필요한 짧은 조각을 병합했다. v85/v86 파일과 다른 구간의 지도 요소는 그대로 보존했다. 실행 중인 ROS/MORAI는 변경하거나 재시작하지 않았다.

- 지도: `../lanelet_map/lanelet2_map_v87.osm`
- SHA256: `70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575`
- 기준 v86 SHA256: `55ee0ce9c4b8489941994cb50e29eb3526d17e7512a41bb86886ebd5576e86bd`
- [실제 v86 / v87 비교 SVG](kcity_v87_highway_gis_compaction.svg) · [PNG](kcity_v87_highway_gis_compaction.png)

## 병합 결과

| 구간 | v86 주행 순서 | v87 |
|---|---|---|
| 진입 본선 | 201457 → 200099 → 201467 | **201457 → 201467** |
| 진입 소실 차로 | 201438 → 201468 | **201468**, 삼각형 |
| 병목 본선 | 200127 → 201473 → 201439 | **201439** |
| 병목 소실 2차로 | 200130 → 201474 → 201441 | **201441**, 삼각형 |
| 병목 소실 3차로 | 200134 → 201475 → 201446 | **201446**, 삼각형 |

진입 본선의 `201457` 분할은 의도적으로 유지했다. 옆 차로의 `201456`과 `201468`을 각각 유지하면서 본선만 전부 합치면 Lanelet2의 전체 경계 공유 조건이 깨져 두 LC 연결이 사라진다. 실제 RoutingGraph 비교로 확인했다. 따라서 `201456 ↔ 201457`, `201468 → 201467`이 각각 성립하도록 분할 하나를 남겼으며, 해당 분할점에서도 경계는 같은 직선이다.

같은 횡단면의 LC를 보존하기 위해 진입의 `201435+201466`, `201436+201465`, `201437+201464` 및 병목의 `200148+201472+201440`도 각각 마지막 ID로 병합했다. 이 조정 역시 진입·병목 구간 안에 한정된다. 최종 lanelet 수는 1315 → 1302다.

## 형상과 연결

- 진입 본선/소실 차로 경계는 GIS `B2256W000038`, 본선 안쪽은 `B2256W000041`, 외곽 연결점은 `B2256W000061`을 참고했다.
- 병목 본선/2차로 경계는 GIS `B2256W000043`을 참고했다. 2/3차로 사이에는 대응 B2 도색 선형이 없어 A2_LINK와 소실 끝점을 기준으로 가상 경계를 구성했다.
- 기존 외부 접속점은 고정했다. 원본 GIS 자체의 작은 굴곡을 그대로 복제하지 않고 직선화했으므로 GIS와 완전 일치하는 것은 아니다. 경계에서 원본 GIS까지 1m 간격으로 측정한 최대 거리는 진입 약 **0.164m**, 병목 약 **0.354m**다.
- 기존 경계 노드 중 좌표를 옮긴 것은 국소 접속점 3개뿐이다. 나머지 변경은 이 구간 경계의 중간점 정리와 중심선 재생성이다. 영역 밖 primitive와 기존 노드 좌표는 동일하다.
- 소실 차로 `201468`, `201441`, `201446`은 정상 후속 연결이 없고, 본선 방향의 LC 연결이 있다. 반대 방향으로 소실 차로에 새로 들어가는 LC는 막았다. 이 정책은 병합된 경계 전체에 적용된다.
- `201456 → 200128`, `201468 → 200128`, `201446 → 200185`, `201441 → 200185` 경로가 존재하며 모두 LC를 거쳐야 한다. 정상 후속 연결만으로 우회 합류할 수 없다.

이 결과는 지도에서 LC가 가능한 구조라는 뜻이다. 실제 LC 승인 시점, 가감속 gap 선택, 취소 여부와 충돌 회피 성능은 주행 검증으로 별도 확인해야 한다. 정지 방지나 가감속 합류 로직을 이번 지도 작업에서 구현한 것은 아니다.

## 실행 설정

후속 배포 작업에서 기본 실행도 v87로 전환했다. 이전 버전 지도 파일은 보존했다. 명시적인 v87 진입점은 아래와 같다. 이미 실행 중인 스택과 중복 실행하지 말고 다음 테스트 시작 시 사용한다.

```bash
cd /home/jihun/AjouNice2026
source /opt/ros/noetic/setup.bash
source planning_ws/install_isolated/setup.bash
source base_ws/devel_aw149/setup.bash
export AJOUNICE_ROOT="$PWD"
roslaunch ajounice_planning_integration competition_planning_v87.launch
```

새 지도와 맞는 `kcity_v87_competition_route.yaml`, `competition_route_requester_v87.param.yaml`, `mission_planner_kcity_v87.yaml`, `lanelet_speed_limits_v87.yaml`을 함께 선택한다. 삭제 ID를 새 ID로 치환하고 중복을 정리했다. waypoint와 fallback cutoff의 세계 좌표는 보존하고, 병합된 차로의 시작점에서 계산하는 `lanelet_s_m`만 재산정했다.

속도 덮어쓰기 파일은 지도 SHA가 정확히 맞아야 로더가 수락한다. 기존 v86 launch가 참조하던 기본 파일은 v85 SHA에 고정되어 있어, v87에서는 새 전용 파일을 명시했다. 기존 파일은 수정하지 않았다.

기존 50km/h cap은 병합 대상 중 가장 낮은 값으로 보존하여 `201468`, `201439`, `201441`, `201446`에 적용한다. **`201446`은 이전 앞 조각에 있던 50km/h cap이 병합된 전체 차로에 적용되므로, 마지막 옛 201446 구간도 50km/h가 된다.** 다른 속도 정책과 LC·Cruise·제어 파라미터는 이번 작업에서 변경하지 않았다.

규정 모니터를 별도로 사용하는 경우 `roslaunch competition_rule_monitor competition_rule_monitor_v87.launch`를 선택한다. 지도 SHA와 고속 차로 개수(병합 후 39개)가 맞는 새 `kcity_v87_rules.yaml`을 사용한다. GT localization의 `official_start_config`가 필요하면 이 새 YAML 경로로 지정할 수 있다(공식 시작 위치 자체는 동일).

새 launch/config 6개만 설치 경로에 추가했고 기존 설치 파일은 덮어쓰지 않았다. 후속 배포에서 범용 기본 launch와 관련 소비자의 기본값도 v87로 전환했다. [배포 검증](kcity_v87_runtime_release.md)을 참조한다.

## 검증 결과

- [구조·범위·RoutingGraph·GIS·route 검증](kcity_v87_highway_gis_compaction_validation.json): **PASS**. 전체 유효 polygon, 국소 중심선 내부, 참조 무결성, 세 삼각형, LC 11쌍, route/mission ID와 waypoint 검사 통과.
- [실제 VMB 결과](kcity_v87_highway_gis_compaction_actual_vmb.json), [화면](kcity_v87_highway_gis_compaction_actual_vmb.png): **오류 0개**.
- [VMB 전후 비교](kcity_v87_highway_gis_compaction_vmb_comparison.json): ID 병합을 반영한 신규 경고 대상 0개. 기존 중심선 연결 경고는 40 → 13개. 기존 속도 임계값(30km/h) 경고 1199개, 고립 점 899개, intersection_area 경고 60개는 남아 있다. 요청 범위 밖을 고치지 않았다.
- [launch 해석·기존 파일 보존 검증](kcity_v87_highway_gis_compaction_launch_validation.json): **PASS**. 신규 설정 source/install 일치, 이전 지도 전체와 기존 LC 소스 파일 SHA 동일. 차이는 지도 및 관련 설정 선택과 mission ID 치환뿐이다.
- [실제 native map-loader 속도 적용 함수 오프라인 검사](kcity_v87_highway_gis_compaction_native_caps.json): **PASS**. 최종 지도 SHA와 4개 cap을 수락했고 실제 lanelet에 50km/h 적용을 확인했다.
- 실시간 ROS 노드 실행, bag replay, MORAI 폐루프 주행: **미실시**.

재현 스크립트는 `../scripts/kcity_map_workflow/{build_v87_highway_gis_compaction,prepare_v87_profiles,validate_v87_highway_gis_compaction,visualize_v87_highway_gis_compaction}.py`다. build/prepare는 기존 결과를 덮어쓰지 않는다. 최초 형상 작업 당시 지도는 로컬 파일이었다. 후속 배포에서는 Drive quota 초과에 따라 v87 OSM만 Git 관리 예외로 추가했다. 배포 상태는 [배포 문서](kcity_v87_runtime_release.md)를 따른다.
