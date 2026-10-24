# v87 기본 실행 및 Route A/B 배포

기본 planning·RViz·예측·신호등·규정 모니터가 모두 `lanelet2_map_v87.osm`을 사용하도록 전환했다. 기존 v85/v86 지도는 수정하지 않았다. LC·Cruise·제어 파라미터, TF 수치, estimator·인지 정책은 변경하지 않았다. 이미 실행 중인 ROS/MORAI 세션을 재시작하지 않았으므로, 새 기본값은 다음 실행부터 적용된다.

## 실행

```bash
cd ~/AjouNice2026
source scripts/workspace/source_aw149.sh
roslaunch ajounice_planning_integration competition_planning.launch
# 버전을 이름으로 명시해도 동일한 지도/mission 설정을 사용한다.
# roslaunch ajounice_planning_integration competition_planning_v87.launch
```

기존 `docs/common_alias.txt`의 planning·tracking·traffic·RViz alias는 기본 launch를 호출하므로 별도 버전 인자가 필요 없다. localization/controller/detector는 기존 실행 계약대로 별도로 시작한다. 실행 중인 스택 위에 중복 실행하지 않는다.

## 일치시킨 항목

- `map.launch`: v87 OSM, v87 전용 속도 cap 파일, projector 발행기의 지도 경로와 SHA.
- `planning.launch`: `mission_planner_kcity_v87.yaml`.
- 기본 requester 설정: `kcity_v87_competition_route.yaml`. Route A/B/fallback 모두 병합 후 ID를 사용한다.
- planning RViz·속도 편집기: 같은 v87 경로 정의와 cap 파일.
- Map-CV 예측·신호등 경로 helper·규정 모니터·map visualizer·map TF launch: 같은 v87 OSM.
- localization의 공식 시작점 YAML: v87 파일. 공식 시작점 좌표·yaw는 v85와 동일하다.
- 현재 실행/평가 도구의 지도 기본값도 v87로 맞췄다. 과거 버전 YAML, 지도 제작 이력, 기존 시나리오 좌표의 출처 주석은 보존했다.
- 기존 v86 비교 launch는 미배포 과거 작업 파일이며 이번 배포 대상/기본 진입점에 포함하지 않았다.

v87 지도 SHA256은 `70f2cdf7ee9dee3333d6553787525890c0a73c4fbcdfdcc9fc95a5c92c687575`다. [릴리스 manifest](../../configs/map/kcity_v87_release.yaml)에 지도/설정 경로를 기록했다.

## Route A/B 연결 검증

[실제 production DefaultPlanner 검증 JSON](kcity_v87_runtime_route_audit.json)을 참조한다. 별도 ROS master에서 설치된 실제 plugin에 v87 binary map과 production 차량 치수·mission 정책을 입력했다. 가짜 route 응답을 사용하지 않았다.

- Route A 전체: **48 section**, 설정된 preferred 순서와 일치.
- Route B 전체: **58 section**, 설정된 preferred 순서와 일치.
- Route B를 trigger·중간 waypoint에서 요청하는 3개 경우와 허용된 모든 시작 lanelet의 중심선 중간점 35개를 추가 검사했다.
- 총 **40개 실제 경로 생성**, **1199개 section 간 전이** 통과. 각 section의 ID가 지도에 존재하고, preferred가 해당 section의 primitive에 포함되며, 허용된 횡방향 LC 및 정상 후속 연결을 따라 끝 section까지 실제로 이어지는지 검사했다. 전체 생성 결과는 설정 경로 또는 허용된 정확한 suffix와 일치한다.
- `201468`, `201441`, `201446` 소실 차로의 자동 후속 합류는 없고 LC 연결은 유지한다. [지도 검증](kcity_v87_highway_gis_compaction_validation.json)에서 별도로 확인했다.
- fallback은 실제 서비스가 고정 lanelet 경로를 직접 요청하는 방식이다. A/B waypoint planner 검사와 구분하며, 해당 ID/경로 도달 가능성과 cutoff 좌표는 지도 검증 및 requester 테스트로 확인했다.

검사 중 곡선 차로의 양 끝 평균점을 사용해 옆 차로에서 출발한 잘못된 표본을 발견했다. 검사 도구를 실제 중심선 호길이 중간점으로 수정한 후 전수 검사했다. 지도나 LC 로직을 그 잘못된 표본에 맞춰 바꾸지 않았다.

재현:

```bash
source scripts/workspace/source_aw149.sh
/usr/bin/python3 kcity_map/scripts/kcity_map_workflow/validate_v87_runtime_routes.py
```

구성된 mission-planner build의 include/link flags로 별도 probe만 컴파일한다. production 바이너리는 재빌드/교체하지 않는다. ROS는 임의의 loopback 포트에서 시작하고 검사 종료 시 검사 프로세스만 정리한다. MORAI 폐루프 주행/LC 성능 검증은 수행하지 않았다.

## 추가 검사

- [기본 launch 12개 해석](kcity_v87_runtime_launch_audit.json): 모두 성공, map·route·mission·속도·규칙 경로가 v87로 일치. 노드 실행 없이 `--dump-params`로 검사했다.
- 지도 관련 pytest 계약 **69개**, requester private-master ROS 테스트 **19개** 통과.
- 실제 map-loader 속도 적용 함수: [PASS](kcity_v87_highway_gis_compaction_native_caps.json).
- 동일한 최종 OSM에 대한 실제 VMB: [오류 0개](kcity_v87_highway_gis_compaction_actual_vmb.json). 기존 경고는 남아 있으며 이번 배포 작업에서 지도 바이트는 변경하지 않았다.
- 기존 dirty RViz/개인 network 설정/LC YAML과 이전 OSM 전체 SHA를 보존했다. 이번 커밋에서 제외한다.
- 전역 perception 문서 검사는 기존 `docs/bevfusion/README.md`, `docs/bevfusion/CURRENT.md`, `dataset/morai_od/docs/BASE5_V2_CURRENT.md`의 분량 제한 3건으로 실패한다. 세 파일의 줄 수는 기존 HEAD와 동일하며 이번 지도 변경에 포함하지 않았다. 링크/원문 보존 관련 신규 오류는 없다.

## 지도와 Git 배포

팀 Drive에 v87만 immutable 업로드를 시도했지만 `403 storageQuotaExceeded`로 거절됐다. 다른 자산을 삭제하거나 원격 파일을 덮어쓰지 않았다. 사용자 요청에 따라 **v87 OSM만 Git 예외로 직접 포함**했다. 따라서 이번 릴리스가 반영된 브랜치를 checkout하면 v87 OSM도 함께 받으며, v87 지도 자체를 받기 위해 Drive 인증이 필요하지 않다. 다른 대용량 지도 자산은 기존 Drive 정책을 유지한다.

설정/지도/검증 도구/릴리스 문서만 현재 planning 브랜치와 개발 브랜치에 반영한다. 현 원격에는 `dev`가 없어 기존 개발 브랜치 `develop`을 대상으로 한다. 서로 다른 branch의 다른 변경을 통째로 합치지 않고 이 릴리스 커밋만 반영한다.
