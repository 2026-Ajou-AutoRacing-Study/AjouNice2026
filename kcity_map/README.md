# K-City 지도

현재 인지 launch의 기본 지도는 `lanelet_map/lanelet2_map_v88.osm`이다.
실제 실행의 map 인자와 설치본을 먼저 확인하며, 파일명 숫자가 크다는 이유로 바꾸지 않는다.

2026-10-26: 새 통합 자산은 **v94 = 팀장님 v88-2 도로 수정 + 검증한 v93 정지선 좌표**다.
[통합 manifest](../configs/map/kcity_v94_merged_asset.yaml)에 원본 SHA·병합 범위·Drive 검증 상태를 둔다.
v88-2의 도로 재구성·속도 제한은 보존하고 정지선8개 node와버전표기만 바꿨다. v94는 lanelet1295개다.
정지선 위치는 v87 대비 A+0.75/B+2.6/C+0.35/D+2.6m로 v93과 같으며 상대 이동을 중복 적용하지 않았다.
안정 정차 차이2.34px는 [v93 계열의 실측](../docs/perception/traffic_light/261026_TIMED_YELLOW_CANDIDATE.md#pixel-alignment-v93)이지 통합 뒤 새 실주행 결과가 아니다.
**운영 기본은 v88**을 유지한다. `map_version:=94`는 지도·SHA·Route A/B를 함께 선택하는 명시적 profile이며 [팀 업데이트](../docs/runbooks/261026_TRAFFIC_V94_TEAM_UPDATE.md)를 따른다. v94로 자동 전환하거나 이전 자산을 삭제하지 않는다.

| 구분 | 경로·기준 |
| --- | --- |
| Lanelet2 | `lanelet_map/lanelet2_map_v88.osm` |
| Projector | `lanelet_map/map_projector_info_v1.yaml` |
| Pointcloud | `lanelet_map/pointcloud_map_v1.pcd` |
| 속도 상한 | v88 OSM attributes: 대상 양쪽 차선 150 → 80 → 70 km/h. 별도 override 없음 |
| 최신 제작 검증 근거 | `../configs/map/kcity_v88_release.yaml` |
| 과거 제작·검증·그림 | [legacy 안내](legacy/README.md) |

## 자산 준비

v88 OSM은 Drive 용량 제한에 따른 명시적 예외로 Git checkout에 포함한다.
PCD·MGeo·GIS·다른 버전 OSM·버전별 route 같은 큰 자산은 팀 Drive로 공유한다.
설정·현재 도구·이력은 Git으로 관리한다. 자산 경로는 이 저장소의 `kcity_map/`을 유지한다.

```bash
# rclone remote는 본인 계정으로 설정하고 인증파일은 공유하지 않는다.
bash scripts/map/download_kcity_map_assets.sh --dry-run
bash scripts/map/download_kcity_map_assets.sh
```

위 명령은 저장소 루트에서 실행한다. 기본 remote는 `ajou_drive`, Drive 경로는
`kcity_map`이다. `KCITY_MAP_RCLONE_REMOTE`·`KCITY_MAP_DRIVE_PATH`로 변경할 수 있다.
두 스크립트는 immutable 정책과 크기·MD5 검사를 사용한다. 기존 자산을 삭제하거나
같은 경로의 다른 내용을 덮어쓰지 않는다. 업로드는 별도 승인 후
`scripts/map/upload_kcity_map_assets.sh --dry-run`으로 먼저 확인한다.
이 PC의 후속 Drive 작업은 `KCITY_MAP_RCLONE_REMOTE=ajou_drive_private`를 명시한다.
전용 인증이 만료되면 갱신을 요청하며 공용 client인 `ajou_drive`로 우회하지 않는다.

## 현재 도구와 역사 재현

- 실제 실행 지도 확인: [check_runtime_map_version.py](../scripts/map/check_runtime_map_version.py).
- 범용 지도 검사·변환: [workflow 안내](scripts/kcity_map_workflow/README.md).
- 코스 산출물: [competition_route 안내](competition_route/README.md). 과거 v41/v44 예시를
  현재 planning route나 대회 기본값으로 사용하지 않는다.
- 버전 전환 전용 도구는 [legacy](legacy/README.md)에 원문 보관했다.

이번 정리는 자산 다운로드·재생성·현장 지도 교체·속도 정책 변경을 수행한 것이 아니다.

## v88 highway 변경

- 선호 경로: `200360 → 201479 → 201451` (150 → 80 → 70 km/h).
- 옆차선: `200185 → 201448 → 201452` (동일 속도).
- 기존 201479의 꺾임점에서 분할하고 앞·뒤를 각각 직선으로 병합했다.
- 201451·201452의 형상과 좌표는 v87과 동일하며 속도만 변경했다.
- 바깥 200147·201449의 구간 경계도 정렬해 기존 200147 ↔ 200185 차선변경 연결을 유지했다.
- 전체 형상·그래프 검사, 설치된 Mission Planner 36 경로/1,005 전이, 설치 launch 18개 해석을 통과했다.
- 실행 중인 ROS 세션은 교체하지 않았다. v88 폐루프 주행과 VMB UI 검증은 미실행이다.
- Drive 업로드는 사용자가 수행한다.
