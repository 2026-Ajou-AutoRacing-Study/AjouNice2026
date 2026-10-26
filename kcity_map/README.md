# K-City 지도

현재 소스 launch의 기본 지도는 `lanelet_map/lanelet2_map_v94.osm`이다.
실제 실행의 map 인자와 설치본을 먼저 확인하며, 파일명 숫자가 크다는 이유로 바꾸지 않는다.

| 구분 | 경로·기준 |
| --- | --- |
| Lanelet2 | `lanelet_map/lanelet2_map_v94.osm` |
| Projector | `lanelet_map/map_projector_info_v1.yaml` |
| Pointcloud | `lanelet_map/pointcloud_map_v1.pcd` |
| 속도 상한 | v94 OSM attributes: 대상 양쪽 차선 150 → 80 → 70 km/h. 별도 override 없음 |
| 최신 제작 검증 근거 | `../configs/map/kcity_v94_release.yaml` |
| 과거 제작·검증·그림 | [legacy 안내](legacy/README.md) |

## 자산 준비

v94 OSM은 Git checkout에 포함한다. 기본 launch와 설치본도 v94로 맞췄다.
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

## 현재 도구와 역사 재현

- 실제 실행 지도 확인: [check_runtime_map_version.py](../scripts/map/check_runtime_map_version.py).
- 범용 지도 검사·변환: [workflow 안내](scripts/kcity_map_workflow/README.md).
- 코스 산출물: [competition_route 안내](competition_route/README.md). 과거 v41/v44 예시를
  현재 planning route나 대회 기본값으로 사용하지 않는다.
- 버전 전환 전용 도구는 [legacy](legacy/README.md)에 원문 보관했다.

v94는 다운로드 폴더의 제공 파일을 그대로 복사했다. v88 대비 신호등 A/B/C/D 정지선 끝점 8개 좌표만 달라졌으며 차로 형상·연결·속도는 동일하다. 관련 패키지 빌드·설치 후 v94 자동화의 첫 Route B 생성·주행 시작을 확인했다. 전체 캠페인 결과와 VMB UI 검증은 별도다.

## v88에서 계승한 highway 형상

- 선호 경로: `200360 → 201479 → 201451` (150 → 80 → 70 km/h).
- 옆차선: `200185 → 201448 → 201452` (동일 속도).
- 기존 201479의 꺾임점에서 분할하고 앞·뒤를 각각 직선으로 병합했다.
- 201451·201452의 형상과 좌표는 v87과 동일하며 속도만 변경했다.
- 바깥 200147·201449의 구간 경계도 정렬해 기존 200147 ↔ 200185 차선변경 연결을 유지했다.
- v88 당시 검증: 전체 형상·그래프 검사, Mission Planner 36 경로/1,005 전이, 설치 launch 18개 해석 통과. v94의 신규 폐루프 검증 결과는 아니다.
- v94 첫 Route B 생성·주행 시작을 확인했다. 전체 폐루프 캠페인과 VMB UI 검증 완료를 의미하지 않는다.
- Drive 업로드는 사용자가 수행한다.
