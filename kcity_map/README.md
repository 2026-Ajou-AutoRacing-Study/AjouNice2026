# K-City 지도

현재 planning·인지·지도 시각화 launch의 소스 기본 지도는 `lanelet_map/lanelet2_map_v95.osm`이다.
실제 실행의 map 인자와 설치본을 먼저 확인하며, 파일명 숫자가 크다는 이유로 바꾸지 않는다.

2026-10-27: 로컬 v95를 기반으로 음영 직선도로 양쪽 레인렛 `200374`, `200471`의
`speed_limit`을 59 → **40 km/h**로 변경한 `lanelet_map/lanelet2_map_v96.osm`을 생성했다.
[v96 manifest](../configs/map/kcity_v96_shadow_speed.yaml)에 원본·산출물 SHA와 검증 범위를 기록했다.
형상·연결·나머지 속성은 동일하며, 소스 기본 지도·설치본·실행 중 노드는 전환하지 않았다.

2026-10-27: **v95 = v94 + 200311 중심선의 109.5–132.5 m 구간 보정**이다.
47개 점만 최대 1.82 cm 이동했고 도로 경계·속도·연결은 보존했다.
[자산 manifest](../configs/map/kcity_v95_centerline_asset.yaml)에 SHA와 수정 범위를 둔다.
지도 파일은 Git에 포함하지 않는다. 별도로 받은 v95를 다음과 같이 검증·배치한다.

```bash
python3 scripts/map/prepare_kcity_v95.py --source /path/to/lanelet2_map_v95.osm
python3 scripts/map/prepare_kcity_v95.py --check
```

v95 소스 전환은 실행 중인 자동화나 설치본을 바꾸지 않는다. v95 실제 주행 검증은 아직 없다.
아래 v94 설명은 v95가 계승한 자산 이력이며, v94 준비 도구는 명시적 과거 버전용이다.

2026-10-26: 새 통합 자산은 **v94 = 팀장님 v88-2 도로 수정 + 검증한 v93 정지선 좌표**다.
[통합 manifest](../configs/map/kcity_v94_merged_asset.yaml)에 원본 SHA·병합 범위·Drive 검증 상태를 둔다.
v88-2의 도로 재구성·속도 제한은 보존하고 정지선8개 node와버전표기만 바꿨다. v94는 lanelet1295개다.
정지선 위치는 v87 대비 A+0.75/B+2.6/C+0.35/D+2.6m로 v93과 같으며 상대 이동을 중복 적용하지 않았다.
안정 정차 차이2.34px는 [v93 계열의 실측](../docs/perception/traffic_light/261026_TIMED_YELLOW_CANDIDATE.md#pixel-alignment-v93)이지 통합 뒤 새 실주행 결과가 아니다.
**당시 소스 기본은 v94**였다. 인자 없이 실행해도 지도·SHA·Route A/B를 함께 선택한다. 설치본 갱신과 v94 자산 준비는 [팀 업데이트](../docs/runbooks/261026_TRAFFIC_V94_TEAM_UPDATE.md)를 따른다. 실행 중인 노드와 개인 캠페인 snapshot은 이 소스 변경으로 전환되지 않는다.

| 구분 | 경로·기준 |
| --- | --- |
| Lanelet2 | `lanelet_map/lanelet2_map_v95.osm` |
| Projector | `lanelet_map/map_projector_info_v1.yaml` |
| Pointcloud | `lanelet_map/pointcloud_map_v1.pcd` |
| 속도 상한 | v88 OSM attributes: 대상 양쪽 차선 150 → 80 → 70 km/h. 별도 override 없음 |
| 최신 제작 검증 근거 | `../configs/map/kcity_v95_centerline_asset.yaml` |
| 과거 제작·검증·그림 | [legacy 안내](legacy/README.md) |

## 자산 준비

기존 v88 OSM은 Drive 용량 제한에 따른 명시적 예외로 Git checkout에 남긴다. 기본 v94 OSM은 팀 Drive 자산이므로 새 PC는 `python3 scripts/map/prepare_kcity_v94.py`로 먼저 준비한다.
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
