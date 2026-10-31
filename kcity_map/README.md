# K-City 지도

현재 planning·인지·측위 지도·시각화의 기본 지도는 **v101**이다.
v101은 사용자가 제공한 v100의 기존 노드·차선·중심선·속도·경로 연결을 보존하고,
음영 직선 구간 `200374`·`200471` 바깥에 GIS 기반 `road_shoulder` Lanelet
`234976`·`235226`을 추가한 지도다. 차선과 숄더는 기존 경계 Way를 공유하며,
공유 경계는 흰색 실선, 실제 바깥 경계는 `road_border`로 표현한다.
새 숄더의 제한속도는 10 km/h다. 숄더 사용 정책과 expandable 설정은 별개다.
[현재 manifest](../configs/map/kcity_v101_shadow_shoulders.yaml)에 실제 파일 SHA를 고정한다.
지도 파일·작업용 그림·검증 산출물은 Git에 포함하지 않는다.
별도 제공받은 v101을 다음 명령으로 검증·배치한다.

```bash
python3 scripts/map/prepare_kcity_v101.py --source /path/to/lanelet2_map_v101.osm
python3 scripts/map/prepare_kcity_v101.py --check
python3 scripts/map/check_runtime_map_version.py --installed
```

클린빌드도 v101을 검사한 뒤 진행한다. 파일이 없거나 SHA가 다르면 중단하며 구버전으로 대체하지 않는다.
일반 planning·통합 stack·인지 launch와 업데이트 도구는 v101 이외 버전·지도 경로를 거부한다.
과거 버전별 launch는 이력 보존용이며 현재 공통 planning 진입점에서 실행이 거부된다.
원본 v100부터 존재한 Lanelet `200289` 자기교차 1건은 보존됐다. v101의 새 숄더는
형상·공유 경계·Lanelet2 로드·기존 routing 연결 검사를 통과했다. 폐루프 주행 검증은 별도다.
설치 반영 후 실행 중인 노드는 재시작해야 새 지도를 로드한다.

이전 v99는 v98 대비 Lanelet `201096` 제한속도를 59 → 30 km/h로 변경한 지도다.
[당시 manifest](../configs/map/kcity_v99_speed_limit.yaml)와 준비 도구는 이력으로 보존한다.
아래 설명은 이전 자산 이력이다.

이전 v98은 중심선 연결을 수정한 지도다.
v98은 v97의 Lanelet `201439` 중심선 Way `105522` 끝점 참조를
`229565`에서 다음 Lanelet `200360`의 `233927`로 연결한 지도다.
9.0502 cm 간격을 없애며 기존 굴절은 보존한다. 경계·속도·나머지 형상·경로 정책은 변경하지 않는다.
[현재 manifest](../configs/map/kcity_v98_centerline_connection.yaml)에 입력·출력 SHA를 고정한다.
지도 원본·작업용 그림·검증 산출물은 Git에 포함하지 않는다.
기존 v97 자산이 있으면 아래 준비 도구가 검증 후 v98을 재현한다. 다른 내용의 기존 파일은 덮어쓰지 않는다.

```bash
python3 scripts/map/prepare_kcity_v98.py
python3 scripts/map/prepare_kcity_v98.py --check
python3 scripts/map/check_runtime_map_version.py --installed
```

소스 변경과 설치·실행 중인 노드의 적용은 별개다. 설치 반영 후 관련 노드를 재시작해야 적용된다.
v97 이하 명시적 프로필은 과거 버전 복귀용으로 보존한다. 아래 설명은 이전 자산 이력이다.

v97 전환 당시 planning·인지·지도 시각화 launch의 소스 기본 지도는 `lanelet_map/lanelet2_map_v97.osm`이었다.
실제 실행의 map 인자와 설치본을 먼저 확인하며, 파일명 숫자가 크다는 이유로 바꾸지 않는다.

v97은 사용자가 제공한 지도이며 `200968`의 제한속도는 59 → 40 km/h다.
[v97 manifest](../configs/map/kcity_v97_speed_limit.yaml)에 실제 파일 SHA를 고정한다.
VMB 재저장에 따른 좌표 정밀도·속성·멤버 차이도 있어 v96과 바이트/형상이 동일하다고 간주하지 않는다.
Route A/B와 Mission 정책은 v96을 계승하며 지도 로드·경로 연결은 별도 검증한다.
`python3 scripts/map/prepare_kcity_v97.py --check`로 자산을 확인한다.
당시 설치된 기본값은 v97이었다. 현재 기본값은 문서 상단의 v101를 따른다.

2026-10-27: 로컬 v95를 기반으로 음영 직선도로 양쪽 레인렛 `200374`, `200471`의
`speed_limit`을 59 → **40 km/h**로 변경한 `lanelet_map/lanelet2_map_v96.osm`을 생성했다.
[v96 manifest](../configs/map/kcity_v96_shadow_speed.yaml)에 원본·산출물 SHA와 검증 범위를 기록했다.
형상·연결·나머지 속성은 동일하다. 런치와 라우트 리쿼스터의 소스 기본값은 v96으로 전환했다.
설치본·실행 중인 자동화는 바꾸지 않았다. 지도 파일은 Git에 포함하지 않는다.

```bash
python3 scripts/map/prepare_kcity_v96.py --source /path/to/lanelet2_map_v96.osm
python3 scripts/map/prepare_kcity_v96.py --check
```

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
| Lanelet2 | `lanelet_map/lanelet2_map_v101.osm` |
| Projector | `lanelet_map/map_projector_info_v1.yaml` |
| Pointcloud | `lanelet_map/pointcloud_map_v1.pcd` |
| 속도 상한 | v88 OSM attributes: 대상 양쪽 차선 150 → 80 → 70 km/h. 별도 override 없음 |
| 최신 자산 검증 근거 | `../configs/map/kcity_v101_shadow_shoulders.yaml` |
| 과거 제작·검증·그림 | [legacy 안내](legacy/README.md) |

## 자산 준비

기존 v88 OSM은 Drive 용량 제한에 따른 명시적 예외로 Git checkout에 남긴다. 현재 v97 OSM은 별도 제공 자산이므로 새 PC는 위 `prepare_kcity_v97.py --source ...` 명령으로 먼저 준비한다.
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
