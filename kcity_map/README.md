# K-City Map

K-City 지도 설정과 제작·검증 스크립트는 Git으로 관리하고, 대용량 지도
자산은 팀 Google Drive로 공유합니다. Google Drive에서 파일을 내려받아도
경로는 항상 `~/AjouNice2026/kcity_map/`으로 유지되므로 기존 ROS launch와
도구의 경로를 바꿀 필요가 없습니다.

## 관리 범위

Git에서 관리하는 파일:

- `README.md`, `.gitignore`
- `2026_molit_comp_sample_scene.json`
- `lanelet_map/map_projector_info_v1.yaml`
- `lanelet_map/lanelet2_map_v87.osm` (명시적 Git 배포 예외, 아래 설명)
- v87 route/requester/mission/rules 실행 설정
- `competition_route/README.md`
- `competition_route/How to visualize competition route.txt`
- `scripts/kcity_map_workflow/`

Google Drive에서 관리하는 파일:

- `lanelet_map/*.osm`
- `lanelet_map/*.pcd`
- `mgeo_map/`
- `gis_map/`
- `example_map/`
- `competition_route/v*/`

현재 기준 자산:

- Lanelet2: `lanelet_map/lanelet2_map_v87.osm` (Git checkout에 포함)
- Pointcloud: `lanelet_map/pointcloud_map_v1.pcd`
- Projector: `lanelet_map/map_projector_info_v1.yaml`
- Competition route: `base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml`

## v87 기본 실행

`competition_planning.launch`와 명시적 `competition_planning_v87.launch`는 모두 v87을 사용합니다.
RViz·예측·신호등·규정 모니터도 같은 지도 버전으로 맞췄습니다.

공용 requester 실행 코드는 v87 전용
[requester 설정](../base_ws/src/planning_addons/competition_route_requester/config/competition_route_requester_v87.param.yaml)과
[Route A/B 설정](../base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml)을 사용합니다.
기본 requester 설정도 같은 v87 Route A/B를 선택합니다.

지도 제작 중 생성한 SVG·PNG·검증 결과와 일회성 제작 스크립트는
Git에서 제외된 `planner_logs/analysis/` 아래에 보관합니다.

v87은 사용자 요청으로 OSM 자체도 Git에 배포합니다. 팀 Drive 업로드는
`storageQuotaExceeded`로 거절되어, 이 버전만 `.gitignore` 예외로 추가했습니다.
기존 지도 자산의 Drive 관리 정책과 이전 버전 파일은 보존합니다.

## 최초 준비

Ubuntu에서 rclone을 설치하고 자신의 Google 계정으로 `ajou_drive` remote를
한 번 설정합니다. OAuth 설정 파일인 `~/.config/rclone/rclone.conf`는
절대 Git에 올리거나 다른 사람과 공유하지 않습니다.

```bash
sudo apt update
sudo apt install -y rclone
rclone config
```

Drive remote 이름은 `ajou_drive`로 통일합니다. 스크립트는 팀 공유 폴더
`AjouNice2026`의 폴더 ID를 루트로 사용하므로 일반 공유 폴더도 동일하게
접근할 수 있습니다.

## 지도 다운로드

저장소 루트에서 먼저 변경 내용을 확인한 후 내려받습니다.

```bash
# 실제 변경 없이 다운로드 대상을 확인
scripts/map/download_kcity_map_assets.sh --dry-run

# 누락된 지도 자산 다운로드 및 Drive MD5 검증
scripts/map/download_kcity_map_assets.sh
```

다운로드는 원격에 있고 로컬에 없는 파일을 추가합니다. 로컬에만 존재하는
파일을 삭제하지 않으며, 같은 경로의 내용이 다르면 덮어쓰지 않고 오류로
중단합니다. 다운로드 후 `rclone check`가 Drive에 있는 모든 지도 파일과
로컬 파일의 크기 및 MD5를 직접 비교합니다.

## 새 지도 업로드

새 버전은 기존 파일을 덮어쓰지 않고 버전 번호를 올립니다. 현재 Git 배포
지도는 v87이며, 이전 버전 지도·라우트 설정과 제작 산출물은 Git에 포함하지 않습니다.

```bash
# 업로드 대상과 충돌 여부를 실제 변경 없이 확인
scripts/map/upload_kcity_map_assets.sh --dry-run

# 신규 지도 업로드 및 Drive MD5 검증
scripts/map/upload_kcity_map_assets.sh
```

업로드 후 `rclone check`가 모든 로컬 지도 파일과 Drive 파일의 크기 및
MD5를 직접 비교합니다. 동일한 원격 경로에 내용이 다른 파일이 있으면
`--immutable` 정책에 의해 업로드가 실패합니다. 지도 자산만 추가하거나
다운로드한 경우에는 Git 커밋이 필요하지 않습니다. Git 커밋은 설정,
제작·검증 스크립트 또는 문서가 변경될 때만 수행합니다.

## 환경값 변경

기본값은 다음과 같습니다.

```text
KCITY_MAP_RCLONE_REMOTE=ajou_drive
KCITY_MAP_DRIVE_ROOT_FOLDER_ID=12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP
KCITY_MAP_DRIVE_PATH=kcity_map
```

다른 remote나 테스트 폴더를 사용할 때만 실행 전에 환경변수로 덮어씁니다.

```bash
export KCITY_MAP_RCLONE_REMOTE=my_drive
export KCITY_MAP_DRIVE_PATH=kcity_map_test
scripts/map/upload_kcity_map_assets.sh --dry-run
```

## 맵 제작 스크립트

맵 제작 도구는 `kcity_map/`을 작업 디렉터리로 사용합니다.

```bash
cd ~/AjouNice2026/kcity_map

python3 scripts/kcity_map_workflow/generate_drive_course_from_map.py \
  --map lanelet_map/lanelet2_map_v87.osm \
  --reference-csv /path/to/reference_lanelets.csv \
  --out-dir competition_route/v87 \
  --output-prefix drive_course_v87
```

일부 Autoware 연동 검증 스크립트는 선택적으로 상위 워크스페이스의
`autoware_data/`를 사용합니다. Autoware 자체는 이 저장소에 포함하지
않습니다.
