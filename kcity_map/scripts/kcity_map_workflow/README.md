# K-City Map Workflow Scripts

현재 지도에 재사용할 수 있는 생성·변환·검증 도구를 보관합니다.
이전 버전 전용 제작·검증 스크립트와 ROS2 실험용 probe는 배포하지 않습니다.

기본 맵은 `lanelet_map/lanelet2_map_v87.osm`입니다. 입력·출력을 수정하는
도구는 경로와 출력 버전을 명시해야 하며 배포 지도를 직접 덮어쓰지 않습니다.
그림·검증 기록·일회성 스크립트는 Git 제외 경로인 `planner_logs/analysis/`에 둡니다.

v87 OSM은 Git 배포에 포함합니다. Pointcloud·GIS 등 별도 지도 자산은
Google Drive의 `kcity_map/` 아래에서 공유합니다. 업로드와 다운로드에는
저장소 최상위의 동기화 스크립트를 사용합니다.

```bash
bash scripts/map/upload_kcity_map_assets.sh
bash scripts/map/download_kcity_map_assets.sh
```

포함 경로는 `configs/map/kcity_map_drive.filter`가 정의합니다. 기존 원격
파일을 교체하는 작업은 대상 버전과 체크섬을 먼저 확인한 뒤 수행합니다.

실행 전에 각 스크립트의 `--help`로 필요한 Python 패키지와 입·출력 경로를 확인하세요.

과거 버전 전용 도구는 [legacy](../../legacy/README.md)에 보존했다. 원래 Git tree에서 재현하며 현재 배포 도구로 자동 사용하지 않는다.
