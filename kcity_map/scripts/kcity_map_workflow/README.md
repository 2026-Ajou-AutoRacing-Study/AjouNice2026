# K-City Map Workflow Scripts

맵 생성·변환·검증과 대회 코스 산출에 사용한 Python 스크립트를 보관합니다.

기본 맵 경로는 저장소 루트를 기준으로 설정되어 있습니다. `autoware_*` 스크립트와 일부 버전 전환 스크립트는 추가 검증을 위해 저장소 상위의 로컬 `autoware_data/`를 선택적으로 참조합니다. Docker, Autoware 소스와 런타임 데이터는 이 저장소에 포함되지 않습니다.

큰 맵과 지도 버전별 대회 코스 산출물은 Git에 넣지 않고 Google Drive의
`kcity_map/` 아래에서 공유합니다. 업로드와 다운로드에는 저장소 최상위의
동기화 스크립트를 사용합니다.

```bash
bash scripts/map/upload_kcity_map_assets.sh
bash scripts/map/download_kcity_map_assets.sh
```

포함 경로는 `configs/map/kcity_map_drive.filter`가 정의합니다. 기존 원격
파일을 교체하는 작업은 대상 버전과 체크섬을 먼저 확인한 뒤 수행합니다.

실행 전에 각 스크립트의 `--help`로 필요한 Python 패키지와 입·출력 경로를 확인하세요.
