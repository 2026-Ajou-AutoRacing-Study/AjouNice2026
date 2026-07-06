# K-City Map

K-City 지도 원본/가공 파일을 보관하는 디렉터리입니다.

## Directory Layout

- `lanelet_map/`: Lanelet2 `.osm`, projector YAML, pointcloud map 등 Lanelet 기반 map asset
- `mgeo_map/`: MORAI MGeo JSON map
- `gis_map/`: K-City GIS shapefile 원본

## Git Policy

- `.gitignore`에서 `kcity_map/` 전체를 ignore합니다.
- 따라서 새로 생성되거나 복사한 map asset은 기본적으로 git에 올라가지 않습니다.
- 단, 아래 파일들은 ignore rule 추가 전에 이미 git이 추적하던 파일이라 GitHub에 남아 있습니다.
  - `kcity_map/README.md`
  - `kcity_map/mgeo_map/global_info.json`
  - `kcity_map/mgeo_map/link_set.json`
  - `kcity_map/mgeo_map/node_set.json`
- 이미 tracked 상태인 파일은 `.gitignore`만으로 자동 제거되지 않습니다.

## Notes

- 비공개 map 또는 용량이 큰 map은 git에 직접 업로드하지 않습니다.
- 공유가 필요한 map은 별도 저장소/드라이브/다운로드 스크립트 등으로 관리하는 방식을 우선 검토합니다.
- 새 map 파일을 의도적으로 commit해야 하는 경우에는 `.gitignore` 예외 또는 `git add -f` 사용 여부를 먼저 명확히 정합니다.
