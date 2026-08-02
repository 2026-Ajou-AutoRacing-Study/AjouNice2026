# Competition Route Artifacts

대회 주행 코스 산출물은 지도 버전별로 보관합니다.

- 출력 규칙: `competition_route/vXX`
- 기준 코스 좌표: `competition_route/v41/drive_course_v41_lanelets.csv`
- 생성 스크립트: `scripts/kcity_map_workflow/generate_drive_course_from_map.py`

`attempt1`, `attempt2`, `attempt3` 같은 하위 폴더는 사용하지 않습니다. 동일 지도 버전을 다시 생성해야 하면 명시적으로 재생성하고, 일반적인 새 작업은 새 버전 폴더를 만듭니다.

저장소 루트에서 실행 예시:

```bash
python3 scripts/kcity_map_workflow/generate_drive_course_from_map.py \
  --map lanelet_map/lanelet2_map_v44.osm \
  --reference-csv competition_route/v41/drive_course_v41_lanelets.csv \
  --out-dir competition_route/v44 \
  --output-prefix drive_course_v44
```
