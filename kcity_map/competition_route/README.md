# Competition Route Artifacts

현재 배포 Route A/B는 다음 설정을 사용합니다.

`base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml`

아래 코스 산출물은 로컬 작업용이며 Git에 포함하지 않습니다.

- 출력 규칙: `competition_route/vXX`
- 기준 코스 좌표: 실행 시 `--reference-csv`로 명시
- 생성 스크립트: `scripts/kcity_map_workflow/generate_drive_course_from_map.py`

`attempt1`, `attempt2`, `attempt3` 같은 하위 폴더는 사용하지 않습니다. 동일 지도 버전을 다시 생성해야 하면 명시적으로 재생성하고, 일반적인 새 작업은 새 버전 폴더를 만듭니다.

저장소의 `kcity_map/`에서 실행 예시:

```bash
python3 scripts/kcity_map_workflow/generate_drive_course_from_map.py \
  --map lanelet_map/lanelet2_map_v87.osm \
  --reference-csv /path/to/reference_lanelets.csv \
  --out-dir competition_route/v87 \
  --output-prefix drive_course_v87
```

과거 제작 이력은 [legacy 안내](../legacy/README.md)로 분리했다. 현재 route를 과거 산출물로 대체하지 않는다.
