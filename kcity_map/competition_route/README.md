# Competition Route Artifacts

대회 주행 코스 산출물은 지도 버전별로 보관합니다.

- 출력 규칙: `competition_route/vXX`
- 단일 기준 파일: `competition_route/vXX/route_definition.yaml`
- 기준 코스 좌표: `competition_route/v41/drive_course_v41_lanelets.csv`
- 생성 스크립트: `scripts/kcity_map_workflow/generate_drive_course_from_map.py`

`attempt1`, `attempt2`, `attempt3` 같은 하위 폴더는 사용하지 않습니다. 동일 지도 버전을 다시 생성해야 하면 명시적으로 재생성하고, 일반적인 새 작업은 새 버전 폴더를 만듭니다.

v60 preferred/candidate 설정을 반영해 재생성하는 예시(`kcity_map`에서 실행):

```bash
python3 scripts/kcity_map_workflow/generate_drive_course_from_map.py \
  --map lanelet_map/lanelet2_map_v60.osm \
  --reference-csv competition_route/v60/drive_course_v60_lanelets.csv \
  --route-definition competition_route/v60/route_definition.yaml \
  --out-dir competition_route/v60 \
  --output-prefix drive_course_v60
```

`route_definition.yaml`에는 맵 버전/해시, 반드시 선호 경로에 포함할 lanelet,
preferred/candidate 목록과 candidate 할당을 함께 둡니다. 플래너와 산출물 생성기는
이 파일을 같이 읽습니다. 생성기는 결과 폴더의 `route_definition.yaml`도 정규화해
다시 기록하고, 직진 successor 뒤에 이어지는 의도된 차선 변경까지 topology로
검증합니다.
