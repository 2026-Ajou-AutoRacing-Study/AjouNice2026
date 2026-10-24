# 지도 범용 도구

현재 지도·자산 준비 기준은 [지도 README](../../README.md)를 따른다.

- `audit_kcity_maps.py`, `audit_lanelet_polygons.py`: 지도 구조·정렬 검사.
- `generate_drive_course_from_map.py`: 명시한 map/reference CSV로 코스 생성.
- `simplify_lanelet_osm.py`, `sort_invalid_lanelet_bounds.py`,
  `transplant_way_geometry.py`, `drop_curvature_warning_points.py`: 명시적 지도 변환.
- `autoware_*`: 기존 Autoware 검증 probe의 호환 경로. 현재 ROS1 운영이나 자동 지도 수정
  도구가 아니며, 이번 정리에서는 구현·실행 조건을 바꾸지 않았다.

버전별 build/fix/validate/visualize는 [제작 이력](../../legacy/README.md)으로 분리했다.
보관된 도구의 `__file__`·sibling import는 원래 위치 기준이므로 당시 Git tree에서 재현한다.
현재 도구도 실행 전에 `--help`와 입력·출력·로컬 의존성을 확인한다.
큰 지도와 코스 산출물은 Git이 아닌 Drive 관리 대상이다.
