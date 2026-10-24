# 재현용 MORAI 설정 archive

현재 정책과 동결 코드가 직접 참조하는 입력은 [상위 README](../README.md)를 따른다.
이곳에는 현재 코드·테스트의 직접 참조가 없는 작은 검토/선택 설정만 이동했다.
원본 데이터·수집 출력·native GT·모델·ignored 대용량 자산은 이동하지 않았다.

- `base3_static_lraspp_selection_v1.json`: 과거 segmentation 선택.
- `base5_*sparse_camera_review_261004.json`: 초기 Base5 sparse camera 검토.
- `generated/static_classcentric_base3_batch008_dynamic_ped_*.json`: 당시 pilot/replacement 설정.
- `scenario_runner_asset_catalog.yaml`: Scenario Runner 예제에서 관찰한 참고 catalog; 전체 MR 자산 목록이 아니다.

이동 전후 경로·원문 SHA는 [dataset archive manifest](../../docs/legacy/MANIFEST.md)에 있다.
재현 시 당시 source·catalog·단위를 명시하며 현재 output의 config를 덮어쓰지 않는다.
