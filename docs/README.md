# AjouNice2026 문서 진입점

현재 작업은 아래 router만 우선한다.

1. [`ACTIVE_PRIORITIES.md`](ACTIVE_PRIORITIES.md): 실제 미완료 작업.
2. [`bevfusion/README.md`](bevfusion/README.md): production perception과 다음 실험.
3. [`localization/README.md`](localization/README.md): production localization.
4. [`../dataset/morai_od/docs/README.md`](../dataset/morai_od/docs/README.md): 현재 dataset.
5. [`../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md`](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md):
   production Tracking·Map-CV 계약.
6. [`runbooks/morai_full_stack_by_terminal.md`](runbooks/morai_full_stack_by_terminal.md):
   full-stack 실행 순서.
7. [`competition/README.md`](competition/README.md): 대회 최종 규정 v1.2와 과거 규정 구분.
8. [`../kcity_map/README.md`](../kcity_map/README.md): 현재 지도·자산 준비와 제작 이력.
9. [`../configs/README.md`](../configs/README.md): 센서·네트워크와 분야별 설정 안내.

`legacy/`·과거 detector는 재현용이다. 날짜나 분량으로 CURRENT를 덮어쓰지 않는다.
초기 토픽·LiDAR·지도 정렬 설계는 `legacy/design_2605/`에 보관했으며 실제 launch와
`rostopic info`가 현재 인터페이스의 기준이다. 원문·검색·재현 규칙은
[문맥 관리 안내](maintenance/CONTEXT_MAINTENANCE.md)를 따른다.
