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

`legacy/`가 붙은 문서, `docs/jsw/`, `external_docs/`, 과거 detector 폴더는 재현·참고
기록이다. 날짜가 최신이거나 내용이 자세하다는 이유만으로 CURRENT 실행값을 덮어쓰지
않는다. `topic_interface.md`와 `topic_interface_example.md`는 설계 참고이며 실제 launch와
`rostopic info`를 우선한다.
