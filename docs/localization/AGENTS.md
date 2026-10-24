# Localization documentation agent context

이 디렉터리에서는 `README.md`가 문서 router다. 다음 순서로 읽는다.

1. `README.md`
2. `COMPETITION_LOCALIZATION_CURRENT_STATE.md`
3. `BAG_CATALOG.md`
4. `CURRENT_EVIDENCE.md`

모든 실험은 `EXPERIMENT_TEMPLATE.md`의 provenance를 기록한다. 특히 MR/MRP,
`gt_source`, estimator/evaluator heading offset과 time policy를 생략하지 않는다.
`legacy/`는 역사·재현용으로만 사용한다. 명시적 재현 요청 또는 현재 근거의 특정
인용을 확인할 때만 해당 파일을 읽고, 완료된 실험/PLAN을 새 현재 정책으로 사용하지 않는다.
센서·네트워크·validator 및 planning/control의 정책을 이 문서에서 추론하지 않는다.
