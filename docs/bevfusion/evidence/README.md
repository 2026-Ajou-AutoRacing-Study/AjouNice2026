# BEVFusion 상세 근거 보관

현재 모델·실행 결정은 [CURRENT](../CURRENT.md), 근거 선택은 [상위 README](../README.md#evaluation)를 따른다.
이 디렉터리는 기본 검색에서 제외한다. 필요한 anchor의 절만 명시적으로 읽는다.

- `261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md`: 65절의 입력·학습·검증·production 승격 근거. 과거 ‘E24 유지’를 현재 결정으로 읽지 않는다.
- `261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md`: 완료된 동결 학습·재개·전송·백업의 원문 이력. 당시 endpoint·가격·진행 상태는 현재값이 아니다.

동결 source·resume 실행기·모델 config·후보 검증 replay는 이동하지 않았다.
정리 전 원문 SHA256와 old→new는 ignored `artifacts/cleanup_261024/perception_moves.json`에 기록했다.
보존 문서는 baseline Git 원문 bytes를 유지하므로 내부 상대 링크의 기준은 manifest의 `old_path`다.
이력 내부 링크를 현재 실행 경로로 해석하지 말고, 현재 README의 직접 링크·anchor에서 필요한 근거를 선택한다.
