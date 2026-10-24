# 현재 문서와 과거 기록 관리

문서·도구 정리 기준: 2026-10-24. 운영 알고리즘 변경이 아닌 문맥·경로 정리다.

## 읽는 순서

저장소 `AGENTS.md` → `docs/README.md` → 해당 분야 README → CURRENT → 필요한 근거만 읽는다.
CURRENT는 실행 기준, EVIDENCE는 조건부 실험 결과, RUNBOOK은 실행 절차다.
오래된 계획·실패 후보·대체된 문서를 CURRENT와 병렬인 권위로 취급하지 않는다.

## 보관과 검색

- 분야별 `legacy/`는 재현용 원문이다. 삭제 대신 원래 내용과 이동 경로를 보존했다.
- `.ignore`는 일반 `rg` 검색에서 역사 자료와 구 detector를 제외한다. Git 추적·빌드·실행에는
  영향을 주지 않는다. 특정 과거 자료가 필요하면 경로를 직접 열거나 `rg --no-ignore`를 쓴다.
- [이동·원문 SHA 목록](context_archive_261024.json)에서 `original_path`와
  `archive_path`를 찾는다. `preserve_before_rewrite`는 현재 안내를 줄이기 전의 사본이다.
- 보관 원문의 상대 링크·고정 경로·`__file__` 기반 경로는 당시 구조 그대로다. 보관 도구를
  새 위치에서 바로 실행한다고 보장하지 않는다. 재현은 manifest의 원본 commit과 경로를
  별도 worktree에 복원해 수행하며, 현재 수집기·스택에 덮어쓰지 않는다.
- 외부에서 계속 참조하는 일부 옛 경로에는 내용 없는 이동 안내만 남길 수 있다.
  호환 안내는 운영 설정이나 옛 문서 전체의 복제본이 아니다.

## 이동하지 않은 코드

현재 build/launch/import가 참조하는 Fast-LIO·FastBEV·PointPainting 및 submodule은
폴더명만 보고 제거하지 않는다. 호환용임을 명시하고 기본 탐색에서 분리한다.
Base5 수집기의 script inventory와 runtime/resume 계약은 동결 상태를 유지한다.
센서·네트워크·validator는 해당 담당자가 정리하며, planning/control은 이 정리의 수정 범위가 아니다.

2026-10-24 통합 정리는 이동 420개·원문 snapshot 24개·보호된 planning 링크용 그림 복사 1개를
manifest에 기록했다. 필수 mirror·Sensor4/5·validator source·동결 수집 profile은 원경로를
유지하고 역할표의 내부 호환 항목으로만 안내한다. JSON 값·운영 모델·launch 기본값은 바꾸지 않았다.
현재 측위/모델/데이터 문서와 인덱스만 갱신하고, 과거 실험 수치와 원문을 고쳐 쓰지 않았다.

## 이후 갱신

새 채택 결과는 CURRENT의 해당 항목을 교체하고 상세 근거만 연결한다. 완료된 계획을 계속
위에 덧붙이지 않는다. 폐기·기각 이유는 보관하되 기본 실행 안내로 노출하지 않는다.
이동 후에는 현재 진입점 링크, 활성 코드의 의존 경로, 원문 SHA와 동결 코드 불변을 검사한다.

```bash
python3 scripts/docs/check_perception_context.py
python3 scripts/maintenance/check_context_archive.py
```
