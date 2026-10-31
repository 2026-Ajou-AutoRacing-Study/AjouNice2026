# 지도 제작 이력

버전별 제작·검증 스크립트는 `version_workflow/`, 과거 보고서·그림은 `reports/`로 이동했다.
현재 지도 자산이나 속도 상한을 바꾼 것이 아니다. 최신 v85 산출 검증 보고서는 `../docs/`에 남겼다.

원문은 byte 단위로 보존했다. 원래 경로·SHA는
[전체 manifest](../../docs/maintenance/context_archive_261024.json)를 참고한다.
`ROOT = Path(__file__).parents[...]`와 sibling import 등은 원래 구조 기준이므로 보관 위치에서
바로 실행하지 않는다. 필요한 당시 commit을 별도 worktree에서 복원해 재현한다.
