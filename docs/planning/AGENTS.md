# 플래닝 문서와 로스백 분석 결과

- `docs/planning/`에는 재사용 가능한 설계, 실행 계약, 통합 지침을 둔다.
- 개별 로스백 분석 Markdown, 이미지, CSV, JSON, 재생 로그는 저장소 루트의
  `planner_logs/analysis/<run_id>/`에 함께 저장한다. 이 폴더는 Git 제외 대상이다.
- 여러 백의 일회성 비교 보고서도 `planner_logs/analysis/<comparison_id>/`에 저장한다.
- 기존 일회성 Markdown은 `planner_logs/analysis/reports/`에 모아 두었다.
- 과거 분석 보고서를 현재 코드·실행값·최신 백의 증거로 대체하지 않는다.
- 분석 아티팩트를 커밋하거나 `docs/planning/`에 복제하지 않는다.
