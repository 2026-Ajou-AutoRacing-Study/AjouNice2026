# Base5 초기화와 최적화 계획의 이전 경로

문서 상태: COMPATIBILITY ROUTER. 2026-10-12에 현재 상태와 진행 이력을 분리했다.
현재 모델·완료 학습·남은 작업의 단일 기준은 [CURRENT](CURRENT.md)다.
이전 계획의 명령·숫자·실측은 [원문 보존 목록](../perception/legacy/context_261012/README.md)에 있다.
이 경로는 기존 링크를 유지하며 현재 실행 상태를 별도로 복사하지 않는다.

## 기존 링크 호환

<a id="base5-current-plan-hub"></a>
현재 기준: [CURRENT](CURRENT.md). 당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub).

<a id="base5-training-inference-memory-decision-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-training-inference-memory-decision-261011).

<a id="base5-training-compile-short-gate-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-training-compile-short-gate-261011).

<a id="base5-depth25-local-optimization-plan-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-depth25-local-optimization-plan-261011).

<a id="base5-학습-초기화와-bev-연산-최적화-계획"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-학습-초기화와-bev-연산-최적화-계획).

<a id="현재-계획-허브--2026-10-12-원거리-평가기억-ab-완료와-후속-전달-안정화"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#현재-계획-허브--2026-10-12-원거리-평가기억-ab-완료와-후속-전달-안정화).

<a id="문서-갱신-규칙"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#문서-갱신-규칙).

<a id="학습-48gbbatch-4와-대회-추론-24gb의-분리--2026-10-11-1603-kst-합의"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#학습-48gbbatch-4와-대회-추론-24gb의-분리--2026-10-11-1603-kst-합의).

<a id="학습-compile의-짧은-4090-검사--2026-10-11-1445-kst-합의"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#학습-compile의-짧은-4090-검사--2026-10-11-1445-kst-합의).

<a id="2026-10-11-최신-합의-depth-25cm-후보와-본학습-전-로컬-최적화"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#2026-10-11-최신-합의-depth-25cm-후보와-본학습-전-로컬-최적화).

<a id="학습-후보와-성능-기대의-경계"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#학습-후보와-성능-기대의-경계).

<a id="다음-실행-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#다음-실행-순서).

<a id="2026-10-10-상태-갱신"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#2026-10-10-상태-갱신).

<a id="sunny13-수집-종료-후-묶음-실험-합의"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#sunny13-수집-종료-후-묶음-실험-합의).

<a id="선택과-종료-기준"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#선택과-종료-기준).

<a id="결정과-남은-선택"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#결정과-남은-선택).

<a id="base3e25-초기화의-실제-이력"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base3e25-초기화의-실제-이력).

<a id="base5-초기화-추천과-로딩-검증"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-초기화-추천과-로딩-검증).

<a id="bevpoolv2-검토-근거와-한계"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#bevpoolv2-검토-근거와-한계).

<a id="최적화-ab-구현-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#최적화-ab-구현-순서).

<a id="수집-중-가능한-검증과-종료-후-비용-시험"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#수집-중-가능한-검증과-종료-후-비용-시험).

<a id="수집-종료부터-학습배포까지"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#수집-종료부터-학습배포까지).

<a id="tensorrt-후속-시험의-범위와-작업-예산"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#tensorrt-후속-시험의-범위와-작업-예산).
