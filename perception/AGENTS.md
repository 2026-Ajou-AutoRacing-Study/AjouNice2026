# 인지 모델 작업 지침

## 시작 문서

[인지와 데이터셋 작업 안내](../docs/perception/README.md)에서 **현재 작업에 해당하는 행만** 선택한다.
모델 작업의 현재 기준은 [BEVFusion CURRENT](../docs/bevfusion/CURRENT.md)다.
수집·라벨 변환을 건드릴 때만 [Base5 데이터 CURRENT](../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 추가한다.
Base3·Base4·PointPainting·Fast-BEV 문서는 명시적인 재현 작업에서만 읽는다.

## 작업 경계

- 실제 production과 opt-in 시험 모델을 구분한다. 최신 checkpoint를 자동 승격하지 않는다.
- 데이터 원본·native annotation·완료 ledger·동결 학습 source와 resume 계약을 보존한다.
- 후보 설정을 바꿀 때 입력·좌표·강도·클래스·후처리·학습/추론 옵션의 차이를 명시한다.
- GPU 비용 시험, 정확도 평가, 실제 ROS 입력, tracker, 폐루프 검증을 서로 대체하지 않는다.
- 다른 agent의 planning_validator·planning·MORAI 세션은 작업에 포함되지 않으면 변경하지 않는다.
- 코드·데이터를 바꾸기 전에 해당 하위 지침과 실행 config/launch를 확인한다.

## 문서와 정리

AGENTS에는 읽기 경로와 안정적인 규칙만 둔다. 날짜별 진행 결과를 앞에 계속 덧붙이지 않는다.
현재 결정은 해당 CURRENT의 기존 항목을 갱신하고, 실측은 EVIDENCE의 관련 절에 남긴다.
근거 전체를 한 번에 읽지 말고 필요한 anchor의 절을 선택한다.
검색은 기본적으로 `legacy/`를 제외하고 과거 재현이 필요할 때만 포함한다.
정상적인 한국어 띄어쓰기와 ‘~다’ 문체를 사용하며 명령·경로·수치·해시는 보존한다.
여러 대답에 걸친 한 작업이 끝나면 관련 구현·검증·문서를 묶어 현재 브랜치에 일반 commit/push한다.
작은 검사마다 commit하지 않으며 raw·민감 정보·ignored 산출물을 강제로 추적하지 않는다.
