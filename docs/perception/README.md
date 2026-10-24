# 인지와 데이터셋 작업 안내

문서 상태: ROUTER. 정리 기준: 2026-10-24.
에이전트는 아래에서 **현재 요청에 해당하는 한 행**을 선택한다.
모델·데이터·ROS·후단을 모두 바꾸는 작업이 아니라면 다른 영역의 문서를 미리 전부 읽지 않는다.

## 작업별 시작점

| 작업 | 먼저 읽을 문서 | 필요한 경우에만 추가 |
|---|---|---|
| 다음 인지 작업·새 bag 수령 후 우선 분석 | [활성 우선순위](../ACTIVE_PRIORITIES.md#perception-priorities) | 단계별 지연 입력은 같은 문서의 [확인 목록](../ACTIVE_PRIORITIES.md#perception-latency) |
| Base5 학습·checkpoint·공간 격자·추론 최적화 | [모델 CURRENT](../bevfusion/CURRENT.md) | [근거 인덱스](../bevfusion/README.md)에서 관련 절 |
| Base5 생성·수집·resume·부족분 보충 | [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md#capture) | [통합 생성·수집 근거](../../dataset/morai_od/docs/261018_base5_unified_generation_capture.md) |
| Native GT·JPEG·가시성·학습 클래스 변환 | [데이터 GT 계약](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md#gt) | 모델 입력을 바꾸면 [모델 CURRENT](../bevfusion/CURRENT.md#model) |
| BEVFusion ROS bridge·배포·모델 선택 | [모델 실행 기준](../bevfusion/CURRENT.md#runtime) | [패키지 README](../../base_ws/src/perception/bevfusion_ros/README.md), production 작업이면 handoff |
| 신호등 YOLO·Python·GPU 환경 | [신호등 README](../../base_ws/src/perception/traffic_light_detection/README.md) | 현재 launch와 대상 호스트 실행 상태 |
| Tracker·UNKNOWN policy·prediction | [후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md) | [검출·추적 근거 인덱스](../bevfusion/README.md#evaluation) |
| Base3·Base4·PointPainting·Fast-BEV 재현 | [재현 문서 선택](../../dataset/morai_od/docs/README.md#reproduction) | 명시된 당시 source·config·dataset 계약 |

## 상태와 문서 역할

- CURRENT: 지금 선택한 계약·완료 상태·남은 작업을 해당 영역에서 한 번만 관리한다.
- ROUTER/README: 관련 문서를 고르는 안내다. 날짜별 결과를 복사하지 않는다.
- RUNBOOK/HANDOFF: 실행 순서·명령·안전 경계다. 실행 중 여부는 실제 프로세스·journal에서 확인한다.
- EVIDENCE: 당시 표본·설정·수치·실패와 재현 근거다. 필요한 anchor의 절만 읽는다.
- LEGACY/REPRODUCTION: 과거 설계·실험·데이터 계약이다. 현재 기본값으로 자동 선택하지 않는다.

[모델 CURRENT](../bevfusion/CURRENT.md)와 [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)는
각각 다른 사실을 소유한다. 모델 설정을 데이터 상태 문서에 길게 복사하거나 그 반대로 하지 않는다.
운영 alias·resolved config·동결 계약이 문서와 다르면 차이를 확인하고 해당 CURRENT만 수정한다.

## 범위와 갱신 규칙

이 정리는 인지·데이터셋 문서에 한정한다. planning·localization·planning_validator의
소유권·실행 설정은 변경하지 않는다. TF·시간축을 실제로 변경하는 작업에서만 해당 영역 문서를 읽는다.

AGENTS에는 안정적인 규칙과 읽기 경로만 둔다. 새로운 ‘최신 결과’ 단락을 계속 앞에 넣지 않는다.
결정이 바뀌면 CURRENT의 기존 항목과 검증 날짜를 갱신하고 근거를 연결한다.
기본 검색은 저장소 `.ignore`를 따르며 `legacy/`·`evidence/`·FastBEV/PointPainting 이력을 제외한다.
과거 재현이나 특정 근거 확인 때만 `rg --no-ignore <명시적 경로>`로 포함한다.
원본·동결 실행·후보 설정을 섞지 않으며 실험 검증을 production 승격으로 읽지 않는다.

Context 문서를 바꾼 뒤 [문서 검증기](../../scripts/docs/check_perception_context.py)를 실행한다.
명령은 `/usr/bin/python3 scripts/docs/check_perception_context.py`이며 링크·기존 anchor·원문 SHA256·문서 길이·계약 표기를 검사한다.
검증기는 ROS·GPU·네트워크·수집 상태를 변경하지 않는다.

## 보존된 이전 진입점

[2026-10-12 원문 스냅샷](legacy/context_261012/README.md)에 중복 이력을 무손실로 보존했다.
2026-10-24 정리의 현재 모델·후단 원문은 [BEVFusion 이력](../bevfusion/legacy/context_261024/CURRENT.md)과 [후단 이력](../../base_ws/src/perception_object_pipeline/docs/legacy/context_261024/TRACKING_PREDICTION_CURRENT.md)에 있다.
기존 자주 참조된 파일·anchor는 현재 문서나 해당 이력으로 연결되는 호환 경로를 유지한다.
