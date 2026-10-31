# MORAI 데이터셋 문서 라우터

문서 상태: ROUTER. 정리: 2026-10-24.
현재 작업은 [Base5 CURRENT](BASE5_V2_CURRENT.md)에서 시작하고 필요한 근거 하나만 선택한다.

| 작업 | 문서 |
|---|---|
| 승인 30,750장·6환경·10개 cohort·실제 입력/ledger | [Base5 CURRENT](BASE5_V2_CURRENT.md#data) |
| 공통 생성·수집·복구·Sunny11/13/15 전달 | [통합 실행 근거](261018_base5_unified_generation_capture.md) |
| Foggy11/13/15 마감·신규 1장·Sunny 장애물 750장 | [후속 수집·전달 근거](261019_base5_foggy13_reinforcement_and_capture.md) |
| native GT·JPEG·원본 보존·학습 class | [GT 계약](BASE5_V2_CURRENT.md#gt) |
| MR 자산 eligibility·공식 제외·원래 의미 | [MR 자산 계약](MR_ASSET_COLLECTION_CURRENT.md) |
| 모델·checkpoint·실제 학습/추론 | [BEVFusion CURRENT](../../../docs/bevfusion/CURRENT.md) |

CPU preflight·소량 검증·데이터 준비와 본수집/학습 시작 승인은 별개다.
실시간 상태는 해당 실행 journal에서 확인하며 완료 출력·부모 ledger를 다시 합산하지 않는다.

<a id="reproduction"></a>

## 명시적인 재현만

Base3-Online, Base4 rosbag, Base5 v1~v158의 계획·옛 CURRENT·날짜별 실험은
[legacy 계보와 manifest](legacy/README.md)에서 요청한 세대만 고른다.
기본 검색은 `legacy/`를 제외한다. 과거 명령·class·단위·resume 옵션을 현재 기본값으로 사용하지 않는다.
동결 scripts·config 경로가 남아 있어도 최신 수집 진입점이라는 뜻은 아니다.
