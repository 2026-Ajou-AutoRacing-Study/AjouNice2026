# MORAI 생성·변환 입력

현재 작업은 [Base5 CURRENT](../docs/BASE5_V2_CURRENT.md)와
`scripts/portable_base5/generation.py`의 공통 6환경 진입점을 따른다.

## 현재 정책 입력

| 파일 | 역할 |
|---|---|
| `morai_asset_catalog_mr_base5_261005.json` | MR 자산 eligibility·원래 의미·제외 이유 |
| `base5_asset_appearance_policy_v1.json` | 정적/이동 외관 역할 |
| `base5_stationary_geometry_mr_261006.json` | spacing용 envelope; native GT의 대체물이 아님 |
| `../../../configs/dataset/base5_midrange_reinforcement_v163.json` | 공통 중거리 보강 프로파일 |

## 동결 경로와 재현

이곳의 Base3/OD180·초기 Base5 입력 중 코드·테스트가 직접 참조하는 파일은
기존 source/runtime hash·기본 경로 재현을 위해 원래 이름으로 남겼다.
`base5_independent_5k_sunny11.json`은 역사적 호환 alias로 실제 hour가 13이다.
이 이름만 보고 Sunny11 수집을 실행하지 않는다. 환경·seed·release는 공통 진입점에서 명시한다.
구 `base5_independent_v2.json`·`base5_static_v1.json`·roundabout v118은 현재 배치 승인이 아니다.

직접 코드 참조가 없는 옛 검토/selection/Scenario Runner 입력은 [legacy](legacy/README.md)로 분리했다.
현재 manifest·동결 config를 in-place로 바꾸거나 과거 output에 새 코드를 강제 resume하지 않는다.
이전 설명의 원문은 [정리 전 snapshot](../docs/legacy/context_261024/configs_README.before.md)에 있다.
