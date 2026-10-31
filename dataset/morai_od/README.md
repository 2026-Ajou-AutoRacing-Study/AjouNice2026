# MORAI perception dataset

MORAI 3D object detection의 시나리오 생성·센서 수집·검증·변환 코드를 관리한다.
현재 지원 입력은 **30,750장 / train 26,136 / val 4,614**, Sunny/Foggy 11·13·15시의
6환경과 Sunny 장애물 추가 보강이다. 원본 데이터나 학습 입력을 이 코드 트리로 옮기지 않는다.

## 시작점

- 현재 데이터·원본 보존·수집/복구 계약: [Base5 CURRENT](docs/BASE5_V2_CURRENT.md)
- 작업별 근거 선택: [문서 라우터](docs/README.md)
- 학습·checkpoint·추론: [BEVFusion CURRENT](../../docs/bevfusion/CURRENT.md)
- 과거 세대의 명시적 재현: [legacy 계보](docs/legacy/README.md)

## 코드와 입력

| 경로 | 역할 |
|---|---|
| `../../scripts/portable_base5/` | 새 환경 공통 생성·실행·전달 진입점 |
| `scripts/`·`tests/` | 동결 수집의 원형 구현·회귀·재현 source |
| `configs/` | 현재 자산 정책과 동결 코드가 참조하는 호환 입력 |
| `../../configs/dataset/` | 현재 공통 보강 프로파일 |
| `manifests/`·`metadata/`·`scenarios/` | 작은 fixture·고정 split과 로컬 생성물 |
| `docs/` | CURRENT·최신 근거·명시적 legacy archive |

`scripts/`와 `tests/`는 과거 release가 파일 목록·SHA·로컬 import closure까지 고정한다.
옛 파일을 임의로 rename하거나 새 helper를 이 디렉터리에 넣지 않는다.
구 수집기·실험 config의 존재는 실행 승인이나 현재 기본값을 의미하지 않는다.
raw·native GT·실패 근거·선택 ledger·JPEG·압축본·모델은 이번 문서 정리의 이동/삭제 대상이 아니다.
