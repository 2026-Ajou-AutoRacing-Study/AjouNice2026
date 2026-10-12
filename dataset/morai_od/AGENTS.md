# MORAI 데이터셋 작업 지침

## 시작 문서

현재 Base5 생성·수집·보충·GT 변환은 [Base5 CURRENT](docs/BASE5_V2_CURRENT.md)에서 시작한다.
[문서 인덱스](docs/README.md)는 다른 작업을 선택할 때만 읽는다.
학습·추론 설정까지 바꿀 때만 [BEVFusion CURRENT](../../docs/bevfusion/CURRENT.md)를 추가한다.
Base3-Online·Base4 rosbag 수집은 명시적인 재현 요청에서만 해당 계약을 읽는다.

## 보존과 안전

- 완료 원본·native GT·실패 근거·selected ledger·source/runtime 해시를 보존한다.
- 구 output에 변경된 코드나 다른 manifest를 강제로 resume하지 않는다.
- 새 후보·보충은 새 경로와 명시적인 계보를 사용한다. 이미 포함된 부모 성공분을 다시 합산하지 않는다.
- 배치 수·승인 장면 수·가시 component GT 수·학습 클래스 수를 구분한다.
- 객체의 원래 의미·asset·parent/component 관계를 보존한다. 학습 클래스 변경은 파생 라벨 정책이다.
- 생성·CPU preflight·소량 live 검증을 본수집 완료로 해석하지 않는다.
- 수집 통신과 실주행 streaming의 네트워크 설정을 혼동하거나 중복 collector를 실행하지 않는다.
- 무수신 CollisionData·통신/Park 불확실·hash 불일치를 일반 장면 부족분으로 우회하지 않는다.
- 움직이는 외관·두 waypoint 높이·심각한 부유 검사 등 현재 QA는 런북과 동결 release를 따른다.
- MORAI 조작·대량 수집·기존 데이터 삭제는 해당 사용자 요청 범위에서만 수행한다.

## 문서와 정리

AGENTS에 날짜별 작업 이력을 쌓지 않는다. 현재 상태는 Base5 CURRENT에서 한 번만 관리한다.
진행 결과는 관련 EVIDENCE에 기록하고 CURRENT에는 결론·경로·근거만 남긴다.
기본 검색에서 `legacy/`를 제외한다. 과거 명령은 명시적인 재현 작업에서만 사용한다.
정상적인 한국어 띄어쓰기와 ‘~다’ 문체를 사용하며 명령·경로·해시·수치는 보존한다.
한 작업이 끝나면 관련 변경·검증·문서를 묶어 현재 브랜치에 일반 commit/push한다.
