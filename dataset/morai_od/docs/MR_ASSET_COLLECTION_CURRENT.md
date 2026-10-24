# MR 자산의 현재 eligibility와 원래 의미

문서 상태: CURRENT. 근거 확인: 2026-10-12.
대회용 MR 자산 eligibility와 수집 원본의 의미를 관리한다.
장면 배치·현재 수집은 [Base5 CURRENT](BASE5_V2_CURRENT.md)를 따른다.

## 공식 출제 범위와 제외

- 대회 F2의 전체 객체 에셋이 GPS 음영·정적/동적 장애물 후보가 될 수 있다. 전체 균등 출제라는 뜻은 아니다.
- 기존 정적 장애물 자산이 움직이는 예외는 없다. CargoBox 등 rigid static에 이동 속도를 넣은 진단을 학습 분포로 쓰지 않는다.
- `OBJ_G80_Open_Door`(40100034)는 공식 출제 제외다. 문 닫힌 일반 G80까지 제외하지 않는다.
- 고주로의 정차 차량·정적 장애물은 배치 예정이 없다는 답변을 받았다. 터널은 별도 분포다.
- 공통 Z/waypoint 설정 문제와 에셋별 pivot·경로 지면을 구분한다. 다른 모든 자산의 동일 버그가 확정된 것은 아니다.

공식 근거는 [MORAI 답변](legacy/morai_reference/260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준)을 따른다.
실제 launcher 옵션 삭제나 모든 지면 조건의 성공을 이 답변만으로 인증하지 않는다.

## 현재 source와 의미

- [Base5 MR catalog](../configs/morai_asset_catalog_mr_base5_261005.json)의 eligibility·제외 이유를 사용한다.
- [Appearance 정책](../configs/base5_asset_appearance_policy_v1.json)은 원래 정적/이동 역할을 구분한다.
- [정지 배치 형상](../configs/base5_stationary_geometry_mr_261006.json)은 spacing용 envelope다. native GT를 대체하지 않는다.
- [ObjectStatus BBox catalog](../configs/morai_asset_bbox_offsets_status_size_v2.json)는 bag 평가용이며 native annotation과 다르다.
- MR에 MRP 교집합·quarantine을 적용하지 않는다. 과거 catalog는 재현 source로 보존한다.
- 일반 표지판·가로등·나무·효과 등 배경/ignore 정책은 우리 supervision 정책이며 공식 출제 금지와 다르다.

원본 animal subtype과 복합 parent/component 관계를 유지한다.
현재 학습의 Animal→Pedestrian 매핑은 [모델 CURRENT](../../../docs/bevfusion/CURRENT.md#model)의 파생 정책이다.
과거 Animal→Obstacle 후보나 4클래스 pilot을 자동 재사용하지 않는다.
OBJ 차량은 Vehicle이며 같은 부모 UID의 여러 native GT를 하나로 덮어쓰지 않는다.

카탈로그에 포함됐다는 것과 모든 pose·지면·bag pivot 보정이 검증됐다는 것은 다르다.
선택한 release의 실제 QA·unknown asset 처리·보정 provenance를 확인한다.
부족한 보정을 추측해 라벨을 확정하거나 unknown 자산을 몰래 background로 바꾸지 않는다.

## 과거 검수

MR/MRP 목록 비교·복구 네 종·초기 pool 수량·보정 실험은
[개편 전 원문](../../../docs/perception/legacy/context_261012/README.md)과 해당 날짜의 EVIDENCE에서만 재현한다.
현재 pool 크기와 122종 coverage는 실제 release/adopted manifest로 확인한다.
