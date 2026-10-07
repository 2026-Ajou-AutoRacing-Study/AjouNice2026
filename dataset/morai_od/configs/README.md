# Generation configs

## 현재 Base5 기본 환경

v112의 `mission_dynamic_observation.vehicle_primary_scenes=12`는 동적 미션250셀의4.8%다.
길이6m 이하12차종,도로 link 기반 정지 외관 캡처이며 차량 양보 관측만8–20m다.
일반 사람12슬롯만 교체하고 희귀 복합/동물 quota를 유지한다. 구manifest/resume에 새코드를 섞지 않는다.

v111의 `stationary_mission_assets`가 정적 회피 슬롯의 자산 범위를 정한다.
기존600 obstacle 미션 슬롯에9개 pool을 할당하고, other_vehicle50 슬롯은 정적 차량으로 유지한다.
모든122종이 정적 미션에 들어가며 속도0/이동경로없음이다. family를 GT 클래스로 해석하지 않는다.

2026-10-07: Sunny13시가 현재 기본값이다.
`base5_independent_5k_sunny13.json`을 명시적 config로 사용한다.
`base5_independent_5k_sunny11.json`은 기존 generator/resume 경로 호환을 위해 이름만
남긴 동일 내용의 alias이며 hour도13이다. 실제 환경은 MR에서 확인하고,
기존 수집의 frozen manifest를 in-place로 바꾸지 않는다.
[정정 근거·기존 산출물 처리](../docs/261007_base5_sunny13_environment_correction.md).

## 기존 설정 이력

대량 생성기의 anchor 선택, 객체 수·거리, 날씨·시간 조합, seed와 split 정책을
YAML 설정으로 관리할 디렉터리다. 현재 프로토타입은 명령행 인자를 사용한다.

`scenario_runner_asset_catalog.yaml`은 설치된 Scenario Runner v1.7.0의 XOSC
예제에서 실제로 관찰한 MORAI 모델명을 정리한 참고 카탈로그다. MORAI 전체
Asset 목록이 아니므로 생성 전 실제 시뮬레이터에서 로드 검증해야 한다.

`morai_asset_catalog_260807.json`은 MORAI Scenario Edit에서 확인한
차량·보행자 관련·curated 장애물 109개 고유 DataID와 표시명, 1차 ontology,
생성·학습 정책을 담는다. 원본 시나리오와 `optionList.byte`의 SHA-256을
기록하며 `scripts/extract_morai_asset_catalog.py`로 재생성한다.

`od_180_pilot.yaml`은 180-run PointPainting/향후 BEV 공용 파일럿의
anchor, 환경, split, 객체 수·거리, visibility와 충돌 제약 초안이다.
위 실제 카탈로그를 참조한다. 사람 결합형 obstacle은 제외하며,
`pedestrianList`의 자전거 탑승자·사람+동물 복합 모델과 자산별 footprint는
label audit 후 확정한다.

`fastbev_sunny13_1500_v1.json`은 완료된 Sunny 13시 286 scene에서 250 scene,
1,500 frame을 재현 가능하게 고르는 Fast-BEV baseline 설정이다. source capture,
Sensor_6 calibration, 고정 seed, 175/38/37 scene split과 coarse class를 정의한다.
