# 정적 장애물 시나리오 자동 검증 결과

- 작성일: 2026-10-04
- 대상: K-City 정적 장애물 시나리오 17종
- 검증 구성: MORAI 시뮬레이터 + 플래닝 검증기

## 1. 시나리오 구성

- 공통 시작점: `static-startline`
  - 위치: `(-78.673, -227.793, 28.734)`
  - MGeo 시작 링크: `A2256W000213` (ratio `0.037013`)
- 장애물 위치 시나리오: `pos01` ~ `pos16` (16종)
- 차량 장애물 시나리오: `car01` (1종, 마지막 순서)
- 총 17개 시나리오를 순차 실행하도록 자동화 구성 완료

## 2. 자동검증 결과 요약

| 검증 모드 | PASS | FAIL | 시스템 오류 | 실패 시나리오 |
| --- | ---: | ---: | ---: | --- |
| GT 기반 | 13 | 4 | 0 | `pos04`, `pos07`, `pos13`, `car01` (모두 TIMEOUT) |
| BEVFusion 기반 | 15 | 2 | 0 | `pos13` (TIMEOUT), `pos15` (COLLISION) |

> PASS 기준: 목적지 도착 및 충돌 없음

## 3. 모드별 상세 결과

### GT 기반 검증

- PASS: `pos01`, `pos02`, `pos03`, `pos05`, `pos06`, `pos08`, `pos09`, `pos10`, `pos11`, `pos12`, `pos14`, `pos15`, `pos16`
- FAIL (TIMEOUT): `pos04`, `pos07`, `pos13`, `car01`
- 실패 공통 양상: 주행 중 목적지 이전에서 정지했으며, 충돌은 관측되지 않음

### BEVFusion 기반 검증

- PASS: `pos01` ~ `pos12`, `pos14`, `pos16`, `car01`
- FAIL (TIMEOUT): `pos13`
- FAIL (COLLISION): `pos15`
- 시스템 오류 없이 17개 전체 시나리오 실행 완료

## 4. GT / BEVFusion 비교

- 두 모드 모두 실패: `pos13` (목적지 이전 정지에 따른 TIMEOUT)
- GT에서는 TIMEOUT, BEVFusion에서는 PASS: `pos04`, `pos07`, `car01`
- GT에서는 PASS, BEVFusion에서는 COLLISION: `pos15`
- 나머지 12개 위치 시나리오는 두 모드 모두 PASS

## 5. 산출물 위치

### 시나리오 및 실행 설정

- 생성 스크립트: `base_ws/src/planning_addons/planning_scenario_validator/scripts/generate_user_static_obstacle_campaign.py`
- GT 캠페인 설정: `base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/workspace/user_static_obstacle_17_gt.yaml`
- BEVFusion 캠페인 설정: `base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/workspace/user_static_obstacle_17_bev.yaml`
- 생성 시나리오: `base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/workspace/scenarios/user_static_01_pos01.json` ~ `user_static_17_car01.json`

### 최종 검증 로그

- GT: `planner_logs/validation/user_static_obstacle_17_gt_261004_165741/summary.csv`
- BEVFusion: `planner_logs/validation/user_static_obstacle_17_bev_261004_172418/summary.csv`

## 6. 후속 확인 항목

- [ ] `pos13`: 두 모드 공통 TIMEOUT 원인 분석 (경로·속도·정지 조건 확인)
- [ ] `pos15`: BEVFusion 입력에서 발생한 충돌 객체와 회피 경로 확인
- [ ] `pos04`, `pos07`, `car01`: GT/BEVFusion 결과 차이의 인지·플래닝 입력 비교
- [ ] 수정 후 동일한 17개 캠페인을 재실행하여 결과 갱신
