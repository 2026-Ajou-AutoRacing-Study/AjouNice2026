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

## 7. 자동검사 전체 수치 (최종 판정 34건)

| 시나리오 | GT 판정 | GT 시간 | GT 최소 목표거리 | GT 충돌 | BEVFusion 판정 | BEV 시간 | BEV 최소 목표거리 | BEV 충돌 | 비교 |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| `pos01` | PASS | 17.26초 | 0.417 m | 0 | PASS | 17.90초 | 0.247 m | 0 | 공통 PASS |
| `pos02` | PASS | 16.24초 | 0.106 m | 0 | PASS | 18.08초 | 0.197 m | 0 | 공통 PASS |
| `pos03` | PASS | 16.24초 | 0.129 m | 0 | PASS | 16.47초 | 0.118 m | 0 | 공통 PASS |
| `pos04` | TIMEOUT | 90.01초 | 32.204 m | 0 | PASS | 19.27초 | 0.258 m | 0 | BEVFusion PASS |
| `pos05` | PASS | 16.05초 | 0.128 m | 0 | PASS | 17.27초 | 0.229 m | 0 | 공통 PASS |
| `pos06` | PASS | 16.25초 | 0.165 m | 0 | PASS | 16.47초 | 0.130 m | 0 | 공통 PASS |
| `pos07` | TIMEOUT | 90.03초 | 27.974 m | 0 | PASS | 18.70초 | 0.256 m | 0 | BEVFusion PASS |
| `pos08` | PASS | 16.26초 | 0.262 m | 0 | PASS | 19.68초 | 0.270 m | 0 | 공통 PASS |
| `pos09` | PASS | 16.25초 | 0.131 m | 0 | PASS | 16.67초 | 0.207 m | 0 | 공통 PASS |
| `pos10` | PASS | 17.27초 | 0.112 m | 0 | PASS | 18.47초 | 0.248 m | 0 | 공통 PASS |
| `pos11` | PASS | 16.66초 | 0.269 m | 0 | PASS | 20.71초 | 0.266 m | 0 | 공통 PASS |
| `pos12` | PASS | 16.27초 | 0.123 m | 0 | PASS | 16.70초 | 0.256 m | 0 | 공통 PASS |
| `pos13` | TIMEOUT | 90.09초 | 38.654 m | 0 | TIMEOUT | 90.20초 | 39.627 m | 0 | 공통 TIMEOUT |
| `pos14` | PASS | 16.66초 | 0.218 m | 0 | PASS | 18.29초 | 0.251 m | 0 | 공통 PASS |
| `pos15` | PASS | 18.06초 | 0.105 m | 0 | COLLISION | 11.26초 | 20.814 m | 1 | BEVFusion 충돌 |
| `pos16` | PASS | 16.27초 | 0.240 m | 0 | PASS | 16.91초 | 0.238 m | 0 | 공통 PASS |
| `car01` | TIMEOUT | 90.13초 | 41.291 m | 0 | PASS | 18.91초 | 0.272 m | 0 | BEVFusion PASS |

- GT 합계: 17건 / **13 PASS, 4 TIMEOUT**, 실행시간 합계 215.74초, 충돌 0건
- BEVFusion 합계: 17건 / **15 PASS, 1 TIMEOUT, 1 COLLISION**, 실행시간 합계 281.76초, 충돌 1건
- 최종 판정 기준 시스템 오류: 0건

## 8. 실패 원인 수치

| 모드 | 시나리오 | 실패 유형 | 관측 내용 |
| --- | --- | --- | --- |
| GT | `pos04` | TIMEOUT | 81.59 m 주행 뒤 정지, 최대 속도 14.32 m/s, 남은 목표거리 32.204 m |
| GT | `pos07` | TIMEOUT | 85.69 m 주행 뒤 정지, 최대 속도 14.06 m/s, 남은 목표거리 27.974 m |
| GT | `pos13` | TIMEOUT | 75.29 m 주행 뒤 정지, 최대 속도 13.81 m/s, 남은 목표거리 38.654 m |
| GT | `car01` | TIMEOUT | 72.72 m 주행 뒤 정지, 최대 속도 13.48 m/s, 남은 목표거리 41.291 m |
| BEVFusion | `pos13` | TIMEOUT | 74.34 m 주행 뒤 정지, 최대 속도 13.87 m/s, 궤적 앞 5점 최대 속도 7.04 m/s, 남은 목표거리 39.627 m |
| BEVFusion | `pos15` | COLLISION | `/CollisionData` 충돌 1건, 11.26초에 종료, 목표까지 20.814 m 남음 |

> `car01`의 GT 첫 시도에는 일시 오류 기록이 남아 있으나 자동 재시도 후 TIMEOUT으로 최종 판정됐습니다. 표에는 최종 시도 결과만 반영했습니다.

## 9. 결과 파일 및 영상

- GT CSV: `planner_logs/validation/user_static_obstacle_17_gt_261004_165741/summary.csv`
- BEVFusion CSV: `planner_logs/validation/user_static_obstacle_17_bev_261004_172418/summary.csv`
- 케이스별 기록: 각 `success/`, `failure/`, `error/` 폴더의 `planbag_*.json` 및 `planbag_*.mp4`
- `planbag`은 검증기 실행 기록(JSON/MP4)이며, ROS 원본 `.bag`은 별도 `rosbag record` 실행 시에만 생성됩니다.
