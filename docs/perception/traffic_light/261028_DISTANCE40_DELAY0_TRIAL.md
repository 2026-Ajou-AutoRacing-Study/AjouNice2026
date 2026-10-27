# 40m 판단 시작 · 응답지연 0초 단일 주행

상태: EVIDENCE / 2026-10-28. 요청한 짧은 로컬 후보 실험이며 PR·운영 승격은 하지 않았다.

## 설정과 의미

- 실제 조회 환경 Cloudy13, 강제 신호 제어 없음. Sunny13으로 간주하지 않는다.
- 실제 ROS1 planning/controller + localization + od_gt, BEVFusion 동시 부하 시험 아님.
- planning/Map-CV/RViz/traffic는 명시적 v97 지도. 원격 v98 통합 시험이 아니다.
- 공통 `delay_response_time=0.0`을 실행 snapshot에서 확인했다. `system_delay=0.5`는 별개이며 변경하지 않았다.
- 후보 `observation_start_depth_m=40.0`: **카메라→신호 housing의 전방 깊이**다. 정지선까지 40m가 아니다.
- 같은 미션의 현재 이미지·현재 지도 연결이 40m 안에서 처음 성립하면 색 전달을 시작하고,
  이후 거리가 커지거나 누락돼도 게이트는 닫지 않는다. 기존 UNKNOWN/no_lamps 처리는 유지한다.
- 게이트 밖의 색은 phase/hold 이력에도 넣지 않는다. 관측 전에는 기존 unseen defer로 경로를 연다.
- 황색 예외는 이번 시험에만 ON. 첫 관측 황색·정차 중 황색을 새 출발 허가로 쓰지 않는다.
- 운영 launch의 거리 게이트는 **0(비활성)** 그대로다. 같은 미션 내 teleport 초기화는 이 후보에 미구현이다.

## 결과

| 미션 | 색 사용을 시작한 실제 깊이 | 첫 사용 색 | 원본 MGeo선 통과 native 상태 |
|---|---:|---|---|
| A | 38.40m | R | G(16) |
| B | 39.91m | G | G(16) |
| C | 38.78m | R | 양쪽 G+좌화살표(48) |
| D | 31.37m | Y | R+좌화살표(33) |

40m 이내 첫 처리 프레임부터 열기 때문에 정확히 40.0m에서 열리지 않는다.
D는 31.37m에서 처음 연결됐다. 따라서 40m 설정이 40m부터 관측을 보장하는 것은 아니다.

- 정상 목표 도착 PASS, 종료 실제 정지·P VERIFIED, wrapper exit0 / supervisor NORMAL.
- A의 raw 황색 3프레임은 깊이 72.44/74.94/64.67m에서 발생했고 모두 최종 판단에서 제외됐다.
  A 최종 색열은 UNKNOWN→R→G→UNKNOWN, B는 UNKNOWN→G→UNKNOWN,
  C는 UNKNOWN→R→G+좌화살표였다. 먼 거리 오판을 차단하는 효과는 이번 기록에서 확인했다.
- D는 UNKNOWN→Y→G→Y→R+좌화살표→UNKNOWN이며, G 단독도 좌회전 허가는 아니다.
  D 통과 말단 19개 planner 표본에서 전체 ZERO 재삽입은 0회였다.
- 원본선 앞뒤 0.25초 native 조회 대조에서 네 미션 모두 방향 허가를 확인했다.
  이는 개발용 준수 proxy이며 공식 패널티·픽셀 정확도 100% 인증은 아니다.
- 주행 창 카메라18.79Hz / 신호8.36Hz / planning9.99Hz / 제어50.00Hz.
  신호 source→bag age 평균33.4ms, p95 50.8ms; planning157.0/225.7ms.
  bag age는 actuator 지연이나 GPU 단독 시간과 다르다.

## 한계와 재현 근거

한 번의 자연 신호 주행이다. 안정적인 이동 G→Y의 황색 STOP/GO 양쪽 분기,
6환경·LAN·BEVFusion 풀스택, 정지 위치 최적성, 모든 과도 감속 부재를 입증하지 않는다.
기존 영상과 환경/신호 위상이 달라 인과 A/B 성능 개선량으로 계산하지 않는다.
40m는 신호까지 거리라 실제 정지선까지 거리는 더 짧을 수 있다. 모든 속도에서 안전한 기준으로 승격하지 않는다.

- 실행/영상: `artifacts/evaluation/traffic_distance40_delay0_261028/run/`.
- 종료/준수/주행 감사: 같은 상위의 `driving_audit.json`, `run/observer/crossings.json`,
  `run/agent_closed_window_audit_v1.json`, `supervisor/summary.json`.
- 실제 캠페인: `planner_logs/validation/distance40_delay0_261028_261028_033403`.
- 게이트·실제 callback·영상·wrapper 관련 테스트 75개 통과. source/install runtime audit PASS.
  별도 installer mock suite의 11개 실패는 fixture에 없는 `perception_object_pipeline/CMakeLists.txt`
  참조 때문이며 이번 수정 파일은 아니다. 전체 저장소 테스트 통과로 표현하지 않는다.
