# v61 lanelet 200392 경계 손실 수정 및 검증

## 판정

2026-09-12: ROS1 BPP 동적 경계 확장의 자기교차 제거 로직을 ROS2 원본 의미로 복구했다.
지도 수정이나 PO 가중치 강화 없이, 관측한 200392 좌회전 계획에서 지도 경계 보존과 차체의 차로 영역 포함을 확인했다.
모든 주행 조건 및 폐루프 추종 성공을 의미하지 않는다.

## 결함과 수정

- 원본: Autoware2ros1/autoware_original/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/drivable_area_expansion/drivable_area_expansion.cpp, expand_bound.
- 대상: planning_ws/src/autoware_behavior_path_planner_common/src/utils/drivable_area_expansion/dynamic_area_core.cpp.
- 기존 ROS1 부호 비교 교차 판정은 정상 이웃 선분의 공통 끝점을 자기교차로 취급할 수 있었다.
- 원본의 교차점 계산, 양 끝점에서 거리 1e-3 초과 조건, 루프 순회 의미를 복구했다.
- 기존 실제 자기교차 테스트의 기대값 2도 원본 알고리즘을 따라 1로 수정했다. 단순히 교차 삭제 기능을 끄지 않았다.
- 양방향 정상 곡선 보존 및 v61 way 100323 좌표의 반복 처리 보존 회귀테스트 추가.
- v61 지도는 변경하지 않았다. 200392에는 left=100323, right=100324가 있고 intersection_area 연결은 없다.

## 빌드/테스트

planning_ws/tools/build.sh --pkg autoware_behavior_path_planner_common 성공.
test_path_shifter 전체 74/74 통과, 이 중 DynamicAreaCore 16/16 통과.
git diff --check 통과. 실행 BPP가 install_isolated의 수정 공유 라이브러리를 로드한 것을 /proc/PID/maps로 확인.

## 실제 ROS 그래프 관측

| 항목 | 수정 전 | 수정 후 유효 샘플 |
|---|---:|---:|
| 지도 왼쪽 경계점에서 BPP 왼쪽 경계까지 최대 최근접 거리 | 6.4559 m | 0.0000793 m |
| 지도 오른쪽 경계점에서 BPP 오른쪽 경계까지 최대 최근접 거리 | 5.9308 m | 0.0000789 m |
| PO 국소 궤적에서 smoother 경로까지 최대 최근접 거리 | 4.7354 m | 0.2794 m / 후속 0.1817 m |
| 후속 샘플의 차체 검사 | 미측정 | 47개 자세, 영역 침범 0개 |

BPP와 smoother의 좌우 경계는 동일했다. PO skip=false 유지.
수정 전 실행 가중치는 lat=10, avoidance.lat=10이었고 수정 후 재시작은 원복된 1,0을 로드했다.
따라서 위 결과는 동일 가중치/동일 위치 A/B 실험이 아니다. 수정 후 차량은 사용자가 공식 출발점으로 이동시켰다.
수정 전 ego=(31.5972,174.4333), 첫 수정 후 ego=(0.0685,118.0094).

차체 검사는 PO 자세에서 앞 3.845m, 뒤 0.79m, 반폭 0.945m 직사각형을 생성했다.
진입 200889 + 좌회전 200392 + 탈출 200178의 지도 폴리곤 합집합을 기준으로 하며 접합/투영 허용 0.02m를 적용했다.
국소 샘플 범위는 map x=(25,42), y=(180,200); 해당 샘플 47개 모두 차량 박스 침범 면적 0이었다.
단일 200392 폴리곤만 검사하면 진입부 점이 외부로 집계되므로, 차체 검증에는 연결 차로 합집합을 사용했다.
이는 이산 궤적 자세 검사이며 연속 swept footprint 검사 또는 실제 제어 추종 검증이 아니다.
샘플은 순차 수신(최대 약 0.2초 차이)이며 완전한 동시각 동기화는 아니다.

추가 샘플에서는 차량이 출발점 뒤로 이동하여 해당 코너 점이 0개였다. 이 샘플은 통과 증거로 사용하지 않았다.

## 현재 실행과 남은 검증

수정된 competition_planning.launch를 실행 중이다. 기존 플래닝 묶음은 BPP 종료 때 함께 종료되어 다시 실행했다.
인지/측위/제어 소스와 지도는 변경하지 않았다. 출발점 제한은 우회하거나 수정하지 않았다.
별도 ROS master 시험 준비는 실제 그래프가 복구된 후 종료했으며 별도 master에서 궤적 검증을 수행한 것으로 집계하지 않는다.

남은 범위: 같은 위치/입력/가중치 고정 재생 비교, 여러 접근 위치의 최적화, 연속 차체 검사, 실제 좌회전 폐루프 주행.
현재 증거는 경계 훼손 결함 수정 및 해당 좌회전의 계획 출력 개선을 지지한다.
