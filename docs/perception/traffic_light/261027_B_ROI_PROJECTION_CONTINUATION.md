# 2026-10-27 B 신호 ROI 불안정성: 실측과 현재 영상 재판독 후보

상태: **로컬 소스·설치본 일치 확인, 오프라인 재생과 로컬 폐루프 주행 검증**. 팀 PC 설치본과 LAN 풀스택에서의 효과는 아직 확인하지 않았다. 이 기록은 이전 [연속 완충 시험](261026_SIGNAL_JITTER_RUNTIME_REMEDIATION.md)을 대체하지 않는다.

이후 지도 허용 범위를3°·90px에서4°·120px로 완화했다. 현재값과 추가 replay·주행 결과는 [선택 허용 범위 대조](261027_SELECTION_TOLERANCE_COMPARISON.md)를 따른다. 아래3° 경계와 수치는 당시 대조 실험의 기록이다.

## 받은 bag에서 확인한 것

팀장님 bag `rosbags/traffic_light/261027_teamlead_011017/planning_rviz_20261027_011017.bag`의 `/map/vector_map`은 `version_map=88`이다. B 접근 중 같은 정지 목표가 약 69.159·69.741·71.046초에 다시 들어갔다가 빠졌고, 차속은 약 16.16→0.42m/s로 떨어졌다. 그 시각의 원본 전방 영상에는 B 후보 신호의 녹색 점등이 보인다. 반면 `/perception/traffic_light_recognition/{map_rois,traffic_signals}`와 `/traffic_light_node/debug_{selection,classifier,runtime}`가 녹화되지 않아 **실제 팀 PC에서 어느 단계가 실패했는지는 특정할 수 없다**. `Traffic map ROI missing` 일곱 건은 약 0초와 125~150초의 `route_missing_or_stale`이며 B의 약 64~74초와 겹치지 않는다. 상세 시각·원본 해시는 로컬 `planner_logs/analysis/261027_teamlead_011017_B_actual/REPORT.md`에 보존했다.

## 후보와 대조 결과

현재 소스·v5 가중치·bag의 v88 지도를 이용해 **같은 원본 JPEG/측위를 오프라인 재생**했다. 기존 코드에서 B의 활성 미션 91콜백 중 판독 31·UNKNOWN 60이었다. 녹색 유지 구간의 간헐적 UNKNOWN 7프레임은 yaw 불확실성 표준편차 약 3.26~3.48°가 기존 3° 상한을 넘은 경우였다. 현재 영상 시각의 nominal 지도 투영과 높은 confidence의 동일 YOLO housing은 남아 있었지만, 이동 중 이전 프레임 박스와의 IoU가 낮아 기존 0.3초 visual continuation도 거부했다.

추가한 보완은 이 **측위 불확실성 상한 초과 사유에만** 이전 정상 연결의 동일 미션/head를 후보로 삼는다. 현재 영상 시각 TF로 만든 좁은 nominal 투영과 현재 YOLO 박스의 중심·크기가 일치하고, 기존 route·원본 age·실제 화각·다른 지도 head 경쟁 검사를 모두 통과해야 한다. 중심 허용 폭은 투영된 housing 크기에 비례하며 최대 8px이다. 색은 저장한 녹색을 재사용하지 않고 **현재 crop을 CNN으로 다시 판독**한다. 따라서 아무 ROI 실패나 오래된 GO를 시간 창으로 덮는 기능이 아니다. `visual_continuation_sec:=0`으로 기존 보완 전체를 끌 수 있다.

패치 후 동일 v88 재생은 B 판독 31→38, UNKNOWN 60→53으로 바뀌었다. 새 7개는 모두 같은 B head의 현재 crop에서 GREEN으로 판독됐고, 나머지 93개 창 프레임의 최종 tuple/head는 변하지 않았다. 별도 v94 A/B/C/D 재생에서는 전체 409개 창 프레임의 최종 tuple/head 변경이 0개였다. 단위·노드 시험에서는 현재 crop이 RED/YELLOW로 바뀌면 과거 GREEN을 유지하지 않고 현재 색을 출력하며, 0.3초 anchor 만료 뒤에는 UNKNOWN이 된다. 이는 좁은 표본에서의 비회귀 근거이며 실제 오검출률·안전성 상한은 아니다. 원본 JSON은 로컬 `planner_logs/analysis/261027_teamlead_011017_B_counterfactual_agent/infer_v88_current_v5/replay.json`, `planner_logs/analysis/261027_teamlead_011017_B_counterfactual_agent/infer_v88_nominal_sizecap_v2/replay.json` 및 `planner_logs/analysis/261026_v94_latest_bag/replay_infer_nominal_sizecap_v2/replay.json`에 있다. 로컬 `install_traffic_light_candidate.sh --check`는 모델·C++ 플러그인·설치 Python과 현 소스의 일치를 확인했다.

## 추가 로컬 폐루프 및 다음 lanelet 가설 (2026-10-27)

현재 v88 지도·자연 신호·v5 인지·실제 ROS1 planner/controller, 로컬 `od_gt` 조건에서 B 근접 출발과 무객체 풀경로를 **새 실행**으로 각각 1회 돌렸다. 이전 실행 snapshot은 재사용하지 않았다. B 근접은 103.22초, 풀경로는 208.96초에 목적지 도착·충돌 0·종료 P 확인으로 PASS했다. 근거 bag은 각각 `planner_logs/validation/traffic_light_compliance_B_261027_132329/success/`와 `planner_logs/validation/traffic_light_compliance_full_regression_261027_132636/success/`에 있으며 둘 다 신호 선택·분류·route ID와 planner 궤적을 녹화했다. 이는 팀 PC의 LAN·BEVFusion 동시 부하나 공식 채점 인증이 아니다.

근접 주행에서는 B 활성 선택 573/593프레임이 동일 B head `C1256W000091`에 맞았다. visual continuation 51회는 모두 이전 박스와 현재 박스의 동일성 검사로 작동했다. 대부분 적색 정차 중이었고 새 `current_nominal_projection` 분기는 0회였다. 풀경로에서는 B 활성 39/74프레임이 모두 같은 head에 연결됐다. 약 16m/s 접근 중 1프레임 `GO→UNKNOWN→GO`가 있었고, 실제 B 정지선 통과 전후 약 1초도 UNKNOWN이었다. 이 실행에서는 B visual continuation 자체가 0회였으므로, **새 분기의 실제 주행 이득은 아직 입증되지 않았다**. 풀경로 B 통과 시 개발용 native 전후 상태는 녹색이었지만 공식 패널티 판정과 같지 않다.

단발 UNKNOWN은 지도/YOLO 연결 실패가 아니라, 현재 입력의 정상 B crop에 대한 CNN `no_lamps` 판독이었다. 앞뒤 입력 프레임의 분류 결과는 GREEN이었다. 기존 저확신·detector 누락 완충은 의도적으로 `no_lamps`를 덮지 않는다. 통과 직전의 연속 UNKNOWN은 처음 `housing_not_fully_in_image`, 이후 `depth_out_of_bounds`여서 실제 화각·깊이 검사를 우회하지 않는 한 지도 ID를 알아도 crop할 수 없다. 풀경로의 16→8m/s 제동은 후기 UNKNOWN보다 약 2.2초 먼저 시작했고, 해당 시점 궤적에는 B 정지점 재삽입이 없고 후속 경로 약 8m/s 감속이 있었다. 따라서 이 제동을 B 미판독 탓으로 돌릴 근거는 없다. 반대로 한 번의 로컬 주행만으로 팀장님 bag의 녹색 중 재제동이 완치됐다고 말할 수도 없다. 분석 JSON은 `planner_logs/analysis/261027_B_fallback_full_route_hesitation.json`과 `artifacts/evaluation/traffic_light_robustness_261025/B_fallback_full_route_261027_v1/traffic_bag_summary.json`에 있다.

`lanelet_route_matcher`는 이미 경로상 **아직 통과하지 않은 다음 미션 신호 ID**를 선택한다. 팀 B 재생의 활성 91콜백 모두 B ID `201365`가 맞았다. 패치 후 남은 UNKNOWN 53프레임 중 34개는 목표 등기구가 너무 작거나 화각 밖/통과 뒤, 16개는 박스·crop/판독 부재, 3개는 불확실성 초과와 YOLO 박스 부재였다. 멀리서 목표 B 대신 **다른 가까운 등기구가 검출된 프레임**도 있으므로, 다음 lanelet ID만 믿고 임의 YOLO 박스를 B로 지정하는 fallback은 안전하지 않고 이번 문제를 해결하지도 않는다. 이 규칙은 **추가하지 않는다**. 근접한 2프레임의 부분 crop 후보나 `no_lamps` 한 프레임 완충도 현재 자료로는 적색 전환·진짜 소등에서의 오허가 위험을 배제하지 못하므로 보류한다.

## 남은 검증

1. 팀 PC에서 source와 설치본의 traffic node/helper를 함께 갱신·재시작하고 실제 resolved 인자·지도 버전을 기록한다. 과거 bag의 실행 commit/SHA는 알 수 없다.
2. 팀 PC에서 B를 다시 통과하면서 전방 JPEG, route group, map ROI, 최종 traffic signal, selection/classifier/runtime·원시/관측 라벨 디버그, behavior path를 같은 bag에 저장한다. 로컬 새 bag은 디버그·궤적을 확인했으나 팀 PC의 고속 접근 재현은 별도다.
3. 신호가 녹색인 동안 UNKNOWN·정지 목표 재삽입이 사라지는지, 반대로 실제 적색/황색·TF 상실·다른 head·화각 이탈에서 부당한 GO가 생기지 않는지 확인한다. 팀 PC 풀스택 지연과 공식 LAN 채점은 이 재생으로 인증할 수 없다.
