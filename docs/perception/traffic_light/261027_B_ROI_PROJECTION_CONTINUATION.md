# 2026-10-27 B 신호 ROI 불안정성: 실측과 현재 영상 재판독 후보

상태: **로컬 소스·설치본 일치 확인, 오프라인 재생 검증**. 팀 PC 설치본과 새 폐루프 주행에서의 효과는 아직 확인하지 않았다. 이 기록은 이전 [연속 완충 시험](261026_SIGNAL_JITTER_RUNTIME_REMEDIATION.md)을 대체하지 않는다.

## 받은 bag에서 확인한 것

팀장님 bag `rosbags/traffic_light/261027_teamlead_011017/planning_rviz_20261027_011017.bag`의 `/map/vector_map`은 `version_map=88`이다. B 접근 중 같은 정지 목표가 약 69.159·69.741·71.046초에 다시 들어갔다가 빠졌고, 차속은 약 16.16→0.42m/s로 떨어졌다. 그 시각의 원본 전방 영상에는 B 후보 신호의 녹색 점등이 보인다. 반면 `/perception/traffic_light_recognition/{map_rois,traffic_signals}`와 `/traffic_light_node/debug_{selection,classifier,runtime}`가 녹화되지 않아 **실제 팀 PC에서 어느 단계가 실패했는지는 특정할 수 없다**. `Traffic map ROI missing` 일곱 건은 약 0초와 125~150초의 `route_missing_or_stale`이며 B의 약 64~74초와 겹치지 않는다. 상세 시각·원본 해시는 로컬 `planner_logs/analysis/261027_teamlead_011017_B_actual/REPORT.md`에 보존했다.

## 후보와 대조 결과

현재 소스·v5 가중치·bag의 v88 지도를 이용해 **같은 원본 JPEG/측위를 오프라인 재생**했다. 기존 코드에서 B의 활성 미션 91콜백 중 판독 31·UNKNOWN 60이었다. 녹색 유지 구간의 간헐적 UNKNOWN 7프레임은 yaw 불확실성 표준편차 약 3.26~3.48°가 기존 3° 상한을 넘은 경우였다. 현재 영상 시각의 nominal 지도 투영과 높은 confidence의 동일 YOLO housing은 남아 있었지만, 이동 중 이전 프레임 박스와의 IoU가 낮아 기존 0.3초 visual continuation도 거부했다.

추가한 보완은 이 **측위 불확실성 상한 초과 사유에만** 이전 정상 연결의 동일 미션/head를 후보로 삼는다. 현재 영상 시각 TF로 만든 좁은 nominal 투영과 현재 YOLO 박스의 중심·크기가 일치하고, 기존 route·원본 age·실제 화각·다른 지도 head 경쟁 검사를 모두 통과해야 한다. 중심 허용 폭은 투영된 housing 크기에 비례하며 최대 8px이다. 색은 저장한 녹색을 재사용하지 않고 **현재 crop을 CNN으로 다시 판독**한다. 따라서 아무 ROI 실패나 오래된 GO를 시간 창으로 덮는 기능이 아니다. `visual_continuation_sec:=0`으로 기존 보완 전체를 끌 수 있다.

패치 후 동일 v88 재생은 B 판독 31→38, UNKNOWN 60→53으로 바뀌었다. 새 7개는 모두 같은 B head의 현재 crop에서 GREEN으로 판독됐고, 나머지 93개 창 프레임의 최종 tuple/head는 변하지 않았다. 별도 v94 A/B/C/D 재생에서는 전체 409개 창 프레임의 최종 tuple/head 변경이 0개였다. 단위·노드 시험에서는 현재 crop이 RED/YELLOW로 바뀌면 과거 GREEN을 유지하지 않고 현재 색을 출력하며, 0.3초 anchor 만료 뒤에는 UNKNOWN이 된다. 이는 좁은 표본에서의 비회귀 근거이며 실제 오검출률·안전성 상한은 아니다. 원본 JSON은 로컬 `planner_logs/analysis/261027_teamlead_011017_B_counterfactual_agent/infer_v88_current_v5/replay.json`, `planner_logs/analysis/261027_teamlead_011017_B_counterfactual_agent/infer_v88_nominal_sizecap_v2/replay.json` 및 `planner_logs/analysis/261026_v94_latest_bag/replay_infer_nominal_sizecap_v2/replay.json`에 있다. 로컬 `install_traffic_light_candidate.sh --check`는 모델·C++ 플러그인·설치 Python과 현 소스의 일치를 확인했다.

## 남은 검증

1. 팀 PC에서 source와 설치본의 traffic node/helper를 함께 갱신·재시작하고 실제 resolved 인자·지도 버전을 기록한다. 과거 bag의 실행 commit/SHA는 알 수 없다.
2. B를 다시 통과하면서 전방 JPEG, route group, map ROI, 최종 traffic signal, selection/classifier/runtime·원시/관측 라벨 디버그, behavior path를 같은 bag에 저장한다. 녹화기가 핵심 토픽들의 존재를 검사하도록 보완했지만 새 bag 검사는 별도다.
3. 신호가 녹색인 동안 UNKNOWN·정지 목표 재삽입이 사라지는지, 반대로 실제 적색/황색·TF 상실·다른 head·화각 이탈에서 부당한 GO가 생기지 않는지 확인한다. 팀 PC 풀스택 지연과 공식 LAN 채점은 이 재생으로 인증할 수 없다.
