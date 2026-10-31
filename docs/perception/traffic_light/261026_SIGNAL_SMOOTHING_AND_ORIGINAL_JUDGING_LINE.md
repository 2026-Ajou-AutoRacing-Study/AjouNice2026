# 짧은 판독 실패 완충과 원본 MORAI 판정선

2026-10-26. 이전 [UNKNOWN 정책 검토](261026_UNKNOWN_HOLD_ROUTE_REVIEW.md) 후 사용자가
움찔 방지 구현·원본 정지선 기준 황색 계산·새 팀장님 bag 분석을 요청했다.
현재 기본은 [CURRENT](CURRENT.md), 새 bag의 일회성 분석은 `planner_logs/analysis/261026_154234_signal_hesitation/`에서 구분한다.

## 짧은 저확신: 불필요한 STOP을 넣지 않는 방법

- 기존 직접 관측+판단 허가 3회는 유지한다. 다수결 7/4로 돌아가지 않는다.
- 같은 미션·같은 physical head의 정확한 이미지 stamp에 대응하는 정상 지도 투영과,
  CNN `low_confidence_bits`가 확인된 경우에만 이전 녹색 포함 관측을 짧게 보존한다.
- 이미지 정렬 pose와 최신 pose **둘 다 속도 1m/s 이상**, 유효한 source/receipt 시각이어야 한다.
  정차·속도 누락/NaN·과거 packet은 완화하지 않는다. 이 속도도 지연된 측위이므로 급정차 이후까지 절대 보장은 아니다.
- `low_confidence_hold_sec=0.3`이 현재 launch 기본이다. 마지막 신뢰 관측의 처리 시각부터
  최대 0.3초이며 동시에 **원본 source age 0.5초 이내**여야 한다. `0`이면 이 완충만 OFF다.
- UNKNOWN spam·동일 JPEG 재처리는 기한을 연장하지 않는다. 이전 원본 stamp로 보내므로
  새로운 허가 3회로 허위 계수되지 않는다. 기존 detector-only hold는 원본 age 0.2초 그대로다.
- 실제 known R/Y, 모순, 확신 소등, inference 오류, map/TF/identity 실패는 즉시 반영한다.
  저확신 결과라도 확신 Y bit 또는 이전 캐시에 없던 확신 R bit가 있으면 보존하지 않는다.
- D의 기존 R+좌화살표에서 arrow만 불확실한 경우는 이 한도 안에서 잔류할 수 있다.
  실제 변화가 저확신에 가려지면 최대 기한까지 대응이 늦을 수 있는 tradeoff를 없앤 것은 아니다.
- 이는 **조건에 맞는 짧은 저확신 때문에 제동 명령을 새로 넣지 않는 것**이다. 장기 실패·화각 이탈·실제 적색에
  필요한 제동까지 없애거나 모든 움찔의 소멸을 보증하지 않는다. 모델·confidence threshold는 변경하지 않았다.

## 일찍 서는 선과 채점되는 선의 분리

| 용도 | 기준 |
|---|---|
| 통상 정차 위치 | 팀장님 Lanelet `ref_line` + 기존 `stop_margin:0.0` |
| 황색 통과 ETA/제동 딜레마 거리 | 원본 MORAI MGeo type530 선까지 실제 앞차축의 경로 거리 |
| 황색 master OFF의 기본 GO_OUT | 기존 Lanelet 기준 유지, 아직 공식 도색선 준수 해결은 아님 |
| 황색 master ON 명시적 후보의 GO_OUT | 원본 MGeo 앞차축 통과 기준 |
| route helper의 현재 미션 해제 | 원본 MGeo 선의 경로 진행률 이후; 공식선 전 ID 조기 해제 방지 |

원본 profile은 `base_ws/src/perception/traffic_light_detection/config/traffic_signal_judging_lines.yaml`이다.
A/B/C/D의 MGeo 선 ID는 각각 B2256W000556 / B2256W000447 / B2256W000235 / B2256W000252다.
원본 `lane_boundary_set.json` 좌표·SHA와 기존 `map←morai_map` 변환을 검사한다.
이것은 움직이는 Ego/native API를 대회 판단 입력으로 넣는 기능이 아니라 고정 지도 자료다.
앞축은 rear-axle `base_link`에서 +3.0m를 한 번 변환한다. bumper offset을 다시 빼지 않는다.

원본선이 경로에 없거나 geometry가 무효면 황색 override를 허용하지 않는다. 짧아진 경로를
이동된 Lanelet 선으로 대체해 ETA를 짧게 만들지 않는다. 정상 정차 위치는 바꾸지 않는다.
황색 후보에서 map 정차 목표가 이미 뒤라면 현 위치의 제동 요구를 삽입하고, 공식선 통과 뒤에만 해제한다.

**1초 timeout, 최소 황색 시간 가정, 도달 ETA는 다른 값이다.** timeout은 기존 1초 유지이며
최소 황색 2초·여유 0.2초는 timed 후보의 기존 비활성 설정 그대로다. 사용자가 말한
“1초 안에 통과할 수 있나”의 거리 기준을 원본선으로 고친 것이지 황색 지속 시간을 공식 보장한 것이 아니다.
황색 master와 timed flag는 둘 다 기본 OFF다. master ON/timed OFF는 기존 제동 딜레마 분기다.

## 기본 GO_OUT을 한꺼번에 바꾸지 않은 이유

기존 실제 D bag에는 원본선 약7.9m 전부터 0.06m 전까지 `no_selected_crop`가 이어진 구간이 있다.
원본선 GO_OUT을 무조건 기본으로 켜면 기존 통과가 화각 이탈 STOP으로 바뀔 수 있다.
이번 soft CNN 완충은 이 FOV 상실을 덮지 않는다. 사용자의 이전 GO_OUT 보류를 존중해
기본 황색 OFF 주행은 보존하고, 황색 ON 후보에서만 원본선 crossing 처리를 묶었다.
`original_judging_D_visibility_gap_261026_v1.json`은 과거 입력의 반사실 분석이지 새 정책 실주행 결과가 아니다.

route helper의 원본선 진행은 기존 Foggy13 bag 읽기 전용 재생에서 A→B→C→D→없음으로 유지됐고,
후반 지난 미션 재선택은 없었다. 입력·profile SHA는
`artifacts/evaluation/traffic_light_robustness_261025/original_judging_route_replay_261026_v1_receipt.json`에 있다.

## 설치·검증 경계

이번에는 C++ ABI·새 고정 YAML·Python helper가 함께 바뀐다. 팀 PC는 pull만 하지 말고 기존
`bash scripts/runtime/install_traffic_light_candidate.sh`로 재빌드·설치한 뒤 스택을 재시작해야 한다.
새 launch만 있고 설치된 profile이 없으면 roslaunch가 시작 전에 실패할 수 있으므로 설치 확인을 생략하지 않는다.
가중치 재학습·재업로드는 필요하지 않다. 라이브 MORAI·차량 제어를 자동 시작하지 않는다.

회귀 시험은 관측 callback·실제 C++ 모듈·독립 원본 geometry를 분리해 수행한다.
조건을 만족한 G→저확신2개→G 시퀀스의 source-preserving 출력은 불필요한 STOP 삽입을 방지하고,
기한 만료·정차·실제 R/Y·지도 실패는 기존 STOP 경로를 유지한다. 신규 LAN 풀스택 주행 인증은 아니다.

### 마감 확인

- Python 현행 인지291 + 분석기10 + 표본 재계산2 = **303 PASS**. 옛 정책 기대값 테스트는 별도 제외했다.
- 실제 C++ 모듈/guard/원본 geometry/legacy plugin **107 PASS**.
  `planner_logs/analysis/signal_smooth_official_line_261026_v2/`의 XML이며 다른 실행과 중복 합산하지 않는다.
- 실제 모듈의 같은 G→간헐 실패2개→G 시퀀스는 기존 UNKNOWN 경로 STOP 삽입4회,
  조건을 통과한 source-preserving 관측 경로0회였다. 실제 red는 양쪽 즉시 STOP이다.
- 설치 launch3종의 원본선4개/timeout1초 해석, GPU 모델 로드·ROS import 및 source/install 일치를 확인했다.
  이는 새 실주행의 무움찔 인증과 다르다. 실제 주행 노드는 시작하지 않았다.

## 수신한 팀장님 bag과 원인을 섞지 않기

사용자가 제공한 Drive 파일은 전용 `ajou_drive_private`로 받아 MD5를 대조했다.
실제 bag은 지도v87이며 traffic node 실행 흔적은 있지만 traffic 결과·debug·실행 설정은 녹화되지 않았다.
당시 모델/guard 버전은 확정할 수 없다. 일회성 수치·원본/이미지는
`planner_logs/analysis/261026_154234_signal_hesitation/REPORT.md`와
`planner_logs/analysis/261026_154234_signal_candidate_samples_v1/REVIEW.md`를 따른다.

현재 모델의 선택된30개 원본 표본 재계산에서는 primary16 + 기존 근접640 복구6으로22개를 선택했고,
선택한22개의 CNN soft 저확신은0개였다. 따라서 새 저확신 완충이 이 bag의 반복 제동을 해결했다고
주장하지 않는다. 화각 밖·per-head 투영 거부는 완충으로 덮지 않았고, yaw3→5도 후보는
최종 선택 이득0이라 기본으로 바꾸지 않았다. 현재 근접640은 이미 ON이라 당시 배포 여부도 분리한다.

원본 주행에는 A/C/D 재출발 후 명확한 재감속8개와 별개 optimizer 실패 제동이 있다.
후자는 신호 판독 보완만으로 고쳤다고 할 수 없다. 최신 정차선 지도+설치 확인 후 재녹화에서
traffic/debug와 planning 원인을 함께 기록하는 것이 다음 실주행 검증이다.
