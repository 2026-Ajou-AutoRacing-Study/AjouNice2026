# UNKNOWN·단발 누락·지도 선택 정책 재검토

2026-10-26. 사용자 요청으로 GO_OUT·지도 정지선 통과 기준은 이번 검토에서 제외했다.
운영 소스·설정·가중치·실행 중 스택은 변경하지 않고 회귀 테스트와 설명만 보완했다.
현재 기준은 [CURRENT](CURRENT.md), 직전 timeout 변경은 [1초 복구 기록](261026_TIMEOUT_RESTORE_AND_POLICY_AUDIT.md)을 따른다.

## 결론

| 대상 | 판단 | 근거와 한계 |
|---|---|---|
| 최신 UNKNOWN STOP | 유지 | 미확인 신호로 새 출발하지 않는다. 정상 허가 관측 3개가 돌아오면 복구한다. 지속적으로 못 읽으면 계속 정차할 수 있으며 자동 시간 경과 GO는 추가하지 않았다. |
| 같은 head 검출 누락 보존 | 0.2초 유지 | 이전 원본 관측 시각을 보존한다. 새 R/Y가 오면 즉시 교체하고 CNN UNKNOWN·모순·지도/TF 실패는 캐시를 지운다. 전환을 못 본 동안은 최대 보존 기한만큼 이전 관측이 남는 한계가 있다. |
| 지도 연결·route 진행 이력 | 유지 | 다른 신호/이미 지난 미션을 다시 선택하지 않는다. 경로 이탈·반대 heading에 잠깐 거부돼도 유효 pose가 돌아오면 재선택한다. 지도/측위가 지속적으로 부적합하면 복구를 보장하지 않는다. |

0.2초는 callback 간격이 아니라 **원본 JPEG 시각부터의 총 age**다. 입력이 이미 150ms 오래됐으면
남는 보존 예산은 약 50ms뿐이다. 원본이 210ms 오래된 시험에서는 60ms callback 간격이어도
보존하지 않았다. 재전달한 동일 stamp는 판단의 새로운 녹색 확인 횟수로 세지 않는다.

## UNKNOWN과 지도 helper 실패는 다르다

- 올바른 미션 ID가 정해졌고 이미지 판독/투영만 실패하면 그 ID의 빈 `elements`가 발행돼
  최신 UNKNOWN으로 STOP 판단을 받는다. 지속 발행 중에는 1초 timeout으로 GO가 되지 않는다.
- route helper가 실패/만료해 **ID 자체가 0**이면 실제 미션 ID의 관측을 공급하지 못한다.
  planner에 남은 해당 ID의 관측이 만료되거나 처음부터 없으면 기존 competition fail-open GO가 적용된다.
  ID0 발행을 곧바로 해당 미션의 최신 UNKNOWN STOP이라고 설명하면 틀리다.
- 이는 신호등 미실행 때 주행 가능해야 한다는 기존 호환 정책과 연결된다. 별도 장애 분류/정책을
  사용자 동의 없이 추가하지 않았다. 최신 UNKNOWN의 장기 정차와 helper 장애의 fail-open을 구분해야 한다.
- 패키지 README에 남아 있던 “가드 ON이면 missing/stale도 STOP”이라는 이전 설명을 정정했다.

## 검증

- 현행 인지 Python 테스트 **260개 PASS**. 신규 5개는 실제 callback+가짜 모델/ROS의 UNKNOWN
  반복·관측 age 예산·미션/ID0 변경, route의 일시적 횡이탈·반대 heading 뒤 재획득을 확인한다.
- 실제 C++ 테스트 **40개 PASS**: guard17, 실제 module18, experimental5.
  module 신규 1개는 A/B/C/D와 빈 elements/명시적 UNKNOWN 양쪽에서 2초간 최신 UNKNOWN STOP,
  이후 서로 다른 허가 관측 3개 복구 및 동일 관측 재사용의 추가 계수 방지를 확인한다.
- Python의 과거 `test_signal_policy.py`는 옛 정책 기대값 재현용이라 현행 집계에서 제외했다.
- C++는 private ROS master 11374에서 실행하고 종료했다. 차량 명령·MORAI 제어·live 스택 재시작은 없다.
  테스트 중 XmlRpc acceptConnection 경고는 있었지만 세 실행 모두 종료코드0·XML failure0이다.
  XML은 `planner_logs/analysis/traffic_signal_review_261026/`에 보관했다.

## 기존 bag 근거의 적용 범위

`artifacts/evaluation/traffic_light_robustness_261025/route_progress_guard_v1/report.json`의
두 기존 bag 재생에서 legacy는 진행 후 D/C/B/A를 재선택했지만 guard ON은 A→B→C→D→없음을
유지했다. 현재 `lanelet_route_matcher.py` SHA256은 당시 근거와 동일한
`37e7856300320e6dde32f91043dc649edd882421f36d5783e7b748a8bd9fe3db`다.
이는 경로 선택 근거이지 매 프레임 신호등 identity 정답률·모든 CP 복구 인증은 아니다.

새 MORAI 실주행·LAN 동시 부하·가중치 정확도 재평가는 하지 않았다. 기존 bag의 전체 missing 수에는
정상 화각 밖/미션 접근 전/통과 후가 섞이므로 그 수를 정지선 앞 신호 판독 실패율로 사용하지 않는다.
