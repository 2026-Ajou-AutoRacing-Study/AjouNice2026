# 짧은 UNKNOWN 완충 후보 — 옛 강건성 감사 후 비교

문서 역할: EVIDENCE. 기록일: 2026-10-27. 운영 기준은 [CURRENT](CURRENT.md)다.
후속 상태: 아래는 승격 전 시험 기록이다. 이후 사용자 승인으로 유예0.3초와 색 증거 보존을 함께 운영 승격했다. [배포 정리](261027_UNKNOWN_GRACE_RELEASE.md)를 보며, 아래 당시 기본0·미PR 기록을 현재값으로 읽지 않는다.
이는 [옛 파이프라인 감사](261027_LEGACY_ROBUSTNESS_AUDIT.md)에서 확인한
짧은 판독 단절 완화 원리를 현재 입력·시간 계약으로 제한해 시험한 기록이다.
**운영 기본 변경·PR/병합은 하지 않았다.**

## 원인과 후보 범위

- 이전 직접 관측 주행 D의 작은 출발 재제동은 카메라 시각의 TF 조회가 약0.526ms
  미래 외삽을 요구하여 한 프레임의 지도 연결이 실패한 경우였다.
  CNN 오분류나 공분산 초과로 단정하지 않는다. 인접 TF가 observer에 도착했다는 사실만으로
  별도 subscriber/buffer인 ROI 노드에도 제때 도착했다고 단정할 수 없다.
- 기존 detector/CNN 공백 완충은 정상 지도·동일 head 등 조건을 요구하여 이 TF 실패를 처리하지 못했다.
  좌표변환을 최신값으로 바꾸거나 다른 신호등을 임의 연결하지 않고 판단 단계의 짧은 공백을 비교했다.
- 후보 `signal_compliance_unknown_grace_sec`는 **기본0**, 시험 허용0~0.3초다.
  이미 확인된 방향 허가 뒤의 순수 UNKNOWN에서만 직전 실제 G의 source/처리 시각부터
  정해진 기한 안에 허가를 유지한다. UNKNOWN을 새 G로 발행하지 않으며 원본 stamp도 그대로다.
- UNKNOWN 반복·중복 프레임은 기한을 갱신하지 않는다. 적색·황색·색이 있는 모순/부분 판독은
  이 유예 대상이 아니다. 다른 미션·모듈 초기화·시간/source 역전·stale/미수신도 유예하지 않는다.
  기존 competition 미수신/timeout GO는 별도 경로이며 이 후보가 강화한 것이 아니다.
- 옛 7/4·D1.5초 hold·profile 램프 추가·fresh UNKNOWN GO·황색 GO를 복원하지 않았다.
  황색 onset lower bound도 마지막 실제 G에서만 계산하며 UNKNOWN 시각으로 이동시키지 않는다.

## 원시 색 증거와 표준 메시지

기존 canonical 빈 배열은 단순 판독 불가와 색이 있지만 모순인 경우를 구별하지 못했다.
유예 시험은 `preserve_unreadable_color_evidence=true`를 함께 켜서 이 차이를 보존한다.
기본은 false이며 `--unknown-grace-sec`가 양수인 명시적 wrapper 시험에서만 함께 선택한다.

현재 검증된 crop에서 확실한 양성 bit 또는 관측 색이 있지만 최종 UNKNOWN이면,
그 **실제로 관측한 색만** UNKNOWN shape/status로 전달한다. 방향 허가 램프를 합성하지 않는다.
예를 들어 `R+G` 모순·`Y=1/arrow=?`·`G=1/arrow=?`는 알려진 색 증거가 있어 즉시 STOP한다.
no_lamps·확실한 양성 색 없는 저확신·지도/검출 누락은 빈 배열로 남긴다.
최종 phase는 UNKNOWN이고 원본 confidence/stamp를 보존한다.
debug의 `wire_unreadable_color_evidence`와 실제 canonical 메시지로 이 계약을 구분한다.

이 조합 없이 새 planner 인자만 다른 인지 노드에 켜면, 그 노드가 빈 배열에 숨긴 모순도
단순 공백으로 볼 위험이 있다. **임의 혼합 배포를 권하지 않으며 운영 기본으로 켜지 않았다.**

## 같은 입력 비교 계약

실제 C++ `Guard` 헤더를 포함한 독립 재생기로 source·수신 시각을 보존하여 비교한다.
단일 인지 callback뿐 아니라 동일 메시지를 반복 읽는20Hz 판단의 위상도 바꾼다.
표시되는 GO/STOP은 상태 머신의 정지 요구 proxy이며 반사실적 차량 주행이나 공식 신호 판정이 아니다.
색 증거 표식을 과거 데이터에서 복원하는 대조는 원본 wire 대조와 별도로 표시한다.

0.2초 조건은 과거 D 공백을 완전히 없애지 못했다. 마지막 실제 G 원본 stamp+0.2초가
다음 G canonical 수신보다 약79.48ms 빨라, 조사한20Hz 위상에서50~100ms의 STOP이 남았다.
원래 조건은100~150ms였고 한 위상에서는 개선이 없었다.
따라서 “한 프레임 누락이니0.2초로 모두 해결”이라고 결론내리지 않는다.
0/0.2/0.3초와 허가1/2회를 분리해 비교한다. 허가2회가 초반 움찔을 줄인다면
기존 허가를 유지한 효과인지, 첫 출발을 늦춘 효과인지 구분해야 한다.

직접 관측의 완료 실행3개에서 원본 C++ Guard로 다음을 확인했다.
아래 범위는20Hz 판단 위상0/10/20/30/40ms의 민감도이며 실제 planner callback 측정은 아니다.

| 원본 구간 / 허가1회 | grace0 | grace0.2 | grace0.3 |
| --- | ---:| ---:| ---:|
| 이전 current B의 단발G 뒤 판독 공백 | STOP250~300ms | 250~300ms | 200~250ms |
| confirm2 B의 raw Y 한 번 | STOP100~150ms | 100~150ms | 100~150ms |
| 새 current D의 TF 공백 | STOP100~150ms | 50~100ms | 0ms |

0.3초가 모든 판독 공백을 없애는 것은 아니며 명시적 Y의 STOP은 의도대로 남는다.
허가2회에서는 세 구간의 첫G가 아직 확정된 허가가 아니므로 그때 출발을 보류한다.
이는 기존 GO 유지 효과가 아니다. 새 current D의 허가2회는 첫G부터 실제 허가 채택까지
source 약372ms/수신 약382ms를 더 기다렸고, 다른 정차 후 허가 구간은 약101~152ms였다.
`defer_unseen_until_observed`·GO_OUT·정지점 삽입·차량 제어를 전체 재현한 것은 아니므로
이 수치를 실제 재제동·완주 시간 개선으로 읽지 않는다.

3개 실행의 available 비교340,336쌍 중 명시적 STOP/YELLOW171,728쌍에서 지연 위반은0이었다.
변경802쌍은 UNKNOWN의 STOP→GO였다. 원본 wire와 색 증거 표식의 두 stream이 포함되어
분모가 중복되며 초기/미션 종료 후 UNKNOWN도 들어 있다. 독립 사건 수·정확도·안전 인증이 아니다.
이3개 실행에서는 부분 색 표식으로 바뀐 UNKNOWN이0개여서 그 경계는 별도 합성·노드·모듈 반례로 검사했다.
근거는 [전체 같은 입력 대조](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/guard_three_production_runs_03.json)다.

## 검증·자료

- 원인과 원본 증거: `artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/`.
  `summary.json`, `baseline_D.json`, `vote7_C.json`, `old_baseline_B.json`은 원본·재생·추정을 구분한다.
- 실제 주행·같은 입력 결과는 아래 절을 따른다.
- 후보 인지·모델·지도·시간 보존과 판단 동작은 별도다. test PASS만으로 실대회 준수·LAN 안정성을 인증하지 않는다.
- 후보가 실제 소등/가림의 시작을 단순 공백으로 보면 이전 허가가 짧게 남을 수 있다.
  실제 적색으로 바뀌어도 색 증거 자체가 없으면 마지막 실제 G의 원본 stamp부터 최대0.3초 안에서
  이전 허가가 남을 수 있다. 새 R/Y를 명시적으로 관측하면 즉시 반영하지만 보이지 않는 전환까지 알 수는 없다.

## 실제 주행: 0.3초·허가1회

`traffic_light_unknown_grace_audit_261027_190110`의 실제 planner·PP/PI·localization 전체 경로는
**PASS·충돌0·종료 정지/P 확인**이었다. Sunny13, Lanelet v88, 객체는 `od_gt`이며
BEVFusion 동시 부하·팀4090·LAN 검증은 아니다. native 강제 SET 없이 자연 신호를 사용했다.
황색 OFF·허가1회·phase 투표0, grace0.3·색 증거 보존 ON을 실제 runtime/파라미터에서 확인했다.
과거 frozen 실행을 변경하지 않았으며 새 설정 `full_regression.yaml`에 별도 campaign을 만들었다.

| 미션 | 실제 관측·제어 | 검증 해석 |
| --- | --- | --- |
| A | R/UNKNOWN 변동 뒤 G로 출발. raw G→0.3m/s 약0.380초 | R과 판독 불가는 모두 STOP이므로 문자열 변동을 반복 출발로 세지 않음 |
| B | 관측된 G 구간 내부 공백0, 통과 | 과거 B의 긴 CNN 공백을 이번 주행에서 재현한 것은 아님 |
| C | R 정차 후 G+좌화살표 출발. 약0.425초 | 직진 허가. 내부 허가 공백0 |
| D | R+좌화살표 출발. 약0.400초, PTS151.3~154.1 출발 이후 제동0·속도 상승 | 과거 TF 단절과 작은 재제동은 이번 입력에 재현되지 않음 |

위 지연은 첫 raw 허가 원본 stamp→실제 속도0.3m/s 수신이다.
서로 다른 자연 주행의 기술 통계이며 후보의 인과적 속도 향상·저하 추정치가 아니다.
네 원본 MGeo선 앞바퀴 통과는 ±0.25/0.5초 native 조회 대조에서 모두 방향 허가였지만
공식 패널티나 모든 신호 전환의 안전 인증은 아니다.

실제 전방 녹화에서 자른 영상(화면 PTS는 전체 원본 기준):

- [A 정차·출발 89–96초](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/review_clips/unknown_grace_A_89-96.mp4)
- [B 통과 96–104초](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/review_clips/unknown_grace_B_96-104.mp4)
- [C 정차·출발 119–126초](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/review_clips/unknown_grace_C_119-126.mp4)
- [D 정차·출발 149–157초](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/review_clips/unknown_grace_D_149-157.mp4)
- [전체 전방 265.4초](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/unknown_grace/front_video/front_evidence.mp4)

이 녹화는 `--front-video-only`로 실제 annotated camera/telemetry만 저장했으며 데스크톱을 촬영하지 않았다.
선택 이미지/원본 stamp·재사용 여부·runtime 이미지 일치 여부·telemetry age는 sidecar에 있다.
영상의 `Timed candidate CONFIG enabled=OFF`는 **황색 시간제한 후보**이며 UNKNOWN 유예 OFF라는 뜻이 아니다.
UNKNOWN 유예0.3초는 label·실제 planner 파라미터 사본·C++ trace로 별도 확인했다.
[완료 주행 비교 JSON](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/comparison_actual.json)을 함께 본다.
[설치·원본 시각·실제 제어 감사](../../../artifacts/evaluation/traffic_light_robustness_261025/legacy_robustness_audit_261027_v1/actual_03_audit.json)에는 같은 입력 분석과 실제 발동의 한계를 분리했다.

### 실제 발동과 한계

- 고유 runtime2,055개 모두 직접 관측·phase 투표0·색 증거 보존 ON이었다.
  실제 색 증거 표식은0개여서 모순/부분 양성 경계의 비회귀는 노드·모듈 반례 검사 범위다.
- 완전하게 읽힌 C++ DEBUG 표본278행은 grace0.300/active0이었다.
  해당 trace가0.2초 throttle이고 일부 로그가 섞였으므로 **유예 발동 자체가0회였다는 증거는 아니다.**
  B/D의 마지막 화각 이탈 UNKNOWN은 원본 G age 약0.139/0.155초로 유예 가능 기한 안이었다.
  실제 branch 발동 횟수나 기존 출력과의 인과적 차이는 이 주행에서 확정하지 않는다.
- B의 첫 끝단 UNKNOWN 수신→Lanelet GO_OUT은 약30ms, D는 약140ms였다.
  Lanelet GO_OUT은 MGeo 원본선 통과보다 각각 약0.489/1.567초 빨랐다.
  카메라 확보용 정차선과 채점선이 다르므로 끝단 UNKNOWN을 모두 ‘채점선 통과 후’로 설명하면 틀린다.
  이번에는 해당 출발 후 불필요한 재제동이 관찰되지 않았지만 모든 전환 조건을 인증하지 않는다.
- 0.3초 후보의 특정 D 단절 효과는 **과거 동일 입력 재생**으로 입증한 Guard 상태 요구이며,
  새 주행의 매끄러운 D가 그 효과의 실제 재현인 것처럼 연결하지 않는다.
- 2회 허가 확인은 이번 실제 주행에 적용하지 않았다. 그 비교는 위 오프라인 결과에 한정한다.

## 최종 상태

관련 Python **496개**, 별도 ROS master에서 traffic C++ **132개** PASS.
소스·설치 plugin/helper/launch 일치 검사도 통과했다. 원래 모델 가중치·지도·센서 시간은 바꾸지 않았다.
시험 노드·제어기는 내려갔고 마지막 정지/P를 확인했다. MORAI는 켜 두었다.
잔여 ROS 시험 값도 grace0·색 증거 보존false·디버그false·phase 투표0으로 복구했다.
운영 기본값은 그대로이며 후보 승격·PR/병합은 하지 않았다.

판단은 **7/4 전체 복원보다 ‘짧은 공백만 제한적으로 완충’하는 후보가 현재 실패 원인에 더 직접적**이라는 것이다.
긴 CNN 공백·확실하게 잘못 읽은 색·가림 속 신호 전환은 여전히 별도 문제다.
새 주행 PASS를 모든 움찔 제거·공식 신호 준수 보증으로 확대하지 않는다.
