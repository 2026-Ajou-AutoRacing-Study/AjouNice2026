# 황색·근접 판독 종료 반복 검증

종류: EVIDENCE / 2026-10-28. 현재 계약은 [CURRENT](CURRENT.md)를 따른다.
사용자 지시로 **PR·머지·원격 배포는 보류**한다. 아래 완료된 실행과 준비만 된 입력을 구분한다.

후속 설정 변경: 이 시험들 종료 뒤 사용자 요청으로 공통 `delay_response_time`은0.0초로
바뀌었다. 아래0.5초 시험 원본·snapshot·수치는 수정하지 않았으며, 새0초의 실주행 결과가 아니다.

## 공통 조건

- 실제 localization + ROS1 planning + PP/PI + od_gt. BEVFusion·공식 LAN 풀스택 인증이 아니다.
- YOLO/CNN v5, 직접 관측·허가1개·UNKNOWN0.3초·투표OFF.
- 시험에서만 황색 master ON, 의사결정 감속도8m/s², 응답지연0.5초.
- 명시 지도 v97 SHA `da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9`.
- 원격 planning은 v98·응답지연0.0초로 진전했다. 이번 소스/시험에 섞지 않았다.
  따라서 이번 결과를 최신 원격 설정의 실주행 인증으로 쓰지 않는다.
- 원래 전체 출발·목적지·waypoint 유지. 전방 판독 영상과 RViz/MORAI 화면을 기록하고
  D 원본선 앞차축 통과+5초에 영상만 종료, 제어·bag은 정상 목적지 및 정지/P까지 유지한다.
- 각 실행에 passive Hz·source age·CPU 기록이 있다. `/ctrl_cmd`는 header가 없어
  명령 발행→액추에이터 지연 자체를 측정한 값은 아니다.

## 실행 증거와 판정

근거 루트: `artifacts/evaluation/traffic_yellow_repeated_261028/`.

| 실행 | 환경·제어 | 확인된 결과 | 남은 한계 |
|---|---|---|---|
| run_01_sunny13_natural | 재시작 후 Sunny13·SET 없음 | 전체 goal PASS·정지/P. A/B/C/D 원본선 통과 native 방향 허가 | 황색 GO 분기 없음. A 최초 색은 R이므로 첫 관측 Y 검증 아님. D 근접 재제동1건 발견 |
| run_02_sunny13_A_stop | A 30m trigger·Y3초 요청 | 실제 G→Y에서 GO 분기: 목표22.109m, 제동 추정22.567m | 의도한 STOP 조건보다 늦게 읽혀 근접 GO 사례가 됨. A 통과 후 terminal 제어 불일치로 ERROR·정지/P |
| run_02b_sunny13_A_stop | A 45m trigger·Y3초 요청 | 실제 G→Y에서 STOP: 목표28.360m, 제동 추정19.248m. STOP 뒤 재출발 없음 | native Y가 약1.51초 뒤 R로 바뀌어 ERROR·정지/P. 정상 정차 종점·D 재검증은 미완료 |
| run_03_foggy13_no_SET | 추가 재시작 후 Foggy13·SET 없음 | D 출발 이후 양수 궤적 유지·재제동 ZERO 없음. 별도 정지/P 복구 확인 | D+5초 영상 정상 마감 뒤 owner가 외부 신호로 추정되는143으로 종료. 완주 PASS 아님. A 통과 native bracket은 조회 실패가 끼어 미검증 |
| run_04_sunny15_no_SET | Sunny15·SET 없음·시나리오 시각+8초 | 전체 goal PASS·정지/P. A/B/C/D 통과 native 방향 허가. D terminal ZERO0 | A 원거리의 어두운 crop을 Y로 과신한3장과 실제 조기 STOP 요청이 있음. 움직이는 실제 G→Y GO 분기 없음 |
| run_05_sunny11_no_SET | Sunny11·SET 없음·시나리오 시각+16초 | 전체 goal PASS·정지/P. A/B/C/D 통과 native 방향 허가. B/C 단발 공백은 기존 완충 | A의 어두운 crop Y과신2장. 실제 이동 G→Y 분기 검증 아님 |
| run_06_foggy11_no_SET | Foggy11·SET 없음·원본 시각 | 전체 goal PASS·정지/P. A/B/C/D 통과 native 방향 허가. A 의심Y0, D terminal ZERO0 | D 최초 허가 직후 정차 상태의 단발 UNKNOWN이 최종 신호에도 남음. 실제 출발은 연속적이나 누락0 인증은 아님 |
| run_07_foggy15_no_SET | Foggy15·SET 없음·시나리오 시각+8초 | 전체 goal PASS·정지/P. A/B/C/D 통과 native 방향 허가. D terminal ZERO0 | C의 물리 head 교체 중 no_lamps3개가 최종 UNKNOWN으로 남음(R 정차 중·GO 반전 없음). 실제 이동 G→Y GO 분기 검증은 아님 |

요청한 색을 GT로 쓰지 않는다. run02/02b의 전체 완주 실패를 PASS로 바꾸지 않았으며,
분기 확인과 실제 주기 유지·완주를 구분한다. 두 시험 모두 실제 황색이 반복 명령에도
달라질 수 있어, 강제 주기의 지속시간·황색 통과 무패널티를 인증하지 않는다.

수정 후 정상 완주는 run04~07의4환경이다. run01 Sunny13는 수정 전이며 run03 Foggy13는
수정 후 D 통과 영상까지 정상이나 owner 종료로 전체 완주 PASS가 아니다. 여섯 환경 모두가
같은 build에서 정상 완주했다고 요약하지 않는다. 추가 Sunny13 반복 입력(run08)은 준비만 했고 미실행이다.

| 정상 완료 영상 | 현재 카메라·판독·실속도·계획속도 표시 |
|---|---|
| Sunny15 | [전방 영상](../../../artifacts/evaluation/traffic_yellow_repeated_261028/run_04_sunny15_no_SET/front_video/front_evidence.mp4) |
| Sunny11 | [전방 영상](../../../artifacts/evaluation/traffic_yellow_repeated_261028/run_05_sunny11_no_SET/front_video/front_evidence.mp4) |
| Foggy11 | [전방 영상](../../../artifacts/evaluation/traffic_yellow_repeated_261028/run_06_foggy11_no_SET/front_video/front_evidence.mp4) |
| Foggy15 | [전방 영상](../../../artifacts/evaluation/traffic_yellow_repeated_261028/run_07_foggy15_no_SET/front_video/front_evidence.mp4) |

동일 실행의 RViz/MORAI 화면·bag은 각 `agent_run0N_audit.md`에 실제 경로를 보존한다.
영상은 D+5초에 끝나지만 제어·bag은 원래 전체 목적지까지 유지했다.
독립 감사의 [환경별 영상·연속성·Hz/age/CPU 비교표](../../../artifacts/evaluation/traffic_yellow_repeated_261028/agent_round_summary.md)를 함께 본다.

## D 근접 재제동 원인과 수정

run01 D의 순수 UNKNOWN 중 현재 위치의 속도0 계획1개가 영상2프레임에 표시됐고,
실제 약1km/h 재감속·brake 최대0.279가 관측됐다. 같은 구간 native20회는 모두33(R+좌화살표),
CNN이 적색으로 오판한 사례가 아니었다.

가상 Lanelet 선을 지난 뒤 그 선이 잘린 경로 밖으로 빠지면, 원본 채점선 전 정지를 보존하기
위한 기존 fallback이 현재ego를 정차 목표로 사용한다. 이 합성 목표0m를 가상선 통과 판단에도
재사용하면서 UNKNOWN 예외가 잠깐 해제됐다.

수정은 실제 Lanelet 선→앞차축의 유한선 기하거리를 별도로 전달한다. 방향은 기존 원본선의
approach 방향을 사용하고 현재yaw로 통과 방향을 뒤집지 않는다. 기하가 없으면 실제 경로에서
계산한 목표만 대용으로 허용하고 합성 현재위치0은 재사용하지 않는다. 별도 영구 GO latch는 없다.
새 적색·황색·색 모순·stale·원본선 GO_OUT은 기존대로 유지한다.

normal/experimental 양쪽 수정·설치 일치 검사 완료. C++97개(guard36/module38/experimental23),
평가 Python115개 통과(이번 helper/wrapper·짧은 주기·영상·시간 관측·시나리오·supervisor 관련7파일).
잘린 경로·미관측·경계0·색 근거·역방향 재배치 회귀를 포함한다.
설치 library SHA는 `03cca55aa8b0235af61dcb10358c56617e29ef26213ed348836ffc263d7eaf55`다.
run03 D의 출발 후34표시프레임과 terminal UNKNOWN19표시프레임에는 갑작스런 ZERO가 없었다.
후자의 최저 계획속도4.926m/s, brake 최대0.02484였고 실제 속도는17.18→25.52km/h로 증가했다.
새 기하거리 -3.207/-4.354m에서 keep_path_open 로그를 확인했다. 동일 입력 재생 대조나
다른 환경 전체의 완치 인증은 아니다.

run03 주행 창(controller 시작→D+5초)에서 front23.80Hz/traffic8.164Hz/trajectory9.997Hz,
측위·제어50Hz였다. traffic header→bag age 평균/p95=30.47/46.71ms,
trajectory=175.48/246.50ms, CPU busy 평균85.96%였다. owner 종료 뒤 대기 시간을 포함한
전체 요약을 이 주행 창 수치로 사용하지 않는다.

동일 수정의 run04/05/06/07에서도 D terminal UNKNOWN 중 ZERO는0이었다.
최저 계획속도는 각각5.055/5.214/5.372/4.666m/s, 최대 brake는0.0266/0.0359/0.0318/0.0283이었다.
이는 정상 곡률 속도 조정까지 brake0으로 만든다는 뜻이 아니다.
run06은 첫 R+좌허가 직후 detector 누락1개(원본 간격0.123초)가 최종 UNKNOWN으로 남았다.
당시 실제 속도는0→0.3→1.1km/h로 출발했고 재제동은 없었다. 확인 가능한 인접 C++ 결정은
둘 다 허가였으므로, 이 한 gap을 planner 유예가 직접 처리했다고 단정하지 않는다.
C의 no_lamps2개/0.257초도 두 번째는 UNKNOWN이지만 기존 R 정차 중 GO 반전은 없었다.
관측 누락·최종 색 변경·계획 STOP·실제 재제동을 서로 다른 수치로 관리한다.

run07 C에서는016 R→018 선택/no_lamps3개(0.426초)→016 R로 돌아왔다. 서로 다른 물리
head라 이전 head의 색 hold를 적용하지 않았고 최종 UNKNOWN3개가 남았다. R 정차 중이어서
GO 반전·새로운 재제동은 없었으나, 영상 overlay만으로 이 원본 누락이 없다고 판정하면 안 된다.
다른 phase에서도 head 교체가 항상 무해하다는 인증은 아니다. A 의심Y는0, B/D interior gap은0이었다.

정상4회 주행 창의 canonical traffic은8.024~8.138Hz, header→bag age 평균31.22~32.96ms/
p95 47.34~50.72ms였다. planner는9.942~10.009Hz, age 평균171.21~181.60ms/
p95 240.09~264.43ms, ctrl_cmd는50Hz였다. CPU busy 평균86.36~88.12%, p95 90.64~93.72%로
로컬 동시 실행 부하는 실제로 높았다. 이것은 물리 센서 capture→actuator 지연이나 LAN4090 인증이 아니다.
`agent_environment_window_comparison_v1.json`에서 정상4회와 owner 실패run03을 분리한다.

## 시험 제어와 판독 불일치의 구분

- 이동 황색 helper의 기본 전송을 persistent gRPC로 통일했다. UDP는 명시적 과거 재현용이다.
- 의도한 정지 조건을 만들 수 있도록 평가 trigger 상한을30→60m로 확장했다.
- 대상 원본 유한선을 실제 통과하고1초가 지나면 지나간 신호의 terminal SET만 중단한다.
  Y/R 단계의 실제 상태 검사는 유지하고, 입력 freshness·fault heartbeat는 owner 종료까지 유지한다.
- run02b에는 강제 Y 이전에도 `CNN=Y / native GET=G`가 있다. 원인 미확정이다.
  실제 입력과 동일한 source stamp의 원본JPEG8장을 bag에서 재인코딩 없이 추출했다.
  `raw_exact_review_v1/manifest.json`을 따른다. API 조회만으로 CNN 오판이라고 단정하지 않는다.
  후속 원본 픽셀 QC에서 첫 두 표본은 발광 근거가 부족한 어두운 head, 세 번째는 희미한
  녹/청록이었다. 실제 Y 표본과 다르지만 CNN의 Y 확률은0.641/0.700/0.949였다.
  앞의 두 개를 확정G로 재라벨하지 않으며, 작은·어두운 head의 Y 과신 문제로 별도 관리한다.
- run04에서도 A의 Y3개는 exact JPEG 픽셀상 읽을 만한 황색 발광 근거가 없었다.
  p(Y)=0.855/0.879/0.883이며 독립 native 전후 조회는 R이었다. 실제 module은 첫 표본에서
  정차 목표113.720m에 STOP을 요청했다. 이후34.17→22.52km/h 감속이 있으나 곡률 제한과
  신호의 기여를 분리하지 못했으므로, 전부 신호 탓 또는 모두 부드럽게 해결됐다고 하지 않는다.
  실제 G 허가 뒤에는 재STOP이 없었고, D terminal UNKNOWN19표시프레임에도 ZERO0이었다.
  `run_04_sunny15_no_SET/agent_run04_audit.md`와 exact 원본/crop manifest를 따른다.
  작은 housing 높이 gate는 손실까지 비교하는 **오프라인 후보**이며 모델·운영 임계값은 바꾸지 않았다.
  run05의 같은 도로 위치에서는 첫 색 관측 전인데도33.82→22.57km/h로 유사하게 감속했다.
  run04의 초반 속도 dip를 Y 오판이 일으켰다고 단정할 근거가 없으며, 도로 곡률 제한 설명이 더 강하다.
- 현재 native API의 전체 주기표 GET/자동제어OFF는 확인되지 않았다. 로컬 MGeo의 주기표를
  현 시뮬레이터 프로그램과 동일하다고 가정해 덮어쓰지 않았다.

### 작은 housing gate를 운영에 넣지 않은 이유

run01/02b/03/04/05의 classifier·runtime·selection을 정수 source stamp로 정확히 연결한
2,256개에서 detector housing 높이5/6/6.5/7/8px 제한을 오프라인 비교했다.
6.5px는 Y/native-non-Y 불일치14개 중13개를 UNKNOWN으로 바꾸지만 R/G/Y124개도 함께
제거한다(R72/G39/Y13, 화살표0). 이124개 전체를 오판이라고 판정한 것이 아니다.
8px는 불일치14개를 모두 지우지만 API-Y와 일치한 관측도1개 잃으며, Sunny15 A의 최초
API 정합 STOP 근거를 약9.7초 늦춘다. Foggy13의 기록 속도로 계산한 보조 정지 여유도
15.8→2.4m로 줄어든다. 이는 유한선·측위 보간의 반사실 계산이지 gate를 적용한 재주행이 아니다.
따라서 임계값으로 문제를 숨기지 않고, 작은·어두운 housing의 R/Y/G와 판독 불가를
다양한 환경에서 구분하는 CNN 보강을 별도 후속 과제로 남긴다. 가중치는 이번에 바꾸지 않았다.
근거: `offline_quality_gate_v1/report_01_02b_03_04_05_v3.json`.

## 다음 실행의 경계

Sunny/Foggy × 11/13/15시 설정은 같은 생성·실행 로직으로 준비했다. 준비된 YAML/계획은
실행 완료가 아니다. `planned_runs_02b_07_rev2.json`의 `executed=false`는 준비 이력이다.
후속 환경 시험에는 전방 원본 JPEG만 명시적으로 bag에 추가했고 LiDAR 원본은 추가하지 않았다.

첫 관측 Y 별도 stress는 도구만 준비됐고 이번에는 실행하지 않았다. 최초부터 Y만 제어하면
접근 중 Y가 길어질 수 있으므로 자연3초 주기로 표현할 수 없다. 실제 첫 관측 Y와 정상 완주
증거는 아직 남은 검증이다. 자연 주행의 어두운 crop Y 오판을 이 검증의 성공으로 세지 않는다.
환경 반복·D 재제동 회귀·실제 황색 세 조건이 충분히 검증되기 전에는 PR하지 않는다.

## 실행기 비정상 종료와 고아 제어 보호

run03은 D 영상 마감 뒤 약44초까지 주행 상태를 저장하다 상위 owner만 종료됐다.
자체 오류/정상 cleanup, OOM/커널 크래시/디스크 부족 근거는 없고 외부 종료가 의심되나
신호 발신자는 확인하지 못했다. 남은 제어기를 명시적으로 중단하고 실제 정지→단독 P 송신→P를
확인한 별도 receipt를 `owner_recovery/verified_stop_park.json`에 보관했다. 과거 실행 상태를
정상 완료로 덮어쓰지 않았다.

후속 주행은 독립 tmux와 평가 전용 `supervise_traffic_light_trial.py`에서 시작한다.
살아있는 owner의 실제 descendant PID·생성시각만 추적하고, 비정상 종료로 제어기가 남으면
소유 제어기 중단→브레이크·정지 확인→기존 소유 UDP 종료→P 확인→소유 잔여 정리를 수행한다.
실제 확인 실패는 UNVERIFIED이며 복구 성공도 주행 PASS로 바꾸지 않는다. mock10개와
wrapper 도움말 실행 smoke를 확인했다. 이는 supervisor 자체 SIGKILL·통신 단절까지 보장하는
물리 비상정지 기능이 아니며, 일반 production 노드에 의존성으로 추가하지 않았다.

## 마감 상태

run07도 owner0·정상 cleanup·실제 정지/P 확인 후 종료했다. 주행용 tmux·제어기는 남기지
않았고 기존 MORAI/roscore는 유지했다. 날씨·시간만 Sunny13으로 복원하고 Get 일치를 기록했다.
지도를 추가 이동하거나 v98을 병합하거나 가중치를 교체하지 않았다. PR·push·merge는 하지 않았다.
`git diff --check`는 통과했다. 문서 검증의 유일한 오류는 이번 범위 밖 기존
`kcity_map/README.md`의85행/65행 예산 초과이며 신호등 현재문서 링크 오류는 없다.
