# 중복 후보: 실제 ROS와 Cruise 고정 입력 대조

상태: EVIDENCE. 2026-10-24, Asia/Seoul.
[native 후보 검토](OBSERVATION_CONSISTENCY_REVIEW.md)의 후속이다.
`shape_consistent_association_ranking`만 OFF/ON, class 확인 후보는 OFF,
30Hz 추적·3초/0.1초 Map-CV·mixed OFF·최신 tracker 입력 기준 출력 예측 age 0.5초를 고정했다.
개별 객체의 마지막 관측 freshness와 다르며, 기존 정적 coasting 최대1초 정책은 유지한다.
운영 기본값·설치본·실행 중인 MORAI/validator/11311은 변경하지 않았다.

## 무엇을 실행했나

1. 현재 C++ tracker를 private source 사본에서 빌드했다. 운영 설치본은 새 인자가 없는
   구버전이므로 설치 binary와 source launch를 섞어 실행하지 않았다.
2. 독립 localhost master에서 실제 tracker → 실제 Python UNKNOWN 정책 → 실제 Python
   Map-CV를 실행했다. 각 실행의 ROS_HOME/로그/자식 process group을 분리했다.
3. `261024_035142_full_route.bag`에서 기록 TF로 만든 world-cache와 원래 측정 시각으로
   `map` 프레임 ROS DetectedObjects를 재구성했다. 원본 raw 센서·원본 메시지 전체 보존이나
   live TF 변환 검증이 아니다. 같은 유효 2,334입력을 같은 순서로 제공했다.
4. 원본 detection 수신 시각 이후 첫 30Hz tick에서 입력을 처리하고, 입력 generation과
   각 단계 출력을 ACK한 뒤 다음 simulated tick으로 진행했다. header 측정 시각과
   simulated 수신 시각, 실제 wall 처리시간을 별도로 저장했다.
5. OFF 두 실행의 전체 wire payload·association trace가 같은지 먼저 확인한 뒤 ON과 비교했다.

검출 수신 age는 원본 cache 평균 약94.56ms다. 재생에서는 30Hz 처리 tick으로 양자화되어
측정→처리 age 평균111.26ms/p95 131.08ms/최대170.06ms였다. 과거 다른 bag의237ms와
혼합하지 않는다. ACK 재생은 입력 누락을 통제한 기능 대조이며, wall-rate·LAN·풀스택
deadline 검증으로 부르지 않는다. 출력 bag의 record time도 처리 tick이며 header와 구분한다.

## 실제 ROS 결과

- OFF 두 번: 2,334입력의 generation ACK와 각 단계 6,327출력, 전체 직렬화 payload와
  association trace가 정확히 같았다. 예측 경로는 현재 pose 포함31개·간격0.1초(3초)였다.
- ON: 동일 입력·동일 출력 tick 수를 유지했다. 잘못 생성된 트럭 UUID 끝 `0026`의
  30개 출력이 사라졌고 confirmed UUID는 46→45개였다.
- 81개 birth의 `(input_seq, measurement_index)`는 모두 같았다. 나머지 45개 UUID의
  최초 confirmation 입력과 simulated 시각도 같았다. 이 표본에서 신규 객체 취득을
  늦추거나 birth를 영구 억제해서 중복을 없앤 것이 아니다.
- 기존 트럭 UUID 끝 `001e`만 내용이 달라졌다. 나머지 공통 UUID의 객체 필드는 동일했다.
  기존 트럭의 30Hz predicted-copy XY 최대 차이는 약14.91cm, 차체 속도 차이는 약0.239m/s다.
  native 측정 시각의 최대6.8cm와 분모·출력 시점이 다르다. GT 오차로 표현하지 않는다.
- 앞서 남았던 장애물 중복은 그대로다. 분류 후보도 켜지 않았다.

변경된 정상 트럭의 공통 UUID 1,163프레임 분포는 다음과 같다. 대부분 그대로인
프레임도 포함한다. 다른 공통 UUID는 객체 필드가 모두 같았으며, 아래는 GT 오차가
아닌 **OFF/ON 차이**다. 속력 차이와 차체 velocity 벡터 차이도 구분한다.

| 차이 | 평균 | p50 | p90 | p95 | 최대 |
|---|---:|---:|---:|---:|---:|
| 현재 XY(m) | 0.000766 | 0 | 0.000000876 | 0.000406 | 0.14908 |
| 속력 절대차(m/s) | 0.000932 | ≈0 | 0.000000125 | 0.0000907 | 0.19383 |
| Top-1 3초 끝점 XY(m) | 0.003091 | ≈0 | 0.00000240 | 0.000315 | 0.68261 |

경로 개수 변경은 0/1,163이며, 3초 끝점 차이가 1m를 넘은 표본은 없었다.
최대 차이가 평균보다 훨씬 큰 국소 변화이므로 평균만으로 해당 순간을 지우지 않는다.

별도 105~125초 window는 ACK로 발행을 막지 않는 wall-target 재생도 OFF/ON으로 했다.
각각 window 안의 215입력·단계별600출력을 확인했고 트럭 중복 30→0이 재현됐다.
그 전 fast history까지 포함한 수는1,385입력·단계별3,745출력이다. 입력 pacing 지각
최대는 OFF 약50.88ms/ON 약44.78ms라 zero-jitter·deadline 보장으로 표현하지 않는다.
일부 wall latency summary는 fast history와 window가 섞인 값이므로 window 전용 비용으로
인용하지 않는다. 모든 성공 실행의 private port 닫힘·자식 process group 정리도 확인했다.

## 실제 Cruise 코드 대조

현재 `ObstacleCruiseModule::plan()`의 실제 7개 C++ source를 별도로 컴파일해 호출했다.
원본 bag의 optimizer trajectory·odom·accel·decision 수신 시각과 현재 명시 config를 고정하고,
입력 PredictedObjects만 위 실제 ROS 출력으로 교체했다. 같은 ego/path를 강제로 유지하는
모듈 민감도 대조이지 폐루프 제어 주행이 아니다. 현재 이 함수의 smoothed 인자는
미사용이라 raw trajectory를 같은 값으로 제공했다.

기록 commit `e4e96ce8...`와 전체 planner config/map binary가 없어 당시 실행본을 복원한
시험은 아니다. 현재 source·header·binary·사용한 dependency hash를 보존했다.
재사용 build도 source/binary/dependency hash 일치를 확인하고 실행했다.

- 원본 predicted 입력 반복: 원본 optimizer 2,098주기의 실제 Cruise 결과가 A/A exact였다.
- ROS OFF 반복: 첫 예측 이전 3주기는 미래 입력을 보지 않고 제외, 동일 2,095주기의
  전체 결과가 A/A exact였다. 고정 입력과 predicted 입력 SHA도 같았다.
- OFF→ON: 후보 집합·실제 선택 UUID·속도 상한 발행 435주기·clear 횟수는 같았다.
  속도 상한만 29주기(약117.445~120.597초) 달라졌으며 17주기는 낮고 12주기는 높았다.
  차이는 최소−0.06426m/s/최대+0.00505m/s로 최대 절대 약0.23km/h다.
  감속·jerk 제한은 같았다. 최종 제어 brake나 실제 Ego 속도 차이라고 쓰지 않는다.
- 실제 선택 대상은 정상 트럭 `001e`로 같았다. 잘못된 트럭 `0026`은 OFF에서도
  `opposite_direction`으로 제외됐다. 트럭 window의 해당 제외 9건은 ON에서 0건이 됐다.
  이전 원본 기록의 가짜 트럭 `0156`도 현재 코드의 고정 재생에서는 역방향으로 제외됐다.

따라서 이번 중복 제거가 이 주행의 Cruise 급감속을 해결했다는 증거는 없다.
정상 트럭의 관측 재배분으로 위치·속도가 조금 달라져 PID/필터에 작은 상한 차이가
이어진 결과와 일치한다. 가짜 ID만 삭제한 ablation도 아니다.

상한이 달라진 29주기만의 절대 차이(m/s)는 평균0.00991/p50 0.000683/
p90 0.03642/p95 0.05784/최대0.06426이다. 전체2,095주기의 평균은0.000137이고
p50/p90/p95는0이다. 양쪽 모두 상한을 발행하지 않은1,660주기는 **변화가 없음**으로
계산했을 뿐, 0m/s 속도 명령을 냈다는 뜻이 아니다. 발행 유무가 바뀐 주기는 없다.

## 검증 범위와 남은 것

- 로터리·LC는 현재 source의 소비 규칙만 확인했다. 기록에 필요한 map/모듈 상태가 없어
  실제 판단을 복원하지 않았다. Top-K와 3초/3초를 바꾸지 않았다.
- 기존 14장면은 planner 입력이 충분하지 않았다. 11개는 optimizer/odom/accel이 없고,
  나머지 3개도 accel이 없어 임의 입력을 만들어 실제 planner 성능이라고 하지 않았다.
- 다른 Stop 계열 모듈·최종 smoother·controller·폐루프·실제 대회 LAN/24GB PC는 미검증이다.
- 운영 설치본은 자동 교체하지 않았다. 두 후보의 기본값도 여전히 OFF다.
  다음 팀 PC 시험에서는 최신 tracker와 launch를 함께 빌드한 뒤 **중복 후보만** 선택한다.
  실제 새 raw LAN bag에서는 선택 UUID·단계별 시각·제동 이유를 함께 확인한다.

## 근거와 재현 도구

- `scripts/evaluation/replay_tracker_shape_guard_ros.py`: 독립 ROS 3단계의 OFF/OFF/ON 재생.
- `scripts/evaluation/analyze_shape_guard_planner_response.py`: 실제 Cruise 고정 입력 재생.
- ROS 성공 자료: `planner_logs/analysis/261024_shape_guard_ros_v1/full_route_receipt_v2/`.
- Cruise 성공 자료: `planner_logs/analysis/261024_shape_guard_planner_v1/receipt_comparison_v1/`.
- 20초 wall-target 자료: `planner_logs/analysis/261024_shape_guard_ros_v1/realtime_105_125/`.
- 실패한 harness 시험도 별도 폴더에 보존했다. 입력 검증기/시각 ACK 실패를 정책 회귀나
  운영 ROS 장애로 집계하지 않았다. 원본 bag·cache는 수정하지 않았다.
