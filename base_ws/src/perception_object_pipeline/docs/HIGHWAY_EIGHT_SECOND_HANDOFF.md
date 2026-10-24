# Map-CV 시간 상한·지도 종단 운영

2026-10-24 사용자·판단팀 최종 승인 정책. 고주로는 최대 8초, 로터리/일반은 최대 3초다.
상대 NPC는 centerline을 따르고 소멸차선 끝에서 사라진다는 팀장님 설명에 따라,
지도 종단까지의 실제 가능한 시간만 출력한다. 8초/3초 고정 길이를 강제로 채우지 않는다.
이전의 Map8 부족→Map3 선택, CTRV8·직선 연장 초안은 최종 정책이 아니다.
Detector·학습·planner/controller 정책·지도 연결 자체는 이 변경으로 수정하지 않는다.

## 최종 출력

| 상황 | 출력 |
|---|---|
| 고주로 지도 경로가 8초 이상 | Map-CV 8초 |
| 고주로 지도 종단까지 5.6초 | Map-CV 5.6초 |
| 지도 종단까지 1.8초 | Map-CV 1.8초; CTRV 3초로 바꾸지 않음 |
| 로터리/일반 지도 경로 충분 | 최대 3초 Map-CV |
| 현재 지도 매칭 자체가 불가능 | 기존 baseline CTRV(기본 3초) |
| 고주로 지도 위 정지·저속 차량 | 정지 또는 해당 저속의 Map-CV; 저속만으로 탈락시키지 않음 |

현재 pose 포함 최대 31/81개, 간격은 0.1초다. 종단 시 남은 시간 이내의 마지막 0.1초 표본까지
출력한다(최대 0.1초 미만 보수적 절삭). 종단이 한 표본보다 가까우면 현재 pose 하나만 남긴다.
실제 routing graph 종단만 절삭하며 탐색 깊이·순환 제한을 소멸 지점으로 위장하지 않는다.
갈림길은 각 후보의 가능한 길이를 계산하고 기존 Top-K/confidence 계산을 유지한다.
끝점 반복·직선 덧붙이기·CTRV 8초로 채우기는 하지 않는다. 실제 정지의 동일 pose 반복은 정상이다.

## 팀원 설정: 한 파일

`base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml`

```yaml
prediction_horizon: 3.0           # 로터리/일반 및 지도 매칭 실패 시 시간 상한
highway_prediction_horizon: 8.0  # 고주로 Map-CV의 시간 상한
highway_prediction_mode: native
highway_direction_recovery: true # 고주로 도로 방향 근거가 명확할 때만 복구
prediction_time_step: 0.1
```

3/6초는 highway 값만 6.0, 6/8초는 baseline을 6.0으로 바꾸면 된다.
긴 값은 baseline보다 커야 하며 0.1초 단위, 현재 pose 포함 최대 100개(9.9초)다.
부적합한 선택적 mixed 설정은 baseline으로 실행하며 새 기동 거부 조건을 만들지 않는다.
YAML은 source에서 읽어 시간 변경만으로 재빌드는 필요 없다. **차량 정지 상태에서 tracking을
정상 종료·재시작**해야 적용된다. 코드 업데이트 시에는 workspace 증분 빌드·alias 재로딩이 필요하다.

운영 `tracking` alias·`tracking_prediction.launch`·통합 `morai_planning_stack.launch`는
mixed ON이며 시간은 YAML을 따른다. CLI 명시값은 YAML보다 우선한다.

```bash
tracking
tracking prediction_highway_horizon:=6.0
# 고주로 별도 시간 상한을 끄고 baseline으로: 종단 절삭 자체는 유지
tracking prediction_mixed_horizon_enabled:=false
```

기존 validator snapshot 재현을 위해 campaign mixed 기본 OFF와 명시적 시간 인자는 유지한다.
새 campaign은 stack에 `prediction_mixed_horizon_enabled: true`, `prediction_horizon: 3.0`,
`prediction_highway_horizon: 8.0`, `prediction_highway_mode: native`를 명시한다.
기존 snapshot·resume 계약은 수정하지 않는다.

## 고주로 방향과 저속

지도 지역은 현재 start lanelet의 `speed_limit=150` 표식으로 선택하며 실제 속도 강제가 아니다.
방향 복구는 기존 매칭이 실패했을 때만 시도한다. 가장 가까운 도로가 명시된 고주로여야 하고,
기존 3m 거리 gate 안의 비슷하게 가까운 도로들이 진행 방향에 동의해야 한다.
반대 방향이 모호하거나 고주로 밖이면 강제 매칭하지 않는다. 복구는 미래 경로만 바꾸며
현재 pose/twist/size/UUID·score·track 수명·coasting은 바꾸지 않는다.
정지 차량은 현재 위치를 유지한다. |종속도|<0.5m/s의 작은 음수는 미래 예측에서만 0으로 제한한다.
유의한 역주행 추정은 임의로 정방향으로 바꾸지 않고 기존 fallback을 유지한다.

## 근거와 한계

이전 14장면 고주로 3,867 객체-프레임의 90회 fallback은 저속 80회와 방향 불일치 10회였다.
10회 중 7회는 같은 원거리 track의 초기 반대 속도와 뒤따른 coasting이었다. 최근접 GT가
약 4.6→17.4m로 멀어져 실제 차량의 정확한 대응을 확정할 수 없다. 나머지 3회는 출발 장면의
저속 track이며 GT 중심과 약 0.14~0.33m였지만 도로 방향 표식 밖이라 복구 범위를 임의 확장하지 않는다.
따라서 매칭 성공 자체를 검출 AP 개선 또는 false-positive 제거라고 해석하지 않는다.

v87 우측 소멸차선 201441/201446은 앞 방향 연결이 끝난다. 이를 무조건 지도 결함·합류 연결
누락이라고 단정하지 않고, NPC 소멸 정책에 맞춰 유효 경로만 출력한다.
동결 입력 재생 도구는 `scripts/evaluation/qualify_map_terminal_horizon.py`다.
과거 append·CTRV8·map-only 고정길이 시험은 증거로 보존하며 최종 정책의 결과와 구분한다.
8초는 정확도·충돌 방지 인증이 아니며 팀 PC에서 실제 LC 소비·추월 영향은 후속 확인이 필요하다.
