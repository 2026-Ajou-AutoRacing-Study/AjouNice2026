# 고주로 Map-CV 8초·로터리 3초 운영

2026-10-24 사용자·판단팀 승인 정책. LC 준비 3초 + 변경 4초 같은 상황을 위해
고주로의 지도 예측 시간만 늘린다. 모델 재학습·가중치 교체는 필요 없다.
지도 합류 연결 수정과 planner/controller 정책 변경은 이번 범위가 아니다.

## 출력 규칙

| 조건 | 기본 출력 |
|---|---|
| 고주로 차량의 완전한 8초 지도 경로 존재 | Map-CV 8초 |
| 8초는 부족하지만 기존 3초 지도 경로 존재 | 기존 Map-CV 3초 그대로 |
| 3초 지도 경로도 없음(예: 1~2초 길이만 존재) | 기존 CTRV 3초 |
| 로터리·일반도로 | 기존 3초 |
| 보행자·UNKNOWN 장애물 | 기존 3초 정책 그대로 |

갈림길 자체는 실패 조건이 아니다. 기존 Map-CV가 각 분기의 8초 후보를 계산한다.
고주로에서 완전한 지도 경로가 없으면 **CTRV 8초·직선 덧붙이기·끝점 반복을 하지 않는다.**
지도 경로의 길이/유한값 검증 실패나 선택적 계산 예외도 기존 짧은 출력으로 돌아간다.
고주로 8초의 후보 수·순위·confidence·첫 3초는 native 재계산으로 바뀔 수 있다.
3초로 남기는 객체와 모든 현재 상태·UUID·입력은 그대로 보존한다.

Ego 미션 toggle이 아니라 객체별 현재 start lanelet로 구분한다.
현재 지도의 `speed_limit=150`은 구간 표식이며 NPC 속도를 강제하는 값이 아니다.
진입 대기 시간을 추가하지 않는다. 지도 매칭/속도 조건은 기존 predictor의 조건을 따른다.
0.1초 간격에서 현재 pose 포함 31/81개를 출력한다.

## 팀원 설정: 이 파일 한 곳

`base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml`

```yaml
prediction_horizon: 3.0          # 로터리·일반·CTRV fallback
highway_prediction_horizon: 8.0 # 지도 경로가 충분한 고주로 Map-CV만
highway_prediction_mode: native
prediction_time_step: 0.1
```

예를 들어 3/6초는 highway 값만 6.0으로 바꾼다. 6/8초는 baseline을 6.0으로 바꾸면 된다.
긴 값은 baseline보다 커야 하고 0.1초 단위, 현재 pose 포함 최대 100개(최대 9.9초)다.
선택적 mixed 설정이 부적합하면 기존 baseline으로 실행하며 새로운 기동 거부를 만들지 않는다.
설정 변경 후 **차량 정지 상태에서 tracking을 정상 종료·재시작**해야 적용된다.
YAML은 source 경로에서 읽으므로 시간 값만 바꾸는 데 재빌드는 필요 없다.
코드를 처음 업데이트할 때는 workspace 설치/빌드와 alias 재로딩이 필요하다.

운영 `tracking` alias·`tracking_prediction.launch`·통합 `morai_planning_stack.launch`는
mixed ON이며 시간 값을 하드코딩하지 않고 이 YAML을 따른다. CLI 명시값은 YAML보다 우선한다.

```bash
tracking
# 일회성 3/6초
tracking prediction_highway_horizon:=6.0
# 전체 baseline(기본 3초)으로 복귀
tracking prediction_mixed_horizon_enabled:=false
```

기존 validator campaign은 snapshot 재현을 위해 mixed 기본 OFF·명시적 시간 인자를 유지한다.
새 비교는 stack에 `prediction_mixed_horizon_enabled: true`, `prediction_horizon: 3.0`,
`prediction_highway_horizon: 8.0`, `prediction_highway_mode: native`를 명시한다.
기존 실행 snapshot을 수정하거나 resume 보호 검사를 완화하지 않는다.

## 검증과 알려진 지도 경계

v87 우측 소멸차선 201441/201446의 옆 진행도로는 존재하지만 현재 predictor는
`following`만 사용하고 차선변경/합류 연결을 예측에 넣지 않는다. 14장면의 85개 객체-프레임에서
3초 Map-CV는 가능했으나 8초 길이가 부족했다. 이 경우 이제 Map-CV 3초를 보존한다.
예전 CTRV 8초 후보는 첫 3초 위치가 최대 38.27m 달라져 기각했다.
이 85개에는 연속 8초 GT가 없으므로 장기 fallback 정확도가 검증됐다고 표현하지 않는다.

실제 ROS 메시지 변환·입력 불변·분기·지도 종단·1~2초 경로·정지/역방향·예외·config 전달을 검사한다.
재생 도구는 `scripts/evaluation/qualify_highway8_horizon.py`이며 원본 입력·지도·소스 SHA를 남긴다.
당시 append/CTRV 후보 증거는 수정하지 않는다. 최종 map-only 재생은
`artifacts/evaluation/highway8_261024_v1/map_only_final_v1/`에 보존한다.

8초는 시간 범위이지 정확도나 충돌 방지 인증이 아니다. 실제 팀 PC에서 LC 소비·추월 영향은
추가 확인해야 하며, Ego maneuver가 8초보다 길면 여전히 부족할 수 있다.
