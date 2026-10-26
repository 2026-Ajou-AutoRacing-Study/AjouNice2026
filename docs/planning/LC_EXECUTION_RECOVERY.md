# LC 실행 진행과 정적 객체의 이동 전환

이 문서는 LC의 공간 오차 검사와 optimization-base 정적 장애물 수명 관리의 소스 계약을 설명한다. 설치·운영 적용 여부는 별도 확인한다.

## 정적 객체의 이동 전환

`optimization_base`에서 최신 planner 입력의 유한한 속력이 해당 클래스의 기존 `moving_speed_threshold`를 넘으면 그 cycle부터 moving으로 분류한다. `moving_time_threshold`를 추가로 기다리지 않는다.

그 객체의 `stored_objects_`와 `clip_objects_`를 **lost-target 복원 전에 함께 제거**한다. 따라서 이동 입력이 정상화된 후 과거 정적 위치가 복원되는 것을 막는다. perception 객체 자체를 삭제하지 않으며 동적 예측·ObstacleCruise의 검사 대상에는 남는다.

검출 소실만으로 정적 기억을 해제하지 않는다. 속력이 다시 정지 기준으로 내려오면 동일 UUID도 즉시 정적 후보가 될 수 있다. 이 전환에 새 timer·cooldown·속도 기준은 추가하지 않는다. 다른 path-generation mode의 기존 moving 확인 방식은 유지한다.

## LC 진행 검사

후보와 최종 궤적 간 고정 0.5m 거리를 실패 기준으로 사용하지 않는다. 실제 최종 궤적의 geometry와 속도로 예측을 만들고 충돌 검사를 계속한다. 현재/목표 차선 corridor에서 측정 차체 또는 예측 차체가 벗어나는 것은 진행 지연 허용과 분리한다.

진행은 현재 도로 기준의 arc와 횡방향 phase로 비교한다. 같은 횡방향 phase가 계획상 언제 나타나는지 찾아 시간 차이를 계산한다. 단순히 횡오차를 종방향 속도로 나누지 않는다. 경로점 속도가 지도 상한일 수 있으므로 phase clock은 선택된 LC 실행 duration에 맞춘다.

남는 횡편차·방향 차이의 회복시간은 기존 속도별 최대 횡가속과 lateral jerk, 기존 finish lateral tolerance를 재사용해 추정한다. 이는 진행 감시용 추정이며 충돌·도로 경계·최적화기의 운동학 제약을 대체하는 실행 가능성 증명은 아니다.

튜닝값은 하나다:

```yaml
lane_change:
  execution:
    recovery_time: 1.0  # seconds
```

ROS 상대 경로는 `lane_change/execution/recovery_time`, parameter event 이름은 `lane_change.execution.recovery_time`이다. BPP node namespace 아래에서 로드한다. 값은 유한한 양수여야 한다. 시작 시 잘못된 값은 거절하고, 잘못된 갱신값은 이전 값을 유지한다.

- 이 값 이하의 phase 지연·회복시간 추정은 허용한다.
- 회복시간 추정이 이 값을 넘는 상태가 같은 시간 이상 지속되면 재계획을 요청한다. 이것만으로 unsafe 또는 Cancel을 만들지 않는다. 허용 시간과 지속 판정 시간에 같은 값을 사용하므로 **전체 지연이 정확히 이 값 이하라는 뜻은 아니다**.
- 정상 상태가 들어오면 바로 failure 누적을 지운다. 정상 복귀 대기 timer는 없다.
- maneuver 시작/끝이 바뀌거나 clock이 역행하면 기존 누적을 재사용하지 않는다.
- feedback과 ego 편차에 같은 기준을 사용한다. 재계획 요청은 실제 collision/RSS 판정의 frame-count hysteresis와 별개다.
- 실제 collision과 corridor 위반은 진행 timer를 기다리는 사유가 아니다.

기존 `path_miss.velocity_points/lateral_thresholds`는 과거 설정 호환을 위해 수신하지만 새 횡방향 진행 판정의 튜닝값으로 쓰지 않는다. `enable_path_miss_detection` 및 완료점 이후의 longitudinal/target-lane 검사는 유지한다.

feedback 누락·stale·다른 frame·불충분한 경로는 계속 거절한다. 기존 feedback freshness timeout은 속도 상태 전환용 0.5초 대기와 다른 계약이므로 유지한다. 이런 입력 실패와 progress failure를 객체 UUID 증거 없이 `unsafe_trailing=true`로 분류하지 않는다.

## 검증과 적용 경계

회귀에는 즉시 moving 복구, 정적 가림 유지, 다른 mode 호환, 서로 다른 주행 속도의 동일 phase 지연, 지속시간/clock reset, ego 편차 회복, corridor 이탈, 실제 근거리 충돌, 먼 정지차에 대한 인접 LC 후보 안전성 검사, parameter 갱신을 포함한다.

특정 거리의 정지차가 있다고 항상 LC를 승인하지는 않는다. 도로 구조·차체·전후방 객체·실현 가능한 궤적을 통과한 경우에만 가능하다. 단위 안전성 검사는 전체 후보 생성·선택·optimizer·controller 폐루프의 성공 증거가 아니다.

장거리 검출 중심의 흔들림과 tracker 초기 속도 정확도는 별도 인지 문제다. 이 변경은 추정 속도를 정확한 값으로 보정하는 변경이 아니다. 속도 입력이 회복됐을 때 planning이 잘못된 정적 기억 때문에 계속 지연되는 경로를 수정한다.

LC parameter/class 구조가 변경되므로 배포 전 관련 의존 타깃을 일관된 헤더/라이브러리로 빌드해야 한다. 소스/후보 devel 테스트를 설치본이나 MORAI 폐루프 검증으로 해석하지 않는다.

## 준비 중 갱신과 후보 정책

승인 후 준비 중에도 최신 입력에서 후보를 찾는다. 새 안전 후보가 더 일찍 실행 가능하거나 즉시 실행 가능하면 현재 pose·yaw·속도 연결을 확인하고 같은 모듈/RTC 승인 안에서 교체한다. 검색 실패 시 기존 승인을 삭제하지 않는다. 횡이동 중에는 진행 감시의 재계획 요청이 있을 때만 연결 가능한 안전 후보로 교체하며, 연속 연결을 만들지 못한 경우 기존 계획의 실제 최종 궤적 안전 검사를 계속한다. 모든 지연에서 복구 경로를 만들 수 있다는 보장은 아니다.

`trajectory.compact_sampling`은 준비시간별 앞 gap 가속·등속·뒤 gap 감속 대표값을 선택한다. `trajectory.lateral_acceleration_samples`는 물리 map 안으로 제한한 명시적 한계 목록이다. 운영 기본은 준비 0~3초 0.5초 간격, 3전략, 횡가속 [2,3,4,5,6,6.5]의 최대 126 기본 조합이며, 물리 제외/중복으로 실제 생성 수는 줄어든다. 기존 dense sampling은 대표값을 고를 feasible grid로만 사용한다.

통과 후보를 탐색 예산 안에서 비교한다. 준비 지연에 횡가속/종가속 부담을 더한 effort를 0.25초 단위로 비교하고, 비슷한 effort에서는 앞 gap을 선호한다. 앞 gap 판별용 가까운 객체가 전체 안전 검사 대상을 하나로 제한하지 않는다. 특수 cooperative overtake 설정은 false를 유지한다. 위치/차로 ID 분기는 없다.

정상 승인 LC는 목표 차로의 다음 LC 종료 제약을 적용한다. 원차로 종료 정지점을 재삽입하지 않는다. 실제 장애물/신호/다음 목표 차로의 종료 정지점은 삭제하지 않는다. 객체 예측 보간은 원본 마지막 시각/pose를 정확히 보존하되 예측을 연장하지 않는다.
