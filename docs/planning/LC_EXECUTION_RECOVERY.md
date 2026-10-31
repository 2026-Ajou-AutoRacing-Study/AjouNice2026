# LC 실행 진행과 정적 객체의 이동 전환

이 문서는 LC의 공간 오차 검사와 optimization-base 정적 장애물 수명 관리의 소스 계약을 설명한다. 설치·운영 적용 여부는 별도 확인한다.

## 감속 LC 후보 토글

`lane_change.trajectory.allow_decelerating_lane_change`의 기본값은 `false`다.
`true`이면 LC 실행 중 감속하는 후보를 포함하고, `false`이면 일반·Frenet·terminal
LC 후보에서 제외한다. 준비 단계의 감속과 장애물 대응·검증 실패에 따른 안전 감속은
이 토글의 대상이 아니다. 이미 승인되어 실행 중인 LC를 토글 변경만으로 취소하지 않는다.
후보 검색 진단의 `allow_decelerating_lane_change`에서 적용값을 확인할 수 있다.

운영 설정은 `autoware_launch/config/ros1/planning/behavior_path_modules.yaml`의
`lane_change.trajectory` 아래에 둔다. ROS 상대 파라미터 경로는
`lane_change/trajectory/allow_decelerating_lane_change`이며 BPP namespace 아래에서 로드한다.

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

## 후보 선택과 승인 갱신 정책

운영 `trajectory.prepare_duration_samples`는 `[0, 0.5, 1, 1.5, 2, 2.5, 4]`초이고 최대 준비시간은 4초다. 목록은 초기 로드하며, 차량 상태상 가능한 시점 이후의 값만 사용한다. 횡가속도 샘플은 `[1, 2, 3, 4, 5, 6.5]` m/s²다.

`compact_sampling`은 준비의 가속/정속/감속 대표값과 실행의 가속/정속/감속 대표값을 독립적으로 조합한다. 준비시간 0에서는 준비 가속도가 없는 같은 후보를 중복 생성하지 않는다. 제한 전 기본 조합은 최대 342개(즉시 3×6 + 지연 6×3×3×6)이며, 중복 운동/물리 제약/시간 예산으로 실제 수는 줄어든다. 기존 dense grid는 대표값의 물리 가능 범위로 사용한다.

가까운 목표 차선 이동 객체를 기준으로 예측상 앞 갭을 먼저 탐색하고, 나머지 갭 탐색에도 50ms 총 예산의 일부를 남긴다. 통과한 후보는 앞 갭, 짧은 준비시간, 준비 가속→정속→감속, 실행 가속→정속→감속, 낮은 횡가속도, 높은 종료속도 순으로 비교한다. 준비시간 0에는 준비 가속도 순위를 적용하지 않는다. 가속도 ±0.001 m/s² 안은 정속으로 분류한다. 객체가 없으면 갭 순위가 같으므로 즉시 LC가 우선한다. 시간 예산 내 탐색된 후보끼리의 우선순위이며 전수 탐색 보장은 아니다. 갭 선택용 객체가 전체 충돌검사 대상을 제한하지 않는다. cooperative overtake는 false를 유지한다.

```yaml
lane_change:
  execution:
    always_update_approved: false
```

false는 매 주기 최신 입력으로 기존 승인을 검사하되 safe인 동안 geometry와 시작점을 유지한다. 진행 회복 감시의 재계획 요청만으로 safe 승인을 교체하지 않는다. unsafe이면 같은 주기에 새로운 유효·안전 후보를 찾아 교체한다. 현재 pose와 후보 최근접점의 거리·방향·속도 차이만으로 교체를 거절하지 않는다. 고정 1m·10도·2m/s 제한은 제거했다. 후보의 계획/현재 움직임 가설에 대한 충돌 검사를 유지하고, 승인 후 실제 최종 출력의 충돌·차체 corridor 검사를 계속한다. 새 후보가 없으면 기존 unsafe를 safe로 바꾸지 않으며 이후 취소/복귀/감속 판정을 유지한다.

true는 기존 승인 safe 여부와 무관하게 매 주기 유효·안전한 새 후보로 교체를 시도한다. 준비 중 safe 계획의 남은 시작시간보다 늦은 후보로는 교체하지 않아 시작이 계속 미래로 밀리지 않게 한다. unsafe 또는 진행 회복 재계획은 시작을 재설정할 수 있다. 토글은 초기 YAML 또는 `lane_change.execution.always_update_approved` parameter event로 전달한다. ROS 파라미터 서버 값 변경 자체는 노드 자동 재로딩이 아니다.

## 예측 범위와 충돌 판정의 분리

조건부 승인과 예측 대기용 LC 시작 전 정지점 삽입을 제거했다. 객체 예측이 자차의 남은 동작보다 짧다는 사실만으로 unsafe, unsafe_trailing, 취소/어보트를 만들지 않는다. 최초 후보와 승인 후 모두 각 객체가 실제 제공한 예측 시각에서 충돌/RSS 검사를 수행한다. 짧은 경로가 섞여 있어도 더 긴 대안 예측의 알려진 충돌 구간을 생략하지 않는다. 예측이 없는 꼬리는 복제/연장하지 않는다. 이는 전체 동작의 미래 안전을 확인했다는 뜻이 아니다.

`prediction_coverage_short`는 진단 sample의 `coverage_warning`에 기록하며 `unsafe_reason`과 `rejected_pairs`에 넣지 않는다. 실제 충돌/RSS 거절 진단은 범위 관측보다 우선 보존한다. 예측 자체가 없거나 시작 시점이 누락된 입력은 별도 입력 실패로 거절한다. cooperative 후방 거리 완화는 전체 예측이 있을 때만 사용하고, 범위가 짧으면 제공된 모든 경로에 일반 RSS 검사를 적용한다.

`prediction_coverage_deferred` 필드는 공개 구조체 ABI와 진단 키 호환을 위해 남기되 항상 false로 유지한다. 이 값으로 출력을 정지시키거나 승인 경로 검사를 우회하지 않는다.

정상 승인 LC는 목표 차로의 다음 LC 종료 제약을 적용한다. 원차로 종료 정지점을 재삽입하지 않는다. 실제 장애물/신호/다음 목표 차로의 종료 정지점은 삭제하지 않는다. 객체 예측 보간은 원본 마지막 시각/pose를 정확히 보존하되 예측을 연장하지 않는다.

이 절의 변경은 소스/후보 빌드 계약이다. 공개 Parameters/Path/NormalLaneChange 구조가 바뀌므로 설치 배포 시 LC 모듈과 의존 모듈을 함께 재빌드해야 한다. 운영 프로세스를 유지한 채 install 라이브러리만 교체하지 않는다.

## 안전예측 초기상태와 필수 예측 실패

Spatial LC 안전예측은 현재 자차 위치와 같은 시각의 측정 종속도에서 시작한다.
준비 구간은 후보의 준비 가감속을 유지하고, 실행 구간은 기존 공간 속도 프로파일의
가속도 증분을 이어 사용한다. 따라서 준비 중 가속해 앞갭으로 진입하는 후보를
정속으로 바꾸거나 삭제하지 않는다. 도로 제한속도는 상한이며 현재속도가 아니다.
승인 후에는 현재 진행 위치부터 남은 구간을 계산하고 준비 단계를 다시 시작하지 않는다.

경로·속도 후보 생성, 가감속 정책, RSS 기준과 ABORT 복귀 중 재탐색 조건은 유지한다.
변환은 후보 원본을 수정하지 않고 안전검사용 복사본에만 적용한다. 실행 구간의
기존 종가속도·감속·jerk 제한을 만족하지 못하거나 예측이 비면 해당 검사는 실패한다.
승인 재검사는 필수 후보/실제운동 예측이 존재하는지 **후단 궤적을 추가하기 전에**
확인한다. 후단 궤적 하나가 누락된 필수 예측을 대신해 승인되면 안 된다.
