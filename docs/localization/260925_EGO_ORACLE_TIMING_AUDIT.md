# MORAI Ego oracle 프레임 정합 감사

상태: **EVIDENCE**
검증일: 2026-09-25

## 결론

`/Ego_topic.velocity.x`에는 고정 scale bias가 없다. 세 폐루프 bag 모두에서 전체
위치 이동거리와 속도 적분거리는 `0.01%` 이내로 일치했다. 그러나 개별 Ego pose
프레임은 등간격 운동학 상태가 아니다. Ego pose 증분에서 같은 메시지의 속도와
heading으로 예측한 증분을 빼면 다음 residual이 남는다.

| bag | 종 residual p50 / p95 | 횡 residual p50 / p95 |
|---|---:|---:|
| production 장주행 | `3.11 / 12.08 cm` | `0.032 / 0.189 cm` |
| production 짧은 반복 | `2.51 / 9.93 cm` | `0.024 / 0.109 cm` |
| 기각 Q/R | `2.60 / 9.80 cm` | `0.038 / 0.164 cm` |

따라서 기존 평가의 의미는 다음처럼 제한한다.

- 평면 위치 오차, 횡·종 위치 오차와 yaw 오차는 그대로 유효하다.
- **횡 frame jump는 oracle residual이 매우 작아 비교 지표로 유효하다.**
- **종 frame jump는 Ego pose의 프레임 시간 불균일이 지배하므로 절대 성능 및
  미세한 A/B 판정에 사용하지 않는다.**
- steering-rate, steering sign flip, controller lateral error는 oracle frame jump와
  독립적인 폐루프 지표이므로 그대로 유효하다.

## 검증 방법

연속 Ego sample `i, i+1`마다 다음을 계산했다.

```text
predicted_delta = mean(velocity.x) * dt * [cos(mean(yaw)), sin(mean(yaw))]
residual = observed_pose_delta - predicted_delta
```

이동 속도 `>1 m/s`, `0.1 ms < dt < 100 ms`인 pair만 사용했다. MR heading convention인
`-1.347 deg`를 적용하고 `map -> morai_map` 변환 뒤 차량 heading 축으로 종·횡을
분해했다.

0.2~1.0초마다 raw pose에 다시 anchor하는 속도 적분 oracle도 보조 확인했다. 이 경우
production 장주행의 종 frame jump p95는 raw oracle의 `7.53 cm`에서
`0.49~1.13 cm`로 감소했지만 횡 frame jump p95는 `1.31~1.32 cm`로 사실상
변하지 않았다. 이는 위 판정과 일치한다. Anchor 주기는 임의성이 있으므로 이
재구성값을 공식 위치 오차로 대체하지 않는다.

## 평가기 변경

`analyze_localization_stability_bag.py`는 기존 metric을 유지해 과거 결과와 호환한다.
대신 `ego_oracle_quality`에 종·횡 kinematic residual을 함께 출력한다. 새 결과를
해석할 때 localization frame jump가 이 oracle-side residual보다 충분히 큰지 먼저
확인한다.

