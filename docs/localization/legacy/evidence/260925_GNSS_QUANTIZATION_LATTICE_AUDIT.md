# GNSS quantization lattice 좌표계 감사

상태: **EVIDENCE**  
평가일: 2026-09-25

## 결론

세 개의 서로 다른 MR bag에서 MORAI GPS 위·경도 최소 격자는 모두 정확히
`1/600000 deg`였다. 이를 현재 production과 같은 WGS84→UTM 52N 변환으로 투영하면
K-City의 GNSS cell은 map x/y 축에 정확히 평행하지 않고 약 `-1.348 deg` 회전한다.

Quantization-aware controller state의 기존 `grid_yaw=0 deg`는 좌표계상 부정확했다.
후보의 기본 격자각을 `-1.347 deg`로 수정했다. 이 값은 MR heading correction과
수치적으로 거의 같지만, 여기서는 차량 heading 보정이 아니라 UTM meridian
convergence로부터 독립적으로 계산한 GNSS cell 축이다.

기능은 여전히 기본 OFF이며 production controller topic은 바뀌지 않는다.

## Raw bag 교차 확인

| bag | valid GPS | lat/lon step | east cell | north cell | east-axis yaw |
|---|---:|---:|---:|---:|---:|
| `260919_231906...bag` | 6,418 | 1/600000° | 0.147889 m | 0.184985 m | -1.347826° |
| `260920_003403...bag` | 1,558 | 1/600000° | 0.147896 m | 0.184985 m | -1.347810° |
| `260822_005713...bag` | 2,636 | 1/600000° | 0.147896 m | 0.184985 m | -1.347892° |

도로 위치가 달라도 cell 폭과 축은 mm 및 `0.0001 deg`보다 작은 범위에서 일치했다.
기존 `0.148 / 0.185 m` 폭은 맞고 회전각만 빠져 있었다.

## 동일 52초 raw replay A/B

각 실행에서 Local/Global EKF부터 1×로 다시 계산했다. ROS callback 위상 때문에
Global 값도 실행 간 수 mm 변하므로 절대 차이보다 각 실행의 Global→후보 감소량을
함께 본다. 단위는 cm다.

| grid yaw | state | 횡 오차 p50 / p95 | 횡 jump p50 / p95 | 종 오차 p50 / p95 | rate |
|---:|---|---:|---:|---:|---:|
| 0° | Global | 1.521 / 5.476 | 0.178 / 1.292 | 1.406 / 5.608 | 49.96 Hz |
| 0° | 후보 | **1.097 / 4.640** | **0.022 / 0.184** | 1.405 / 5.608 | 49.96 Hz |
| -1.347° | Global | 1.534 / 5.440 | 0.178 / 1.294 | 1.396 / 5.498 | 49.99 Hz |
| -1.347° | 후보 | **1.160 / 4.308** | **0.022 / 0.174** | 1.396 / 5.498 | 49.99 Hz |

회전 보정은 큰 새 성능 기법이 아니라 기존 interval model의 좌표계 오류 제거다.
보정 후에도 50 Hz, 종 위치 보존, 횡 jump 감소 계약이 유지됐다. 횡 p95의 추가 개선은
방향상 일치하지만 반복 replay 변동이 있으므로 과장하지 않는다.

## 재현

```bash
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh

python3 scripts/evaluation/localization/analyze_gnss_quantization_lattice.py \
  BAG1 BAG2 BAG3 --output /tmp/gnss_quantization_lattice.json

LOCALIZATION_OUTAGE_CONTROL_MODE=quantized \
LOCALIZATION_OUTAGE_GRID_YAW_DEG=-1.347 \
scripts/evaluation/localization/run_gnss_outage_reacquisition.sh \
  INPUT.bag 26 0 rotated_grid_normal
```

`duration=0`은 GPS를 실제로 제거하지 않고 같은 runner로 정상 구간을 재현하기 위한
audit 용법이다.
