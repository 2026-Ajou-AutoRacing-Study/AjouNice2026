# Human-driven MR evaluation scenarios

이 폴더는 detector를 실행하지 않고 raw sensor rosbag을 사람이 직접 주행해 녹화하기 위한
고정 시나리오 묶음이다. 설치·녹화 명령은
[`dataset/morai_od/docs/260923_HUMAN_DRIVING_ROSBAG_CAPTURE_HANDOFF.md`](../../../dataset/morai_od/docs/260923_HUMAN_DRIVING_ROSBAG_CAPTURE_HANDOFF.md)를 따른다.

## 구성

| 파일 | 목적 |
|---|---|
| `ajounice_eval_highway_balanced.json` | 고주로 45/60/75 km/h NPC profile |
| `ajounice_eval_highway_slow_lead.json` | 느린 선행차가 있는 35/50/65 km/h profile |
| `ajounice_eval_highway_mixed_overtake.json` | 차선별 순서가 섞인 65/40/80 km/h profile |
| `ajounice_eval_highway_fast_surround.json` | 빠른 주변차 55/75/85 km/h profile |
| `ajounice_eval_roundabout_{slow,nominal,fast}.json` | 15/20/25 km/h 고정 회전교차로 NPC |
| `ajounice_eval_pedestrian_crossing.json` | 공식 sample의 보행자와 PedPath만 남긴 횡단 장면 |
| `ajounice_eval_obstacle_scale1_six_assets.json` | scale 1 장애물 6종 주행 관측 |

고주로 속도는 본선 정답을 가정한 값이 아니다. MORAI 공식 답변상 본선 속도는 sample과
동일하지 않고 비공개이므로, 서로 다른 상대속도와 추월 상황을 재현하기 위한 고정 평가
profile이다. 실행 시 랜덤화하지 않는 이유는 bag의 조건을 나중에 정확히 재현하기 위해서다.

장애물 6종은 `CargoBox`, `PE_Drum`, `TrafficBarrel`, `concrete-barrier`,
`Concrete_Barricade01`, `PE_Firewall_Orange`이며 모두 scale 1이다. `WoodBox`는 공식 GT
계약 확인 전까지 정식 평가에서 제외한다.

모든 시나리오는 다음 상태로 시작한다.

- Ego Keyboard mode
- Ego speed 0
- gear P
- 사람이 gear를 D로 바꾼 뒤 직접 운전

JSON은 다음 명령으로 재생성할 수 있다.

```bash
python3 dataset/morai_od/scripts/build_human_eval_capture_scenarios.py
```
