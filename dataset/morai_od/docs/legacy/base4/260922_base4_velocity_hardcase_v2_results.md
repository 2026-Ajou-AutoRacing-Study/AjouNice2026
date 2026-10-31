# Base4 고주로 velocity + hard-case V2 보강 결과

> **문서 상태: CURRENT EVIDENCE / TRAINING-READY PILOT.** 짧은 V1/T1 mechanism
> A/B 전에 기존 28-scene 속도 진단셋에 병목 수렴부와 일반 고속주행 횡 offset을
> 보강한 결과다. Production corpus나 독립 test set은 아니다.

## 결론

- raw bag 입력 `53`개 중 Vehicle-positive temporal scene `51`개, frame `651`개가
  최종 학습셋에 남았다.
- split은 scene 단위 `43 train / 8 val / 0 test`, frame은 `543 / 108 / 0`이다.
- 구조 validator와 temporal quality audit 모두 PASS이며 warning/error가 없다.
- 다음 짧은 V1/T1은 같은 raw LiDAR와 같은 split을 사용한다. Ego deskew A/B까지 한
  실험에 섞지 않는다.

Derived dataset:

```text
artifacts/datasets/morai_base4_velocity_hardcase_v2_raw16
container root: /datasets/base4_velocity_hardcase_v2_raw16
```

## 구성

| 구성 | raw scene | 최종 역할 |
|---|---:|---|
| velocity diagnostic | 28 | 합류·순항 NPC 10--70 km/h 속도축 |
| bottleneck selected | 14 | 수렴 전/목, 협로, 출구, 전방·측방·후측방 전이 |
| general lateral high-speed | 11 | 일반 순항 81.7--118.3 km/h, scene-constant 횡 offset |

병목은 r7 성공 10개와 r8/r10의 실제 협로 4개만 사용했다. r8/r10에서 30 m 범위를
벗어나거나 병목을 지난 scene, r9에서 encounter가 병목 뒤로 밀린 scene은 제외했다.
Converter의 5-point·Vehicle-positive·`+-30 m` 계약을 통과하지 못한 병목 scene 2개는
frame이 하나도 남지 않아 최종 scene 수가 51개가 됐다.

일반 합류와 순항 generator에도 다음 횡 offset을 추가했다.

- 합류: `0, 0, -0.10, +0.10, -0.20, +0.20 m`
- 일반 순항/고속: `0, 0, -0.15, +0.15, -0.30, +0.30 m`
- 병목: 기존 `0, 0, -0.10, +0.10, -0.25, +0.25 m`

offset은 scene마다 하나만 골라 전체 waypoint를 평행 이동한다. 매 frame jitter를 넣지
않으므로 가짜 횡속도나 진동을 학습시키지 않는다. 새 일반 고속 run의 최종 retained
frame은 offset별 `-0.30:28`, `-0.15:18`, `0:46`, `+0.15:14`, `+0.30:28`이다.

## 수집 근거

```text
bottleneck r7 runtime:
  /home/luv/morai_cap/experiments/260922_base4_bottleneck_pilot_r7/runtime
bottleneck selected raw:
  rosbags/datasets/base4_temporal/260922_base4_bottleneck_bn24r7
  rosbags/datasets/base4_temporal/260922_base4_bottleneck_bn12r8_narrow_smoke
  rosbags/datasets/base4_temporal/260922_base4_bottleneck_bn12r10_narrow

general lateral runtime:
  /home/luv/morai_cap/experiments/260922_base4_general_lateral_hs12/runtime
general lateral raw:
  rosbags/datasets/base4_temporal/260922_base4_general_lateral_hs12
```

일반 offset run은 `11/12` scene을 완료했다. 실패 1개는 rosbridge control service 지연 뒤
초기 pose가 틀어져 route-deviation gate가 자동 폐기했다. 성공 bag의 capture window는
LiDAR `12.03--12.95 Hz`, camera 약 `29.4--30.3 Hz`, Ego/Object 약 `47.6--48.9 Hz`였다.
Raw bag 전체 edge에서 camera age `100 ms` 초과가 3/1292 camera-frame에 있었지만,
converter가 선택한 최종 651 frame은 camera age max `31.51 ms`로 모두 통과했다.

## 최종 품질 수치

| 항목 | 값 |
|---|---:|
| Vehicle GT | `651` |
| Vehicle 속도 min/p50/p95/max | `2.78 / 12.67 / 19.43 / 21.40 m/s` |
| BBox-center velocity 일치 p50/p95/max | `0.052 / 0.307 / 1.749 m/s` |
| adjacent frame gap p50/p95/max | `0.153 / 0.181 / 0.264 s` |
| camera age p50/p95/max | `2.11 / 25.95 / 31.51 ms` |
| Ego age p50/p95/max | `8.74 / 11.75 / 23.74 ms` |
| Object age p50/p95/max | `8.91 / 11.85 / 23.74 ms` |
| split leakage / unknown asset / excessive speed | `0 / 0 / 0` |

모든 velocity label source는 geometric BBox center 기준
`morai_object_body_kph_plus_yaw_rate_cross_offset`이다.

## 짧은 V1/T1 A/B

```bash
cd ~/AjouNice2026
set -a
source .env.bevfusion.local
set +a
export BASE4_VELOCITY_HARDCASE_V2_ROOT="$PWD/artifacts/datasets/morai_base4_velocity_hardcase_v2_raw16"

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_velocity_hardcase_v2_v1_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_velocity_hardcase_v2_t1_3sweep_e4.yaml
```

두 학습은 동시에 실행하지 않는다. 두 recipe 모두 dry-run에서 RTX 5060 Ti 기준 physical
batch 4, effective batch 4, E4, LR `2e-5`, 올바른 dataset/checkpoint mount를 확인했다.
V1은 E25, T1은 E25에서 5-channel sparse input으로 변환한 초기 checkpoint를 사용한다.

이 val은 같은 생성기 분포의 mechanism check다. 모델 선택은 기존 고주로 dev bag의
OD/velocity/tracker/Map-CV를 함께 보고, 최종 주장은 별도 controller 주행 held-out bag에서
한다.
