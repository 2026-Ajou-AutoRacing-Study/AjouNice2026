# Base4 고주로 1차 실험 dataset

> **문서 상태: CURRENT / TRAINING READY.** 고주로 Vehicle velocity와 causal temporal
> modeling을 처음으로 비교할 수 있을 만큼 속도·상대 위치·차선·차종을 넓힌 1차
> 실험용 dataset이다. Production corpus나 독립 test set은 아니다.

## 결론

- raw dataset은 `159 scene / 2,028 frame`, scan-end Ego deskew twin은
  `157 scene / 1,887 frame`이다.
- 두 dataset 모두 info validator와 temporal quality audit를 통과했다.
- raw와 deskew는 같은 `161`개 입력 bag과 같은 변환 계약에서 만들었다. Deskew 때문에
  5-point Vehicle-positive 조건을 잃은 frame만 deskew 쪽에서 추가로 제외됐다.
- 학습 기본 비교는 `raw V1`, `raw T1`, `deskew V1`, `deskew T1`의 2x2다. Deskew 결과가 실제 고주로
  rosbag의 velocity/tracking/Map-CV까지 개선하기 전에는 production 기본값으로 올리지 않는다.

## 최종 경로

```text
raw:
  artifacts/datasets/morai_base4_highway_first_experiment_v1_raw
  /datasets/base4_highway_first_experiment_v1_raw

ego deskew:
  artifacts/datasets/morai_base4_highway_first_experiment_v1_ego_deskew
  /datasets/base4_highway_first_experiment_v1_ego_deskew
```

## 구성

| campaign | 최종 raw scene | 역할 |
|---|---:|---|
| velocity/hard-case V2 | `51` | 합류 10--70 km/h, 병목, 일반 횡 offset |
| high-speed continuous | `48` | Ego 80--120 km/h, NPC 25--80 km/h 연속속도 |
| competition-balanced `cb80r1` | `60` | 합류와 순항, 네 상대운동 역할, 좌/우, 다중 차선·offset |

`cb80r1`은 80개를 계획해 전부 시도했으며 `60`개가 안전 gate를 통과했다. 실패 20개는
route deviation, actor gate 이탈 또는 최대 주행거리 초과로 자동 폐기했다. 완료 subset은
다음 분포를 유지한다.

- 합류 `16`, post-merge `44` scene.
- planned Ego `50.0--118.5 km/h`, NPC `10--80 km/h`.
- `front_following 7`, `front_overtake 11`, `rear_receding 13`,
  `side_passby 13`, merge interaction `16` scene.
- adjacent actor left/right 각 `13` scene.
- Ego 횡 offset `-0.30, -0.15, 0, +0.15, +0.30 m` 모두 포함.

최종 raw label 기준 실제 분포는 다음과 같다.

| 항목 | 값 |
|---|---:|
| 차량 asset | `35`종 |
| Ego speed min / p05 / p50 / p95 / max | `8.63 / 13.13 / 23.03 / 30.40 / 33.37 m/s` |
| NPC speed min / p05 / p50 / p95 / max | `2.72 / 5.55 / 13.90 / 21.36 / 22.25 m/s` |
| front / rear GT | `893 / 1,135` |
| left / right / same-near GT | `676 / 1,144 / 208` |

좌우 planned quota는 새 campaign에서 대칭이지만 이전 파일럿까지 합친 frame-level 결과는
right가 더 많다. 첫 mechanism 실험에는 충분한 양쪽 표본이 있으나, 좌측 진입 오차가
실제 held-out에서 보이면 다음 수집은 좌측과 same-lane front-following만 targeted refill한다.

## 변환 계약

- MR Ego heading correction `-1.347 deg`.
- LiDAR 시각 Ego/Object pose interpolation.
- geometric BBox center velocity와 회전 중심속도 보정.
- causal sweep `3`, positive past age feature.
- Vehicle `minimum_gt_points=5`, `+-30 m` Cartesian box range.
- scene 단위 train/val, test split 없음.
- camera nearest age `<=100 ms`; 최종 raw max `36.15 ms`, deskew max `42.64 ms`.

Raw temporal audit:

- train/val `1,688 / 340 frame`, `132 / 27 effective scene`.
- velocity consistency p50/p95 `0.046 / 0.275 m/s`.
- NPC velocity p05/p50/p95 `5.554 / 13.896 / 21.356 m/s`.
- split leakage, unknown asset, excessive velocity, audit error 모두 `0`.

Deskew temporal audit:

- train/val `1,580 / 307 frame`, `130 / 27 effective scene`.
- velocity consistency p50/p95 `0.047 / 0.278 m/s`.
- scan-end SE(2) deskew CPU mean/p95 `1.53 / 1.76 ms`.
- point displacement p95의 scene mean/p95 `1.83 / 2.93 m`.
- split leakage, unknown asset, excessive velocity, audit error 모두 `0`.

## 재현

새 accepted bag directory를 추가한 뒤 다음 도구가 과거 두 campaign과 합쳐 raw/deskew를
같은 입력 목록으로 생성한다. Converter는 campaign 디렉터리와 무관하게 rosbag 실제 시작
시각으로 정렬하고, `capture_end`가 아닌 실패 marker window는 학습에서 제외한다.

```bash
python3 dataset/morai_od/scripts/build_base4_highway_first_experiment.py \
  --new-bag-dir rosbags/datasets/base4_temporal/260922_base4_cb80r1_full \
  --calibration /home/luv/morai_cap/experiments/260922_base4_cb80r1/full_runtime/dynamic_metadata/calibration.json
```

## 짧은 학습 A/B

```bash
export BASE4_HIGHWAY_FIRST_EXP_RAW_ROOT="$PWD/artifacts/datasets/morai_base4_highway_first_experiment_v1_raw"
export BASE4_HIGHWAY_FIRST_EXP_DESKEW_ROOT="$PWD/artifacts/datasets/morai_base4_highway_first_experiment_v1_ego_deskew"

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_first_exp_v1_raw_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_first_exp_t1_raw_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_first_exp_v1_deskew_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_first_exp_t1_deskew_e4.yaml
```

네 학습은 동시에 실행하지 않는다. 생성기 내부 val은 mechanism 확인용이고, 모델 선택은
기존 고주로 개발 bag과 새 controller 주행 held-out bag의 OD velocity, tracker AVE,
Map-CV FDE를 함께 사용한다.
