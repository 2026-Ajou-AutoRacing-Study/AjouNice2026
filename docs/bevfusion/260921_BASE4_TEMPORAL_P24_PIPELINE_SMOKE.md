# Base4-Temporal p24 파이프라인 스모크 결과

- 상태: **CURRENT / PIPELINE PASS / PERFORMANCE NOT ESTABLISHED**
- 날짜: 2026-09-21
- branch: `dataset/base4-temporal`

## 결론

고주로 Vehicle 24-scene 제한 파일럿으로 rosbag 수집부터 9D velocity GT, causal
3-sweep dataloader, V1/T1 학습과 velocity metric까지 end-to-end 경로를 통과했다.
`23`개 유효 sequence와 수백 frame은 구현 검증에는 충분하지만 모델 선택이나 일반화
성능을 판단하기에는 부족하다. 이 문서의 1-epoch 수치는 정확도가 아니라 실행 가능성의
증거다.

## 수집·변환 결과

| 항목 | 결과 |
|---|---|
| 계획 | merge 6 + post-merge 18, seed `260921101` |
| 실행 | 24 attempted, 23 completed, 1 failed |
| raw bag | `rosbags/datasets/base4_temporal/p24v1/`의 scene bag 23개 |
| runtime | `/home/luv/morai_cap/260921_b4_p24v1` |
| raw dataset | `artifacts/datasets/morai_base4_temporal_highway_p24v1_raw` |
| deskew A/B | `artifacts/datasets/morai_base4_temporal_highway_p24v1_ego_deskew` |
| split | scene 단위 train 19 / val 4 / test 없음 |
| raw frame·GT | train 395 + val 90, Vehicle GT 485 |
| deskew frame·GT | train 383 + val 84, Vehicle GT 467 |

같은 scene set은 다음 명령으로 재생성한다. 생성물 manifest/scenario는 campaign
artifact라 Git에서 제외되며 generator와 seed가 재현의 기준이다.

```bash
python3 dataset/morai_od/scripts/generate_base4_temporal_highway.py \
  --scene-count 24 \
  --seed 260921101 \
  --campaign-tag p24v1 \
  --network-template configs/network/ego_network_mr_minimal_hybrid.json \
  --install-dir ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025
```

Recorder와 runner의 터미널 순서는
[`../../dataset/morai_od/docs/morai_mr_mrp_hybrid_scenario_capture_runbook.md`](../../dataset/morai_od/docs/morai_mr_mrp_hybrid_scenario_capture_runbook.md)를
따른다. 현재 상태와 audit 예외는
[`../../dataset/morai_od/docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md)에
고정한다.

유효 window의 LiDAR는 약 `11.2--12.6 Hz`, 네 camera는 `28.2--30.3 Hz`,
Ego/Object는 약 `44--49 Hz`였다. Camera nearest age는 대부분 p50
`1.67--3.29 ms`, p95 `27--28 ms`였다. Bag edge의 736 LiDAR candidate 중 2개만
100 ms를 넘었고 converter에서 제외됐다.

Raw와 deskew info validator가 모두 PASS했다. Raw audit의 edge error와 Ego header
1 ms 역행 한 건은 원본을 숨기지 않고 audit에 남기되 converter가 strict filtering과
timestamp sort를 수행한다.

## 구현된 데이터 계약

- sample canonical time: current LiDAR header stamp.
- Ego/Object state: canonical time 앞뒤 message를 보간.
- GT box: `[x,y,z,l,w,h,yaw,vx,vy]`, target Ego frame의 `m`, `rad`, `m/s`.
- MORAI Object body velocity: `km/h -> m/s -> world -> target Ego` 순서로 변환.
- temporal input: current + 직전 2개 LiDAR sweep.
- sweep alignment: 과거 Ego pose에서 current Ego pose로 planar transform.
- point feature: `[x,y,z,intensity,dt]`, 과거일수록 양수 `dt`.
- causal prefix: 두 과거 sweep이 없는 scene 첫 2개 sample은 T1 대상에서 제외.
- sweep source: target에 positive Vehicle이 없어 제외되는 중간 frame도 history 후보로
  보존. Target filtering 때문에 시간축이 건너뛰지 않음.
- GT Object motion compensation: 사용하지 않음.
- split: frame이 아니라 scene 단위.

Dataset parser가 generic MMDetection3D 처리 과정에서 sweep path를 `lidar.bin`으로
축약하던 문제를 막았고, 7D/9D box를 모두 지원한다. V1은 single-frame 9D velocity
head, T1은 같은 head에 causal 3-sweep을 사용한다.

초기 구현은 positive target row만으로 history를 만들면서 일부 second-oldest sweep이
최대 `0.583 s` 떨어지는 문제를 보였다. 모든 유효 LiDAR frame을 별도 sweep bank에
남기는 방식으로 고쳤고, 재변환한 raw dataset의 lag는 p50 `0.129 s`, p95
`0.181 s`, max `0.246 s`이며 `0.25 s` 초과가 0개다. Deskew A/B는 invalid scan을
엄격히 버리므로 4개 lag가 `0.25 s`를 약 `4 ms` 초과하며 이 사실을 audit에 남긴다.

## Checkpoint 초기화

V1은 production E25 spatial checkpoint를 사용한다. T1은 sparse-conv 입력 weight를
4 channel에서 5 channel로 확장한 다음 기존 네 channel을 복사하고 새 `dt` channel을
0으로 초기화한다. 새 velocity head는 checkpoint에 없으므로 정상적으로 새로
초기화된다.

```text
artifacts/detection3d/bevfusion/base4_init/base3_e25_to_base4_t1.pth
sha256 971aaa551e10855e27d1d18eb4dcea746ef13530ede0cc1b224087cccb4655d0
```

재생성 명령은 다음과 같다.

```bash
python3 perception/detection3d/bevfusion/tools/adapt_base3_checkpoint_for_base4_t1.py \
  artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth \
  artifacts/detection3d/bevfusion/base4_init/base3_e25_to_base4_t1.pth
```

## 재현 실행

로컬 경로는 Git에 넣지 않고 shell 환경으로 제공한다.

```bash
cd ~/AjouNice2026

export BASE4_P24_RAW_ROOT="$PWD/artifacts/datasets/morai_base4_temporal_highway_p24v1_raw"
export BEVFUSION_BASE3_E25_FINETUNE_CHECKPOINT="$PWD/artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth"
export BEVFUSION_BASE4_T1_INIT_CHECKPOINT="$PWD/artifacts/detection3d/bevfusion/base4_init/base3_e25_to_base4_t1.pth"

# 설정·mount만 확인
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_v1_e4.yaml \
  --dry-run
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_t1_3sweep_e4.yaml \
  --dry-run

# 1 epoch / split당 16 sample smoke
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_v1_e4.yaml \
  --smoke
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_t1_3sweep_e4.yaml \
  --smoke

# 제한 파일럿 4 epoch 개발 실험
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_v1_e4.yaml
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_highway_pilot_t1_3sweep_e4.yaml
```

실제 확인에서는 V1과 T1 모두 1-epoch run이 checkpoint와 val까지 완료됐다. History
gap 수정 후 T1도 train/val 각 16 frame으로 다시 통과했다. T1 loader sample은
`(N,5)` point, `(M,9)` box와 서로 다른 양수 `dt`를 가졌다. T1 smoke의 Vehicle
mAVE/speed MAE는 16-frame·1-epoch random head 결과이므로 성능표에 사용하지 않는다.

5060 Ti에서 MORAI가 약 `3.7 GiB` VRAM을 점유한 상태로 팀 기본 batch 4를 동시에
실행하면 OOM이 재현됐다. 같은 상태에서 검증용 batch 1 + accumulation 4는 peak
약 `3.2 GiB`로 PASS했다. 이는 dataset 오류가 아니라 동시 실행 자원 문제다. 실제
학습은 MORAI를 끄고 team auto profile을 사용하며, 시뮬레이터와 병행해야 할 때만 local
recipe에서 physical batch를 낮춘다.

## 평가 rosbag 분류

속도는 `/Ego_topic.velocity`를 `m/s`로 해석해 km/h로 변환했다. 고주로 영역은 현재
감사용 map box `x=35--85`, `y=-430--330`으로 잘랐다. `near30`은 Object topic의
객체가 Ego 30 m 안에 있는 Ego sample 비율이다.

| bag | 최대 Ego 속도 | 고주로 구간 | 고주로 내 60 km/h 이상 | 고주로 near30 | 역할 |
|---|---:|---:|---:|---:|---|
| `260821_235409_highway_merge_dynamic_competition_raw.bag` | 118.1 | 100% | 62.7% | 56.7% | 반복 사용된 merge dev |
| `sensor_cam_raw_highway1.bag` | 252.6 | 100% | 88.8% | 67.2% | 비현실적 최고속 포함 legacy stress/dev |
| `planning_rviz_0915_gt_local.bag` | 67.3 | 24.9% | 27.0% | 76.6% | 현장 GT closed-loop 회귀 |
| `planning_rviz_0915_sensor_local_1.bag` | 67.6 | 18.9% | 40.3% | 70.3% | 현장 localization 회귀 |
| `planning_rviz_0915_sensor_local_2.bag` | 67.5 | 18.4% | 36.4% | 55.7% | 현장 localization 회귀 |
| `planning_rviz_0915_sensor_local_3.bag` | 67.5 | 19.2% | 39.0% | 65.6% | 현장 localization 회귀 |

현장점검 네 bag은 실제 LAN/full-stack/control domain이라는 가치가 크지만 약 67 km/h가
상한이므로 100 km/h 추월 성능 test가 아니다. 기존 merge bag은 118 km/h를 포함하지만
이미 반복적으로 epoch와 parameter 선택에 사용했다. `highway1`의 252.6 km/h는 대회
분포 대표값이 아니다. 따라서 당장 V1/T1 비교에는 기존 두 고주로 bag과 현장 bag을
development/regression으로 쓰되, 최종 후보에는 다른 seed와 실제 controller로 주행한
새 80--120 km/h held-out 고주로 bag이 필요하다.

## 다음 gate

1. 4-epoch V1/T1 결과는 구현·학습 안정성과 방향성만 본다.
2. OD center AP/mATE와 Vehicle speed MAE/2D AVE를 함께 확인한다.
3. detector velocity off/on을 같은 tracker 조건으로 비교하고 tracking AVE와 Map-CV
   FDE@1/2/3s를 낸다.
4. T1이 V1보다 geometry를 망치면 sweep ghosting을 확인하고 T2 BEV memory로 넘어갈지
   판단한다.
5. 분포는 실패 slice를 확인한 뒤 수정하며, 지금 23 sequence를 복제해 수량을
   부풀리지 않는다.
6. 100 km/h급 새 held-out control bag 전에는 production 교체 결론을 내리지 않는다.
