# Base3 분포 공백과 Base5 E25-init E2 pilot 결과

상태: **EVIDENCE + PLAN**, 2026-10-05, `dataset/base5`.
Production은 B3-BF0-F40 E25 / Vehicle·Pedestrian·Obstacle `0.40/0.15/0.10` 그대로다.
초기 확인은 학습 로그·기존 평가·학습 annotation 분석이다. 이후 E1 보행자·장애물9-bag A/B를 완료했다. [최신 rosbag 결과](261005_BASE5_E1_MISSION_ROSBAG_AB.md)가 판단 기준이다. 장애물 소폭 개선이지만 공백 미해결, production E25 유지다. 고주로·로터리와 E2는 시간 제약으로 이번 평가에서 제외했다.

## 현재 rosbag 검증 범위

- 기존 Pedestrian 2 + Obstacle 1, 새 Pedestrian 3 + Obstacle 3: **E1 9-bag**.
- baseline은 기존/current-contract E25 산출물이다. tracker·map·catalog 해시 일치를 확인했다.
- 재현: `python3 perception/detection3d/bevfusion/scripts/run_base5_checkpoint_rosbag_ab.py --models e1 --missions pedestrian obstacle`.
- 집계: `python3 perception/detection3d/bevfusion/scripts/summarize_base5_rosbag_ab.py`.
- 출력: `artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005/`. `runtime_manifest.json`은 축소된 실행 범위, `comparison.json`의 status/pending은 완료 여부를 나타낸다.
- 전체 E1/E2 44회 계획은 중단했다. 이미 끝난 E1 검출·pipeline 결과는 재사용하며 production을 변경하지 않는다.

## 결론

- 축소9-bag은 완료했고 평가용 inference를 종료했다. E2·차량 미션은 미실행이다.
- Base3의 허점은 실제 annotation 분포에서도 확인된다. 원본 학습 sample에는 2m 미만 Pedestrian/Obstacle GT가 없고, 20–30m Pedestrian은 2.81%뿐이다.
- 새 bag의 보행자 누락은 주로 20–30m, 장애물 누락은 근접과 원거리 양쪽이다. 후방을 제거해서 해결할 문제가 아니다.
- Base5 복구 객체 E2 학습은 정상 완료했다. **오프라인 best는 E1**이며 Center mAP 96.83%다. 다만 같은 val에서의 E25 시작점 평가가 없으므로 개선량은 아직 알 수 없다.
- 이번 addon은 8/12m 정지 객체 복구 실험이다. 0–2m 실제 통과·부분 가시성이나 20–30m 보행자 보강 완료가 아니다. 높은 val 점수로 이 두 문제의 해결을 주장하지 않는다.
- E1 동일 runtime9-bag A/B까지 완료했다. 다음은 **실패 구간 표적 보강 → rosbag A/B → 고해상도 별도 A/B**다. epoch 증가나 production 교체부터 하지 않는다.

## 1. Base3 실제 학습 GT 감사

입력은 `artifacts/datasets/morai_base3_bevfusion_v1/morai_base3_infos_train.pkl`, 7,632 frame이다.
SHA256: `b31a0f61d1aca94f1c76c78fbfd3792c14313c301204f3ffd3040bdbf18da8c6`.
E25 학습 run `bf-sw-fresh-e40-rtx5060ti-260918-005536/mmengine_config_input.py`의 dataset 계약을 확인했다.

집계는 augmentation **전** retained GT, XY ±30m box 범위 안에서 Ego 중심 반경 ≤30m다.
LiDAR→Ego translation `[0.855, 0, 1.388]` 중 XY를 적용했다. ObjectRangeFilter는 BEV XY 기준이므로 label에 별도 Z 필터를 추가하지 않았다.
숫자는 반복 관측을 포함한 annotation 수이며 독립 scene/asset 개수가 아니다.

| Ego 거리 | Pedestrian GT | 비율 | Obstacle GT | 비율 |
|---|---:|---:|---:|---:|
| 0–2m | 0 | 0% | 0 | 0% |
| 2–5m | 976 | 21.24% | 456 | 4.29% |
| 5–10m | 1,365 | 29.71% | 3,169 | 29.80% |
| 10–20m | 2,125 | 46.25% | 5,944 | 55.90% |
| 20–30m | 129 | 2.81% | 1,065 | 10.02% |
| 합계 | 4,595 | 100% | 10,634 | 100% |

- 최소 원본 거리: Pedestrian 2.208m, Obstacle 2.710m.
- Obstacle의 85.70%가 5–20m다. 실제 회피 중 옆면만 보이거나 화면을 벗어나는 appearance는 위 숫자만으로 충분하다고 볼 수 없다.
- Pedestrian LiDAR point 중앙값은 2–5/5–10/10–20/20–30m에서 각각 132/49/15/**7개**다.
- Obstacle은 같은 구간 96/48/19/12개다.
- 전방 ±45°/좌/우/후방 구분: Pedestrian 3,425/436/585/149, Obstacle 7,658/894/946/1,136.
- 동적 보행자 addon도 포함돼 있다. `batch008_dynamic_ped` 및 replacement의 train frame은 630개다. 따라서 문제를 '정적 데이터만 학습했기 때문'이라고 단정하지 않는다.
- scale 0.95–1.05, rotation ±0.10rad, translation std 0.2m augmentation은 존재한다. 다만 작은 좌표 perturbation은 진짜 초근접 영상·부분 가시성을 만들어 주지 않는다.

### 5-point gate의 감독 공백

`perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py`는 GT box 내부 point가 5개 미만이면 해당 instance를 annotation에서 제거한다. ignore-region을 만드는 방식은 아니다.
따라서 카메라에 보이는 sparse 객체가 positive label에서 빠질 가능성이 있다. 이는 가능한 학습 메커니즘이며, 전체 성능 하락 중 몇 %가 이것 때문인지는 아직 검증하지 않았다.

기존 `conversion_audit.json`의 모든 split 합계 sparse 제외는 Vehicle 98 / Pedestrian 219 / Obstacle 1,206개다. train-only 또는 전부 원거리라는 뜻은 아니다. raw rejected GT의 거리·가시성별 재감사가 필요하다.
5-point를 전역 0으로 변경하지 않는다. 보이는 객체의 sample+UID review positive와 완전 가림 객체를 구분한다. 이번 WoodBox 8m 6장 whitelist가 제한된 선례다.

## 2. 새 미션 bag이 드러낸 공백

원본은 [동일 계약 기존/새 bag 비교](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md)다. 전체 360° / 반경30m이며 후방 GT를 제거하지 않았다.
아래 recall은 전체에서 confidence 순서로 1m matching한 뒤 GT 거리로 분류한 값이다. 거리별로 독립 재매칭한 AP와 혼용하지 않는다.

| 새 bag / 거리 | GT / TP | Recall |
|---|---:|---:|
| Pedestrian 0–10m | 149 / 148 | 99.33% |
| Pedestrian 10–20m | 107 / 105 | 98.13% |
| Pedestrian 20–30m | 88 / 15 | 17.05% |
| Obstacle 0–10m | 749 / 464 | 61.95% |
| Obstacle 10–20m | 743 / 716 | 96.37% |
| Obstacle 20–30m | 775 / 450 | 58.06% |

Pedestrian FN 76개 중 73개(96.1%)가 20–30m다. 새 bag에서는 해당 거리 GT 비중이 25.6%로 Base3 train 2.81%보다 훨씬 높다.
Obstacle FN 637개 중 근접 285 / 중간 27 / 원거리 325다. 새 Obstacle rear-half recall 75.38%는 front-half 68.85%보다 높다. 후방 제외가 핵심 해결책이 아니다.
학습 공백과 실패 분포의 대응은 강한 단서지만, GT 기준·partial view·asset 구성·camera 해상도 각 요소의 독립 기여는 아직 분리되지 않았다.

## 3. 방금 완료된 학습

Run: `bf-sw-base5-nearfield-replay-e2-pilot-rtx5060ti-261004-214609`.
Output: `artifacts/detection3d/bevfusion/team_training/sw/` 아래 동일 run 이름.
재현 근거: `resolved_experiment.yaml`, `experiment_input.yaml`, `source_snapshot/`, `train.log`, `20261004_124616/vis_data/scalars.json`.
[W&B run](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/bf-sw-base5-nearfield-replay-e2-pilot-rtx5060ti-261004-214609): 최종 sync 메시지 확인.

- E25 initialize, spatial 7D / raw cloud / deskew OFF, 입력 256×704, 기존 ±30m 계약.
- canonical dataset `artifacts/datasets/base5_nearfield_restored_reviewed_replay_pilot_261004`: train727 / val124 / test0.
- Base3 replay train600/val100 + 이전 소량 diversity + 복구 5asset addon. addon의 Ego/객체는 정지, 8/12m × yaw0/90°, scale1이다.
- batch4, accum1, LR2e-5, warmup25 update, 2epoch ×182iteration, validation 매 epoch.
- 로그 시각 12:46:16 시작→12:50:39 최종 val: 약4분23초(컨테이너 로그 시간 기준). 학습 step은 이후 약0.52–0.53초다. 실시간 추론 latency가 아니다.
- 18개 logged train window의 loss/gradient 등 수치 finite. 최종 E2 checkpoint와 W&B sync 확인, traceback/OOM 없이 종료.
- logged-window loss 평균 E1 0.2816 → E2 0.2280. epoch 전체 sample 가중 평균은 아니다. 마지막 logged loss 0.2732 → 0.1993, LR 1.075e-5 → 1.003e-6.

| Offline val 지표 | E1 | E2 |
|---|---:|---:|
| Vehicle Center AP | 95.19% | 94.99% |
| Pedestrian Center AP | 98.89% | 98.89% |
| Obstacle Center AP | 96.42% | 96.46% |
| 3-class Center mAP | **96.831%** | 96.778% |
| 3-class IoU AP@0.50 | **91.05%** | 90.58% |
| Vehicle matched center mATE | 0.0907m | 0.0897m |
| Pedestrian matched center mATE | 0.0445m | 0.0439m |
| Obstacle matched center mATE | 0.0531m | 0.0530m |

Center AP는 0.5/1/2/4m 평균이다. val GT는 Vehicle69 / Pedestrian44 / Obstacle145개다.
E1 `best_val_center_mAP_epoch_1.pth`를 첫 runtime 후보로 삼되 E2도 비교용으로 보존한다. 자동 best는 최종 E2가 아니라 E1이다.

### 점수가 높아도 해결됐다고 할 수 없는 이유

1. E25 baseline을 **동일 124-frame val**에서 별도 측정한 결과가 없다. 다른 dataset의 E25 val과 비교해 fine-tuning 개선량을 주장하지 않는다.
2. scene-disjoint지만 인접한 정지 layout이다. 실주행 held-out test가 아니다. 20–30m Pedestrian GT는 **0개**로, 로그 AP=0은 이 구간의 실패 측정값이 아니라 비어 있는 평가다.
3. val의 Obstacle 20–30m GT도 13개뿐이다. 근접 0–10m 높은 AP를 0–2m 회피 성능으로 일반화하지 않는다.
4. loss는 줄지만 E2 Center mAP/IoU AP는 소폭 하락했다. plateau/좁은 분포 적응 신호일 수 있으나 2epoch만으로 과적합을 확정하지 않는다.
5. Center AP만으로 크기·yaw를 보장하지 않는다. E1 Obstacle mAOE 약1.064rad, Pedestrian mASE 약0.183도 남아 있다. 대칭 물체의 yaw 모호성을 감안하되 회피용 footprint 안정성을 별도 확인한다.
6. val의 `best_F1`/`operating` score는 score sweep 결과다. production threshold 고정 실주행 F1로 읽지 않는다. 수만 개의 낮은 score prediction count 역시 그대로 production FP 개수가 아니다.
7. 이번 CargoBox 새 MR reference와 역사적 reference의 size 차이는 미해결이다. 서로 다른 출처의 GT를 근거 없이 덮어쓰지 않았다.

## 4. 다음 실행 순서와 판정

### A. 현재 후보부터 검증 — 추가 수집 전에

- 같은 pilot val에서 E25 baseline을 추가해 E25→E1/E2 class별 변화량을 기록한다.
- frozen runtime으로 E25/E1/E2를 기존·새 Pedestrian/Obstacle bag에 비교한다. 저장된 E25 결과는 contract/hash가 같을 때만 재사용한다. 신규 결과는 `artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005/{e25,e1,e2}/`에 독립 저장하고 기존 결과를 덮어쓰지 않는다.
- threshold 0.40/0.15/0.10, NMS IoU0.5, deskew OFF, single-frame, detector velocity OFF, 동일 tracker binary/파라미터·map v85·bag별 heading/catalog를 고정한다. 각 source에 맞는 GT reference를 보존한다.
- Center AP 외에 0–2/2–5/5–10/10–20/20–30m recall·precision, 방향/asset별 누락, duplicate, ID 변경, box center/폭 변동, Tracking F1, Map-CV FDE를 함께 기록한다. 0–2m GT가 없으면 N/A다.
- 후보가 근접 성능을 개선하면 고주로·로터리 회귀까지 확인한다. 개선 없이 기존 class가 악화하면 E25를 유지하고 더 오래 학습하지 않는다. 반복 사용된 bag은 dev이며 최종 held-out이라는 이름을 붙이지 않는다.
- 위 전체 A는 최초 **계획**이다. 이후 E1 보행자·장애물9-bag만 실행 완료했으며, 그 결과에 따라 production은 E25를 유지한다. E2·차량 미션과 동일124-frame E25 baseline val은 실행하지 않았다.

### B. 이번 addon이 메우지 못한 분포를 표적 보강

- 근접 장애물: Ego가 실제 통과하며 0.5/1/2/3/5m 거리, 좌우 옆면·얇은 방향·부분 시야를 얻는다. 목표 거리와 실제 sensor/GT 거리의 차이를 audit하고 충돌/비정상 센서 구간을 제외한다.
- 원거리 Pedestrian: 20/25/30m, 앞·옆·뒤 방향과 다양한 인물 asset. camera-visible sparse GT는 sample 단위 검토한다. 보이지 않는 GT를 전부 positive로 복구하지 않는다.
- 원거리 Obstacle 및 Vehicle은 별도 quota/manifest로 관리한다. 30m+ 차량은 현 ±30m 학습과 평가 계약에서 별도 ROI 확장 실험으로 분리한다.
- Base3 replay는 유지한다. 현재600장만으로 전역 망각 방지를 보장하지 않으므로 전체 Base3 및 고주로/로터리 회귀를 확인하고 필요한 replay 양을 조정한다.
- 분포 개선 전후 retained/rejected GT를 class×거리×방향×asset×가시성으로 집계한다. scene/sequence 단위 split을 유지한다.

### C. 고해상도는 독립 실험

- 256×704 vs 320×896을 동일 dataset/split/epoch/LR/GT/ROI로 비교한다. 60m ROI나 sparse-label 변경을 동시에 넣지 않는다.
- 원거리 작은 객체에 도움을 줄 가능성은 있지만 현재 정확도 개선은 미측정이다. 5060 Ti forward smoke와 4090 실시간 처리 성능을 구분한다.
- 새 모델의 offline/미션 전체 회귀 및 실제 throughput 확인 전 production checkpoint/alias를 변경하지 않는다.

## 연결 문서

- [수집·reviewed replay 입력](../../dataset/morai_od/docs/261004_base5_restored_replay_results.md)
- [기존/새 E25 동일 계약 비교](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md)
- [근접·원거리·승격·고해상도 pilot](261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md)
