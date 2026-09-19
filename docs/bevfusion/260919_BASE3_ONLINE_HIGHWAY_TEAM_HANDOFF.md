# Base3-Online 고주로 Vehicle 팀 학습 handoff

> **문서 상태: CURRENT.** Base3-Online 팀 학습과 고주로 평가의 실행 기준이다.

## 목표

동일한 rosbag-derived 고주로 Vehicle 데이터만 사용해 다음 두 실험을 비교한다.

1. `online-highway-fresh-e40`: Base3/E25를 사용하지 않고 MORAI 3-class 호환
   nuScenes 초기 가중치에서 새 optimizer로 40 epoch 학습한다.
2. `online-highway-e25-finetune-e12`: production E25 model weight에서 시작하되
   optimizer와 scheduler는 새로 만들어 12 epoch fine-tuning한다.

여기서 `fresh`는 완전 난수 초기화가 아니다. 기존 Base3 학습 정보는 쓰지 않지만
BEVFusion backbone의 안정적인 학습을 위해 팀 공용
`bevfusion_nuscenes_morai3_compatible.pth`를 사용한다. 두 실험 모두 train/val/test는
온라인 고주로 데이터뿐이며 GT sampling도 사용하지 않는다.

## Git 기준

- branch: `perception/base3-online-rosbag-dataset`
- 공용 Docker: `ajounice/bevfusion:team-cu128-v1`
- tracked recipes:
  - `perception/detection3d/bevfusion/configs/team/recipes/online_highway_fresh_e40.yaml`
  - `perception/detection3d/bevfusion/configs/team/recipes/online_highway_e25_finetune_e12.yaml`

2080 Ti와 3090이 같은 YAML을 쓴다. `gpu.profile: auto`가 GPU 이름을 감지해
2080 Ti는 physical batch 2 + accumulation 2, 3090은 batch 4 + accumulation 1로
설정한다. 두 경우 effective batch는 4로 같으며 learning rate도 같다.

## Google Drive 전달물

권장 업로드 폴더:

```text
ajounice2026/model/bevfusion/base3_online_highway_260919/
├── base3_online_highway_raw_rosbag_260919.tar.zst
├── base3_online_highway_motion_v2_dataset_260919.tar.zst
├── README_FIRST.md
└── SHA256SUMS
```

- raw archive: 원본 paired rosbag 7개와 캡처 runtime metadata. 재파싱·감사용이다.
- dataset archive: 바로 학습 가능한 983-frame MMDetection3D dataset. 학습에는
  이것만 필요하다.
- Base3-Static 10K와 E25 weight는 이미 공유된 기존 전달물을 재사용한다.

최종 dataset 계약:

| 항목 | 값 |
|---|---:|
| scene | 334 |
| train / val / test | 692 / 147 / 144 |
| total frame | 983 |
| Vehicle GT (`num_lidar_pts >= 5`) | 1,814 |
| state alignment | LiDAR timestamp pose interpolation |
| fallback extrapolation | bracket가 없을 때만, 최대 0.05 s |
| 학습·평가 범위 | `[-30,-30,-3,30,30,2] m` box range |

이 corpus에는 Vehicle만 있으므로 online val의 3-class `val/center/mAP`는
Pedestrian/Obstacle 0을 포함해 Vehicle metric의 약 1/3로 보인다. 실패가 아니다.
checkpoint 추세는 `val/center/vehicle_mean_AP`, `vehicle_mATE`를 우선 보고, 최종
선정은 production 7-bag으로 한다. 특히 E25 fine-tuning은 Vehicle을 개선하면서
Pedestrian/Obstacle을 망각할 수 있으므로 고주로 두 bag만 보고 production으로
교체하면 안 된다.

## 팀원 PC 최초 준비

저장소와 기존 팀 Docker/E25 전달물을 먼저 준비한다. Ubuntu 20.04/22.04 차이는
Docker 내부 학습에는 영향을 주지 않는다.

```bash
sudo apt update
sudo apt install -y zstd

cd "$HOME/AjouNice2026"
git fetch origin
git switch perception/base3-online-rosbag-dataset
git pull --ff-only
```

dataset archive를 예시 경로에 해제한다.

```bash
mkdir -p "$HOME/ajounice_bevfusion_artifacts/datasets"
tar --use-compress-program=unzstd -xf \
  /다운로드/경로/base3_online_highway_motion_v2_dataset_260919.tar.zst \
  -C "$HOME/ajounice_bevfusion_artifacts/datasets"
```

해제 결과는 다음과 같아야 한다.

```text
$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2/
├── morai_base3_infos_train.pkl
├── morai_base3_infos_val.pkl
├── morai_base3_infos_test.pkl
├── conversion_audit.json
├── validation_audit.json
└── samples/
```

raw archive는 학습에 필수는 아니다. 재파싱할 팀원만 `$HOME`에 해제한다.

```bash
tar --use-compress-program=unzstd -xf \
  /다운로드/경로/base3_online_highway_raw_rosbag_260919.tar.zst \
  -C "$HOME"
```

## W&B 1회 로그인

팀 관리자가 W&B `AjouNice2026` team에 계정을 초대한 뒤 각 PC에서 본인 API key로
한 번 로그인한다. API key는 공유하지 않는다.

```bash
touch "$HOME/.netrc"
chmod 600 "$HOME/.netrc"
docker run --rm -it \
  -v "$HOME/.netrc:/root/.netrc" \
  ajounice/bevfusion:team-cu128-v1 wandb login
```

run name은 `bf-<operator>-<recipe>-<gpu>-<timestamp>`로 자동 생성된다.

## 머신 설정

아래 E25 경로에는 기존에 받은 E25 inference 또는 full checkpoint를 넣을 수 있다.
두 파일의 model state는 동일하고 `finetune`은 optimizer를 복원하지 않는다. full
checkpoint가 있으면 full을 우선한다.

```bash
cd "$HOME/AjouNice2026"

bash perception/detection3d/bevfusion/scripts/configure_team_machine.sh \
  --operator yj \
  --capture-root "$HOME/ajounice_bevfusion_artifacts/base3-static-10k-v1/capture" \
  --info-root "$HOME/ajounice_bevfusion_artifacts/base3-static-10k-v1/infos" \
  --online-highway-root "$HOME/ajounice_bevfusion_artifacts/datasets/base3_online_highway_motion_v2" \
  --nuscenes-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/bevfusion_nuscenes_morai3_compatible.pth" \
  --e25-finetune-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/base3_bf0_f40_e25_inference.pth" \
  --inference-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/base3_bf0_f40_e25_inference.pth"
```

`yj` 대신 PC 사용자에 맞춰 `sw`, `yj`, `jh` 중 하나를 사용한다. 이 명령은
Git 비추적 파일 `.env.bevfusion.local`만 생성·갱신한다.

## 실행 순서

먼저 두 recipe 모두 dry-run으로 mount, checkpoint, GPU profile을 확인한다.

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_fresh_e40.yaml \
  --dry-run

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_e25_finetune_e12.yaml \
  --dry-run
```

각 PC에서 최초 한 번씩 16-sample smoke를 수행한다.

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_fresh_e40.yaml \
  --smoke

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_e25_finetune_e12.yaml \
  --smoke
```

smoke가 끝나면 본 학습을 실행한다. 두 학습을 한 GPU에서 동시에 돌리지 않는다.

```bash
# Experiment A: online rosbag-only fresh
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_fresh_e40.yaml

# Experiment B: production E25 -> online rosbag-only fine-tune
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/online_highway_e25_finetune_e12.yaml
```

결과는 기본적으로
`artifacts/detection3d/bevfusion/team_training/<operator>/` 아래에 저장된다. 모든
epoch checkpoint를 보존하므로 offline best뿐 아니라 고주로 rosbag의 detection,
tracking, Map-CV를 동일 threshold `0.40/0.15/0.10`으로 비교해 production 후보를
선정한다. online-only val 하나로 production 교체를 결정하지 않는다.

### 고주로 rosbag 평가

우선 offline val로 후보 epoch를 줄인 뒤 각 후보 checkpoint에 표준 7-bag suite를
실행한다. 이 suite 안의 `vehicle_highway_merge`와 `vehicle_highway1`이 이번 실험의
1차 비교 대상이다. 서버 시작·healthcheck·종료까지 wrapper가 처리한다.

```bash
cd "$HOME/AjouNice2026"
CKPT="/학습/output/epoch_N.pth"
LABEL="online-highway-$(basename "$CKPT" .pth)"

bash perception/detection3d/bevfusion/scripts/run_checkpoint_rosbag_suite.sh \
  "$CKPT" \
  "artifacts/evaluation/bevfusion/$LABEL"
```

고주로 합류는 1.0x, 고주로1은 기존 검증 계약대로 0.5x 재생된다. 결과는 각
subdirectory의 `summary.json`에 저장된다. Detection만 높고 tracker/Map-CV가
퇴행할 수 있으므로 최종 비교는 다음 순서를 지킨다.

1. `vehicle_highway_merge`, `vehicle_highway1`의 center-distance AP/precision/recall.
2. 같은 detection bag을 tracker에 넣은 speed MAE와 AVE.
3. Map-CV top-1 FDE@1/2/3s와 coverage.
4. 기존 production E25와 동일 bag·동일 threshold로 비교.

원본 학습용 paired bag을 평가에 재사용하지 않는다. 학습·평가 누수를 피하기 위해
위 두 production benchmark bag을 유지한다.

## 확인 및 문제 보고

```bash
nvidia-smi
docker image inspect ajounice/bevfusion:team-cu128-v1 >/dev/null
grep -E '^(BASE3_ONLINE_HIGHWAY_ROOT|BEVFUSION_BASE3_E25_FINETUNE_CHECKPOINT)=' \
  .env.bevfusion.local
```

문제가 생기면 다음 네 가지를 함께 전달한다.

1. `nvidia-smi` 출력
2. 사용한 YAML
3. 해당 run의 `resolved_experiment.yaml`
4. `train.log` 마지막 200줄

W&B API key와 `$HOME/.netrc`는 전달하지 않는다.

## 전달 전 검증 기록

2026-09-19 RTX 5060 Ti에서 다음을 확인했다.

- 두 tracked recipe의 dry-run PASS.
- online dataset의 custom container mount PASS.
- 이미 공유한 inference-only production E25를 `load_from`으로 사용한 16-sample
  AMP train + validation + checkpoint smoke PASS.
- Python/shell/YAML 검사와 Base3-Online alignment/audit 테스트 `10/10` PASS.
- dataset/raw 두 archive의 SHA-256 재검증 PASS.
