# BEVFusion 팀 공용 Docker 학습·추론 handoff

## 목적과 기준

이 구성은 Base3-Static 10K로 MORAI 3-class BEVFusion을 여러 PC에서
독립적으로 학습하고 동일 checkpoint를 ROS online inference에 사용하는
팀 표준이다.

| GPU | CUDA arch | physical batch | accumulation | effective batch |
|---|---:|---:|---:|---:|
| RTX 2080 Ti | 7.5 | 2 | 2 | 4 |
| RTX 3090 | 8.6 | 4 | 1 | 4 |
| RTX 4090 | 8.9 | 4 | 1 | 4 |
| RTX 5060 Ti | 12.0 | 4 | 1 | 4 |

공용 image는 CUDA 12.8, PyTorch 2.7.1과
`7.5;8.6;8.9;12.0+PTX` CUDA extension을 포함한다. GTX 1060은
지원 대상에서 제외한다. Host Conda·PyTorch·CUDA Toolkit은 사용하지
않는다.

## 전달 파일

공유 Google Drive `ajounice2026/model/bevfusion/team_docker_v1/`:

```text
ajounice-bevfusion-team-cu128-v1.image.tar.zst
bevfusion-team-checkpoints-v1.tar.zst
base3-static-10k-v1.tar.zst
base3-smoke-subset-v1.tar.zst
README_FIRST.md
SHA256SUMS
ARTIFACT_MANIFEST.txt
```

Base3·checkpoint는 Docker image와 분리한다.

## PC 최초 설정

Native Ubuntu에 NVIDIA driver, Docker Engine, NVIDIA Container Toolkit, Git,
`zstd`가 필요하다.

```bash
sudo apt update
sudo apt install -y zstd
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

마지막 명령이 실패하면 학습 전 driver·Container Toolkit을 수정한다.

## 압축 해제와 image load

```bash
mkdir -p "$HOME/ajounice_bevfusion_artifacts"
cd "$HOME/ajounice_bevfusion_artifacts"

(cd /path/to/team_docker_v1 && sha256sum -c SHA256SUMS)
tar --use-compress-program=unzstd -xf \
  /path/to/team_docker_v1/bevfusion-team-checkpoints-v1.tar.zst
tar --use-compress-program=unzstd -xf \
  /path/to/team_docker_v1/base3-smoke-subset-v1.tar.zst
tar --use-compress-program=unzstd -xf \
  /path/to/team_docker_v1/base3-static-10k-v1.tar.zst
zstd -dc /path/to/team_docker_v1/ajounice-bevfusion-team-cu128-v1.image.tar.zst \
  | docker load
```

```bash
cd "$HOME/AjouNice2026"
BEVFUSION_IMAGE=ajounice/bevfusion:team-cu128-v1 \
  bash docker/bevfusion/scripts/verify_team_image.sh
```

## W&B

팀 관리자가 W&B `AjouNice2026` team에 회원을 초대한 뒤, 각 PC에서
본인 계정으로 1회 로그인한다. API key를 공유하지 않는다.

```bash
touch "$HOME/.netrc"
chmod 600 "$HOME/.netrc"
docker run --rm -it \
  -v "$HOME/.netrc:/root/.netrc" \
  ajounice/bevfusion:team-cu128-v1 wandb login
```

Project는 `AjouNice2026/morai-3d-object-detection`이다. Run name은
`bf-<operator>-<experiment>-<gpu>-<YYMMDD-HHMMSS>`로 자동 생성된다.

## 팀원별 local 설정

사용자 ID는 `sw`, `yj`, `jh` 중 하나다.

```bash
cd "$HOME/AjouNice2026"

bash perception/detection3d/bevfusion/scripts/configure_team_machine.sh \
  --operator yj \
  --capture-root "$HOME/ajounice_bevfusion_artifacts/base3-static-10k-v1/capture" \
  --info-root "$HOME/ajounice_bevfusion_artifacts/base3-static-10k-v1/infos" \
  --nuscenes-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/bevfusion_nuscenes_morai3_compatible.pth" \
  --e10-full-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/base3_bf0_epoch10_full.pth" \
  --inference-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/base3_bf0_e10_inference.pth"
```

이 명령은 Git에 올라가지 않는 `.env.bevfusion.local`과
`configs/team/experiments/<operator>/*.yaml`을 만든다. API key를 넣지
않는다.

## Smoke test

먼저 smoke subset 경로로 `configure_team_machine.sh`를 실행한 뒤:

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  fresh-e12 --dry-run
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  fresh-e12 --smoke
```

Smoke PASS 후 전체 Base3 경로로 local 설정을 다시 생성한다.

## 본 학습

Fresh는 MORAI 3-class로 변환한 nuScenes weight에서 새 optimizer·scheduler로
시작하며 첫 기준 실험으로 권장한다.

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh fresh-e12
```

Fine-tuning은 Base3 E10 weight를 불러오고 optimizer·scheduler를 새로 시작한다.

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh finetune-e6
```

Exact resume는 local YAML을 `mode: resume`로 바꾸고 full checkpoint를 지정한다.
Model·optimizer·scheduler·epoch 상태를 모두 계속하므로 같은 실험일 때만
사용한다. 이때 YAML의 `epochs`는 추가 epoch 수가 아니라 최종 종료 epoch다.
예를 들어 epoch 10 checkpoint를 epoch 18까지 이으려면 `epochs: 18`로 둔다.
`random-e24`는 strict random-init 연구 control이지 기본
선택이 아니다.

## 재현성과 branch 전환

학습 시작 순간 BEVFusion 코드를 output에 snapshot하고 container에
read-only mount한다. 학습 시작 후 host repository branch를 바꾸어도 실행
중인 코드는 변하지 않는다.

```text
experiment_input.yaml
resolved_experiment.yaml
mmengine_config_input.py
run_manifest.json
command.txt
git_status.txt
source_changes.patch
source_snapshot/
train.log
```

## Online inference

추론은 GPU preset과 무관하게 batch 1이다. ROS master와 UDP gateway 실행 후:

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
bevfusion
```

`bevfusion`은 Docker server·healthcheck·ROS bridge까지 실행하고
`/perception/object_recognition/detection/objects`를 발행한다. Tracking·Map-CV·
planning·control은 별도 터미널이다. 기본 threshold는
Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`이다.

서버는 ready 상태를 내기 전에 4-camera·LiDAR 실제 입력 형상의 CUDA
warm-up을 끝낸다. 따라서 컨테이너를 갓 시작한 직후 첫 센서 프레임에서
수 초간 멈추며 프레임을 잃는 문제가 없어야 한다. 모델 로드와 warm-up을
포함한 시작 시간은 5060 Ti에서 약 20초였으므로 healthcheck가 성공하기 전에
ROS bridge만 먼저 실행하지 않는다.

5060 Ti 검증치는 약 `12.5–12.8 Hz`, end-to-end `78–84 ms`
p50/p95였다. 4090에서는 output Hz, latency p50/p95, max gap, drop
count를 다시 측정해 15 Hz 운용을 승인한다.

## 2026-09-16 호환성 검증 기록

- 이미지 내부 CUDA extension: `sm_75`, `sm_86`, `sm_89`, `sm_120` native
  cubin 및 `sm_120` PTX 확인.
- RTX 5060 Ti: fresh initialization과 기존 Base3 E10 full checkpoint 모두
  physical batch 4로 1 epoch train + validation + W&B sync 완료.
- 기존 production E10 inference checkpoint SHA-256:
  `b2d3f65befaa16382cdd189f9c8ac1aff2d2176e6c48e19a17ab329947eff419`.
- Base3 multi-asset obstacle rosbag: cold-ready 직후 frame coverage `1.000`,
  obstacle center-distance mAP `0.6067`, 1 m F1 `0.7923`.
- 고주로 합류 rosbag: frame coverage `1.000`, vehicle center-distance mAP
  `0.5833`, 1 m F1 `0.7611`.
- 두 rosbag 수치는 기존 E10 평가와 사실상 동일하다. 5060 Ti 측정 latency는
  mean `77.7 ms`, p50 `77.5 ms`, p95 `80.5 ms`였다.

2080 Ti·3090·4090은 해당 architecture binary 포함까지 검증했으며 실제 GPU
smoke는 각 PC에서 최초 1회 수행해야 한다. 특히 host NVIDIA driver가 CUDA
12.8 container를 지원하는지는 위 `nvidia/cuda` 명령으로 먼저 확인한다.

## 문제 발생 시

`nvidia-smi`, `docker image inspect`, `run_manifest.json`, `train.log` 끝 200줄을
함께 전달한다. `.netrc`나 W&B API key는 절대 전달하지 않는다.
