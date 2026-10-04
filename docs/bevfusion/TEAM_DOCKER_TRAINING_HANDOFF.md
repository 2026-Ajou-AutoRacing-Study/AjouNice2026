# BEVFusion 팀 공용 Docker 학습·추론 handoff

> **문서 상태: CURRENT.** 공용 Docker와 일반 Base3 학습 계약이다. Base4 temporal
> 실험은 [`BASE4_TEMPORAL_TEAM_HANDOFF.md`](BASE4_TEMPORAL_TEAM_HANDOFF.md)의
> 추가 dataset·checkpoint·audit 계약을 따른다.

## 목적과 기준

이 구성은 Base3-Static 10K로 MORAI 3-class BEVFusion을 여러 PC에서
독립적으로 학습하고 동일 checkpoint를 ROS online inference에 사용하는
팀 표준이다.

2026-09-18 Base3 Fresh E40 corrected validation·test·mission rosbag 결과와
checkpoint 선택 근거는
[`260918_base3_e40_production_evaluation_results.md`](260918_base3_e40_production_evaluation_results.md)를
참고한다. Offline best는 E26이다. 실제 경로 7-bag detection은 E10이 근소하게
높지만, 최종 시스템 목표인 tracker/Map-CV의 AVE와 FDE는 E25가 우세하므로
production 표준은 E25로 정한다. E10은 rollback baseline으로 보존한다.

| GPU | CUDA arch | physical batch | accumulation | effective batch |
|---|---:|---:|---:|---:|
| RTX 2080 Ti | 7.5 | 2 | 2 | 4 |
| RTX 3090 | 8.6 | 4 | 1 | 4 |
| RTX 4090 | 8.9 | 4 | 1 | 4 |
| RTX 5060 Ti | 12.0 | 4 | 1 | 4 |

위 표는 **학습용 시작 preset**이다. 모든 GPU에서 물리 batch를 동일하게
고정한다는 뜻이 아니다. RTX 5060 Ti의 physical batch 4만 실제 full
train/validation으로 확인했고, 2080 Ti·3090·4090 값은 각 VRAM에 맞춘 팀
시작값이므로 각 PC에서 smoke test와 peak VRAM을 반드시 확인한다.

- RTX 2080 Ti 11 GB: `2 x accumulation 2`로 시작하고 CUDA OOM이면
  `1 x accumulation 4`로 낮춘다.
- RTX 3090/4090 24 GB: 우선 `4 x accumulation 1`을 사용한다. batch를
  키우는 실험은 별도 실험 YAML에서만 수행한다.
- RTX 5060 Ti 16 GB: 검증된 기준인 `4 x accumulation 1`을 사용한다.
- 학습 physical batch를 바꿀 때는 effective batch가 4로 유지되도록
  gradient accumulation을 함께 조정한다. effective batch나 learning rate까지
  바꾸면 동일 실험의 단순 GPU 이식이 아니라 별도 하이퍼파라미터 실험이다.

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
  --inference-checkpoint "$HOME/ajounice_bevfusion_artifacts/bevfusion-team-checkpoints-v1/checkpoints/base3_bf0_f40_e25_inference.pth"
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

## 학습·평가 공간 범위 계약

Base3 B0의 학습 GT, detector 출력과 val/test 평가는 모두 같은
`[-30, -30, -3, 30, 30, 2] m` 중심점 범위를 사용한다. 평가기는 이
축 정렬 사각 범위로 GT와 prediction을 모두 거른 뒤 center-distance와 IoU
지표를 계산한다. 중심거리 30 m를 넘더라도 사각 범위의 대각선 안에 있는
box는 평가에 남는다는 점에 유의한다.

2026-09-18 이전 evaluator는 학습 범위 밖 GT를 val/test 분모에 포함하는
계약 누락이 있었다. 이전 run의 자동 validation history는 재현 기록으로만
보존하고, checkpoint 비교와 best 선정은 수정 evaluator로 만든 별도
`corrected-range30` W&B evaluation run을 기준으로 한다. 이 문제는 학습
loss에는 영향을 주지 않으므로 기존 checkpoint를 폐기하거나 재학습할
사유는 아니다.

### Corrected validation batch 크기

평가 batch는 gradient가 없는 **validation 전용 값**이며 위 학습 physical
batch와 별개다. corrected checkpoint sweep의 기본값 `4`는 RTX 5060 Ti
기준이다. 다른 PC에서는 먼저 E1 checkpoint 하나로 실행하고 다음 시작값을
사용한다.

| GPU | validation 시작값 | 조정 기준 |
|---|---:|---|
| RTX 2080 Ti 11 GB | 2 | OOM이면 1 |
| RTX 3090 24 GB | 4 | 여유가 충분하면 별도 timing에서 8 시험 가능 |
| RTX 4090 24 GB | 4 | 여유가 충분하면 별도 timing에서 8 시험 가능 |
| RTX 5060 Ti 16 GB | 4 | OOM이면 2 |

```bash
python3 perception/detection3d/bevfusion/tools/evaluate_corrected_checkpoint_sweep.py \
  ... \
  --batch-size 2
```

batch를 키우는 목적은 평가 시간 단축뿐이며 모델 성능을 높이지 않는다.
E1에서 batch 1 기준 metric과 동일한지, peak VRAM에 최소 1--2 GB 여유가
있는지 확인한 뒤 전체 E1--E40 sweep을 시작한다. CUDA OOM, dataloader 종료,
metric 불일치가 있으면 `8 -> 4 -> 2 -> 1` 순으로 낮춘다. 한 W&B sweep
도중 batch를 바꾸지 말고, 바꿨다면 새 run으로 다시 기록해 재현성을 지킨다.

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
- production E25 inference checkpoint SHA-256:
  `36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011`.
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
