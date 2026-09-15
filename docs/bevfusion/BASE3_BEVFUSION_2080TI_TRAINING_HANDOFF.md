# Base3 BEVFusion RTX 2080 Ti training handoff

## 이 문서의 목적

이 문서는 다른 PC의 Codex 또는 개발자가 AjouNice2026의 Base3 BEVFusion
학습을 재현하거나 이어서 실험하기 위한 단일 진입점이다. 기존 RTX 5060 Ti
환경은 변경하지 않는다. RTX 2080 Ti용 Docker 이미지는 별도 태그로 빌드하고,
데이터·초기 checkpoint·학습 결과도 서로 분리한다.

RTX 2080 Ti는 CUDA compute capability `7.5`, VRAM `11 GB`이다. 기존
`ajounice/bevfusion:dev-cu128`은 프로젝트 CUDA 확장을 `8.6;8.9;12.0+PTX`로
빌드했으므로 그대로 전달하지 않는다. 팀원용 이미지는 `7.5`를 명시해 새로
빌드한 `ajounice/bevfusion:cu128-sm75-handoff`를 사용한다. 이 전달 이미지는
용량과 build 시간을 줄이기 위해 `sm_75` 전용이며 우리 5060 Ti에서는 기존
`dev-cu128` 이미지를 계속 사용한다.

근거: NVIDIA의 [CUDA GPU compute capability 표](https://developer.nvidia.com/cuda/gpus)와
[CUDA 12.8 release notes](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/).

## 전달물과 놓을 위치

| 전달물 | 권장 위치 | 비고 |
|---|---|---|
| Git repository | 임의 위치 | 스크립트가 repository root를 자동 계산함 |
| Base3 raw dataset | 임의 위치 | `BASE3_CAPTURE_ROOT`로 지정함 |
| Base3 metadata index | `artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1` | 작은 전달 bundle에 포함됨 |
| MORAI-adapted nuScenes init | `artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth` | fresh 학습에 필요 |
| Base3 BF0 E10 full checkpoint | `artifacts/detection3d/bevfusion/handoff/base3_bf0_epoch10_full.pth` | fine-tune에 필요 |
| Docker image archive | 임의 위치 | `docker load`로 등록하므로 저장 위치 무관 |

Base3 raw 약 50GB는 이 bundle에 중복 포함하지 않는다. 기존에 공유된 다운로드
경로에서 받은 뒤 `BASE3_CAPTURE_ROOT`만 지정한다.

전달 archive는 repository root에서 푼다. 두 archive 내부 경로가 위 표의 권장
위치와 일치한다.

```bash
cd /path/to/handoff_archives
sha256sum -c ARCHIVE_SHA256SUMS
cd /path/to/AjouNice2026
tar -xzf /path/to/bevfusion_2080ti_support.tar.gz
tar -xzf /path/to/bevfusion_2080ti_checkpoints.tar.gz
```

Base3 다운로드 위치가 예를 들어 `/data/morai/base3`이면 다음만 설정한다.

```bash
export BASE3_CAPTURE_ROOT=/data/morai/base3
```

`BASE3_CAPTURE_ROOT`는 `batch001_primary` 같은 하위 폴더가 직접 보이는
capture session root여야 한다. 다른 사용자 이름을 `/home/luv`로 흉내 낼 필요가
없다.

## 최소 host 조건

- NVIDIA RTX 2080 Ti와 동작하는 NVIDIA driver.
- CUDA 12.8 컨테이너를 위해 Linux driver `570.26` 이상 권장.
- NVIDIA Container Toolkit과 Docker에서 `--gpus all` 사용 가능.
- Base3 raw dataset 약 `45 GB`, 이미지 archive와 학습 산출물을 위한 추가 여유 공간.

확인:

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

## Docker 이미지 받기 또는 직접 빌드하기

전달받은 이미지 archive를 쓰는 방법:

```bash
gzip -dc /path/to/bevfusion-cu128-sm75-handoff.tar.gz | docker load
BEVFUSION_HANDOFF_IMAGE=ajounice/bevfusion:cu128-sm75-handoff \
  docker/bevfusion/scripts/verify_2080ti_handoff_image.sh
```

직접 빌드하려면 repository root에서 다음을 실행한다.

```bash
docker/bevfusion/scripts/build_2080ti_handoff_image.sh
docker/bevfusion/scripts/verify_2080ti_handoff_image.sh
```

이 빠른 빌드는 기존 5060 Ti용 `dev-cu128`을 parent로 사용해 CUDA extension만
`sm_75` 포함으로 다시 컴파일하며 기존 태그를 덮어쓰지 않는다. parent가 없다면
`docker/bevfusion/Dockerfile`을 `BEVFUSION_CUDA_ARCH_LIST` build argument와 함께
처음부터 빌드할 수도 있다.

## W&B 준비

1. <https://wandb.ai/> 계정을 만든다.
2. 가능하면 팀의 `AjouNice2026` entity/project에 초대받는다.
3. host에서 로그인한다.

```bash
python3 -m pip install --user wandb
wandb login
test -s ~/.netrc && echo "W&B credential ready"
```

스크립트는 host의 `~/.netrc`를 read-only로 container에 mount한다. 초대를 받지
못했다면 config의 entity 권한 오류가 날 수 있으므로 `WANDB_MODE=offline`으로
먼저 smoke test하고, 팀 entity 접근 권한을 받은 뒤 online으로 전환한다.
개인 project를 먼저 쓸 경우 `WANDB_ENTITY=<account>`와
`WANDB_PROJECT=<project>`를 지정할 수 있다.

## 데이터 인덱스 이식

원본 metadata는 취득 PC의 절대경로를 보존한다. 다음 명령은 실제 팀원 경로에서
파일을 읽고, Docker 내부 표준 경로 `/datasets/base3`가 기록된 portable info를
새로 만든다.

```bash
cd /path/to/AjouNice2026
export BASE3_CAPTURE_ROOT=/path/to/extracted/260829_034057_base3_static_10k
perception/detection3d/bevfusion/scripts/prepare_base3_portable_infos.sh
```

최종 프레임 수가 train/val/test `7632/1635/1635`이고 validator가 통과해야 한다.

## 먼저 smoke test

```bash
cd /path/to/AjouNice2026
export BASE3_CAPTURE_ROOT=/path/to/extracted/260829_034057_base3_static_10k
export BEVFUSION_IMAGE=ajounice/bevfusion:cu128-sm75-handoff
export WANDB_MODE=disabled
perception/detection3d/bevfusion/scripts/run_base3_2080ti_smoke.sh
```

8 samples/split의 1 epoch만 학습·검증한다. CUDA OOM, `no kernel image is
available`, 파일 경로 오류가 없어야 본 학습으로 넘어간다.

## 학습 방식 세 가지

### 기존 실험 기준선

- 현재 production 기준은 `B3-BF0 E10`이다.
- 같은 `0.40/0.15/0.10` threshold의 핵심 4-bag macro AP에서 E10은 `0.6462`,
  offline-best E6는 `0.6319`였다. 따라서 offline best만 보고 E6를 기준으로 삼지
  않는다.
- E6에서 시작한 BF1 camera-domain augmentation은 offline mAP를 높였지만 E10 대비
  8-bag macro AP가 `-0.0078`이어서 production으로 채택하지 않았다.
- 자세한 수치와 평가 조건 오류 기록은 `docs/bevfusion/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`,
  실행 production 계약은 `docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`에 있다.

새 실험의 비교 기준은 반드시 E10이며, W&B validation만 좋아졌다는 이유로
production을 교체하지 않는다.

### A. Fresh Base3 학습 — 기본 추천

여기서 fresh는 **Base3 E10에서 이어가는 것이 아니라**, 원래 B3-BF0와 동일하게
MORAI 3-class로 변환한 nuScenes BEVFusion 초기 가중치에서 새로운 optimizer와
12-epoch schedule을 시작한다는 뜻이다.

```bash
export WANDB_MODE=online
export WANDB_RUN_NAME=teammate-2080ti-base3-fresh-e12
perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh fresh
```

재현성과 비교가 가장 명확해 첫 실행으로 권장한다.

### B. Base3 E10 fine-tune — 두 번째 추천

```bash
export WANDB_MODE=online
export WANDB_RUN_NAME=teammate-2080ti-base3-e10-ft6
perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh finetune
```

E10 weights에서 새 optimizer, LR `2e-5`, cosine 6 epoch를 시작한다. optimizer와
scheduler까지 정확히 재개하는 `resume`이 아니다. 기존 BF1의 online camera
augmentation은 rosbag 전체 성능을 개선하지 못했으므로 기본 fine-tune에는 넣지
않았다.

### C. Strict random initialization — 연구 대조군

```bash
export WANDB_MODE=online
export WANDB_RUN_NAME=teammate-2080ti-base3-random-e24
perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh random
```

`load_from=None`으로 시작한다. Base3 규모에서 수렴과 성능 리스크가 커 production
후보로 권장하지 않는다. 사용자가 말하는 “from scratch”가 pretrained 초기화까지
배제한다는 뜻일 때만 수행한다.

## 2080 Ti 메모리 설정

기본은 physical batch `2`, gradient accumulation `2`, effective batch `4`이다.
기존 5060 Ti의 batch 4와 optimizer update당 표본 수를 맞추며 LR `1e-4`를
유지한다. warm-up은 physical loader iteration 증가를 반영해 `1000` iteration으로
자동 보정한다.

OOM이면 다음처럼 바꾼다.

```bash
export BEVFUSION_BATCH_SIZE=1
export BEVFUSION_ACCUMULATION=4
export BEVFUSION_WORKERS=2
perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh fresh
```

batch 1에서는 BatchNorm 통계가 달라질 수 있으므로 batch 2가 우선이다. 2080 Ti는
Ampere 이상과 달리 BF16이 아니라 현재 config의 FP16 AMP를 그대로 사용한다.

학습 시간은 2080 Ti에서 직접 smoke의 step time을 측정한 뒤 계산한다. 5060 Ti의
fresh E12는 validation 포함 약 6시간이었지만, 2080 Ti는 세대와 VRAM 차이 때문에
이보다 상당히 오래 걸릴 수 있어 그 시간을 그대로 약속값으로 쓰지 않는다.

## 성공 기준과 결과 위치

- 매 epoch 직후 validation 수행.
- W&B x축은 `epoch`; PointPainting과 동일한 `val/center/*`, `val/box/*`,
  `val/operating/*`, `val/distance/*`, `epoch/learning_rate_*` 계열을 기록.
- best 기준은 `val/center/mAP`.
- 출력 기본 위치:
  `artifacts/detection3d/bevfusion/training/2080ti_<mode...>`.
- fresh와 fine-tune은 W&B run과 work directory를 절대 공유하지 않는다.

학습 완료만으로 production을 교체하지 않는다. frozen Base3 test와 동일 rosbag,
동일 confidence threshold `Vehicle/Pedestrian/Obstacle = 0.40/0.15/0.10`으로 기존
`B3-BF0 E10`과 비교한다.

## Codex에게 요청할 검증 순서

1. `git status`로 팀원의 진행 중 변경을 보존한다.
2. GPU/driver/Docker/NVIDIA Container Toolkit을 확인한다.
3. image의 `TORCH_CUDA_ARCH_LIST`에 `7.5`가 있는지 검증한다.
4. Base3 capture root와 portable info의 세 split을 검증한다.
5. W&B 로그인과 team entity 권한을 확인한다.
6. disabled W&B smoke test를 통과시킨다.
7. fresh E12를 먼저 수행한다.
8. 시간이 허용되면 E10 fine-tune을 별도 run으로 수행한다.
9. test/rosbag 조건을 고정해 기존 E10과 비교한다.

사용자 파일을 임의 삭제하거나 기존 5060 Ti Docker image/tag를 덮어쓰지 않는다.
경로가 다르면 symlink로 `/home/luv`를 재현하지 말고 위 환경변수와 portable info를
사용한다.
