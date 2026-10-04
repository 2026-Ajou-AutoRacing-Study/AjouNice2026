# Base4-Temporal 팀원 handoff

> **문서 상태: CURRENT.** 2026-09-30 기준 Base4 고주로 velocity/temporal 작업의
> 팀 공용 진입점이다. Base3 production 배포는
> [`BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)를
> 따른다.

## 먼저 알아야 할 결론

- 작업 기준 브랜치는 `perception/base4-temporal-competition-v2`다. 과거
  `dataset/base4-temporal`은 복구 원본 이력이며 새 작업 기준으로 사용하지 않는다.
- 기존 production은 계속 `B3-BF0-F40 E25` 7D이며 detector velocity 발행은 `off`다.
- Base4 V1은 single-frame 9D `vx, vy`, T1은 causal 3-sweep 9D control이다.
- 보정 전 4-epoch 결과에서는 V1이 T1보다 고주로 tracking/prediction 일반화가 나았다.
- 두 모델 모두 detector velocity를 tracker에 넣으면 기존 E25보다 악화했으므로 production이
  아니다. 다음 우선순위는 V1 velocity label/loss와 실제 주행형 속도 분포 개선이다.
- MR rosbag의 heading 보정이 확정돼 다음 학습은 `hwy3h1_mrhc_v2`로 V1/T1을 다시
  비교한다. 기존 `hwy3h1_raw` checkpoint는 파이프라인 EVIDENCE다.
- 팀원의 `perception/base4-temporal-3sweep` 브랜치는 전체 merge하지 않는다. 해당 구현에서
  유효했던 scene leakage, sweep transform, velocity consistency 검사는 현재 audit 도구에
  선별 이식됐다.
- `2026-09-22` 고속 연속속도 파일럿은 `48/48 scene` 수집과 raw/scan-end Ego-deskew
  dataset 검증까지 완료했다. 아직 팀 공용 package나 production checkpoint는 아니며,
  다음 실험은 동일 조건 V1/T1 A/B다. 상세 결과는
  [`../../dataset/morai_od/docs/260922_base4_highspeed_jitter_pilot_results.md`](../../dataset/morai_od/docs/260922_base4_highspeed_jitter_pilot_results.md)를 따른다.
- 첫 통합 실험 deck은 raw `159 scene / 2,028 frame`, scan-end Ego deskew
  `157 scene / 1,887 frame`으로 완성됐고 둘 다 temporal audit PASS다. raw V1/raw T1/
  deskew T1 E4 recipe와 입력 계약은
  [`../../dataset/morai_od/docs/260922_base4_highway_first_experiment_dataset.md`](../../dataset/morai_od/docs/260922_base4_highway_first_experiment_dataset.md)를 따른다.

정량 결과와 판정은
[`260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md`](260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md)를
먼저 읽는다.

## Git 시작점

```bash
cd ~/AjouNice2026
git fetch origin
git switch perception/base4-temporal-competition-v2
git pull --ff-only origin perception/base4-temporal-competition-v2
```

개인 실험은 이 커밋에서 새 브랜치를 파도 된다. 공용 branch에 개인 YAML, checkpoint,
dataset을 커밋하지 않는다.

## Git 밖에서 받아야 하는 파일

다음 파일은 용량 때문에 저장소에 포함되지 않는다.

| 항목 | 팀원 PC 권장 위치 | 비고 |
|---|---|---|
| 공용 Docker image | Docker image `ajounice/bevfusion:team-cu128-v1` | 2080 Ti/3090/4090/5060 Ti 공용 |
| Base4 corrected parsed dataset | `$HOME/ajounice_bevfusion_artifacts/datasets/base4_temporal_highway_hwy3h1_mrhc_v2/` | 약 7.3 GiB, MR heading-corrected CURRENT |
| Base3 E25 inference checkpoint | `$HOME/ajounice_bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth` | V1 초기값 |
| Base4 T1 adapted checkpoint | `$HOME/ajounice_bevfusion_artifacts/checkpoints/base3_e25_to_base4_t1.pth` | T1의 5-channel 초기값 |
| V1/T1 E4 checkpoint | 선택 | 기존 결과 재평가에만 필요 |

Dataset root에는 `morai_base3_infos_{train,val,test}.pkl`, scene data,
`conversion_audit.json`, `validation_audit.json`, `temporal_quality_audit.json`이 함께 있어야
한다. 현재 test PKL은 의도적으로 비어 있으며 train/val은 scene 단위로 분리돼 있다.

현재 기준 fingerprint:

```text
train info sha256 159cf1740c5cb85214dc17bc7e6105bb0629632a084e8eaee8d688d6ddfd57af
val info sha256   acc729f1054cb3d37b8a596b70897c92df9dc1de629546f5de17154f7f952dbb
test info sha256  39e794985ab316598b8d9a0329771abe067b1581c3fdbacb73e4a89d26016dda
T1 init sha256    971aaa551e10855e27d1d18eb4dcea746ef13530ede0cc1b224087cccb4655d0
```

## 최초 머신 설정

공용 Docker 설치와 W&B 로그인은
[`TEAM_DOCKER_TRAINING_HANDOFF.md`](TEAM_DOCKER_TRAINING_HANDOFF.md)를 따른다.
그 문서의 `configure_team_machine.sh`로 본인 operator와 GPU profile을 먼저 만든다.

```bash
cd ~/AjouNice2026

bash perception/detection3d/bevfusion/scripts/configure_team_machine.sh \
  --operator yj \
  --image ajounice/bevfusion:team-cu128-v1 \
  --e25-finetune-checkpoint \
    "$HOME/ajounice_bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth"
```

`yj`는 작업자에 따라 `sw` 또는 `jh`로 바꾼다. W&B API key를 저장소나 YAML에 넣지
않는다.

Base4 전용 경로는 현재 shell에 추가한다.

```bash
export BASE4_HWY3H1_MRHC_V2_ROOT="$HOME/ajounice_bevfusion_artifacts/datasets/base4_temporal_highway_hwy3h1_mrhc_v2"
export BEVFUSION_BASE3_E25_FINETUNE_CHECKPOINT="$HOME/ajounice_bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth"
export BEVFUSION_BASE4_T1_INIT_CHECKPOINT="$HOME/ajounice_bevfusion_artifacts/checkpoints/base3_e25_to_base4_t1.pth"
```

팀원이 shell을 새로 열 때마다 export하기 싫다면 위 세 줄만 개인
`.env.bevfusion.local` 끝에 추가한다. 이 파일은 Git ignore 대상이다.

## 학습 전 필수 gate

```bash
cd ~/AjouNice2026

python3 perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py \
  --info-dir "$BASE4_HWY3H1_MRHC_V2_ROOT" \
  --capture-root "$BASE4_HWY3H1_MRHC_V2_ROOT" \
  --serialized-capture-root /datasets/base4_hwy3h1_mrhc_v2

python3 perception/detection3d/bevfusion/tools/audit_base4_temporal_quality.py \
  --info-dir "$BASE4_HWY3H1_MRHC_V2_ROOT"
```

두 명령이 모두 성공하고 `temporal_quality_audit.json`의 `valid`가 `true`일 때만
학습한다. 이 audit는 다음 오류를 차단한다.

- 같은 scene이 train/val에 나뉘는 leakage.
- future sweep 또는 음수/비정상 `dt`.
- 잘못된 past-to-current SE(3).
- Ego/Object/camera timestamp age 초과.
- km/h를 m/s로 오인한 3.6배 velocity label.
- geometric BBox center trajectory와 velocity target의 불일치.

## Dry-run과 smoke test

2026-09-21 RTX 5060 Ti에서 corrected V2의 V1과 T1 모두 `16 train + 16 val`,
1-epoch smoke를 완료했다. Dataset load, V1 4-channel/T1 5-channel causal sweep,
forward/backward, validation, checkpoint 저장과 W&B sync가 모두 통과했다. Smoke metric은
표본 16개의 초기 velocity head 값이므로 모델 선택에는 사용하지 않는다.

```bash
cd ~/AjouNice2026

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_v1_e4.yaml \
  --dry-run

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_t1_3sweep_e4.yaml \
  --dry-run
```

Dry-run의 dataset root, checkpoint, GPU profile과 effective batch가 의도대로인지 확인한
후 한 모델씩 smoke를 실행한다.

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_v1_e4.yaml \
  --smoke

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_t1_3sweep_e4.yaml \
  --smoke
```

2080 Ti는 physical batch 2와 accumulation 2, 3090/4090/5060 Ti는 batch 4와
accumulation 1이 시작값이다. OOM이면 effective batch 4를 유지하면서 physical batch만
낮춘 개인 YAML을 만든다. MORAI를 같은 GPU에서 실행한 채 학습하지 않는다.

## 재현용 4-epoch 학습

```bash
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_v1_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_hwy3h1_mrhc_v2_t1_3sweep_e4.yaml
```

두 모델을 동시에 실행하지 않는다. 팀 runner는 resolved YAML, Git SHA/diff, dataset info
hash, Docker image, GPU, W&B run name과 source snapshot을 output에 남긴다.

### 2026-09-22 hard-case 보강판

속도 진단에 병목 협로와 일반 고속 횡 offset을 추가한 현재 짧은 A/B 입력은 다음과 같다.

```bash
export BASE4_VELOCITY_HARDCASE_V2_ROOT="$PWD/artifacts/datasets/morai_base4_velocity_hardcase_v2_raw16"

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_velocity_hardcase_v2_v1_e4.yaml

bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base4_velocity_hardcase_v2_t1_3sweep_e4.yaml
```

두 recipe는 같은 `43 train / 8 val scene`, `543 / 108 frame` split을 사용한다. Dataset
구성과 품질 수치는
[`../../dataset/morai_od/docs/260922_base4_velocity_hardcase_v2_results.md`](../../dataset/morai_od/docs/260922_base4_velocity_hardcase_v2_results.md)를 기준으로 한다.

## T1 초기 checkpoint를 다시 만들어야 할 때

T1은 sparse-conv 입력을 XYZI 4채널에서 `XYZI + positive past age` 5채널로 확장한다.
공유된 adapted checkpoint를 우선 사용한다. 재생성이 필요하면 PyTorch가 들어 있는 공용
Docker 안에서 다음 도구를 실행한다.

```bash
python3 perception/detection3d/bevfusion/tools/adapt_base3_checkpoint_for_base4_t1.py \
  /path/to/base3_bf0_f40_e25_inference.pth \
  /path/to/base3_e25_to_base4_t1.pth
```

기존 4개 channel weight는 복사하고 새 `dt` channel은 0으로 초기화한다. velocity head는
새로 초기화되는 것이 정상이다.

## 결과 해석과 금지 사항

- 같은 생성기의 val 성능만 보고 production을 선택하지 않는다.
- 반복 사용된 `고주로 합류`, `highway1` bag은 development evaluation이다.
- 실제 detector velocity fusion은 Vehicle 2D AVE와 Map-CV FDE가 함께 개선될 때만 켠다.
- 현재 T1 raw point concatenation을 더 오래 학습시키는 것을 우선 개선으로 보지 않는다.
- 과거 sweep은 Ego motion만 현재 frame으로 정렬한다. GT Object motion compensation을
  input에 사용하지 않는다.
- scan 내부 deskew는 원본 dataset에 덮어쓰지 않는다. raw와 derived dataset을 분리하고
  online에도 같은 보정이 있을 때만 비교한다.
- `/Object_topic`의 vehicle position은 rear axle이고 size는 unscaled catalog 제원이다.
  학습 GT는 sensor-capture geometric BBox 기준이며, rosbag 평가는 현재 asset catalog
  보정을 사용한다.

## 팀원이 결과와 함께 남길 것

다음 정보 없이는 성능 비교를 승인하지 않는다.

1. Git commit과 branch.
2. 사용 recipe 또는 resolved YAML.
3. dataset info SHA-256.
4. 초기 checkpoint SHA-256.
5. GPU와 physical/effective batch.
6. W&B run URL.
7. best 선정 기준과 epoch.
8. 동일 고주로 bag의 detection, tracking 2D AVE, speed MAE, Map-CV FDE.

Dataset 수집·변환 자체를 이어서 작업할 때는
[`../../dataset/morai_od/docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md)를
우선한다.
