# MORAI Fast-BEV

Fast-BEV 원본 구현은 `third_party/fastbev` submodule에 보존한다. 이 디렉터리에는
MORAI 4-camera dataset adapter, 프로젝트 config, 얇은 학습·평가 launcher만 둔다.

이미지 변환 backend는 Fast-BEV 원본 PIL과 OpenCV를 모두 지원한다. 기본값
`pil`은 기존 실험 재현용이며, 배포 전처리와 맞춘 학습·validation·test는 아래처럼
한 번에 선택한다.

```bash
FASTBEV_IMAGE_BACKEND=opencv \
perception/detection3d/fastbev/scripts/train.sh \
  full1500_stride1_fp16_opencv_v1
```

MORAI의 고정 camera role과 우측통행 분포를 보존하기 위해 production
config의 기본 flip 확률은 BEV horizontal `0.1`, image horizontal `0.2`이다.
비교 실험에서는 다음 환경변수로만 바꾼다.

```bash
FASTBEV_BEV_FLIP_PROB=0.1 \
FASTBEV_IMAGE_FLIP_PROB=0.2 \
FASTBEV_IMAGE_BACKEND=opencv \
perception/detection3d/fastbev/scripts/train.sh <run-name>
```

학습 전에 image flip sampling, PIL/OpenCV projection geometry, 3D annotation
변환과 실제 dataset sample을 한 번에 검사할 수 있다.

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50
export PYTHONPATH="$PWD:$PWD/third_party/fastbev${PYTHONPATH:+:$PYTHONPATH}"
FASTBEV_IMAGE_BACKEND=opencv \
python3 perception/detection3d/fastbev/scripts/validate_augmentation.py
```

`MoraiAugImageMultiViewImage`가 픽셀 resize/crop/flip/rotate backend만 바꾸고,
augmentation 표본과 camera projection 보정 수식은 같은 Fast-BEV 경로를
공유한다. 선택한 backend는 config와 W&B run config에도 기록된다.

```bash
cd ~/AjouNice2026

perception/detection3d/fastbev/scripts/train.sh \
  full1500_stride1_fp16_wandb_v1
```

첫 번째 인자 뒤의 값은 원본 `third_party/fastbev/tools/train.py`로 전달되므로
`--resume-from`, `--cfg-options` 같은 MMCV 옵션을 그대로 사용할 수 있다.

기본 산출물 경로는
`artifacts/detection3d/fastbev/runs/<run-name>/`이다. 데이터 구축과 split 정보는
`dataset/morai_od/`에 있으며 실제 이미지·GT는 Git 밖의
`/home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1`을 사용한다.

상세 환경, 평가 규약, W&B 설정은
`docs/baseline_and_streampetr_transition.md`를 따른다.

Obstacle을 크기 형상별 내부 subclass로 나누고 최종 3-class로 병합한 실험은
`docs/obstacle_shape_subclass_experiment.md`에 재현 방법과 채택 여부를 기록한다.
후속 anchor/regression 가설과 실패 시 개선 우선순위는
`docs/obstacle_regression_roadmap.md`를 따른다.
