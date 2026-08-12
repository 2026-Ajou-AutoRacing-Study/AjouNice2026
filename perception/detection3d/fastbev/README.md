# MORAI Fast-BEV

Fast-BEV 원본 구현은 `third_party/fastbev` submodule에 보존한다. 이 디렉터리에는
MORAI 4-camera dataset adapter, 프로젝트 config, 얇은 학습·평가 launcher만 둔다.

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
