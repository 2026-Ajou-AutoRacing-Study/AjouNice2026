# PointPainting batch-size benchmark.

마지막 갱신: 2026-08-30.

## 목적.

- Base3-Static PointPainting 본학습 전에 RTX 5060 Ti 16 GB에서 batch size별 실제
  처리량과 peak VRAM을 확인한다.
- VRAM 점유율이 아니라 samples/sec, 기존 최적화 조건과 학습 안정성을 함께 사용해
  첫 controlled baseline의 batch size를 결정한다.

## 조건.

- GPU: NVIDIA GeForce RTX 5060 Ti 16 GB.
- 입력 proxy: Base2 single-frame 9-feature, 30 m PointPainting dataset.
  - Base3도 같은 point range, voxelization, 9-feature와 PointPillars 구조를 사용하므로
    본 painting 전 GPU 처리량 proxy로 사용할 수 있다.
- Model/config: S1 geometric augmentation, GT database sampling 없음.
- AMP 활성화, DataLoader workers 4.
- 각 batch size는 독립 process에서 warm-up 15 iterations 뒤 60 iterations를 측정했다.
- 측정 범위는 DataLoader iteration, GPU upload, forward, backward, gradient clipping,
  optimizer step과 CUDA synchronization을 포함한다.

## 결과.

| Batch size | Mean iteration | Samples/sec | Peak allocated | Peak reserved | 판정 |
|---:|---:|---:|---:|---:|---|
| 1 | 24.80 ms | **40.32** | 0.61 GiB | 0.76 GiB | 처리량 상한 진단용 |
| 2 | 50.25 ms | 39.80 | 1.11 GiB | 1.43 GiB | 이점 없음 |
| 4 | 100.02 ms | **39.99** | 2.12 GiB | 2.72 GiB | **첫 baseline 채택** |
| 6 | 162.80 ms | 36.85 | 3.11 GiB | 4.07 GiB | 느림 |
| 8 | 215.60 ms | 37.10 | 4.11 GiB | 5.40 GiB | 느림 |
| 12 | 315.40 ms | 38.05 | 6.11 GiB | 7.54 GiB | 느림 |
| 16 | 425.44 ms | 37.61 | 8.11 GiB | 10.70 GiB | 느림 |
| 24 | OOM | — | — | — | 사용 불가 |

## 판정.

- `BS4`를 Base3 `B3S-PP0`의 batch size로 사용한다.
- BS1은 samples/sec가 BS4보다 0.8% 높을 뿐이며, loss variance와 optimizer update
  수가 커지고 기존 S1/S1g 학습 조건과 달라진다.
- BS8--16은 더 많은 VRAM을 사용하지만 처리량이 BS4보다 4.9--7.8% 낮다. 따라서
  VRAM을 더 채우기 위해 batch를 키울 근거가 없다.
- 첫 controlled baseline은 기존 값인 Adam OneCycle, LR `0.003`, 40 epochs를 그대로
  사용한다. Batch scaling에 따른 LR 변경은 별도 accuracy ablation으로만 수행한다.
- Base3 painted dataset 구축 후 같은 script로 BS4를 짧게 재확인한다. 실제 point-count
  분포가 proxy와 달라 처리량 또는 peak memory가 크게 변할 때만 판정을 다시 연다.

## 재현 명령.

```bash
/home/luv/miniconda3/envs/openpcdet_rtx50/bin/python \
  perception/pointpainting/tools/benchmark_pointpainting_train_batch.py \
  --cfg-file /home/luv/OpenPCDet/tools/cfgs/custom_models/pointpillar_morai_pointpainting_base2_singleframe9_geom_aug.yaml \
  --data-path artifacts/datasets/morai_pointpainting_base2_singleframe9 \
  --batch-size 4 \
  --workers 4 \
  --warmup-iters 15 \
  --measure-iters 60 \
  --amp
```
