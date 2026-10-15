# RTX 4090 해상도와 BEV 범위 확대 처리시간

> 2026-10-10 후속: [다양한 완료5K 검사](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)에서
> 확대 후보의 voxel coordinate와 sparse_shape 축 누락을 발견해 새 opt-in encoder를 수정했다.
> 아래 확대 4090 수치는 수정 전 제한 3샘플의 역사적 비용이다. 새 모델 비용/정확도 인증에
> 재사용하지 말고 새 4090 번들로 재측정한다. 기존 E25 비용/원본 결과는 보존한다.

2026-10-08 23:20:33 팀원 PC 측정, 2026-10-09 결과 분석.
RTX 4090에서 기존 공간 간격을 유지한 320×896 이미지 및 종방향 ±100m 모델은
평균 45.86ms, p95 46.71ms였다. CPU 이미지 준비까지 포함하면 평균 50.93ms다.
고해상도와 범위 확대를 연산 비용 때문에 미리 포기할 근거는 줄었다.
확대 모델의 정확도, 실제 ROS 처리량, 풀스택 동시 실행 성능을 증명하는 결과는 아니다.

## 원본과 무결성

- 반환 원본 ZIP은 Downloads에서 다음 ignored 경로로 옮겼다.
  `artifacts/evaluation/bevfusion/capacity_4090_261008/capacity_20261008_232033_32176.zip`
- 같은 경로의 `capacity_20261008_232033_32176/`에 압축을 해제했다.
- ZIP SHA256: `5c53f9c0b6b6e33b44fa59784464376a156319a85aaf0ef8397d009b6c0c8e76`.
- ZIP CRC 및 내부 SHA256 7개 모두 일치했다.
- 반환 metadata의 bundle manifest는 전달했던 v2 ZIP의 manifest와 동일하다.
- 전체 상태 PASS, 각 설정 정방향 60회와 역방향 30회 결과 8개 모두 valid다.
  순서 반전 평균 차이는 기존 +1.46%, 이미지 +0.46%, 범위 −0.29%, 둘 다 +0.26%다.

## 측정 계약

RTX 4090 24GB, AMD Ryzen 9 7950X, driver 570.133.07,
Docker `ajounice/bevfusion:team-cu128-v1`, batch 1 AMP다.
시작 전 GPU 메모리는 493MiB, 사용률 2~3%였고 GPU 프로세스에는 데스크톱과
브라우저만 표시됐다. 실행 중인 Docker는 없었다. 시작 전 기록에서 MORAI나 다른
GPU 학습·추론 프로세스는 보이지 않았다.

네 설정은 같은 H01 raw 3장을 사용했다. 클래스 3개, query 200개,
voxel XYZ 0.075/0.075/0.20m, camera BEV XY 0.30m, depth 간격 0.50m를 유지했다.
기존 depth 범위 1~60m는 118bin, 확대 1~100m는 198bin이다.
semantic ROI는 기존 X/Y ±30m, 확대 X ±100m/Y ±30m다.
확대 내부 격자만 stride 정합을 위해 X ±100.8m/Y ±32.4m로 padding한다.
같은 설정별 geometry, 클래스/query, retained points, occupied voxels가
5060 Ti의 최종 고정 간격 대조 JSON과 모두 일치했다.

E25의 호환 가중치를 로드했으나 범위 확대 시 depth 출력층은 새 초기화 상태다.
로그의 해당 weight/bias shape mismatch는 이 비용 시험의 의도된 조건이다.
frustum 등 geometry buffer는 설정 기준으로 복원한다.
따라서 이 결과를 학습 완료된 ±100m detector의 정확도 측정으로 사용하지 않는다.

## 네 설정 비교

5060 Ti 원본은 `artifacts/evaluation/bevfusion/capacity_261006/controlled_four_production_baseline.json`이다.
시간은 ms이며 CPU 준비 포함 값에도 ROS 통신·publish·tracker·planner는 들어가지 않는다.

| 설정 | 이미지 | 종방향 범위 | 5060 Ti 모델 평균 | 4090 모델 평균 | 4090 모델 p95 | 4090 CPU 준비 포함 평균 | 모델 속도 배수 |
|---|---|---|---:|---:|---:|---:|---:|
| E25 기존 | 256×704 | ±30m | 66.48 | 27.66 | 29.34 | 32.42 | 2.40× |
| 이미지 확대만 | 320×896 | ±30m | 90.46 | 34.35 | 35.05 | 39.33 | 2.63× |
| 범위 확대만 | 256×704 | ±100m | 96.27 | 36.19 | 37.51 | 40.41 | 2.66× |
| 이미지와 범위 확대 | 320×896 | ±100m | 130.59 | 45.86 | 46.71 | 50.93 | 2.85× |

4090에서 둘 다 확대하면 기존 모델 대비 1.66배다. CPU 준비 포함 p95는
53.20ms이며, 평균 50.93ms의 역수는 약 19.64회/초다. 이는 직렬 처리시간 환산값이지
실제 ROS 출력 Hz가 아니다. 10Hz 입력의 100ms 주기에 비하면 연산 여유가 있지만,
최종 후보의 밀집 장면과 풀스택 동시 실행에서 확인해야 한다.

## 단계별 증가 원인

각 값은 GPU stream critical-path span으로 CPU가 만든 대기도 포함할 수 있다.
순수 GPU kernel 시간이나 FLOPs가 아니며 하위 view 단계는 상위 합계에 다시 더하지 않는다.
CPU 전처리는 모델 밖에서 별도로 측정했다.

| 단계 평균 ms | 기존 | 이미지 확대 | 범위 확대 | 둘 다 확대 |
|---|---:|---:|---:|---:|
| CPU 이미지 decode·resize·crop·pack | 4.74 | 4.95 | 4.19 | 5.04 |
| Camera backbone과 neck 신경망 | 6.49 | 8.21 | 6.41 | 8.22 |
| Camera에서 BEV 변환 | 7.80 | 12.50 | 13.95 | 21.56 |
| LiDAR voxelization | 1.61 | 1.61 | 1.69 | 1.69 |
| LiDAR sparse encoder 신경망 | 7.80 | 7.86 | 7.34 | 7.45 |
| BEV fusion·backbone·neck 신경망 | 0.85 | 0.85 | 2.19 | 2.21 |
| Detection head와 decode | 2.23 | 2.28 | 3.70 | 3.74 |

둘 다 확대할 때 모델 평균은 18.20ms 증가한다. Camera에서 BEV 변환 증가
13.76ms가 전체 증가의 약 76%다. 해당 확대 단계 내부는 geometry 6.81ms,
camera feature/depth 처리 2.48ms, BEV pooling 10.52ms, downsample 0.91ms다.
이미지 확대만으로 모델 +6.69ms, 범위 확대만으로 +8.53ms였다.

확대 시 sparse encoder 비용이 폭증하지 않은 것은 이번 입력의 retained points가
8,644~10,306개, occupied voxels가 6,903~8,252개로 제한됐기 때문이다.
밀집 장면의 최대 부하로 일반화하지 않는다. 특히 확대 camera BEV 단계는
5060 Ti 74.71ms에서 4090 21.56ms로 약 3.47배 빨랐지만 sparse encoder는
12.74ms에서 7.45ms로 약 1.71배다. 모든 단계에 동일 GPU 배수를 적용할 수 없다.

## 메모리와 실행 범위

steady peak allocated는 기존 1,304.5MiB, 이미지 1,947.3MiB,
범위 2,060.0MiB, 둘 다 3,129.6MiB다. 이는 PyTorch allocated 값이며
모델을 켜는 데 필요한 전체 VRAM으로 해석하지 않는다. warmup peak allocated는
최대 11,246.6MiB였고 500ms 간격 telemetry의 전체 GPU 메모리 관측 최대는
10,590MiB였다. 샘플링 간격과 allocator 측정 범위가 달라 두 수치가 같을 필요는 없다.
측정 전체는 약 40.7초이며 장시간 열 안정성 검증은 아니다.

현재 Base5 후보는 4-class, voxel 0.10m, camera BEV 0.40m, depth 1.0m다.
이번 대조는 3-class 및 더 촘촘한 간격이므로 후보와 동일 모델의 실측은 아니다.
이 결과만으로 후보 설정이나 production E25를 변경하지 않는다.

## 다음 검증

현재 Sunny13 수집은 그대로 유지한다. 최종 학습 설정 결정 시 이 결과로
고해상도와 공간 정밀도 유지의 비용을 검토한다. 학습된 후보를 팀원 4090에서
원거리·근접 밀집 raw 장면 및 ROS 입력으로 시험하고, detector부터 판단팀 전달까지
p50/p95 latency·출력 Hz·drop·동시 풀스택 부하를 함께 확인한다.
정확도는 동일 mission rosbag에서 detection·tracking·Map-CV로 별도 판정한다.
