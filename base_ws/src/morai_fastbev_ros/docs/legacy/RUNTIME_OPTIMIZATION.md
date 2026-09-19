# Fast-BEV ROS runtime optimization

## 병목과 변경

기존 온라인 노드는 학습/평가용 MMDetection `Compose`를 매 프레임 실행했다.
증강 자체는 test mode라 비활성화되어 있었지만, 다음 작업은 그대로 반복됐다.

- 네 JPEG 순차 decode
- 네 PIL resize/crop 순차 실행
- projection/augmentation metadata 재계산과 deep copy
- normalization, DataContainer와 metadata 포장
- CPU 전처리와 GPU 추론의 직렬 실행

현재 기본 경로는 다음처럼 동작한다.

1. 시작 시 reference pipeline을 빈 Sensor_6 영상에 한 번 실행해 고정 projection,
   BEV origin, image metadata를 캐시한다.
2. 매 프레임에는 JPEG decode, PIL resize/crop, normalization만 실행한다.
3. 네 카메라 작업을 4개 CPU worker에서 병렬 처리한다.
4. pinned CPU tensor를 GPU로 non-blocking 전송한다.
5. CPU preprocess thread와 GPU inference thread가 최신 프레임 슬롯을 사이에 두고
   동시에 진행한다.

`fixed` backend는 reference MMDetection pipeline과 tensor MAE, max error,
projection error가 모두 정확히 `0`이다. `opencv`는 보간 차이로 픽셀이 완전히
같지는 않지만, 동일 test 222프레임 A/B에서 center mAP가 `0.599873`으로 PIL의
`0.600095`와 `-0.000222` 차이였다. 동일 이미지 888장의 offline paired CPU
변환은 PIL 평균 `14.416 ms`, OpenCV 평균 `8.836 ms`로 약 `1.63x` 빨랐다.
이에 따라 ROS 기본값은 `opencv`로 전환하고 `fixed`는 재현·rollback용으로
유지한다.

학습도 `FASTBEV_IMAGE_BACKEND=opencv`로 같은 image geometry backend를
선택할 수 있다. Deployment-matched 학습의 기본 augmentation은 upstream의
50% flip보다 약하게 설정한다.

- BEV horizontal flip: 10%
- image horizontal flip: 20%
- BEV vertical flip: 0%

실제 값은 W&B run config에 기록하며 `FASTBEV_BEV_FLIP_PROB`와
`FASTBEV_IMAGE_FLIP_PROB`로 비교 실험할 수 있다.

### Deployment-matched 12-epoch 학습 결과

동일한 1,050/228/222 train/val/test split, seed와 FP16 조건에서 기존
PIL/BEV 50%/image 50% baseline과 OpenCV/BEV 10%/image 20% 후보를 비교했다.
두 실험 모두 epoch 11의 validation `center_mAP` best checkpoint를 사용했다.

| test 지표 | PIL 50/50 | OpenCV 10/20 | 변화 |
|---|---:|---:|---:|
| center mAP | 0.6001 | 0.5753 | -0.0248 |
| vehicle mean AP | 0.6874 | 0.6884 | +0.0010 |
| pedestrian mean AP | 0.6645 | 0.6488 | -0.0158 |
| obstacle mean AP | 0.4483 | 0.3888 | -0.0595 |
| IoU mAP@0.25 | 0.5783 | 0.5596 | -0.0187 |
| 순수 inference FPS | 43.75 | 42.85 | -0.90 |
| offline end-to-end FPS | 10.88 | 13.56 | +2.68 |

OpenCV는 모델 연산 자체가 아니라 image transform 경로를 줄이므로 순수 inference
FPS는 동등하고, disk load와 전처리를 포함한 end-to-end FPS가 약 24.7% 증가했다.
반면 이번 학습은 backend와 두 flip 확률을 동시에 바꾼 결합 실험이므로 정확도 하락을
OpenCV 픽셀 보간 하나의 영향으로 단정할 수 없다. 특히 obstacle 성능 하락이 커서,
기존 PIL 모델은 정확도 baseline으로 유지하고 OpenCV 모델은 실시간 배포 후보로
보존한다. 후속 실험은 OpenCV backend를 고정한 채 flip 확률만 분리해 비교한다.

- W&B: <https://wandb.ai/AjouNice2026/morai-fastbev/runs/full1500_stride1_fp16_opencv_flip10_20_v1>
- 로컬 결과: `artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_opencv_flip10_20_v1/`

### OpenCV flip 통제 실험

backend와 flip 효과를 분리하기 위해 OpenCV를 유지하고 BEV/image flip을 기존과
같은 50%/50%로 복원해 동일한 12-epoch 실험을 수행했다. epoch 12의 validation
`center_mAP=0.59425` checkpoint를 222-frame test에 사용했다.

| test 지표 | PIL 50/50 | OpenCV 10/20 | OpenCV 50/50 |
|---|---:|---:|---:|
| center mAP | 0.6001 | 0.5753 | 0.5919 |
| vehicle mean AP | 0.6874 | 0.6884 | 0.6938 |
| pedestrian mean AP | 0.6645 | 0.6488 | 0.6625 |
| obstacle mean AP | 0.4483 | 0.3888 | 0.4194 |
| IoU mAP@0.25 | 0.5783 | 0.5596 | 0.5744 |
| IoU mAP@0.50 | 0.2654 | 0.2547 | 0.2751 |
| offline end-to-end FPS | 10.88 | 13.56 | 13.20 |

OpenCV 50/50은 10/20 대비 center mAP를 0.0166 회복했고 PIL 50/50과의 차이를
0.0082로 줄였다. vehicle과 pedestrian mean AP 및 IoU AP는 PIL과 동등하거나
더 높았지만 obstacle mean AP는 아직 0.0290 낮다. 따라서 이전 정확도 하락의 큰
부분은 flip 확률 축소 영향으로 판단하며, 배포 후보의 기본 학습 정책은 OpenCV
50/50으로 되돌리는 편을 권장한다. 현재 config 기본값 변경은 별도 결정으로 남긴다.
단, obstacle 표본 수·크기 다양성과 yaw label 품질은 별도 개선 과제로 남는다.

- W&B: <https://wandb.ai/AjouNice2026/morai-fastbev/runs/full1500_stride1_fp16_opencv_flip50_50_v1>
- 로컬 결과: `artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_opencv_flip50_50_v1/`

## 2026-08-14 benchmark

입력 bag:

`rosbags/raw_sensor_w_cam/fastbev_30hz_benchmark/260814_010854_fastbev_4cam_30hz_vehicle_dense_raw.bag`

GPU는 RTX 5060 Ti, FP16, 입력은 Sensor_6 네 카메라, 출력은 Fast-BEV detection이다.

| 재생 조건 | 입력 wall Hz | 출력 wall Hz | preprocess mean | inference mean | camera span p95 |
|---|---:|---:|---:|---:|---:|
| 원본 1.0x 짧은 구간 | 24.78 | 24.79 | 20.87 ms | 22.81 ms | 3.0 ms |
| 30 Hz 환산 1.286x | 30.20 | 30.22 | 20.98 ms | 23.25 ms | 3.0 ms |
| 35 Hz stress 1.5x | 35.31 | 35.32 | 20.57 ms | 23.34 ms | 3.0 ms |

원본 bag 전체 평균 header rate는 약 23.32 Hz이며, 위 1.0x 수치는 모니터가
겹친 14.5초 구간의 순간 rate다. 핵심은 세 조건 모두 입력과 출력 wall rate가
일치했고 steady state raw drop이 없었다는 점이다. 초기 warm-up 동안의 prepared
drop은 정상이며 실제 운용에서는 판단 입력을 warm-up 완료 후 사용한다.

## 재현 명령

터미널 1:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
roscore
```

터미널 2:

```bash
cd ~/AjouNice2026
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh bag
```

터미널 3:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
rosrun morai_fastbev_ros benchmark_fastbev_runtime.py \
  --duration 46 \
  --output /tmp/fastbev_runtime_30hz.json
```

터미널 4:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
rosbag play --clock --rate=1.286 --quiet \
  rosbags/raw_sensor_w_cam/fastbev_30hz_benchmark/260814_010854_fastbev_4cam_30hz_vehicle_dense_raw.bag \
  --topics \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed
```

전처리 동등성은 bag에서 동일 시점 네 이미지를 `front.*`, `front_left.*`,
`front_right.*`, `back.*`로 추출한 디렉터리에 대해 다음처럼 확인한다.

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
export PYTHONPATH="$PWD/third_party/fastbev:$PWD${PYTHONPATH:+:$PYTHONPATH}"

python3 base_ws/src/morai_fastbev_ros/scripts/validate_fastbev_preprocessing.py \
  --image-dir /path/to/four_camera_images
```
