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
