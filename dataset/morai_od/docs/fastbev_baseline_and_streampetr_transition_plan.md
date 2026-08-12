# Fast-BEV baseline과 StreamPETR temporal dataset 전환 계획

## 1. 목적과 현재 결정

현재 작업은 `dataset/morai-multisensor-capture` 브랜치에서 계속한다. Fast-BEV는
현재 수집 데이터가 실제 BEV 3D detection fine-tuning에 유효한지 빠르게 확인하는
single-frame baseline으로 사용한다. 이후 baseline을 기준으로 temporal modeling의
효과를 비교할 수 있도록 메타데이터 수집 방식을 보강하고 StreamPETR용 데이터를
새로 수집한다.

순서는 다음과 같다.

```text
현재 Sunny 13시 수집본
→ 정상 1,500프레임 선별
→ 4-camera single-frame Fast-BEV fine-tuning
→ baseline 성능·변환 파이프라인 검증
→ frame metadata와 Ego pose 수집 방식 개편
→ StreamPETR용 연속 데이터 신규 수집
→ temporal 성능 향상 비교
```

Fast-BEV와 StreamPETR 소스는 각각 다음 submodule을 사용한다.

```text
third_party/fastbev
third_party/streampetr
```

## 2. Fast-BEV baseline 데이터 범위

현재 검증된 수집 결과는 286개 완료 scene, 1,716프레임이다. 이 중 품질 검사를
통과한 250개 scene을 골라 정확히 1,500프레임을 Fast-BEV baseline v1으로 사용한다.
각 scene은 6개의 연속 프레임을 가진다.

처음 완료된 250개를 단순 선택하지 않는다. 다음 기준으로 전체 후보를 검사한 뒤
scene 유형과 객체 분포가 유지되도록 250개를 선택한다.

- 같은 MORAI timestamp로 묶인 Camera 4대·LiDAR·GT가 모두 존재할 것
- 네 Camera의 `Intensity.png`가 모두 존재하고 0 byte가 아닐 것
- 학습 GT에 필요한 Camera `BBox3D.txt`와 LiDAR Instance GT가 유효할 것
- scene당 6개 frame이 모두 완전할 것
- ScenarioLoad 당시 기대 actor 수와 실제 actor 수가 일치한 완료 scene일 것
- 충돌, 경로 이탈, 불완전 timestamp group이 발생한 scene은 제외할 것
- `scene_type`, topology, 객체 수, 거리·방향 분포가 한 유형에 과도하게 치우치지
  않도록 선택할 것

한 scene에서 일부 frame만 가져와 1,500장을 맞추지 않는다. temporal 학습을 하지 않는
Fast-BEV 1F에서도 인접 frame의 train/validation/test 누수를 막고 데이터 단위를
일관되게 유지하기 위해 6개 frame 전체를 함께 채택하거나 함께 제외한다.

## 3. 최종 split 정책

생성 manifest에 있던 과거 split은 최종 학습 split으로 강제하지 않는다. 높은
ScenarioLoad 실패율로 실제 정상 scene의 비율과 클래스 분포가 바뀌었기 때문이다.
품질 검사를 통과한 250개 scene을 확정한 후 한 번만 최종 split을 만든다.

| Split | Scene | Frame | 실제 비율 |
|---|---:|---:|---:|
| Train | 175 | 1,050 | 70.0% |
| Validation | 38 | 228 | 15.2% |
| Test | 37 | 222 | 14.8% |
| 합계 | 250 | 1,500 | 100% |

split은 반드시 scene 단위다. 같은 `scene_id`의 6개 frame은 한 split에만 포함한다.
최종 split은 고정 seed로 생성하고 학습 결과를 본 뒤 다시 조정하지 않는다.

split 생성 시 가능한 범위에서 다음 분포를 계층화한다.

- `vehicle_centric`, `pedestrian_centric`, `mixed_vehicle_pedestrian`
- `camera_seam_occlusion`, `obstacle_context`
- straight, curve, intersection, crosswalk, roadside clutter
- 차량·보행자·장애물 수
- 전·좌·우·후방 객체 분포

이번 v1은 Sunny 13시 내부 성능 확인용이다. 다른 시간·날씨에 대한 일반화 성능을
주장하지 않는다.

## 4. Fast-BEV용 데이터 형식

대용량 이미지와 LiDAR 데이터는 Git 저장소 밖에 둔다.

```text
$AJOUNICE_DATA_ROOT/morai_od/processed/fastbev_sunny13_1500_v1/
├── metadata/
│   ├── dataset.json
│   ├── calibrations/sensor6_v1.json
│   ├── scenes.jsonl
│   └── frames.jsonl
├── annotations/
│   ├── boxes_3d.jsonl
│   └── rejected_frames.jsonl
├── splits/
│   ├── train.txt
│   ├── val.txt
│   └── test.txt
└── fastbev_infos/
    ├── morai_infos_train.pkl
    ├── morai_infos_val.pkl
    └── morai_infos_test.pkl
```

Git에는 변환 코드, schema, split scene ID, 작은 통계·검증 보고서만 저장한다.

### 4.1 Calibration

`Sensor_6.json`을 원본으로 `sensor6_v1.json`을 한 번 생성한다. frame마다 고정
intrinsic과 extrinsic을 반복하지 않고 `calibration_id: sensor6_v1`만 참조한다.

저장할 항목은 다음과 같다.

- Camera별 해상도와 horizontal FOV
- Camera별 intrinsic `K`
- distortion 계수와 적용 가정
- `camera_optical_to_ego`, `ego_to_camera_optical`
- `lidar_to_ego`, `ego_to_lidar`
- 좌표축, 단위, 행렬 convention
- 원본 `Sensor_6.json` SHA-256
- BBox3D 재투영 검증 오차

### 4.2 Frame metadata

Fast-BEV v1은 single-frame이므로 exact temporal Ego pose를 학습 입력으로 요구하지
않는다. 각 accepted frame에는 다음을 기록한다.

- `frame_id`, `scene_id`, `frame_index`
- `calibration_id`
- MORAI timestamp stem
- Camera 4대 RGB와 원본 GT 경로
- LiDAR intensity·semantic·instance·3D box GT 경로
- 품질 검증 결과
- 원본 capture run과 scenario 파일 참조

현재 구형 수집 report의 trigger 시점 Ego `x, y, z`는 보조 정보로 보존할 수 있지만,
정확한 센서 timestamp와 일치하는 full pose가 아니므로 temporal ground truth로
간주하지 않는다.

### 4.3 3D annotation

학습용 box는 원본 GT를 덮어쓰지 않고 별도 `boxes_3d.jsonl`로 만든다.

1. Camera 4대의 `BBox3D.txt`에서 camera-visible 객체를 읽는다.
2. `UNIQUEID`가 같은 객체를 합친다.
3. Camera optical 좌표를 Ego/LiDAR 기준으로 변환한다.
4. LiDAR `_Instance.txt`, `_Instance_8Points.txt`와 위치·크기·yaw를 교차 검증한다.
5. 최종 box, class, instance ID, visible camera를 기록한다.

16-channel LiDAR에 관측된 객체만 정답으로 쓰면 카메라에는 보이지만 LiDAR가 놓친
원거리 객체가 false negative label이 될 수 있으므로 Camera GT와 LiDAR GT를 함께
사용한다.

초기 baseline class는 다음 coarse taxonomy를 기본 후보로 한다.

```text
vehicle
pedestrian
obstacle
```

최종 class mapping은 MORAI DataID와 Camera/LiDAR GT 이름의 일치 여부를 audit한 뒤
고정한다. 학습 중 taxonomy를 변경하지 않는다.

## 5. Fast-BEV 4-camera 포팅과 검증 순서

공식 nuScenes 설정의 6-camera 전제를 현재 4-camera 순서로 변경한다.

```text
front
front_left
front_right
back
```

Front/Back은 1280×720, Left/Right는 640×480이므로 단순 강제 resize로 aspect ratio를
왜곡하지 않는다. 공통 학습 canvas에 letterbox 또는 crop을 적용하고, resize·padding에
맞춰 intrinsic을 정확히 갱신한다.

### 5.1 BEV 좌우 반전 증강 정책

현재 Fast-BEV baseline은 train sample마다 50% 확률로 Ego/LiDAR 좌표계의
좌우축을 반전하는 `flip_ratio_bev_horizontal=0.5`를 사용한다. 원본 파일을
변경하는 방식이 아니라 학습 중 3D box의 `y`, yaw와 camera projection을 함께
변환하는 runtime augmentation이며 validation/test에는 적용하지 않는다. 전후 반전은
`flip_ratio_bev_vertical=0.0`으로 사용하지 않는다.

1,050-frame train split처럼 데이터가 작은 첫 baseline에서는 과적합 완화를 위해
`0.5`를 유지한다. 다만 K-City의 우측통행과 좌·우 객체 분포 자체가 유효한 prior일 수
있으므로, baseline 이후 같은 split과 seed로 `0.0` 또는 `0.2`를 비교하는 ablation을
수행할 가치가 있다. 이 비교 전에는 좌우 반전 비율을 학습 도중 변경하지 않는다.

### 5.2 작은 객체용 BEV 출력 격자

MORAI 기본 port의 `M2BevNeck stride=2`는 0.5 m BEV voxel을 다시 downsample하여
1.0 m 간격의 detection grid를 만들었다. 21-frame train subset에서 초기 anchor와
GT의 최대 IoU를 비교했을 때 IoU 0.5 이상 anchor를 갖는 비율은 vehicle 88.1%인
반면 pedestrian 1.3%, obstacle 0%였다. anchor 종류만 늘리는 방식은 중심 격자
오차를 해결하지 못했다.

`neck_3d.stride=1`로 0.5 m grid를 보존한 100-epoch overfit ablation 결과는 다음과
같다. 동일한 21 frame과 기존 anchor를 사용했다.

| 항목 | stride=2 | stride=1 |
|---|---:|---:|
| FP16 inference FPS | 50.93 | 41.87 |
| inference p95 | 21.58 ms | 24.48 ms |
| mAP@0.25 | 0.4373 | 0.9243 |
| mAP@0.50 | 0.2979 | 0.5972 |
| vehicle AP@0.50 | 0.8728 | 0.9526 |
| pedestrian AP@0.50 | 0.0060 | 0.5186 |
| obstacle AP@0.50 | 0.0150 | 0.3205 |

약 18%의 FP16 처리량 감소 후에도 30 Hz를 넘으며 세 클래스가 모두 크게 개선되어,
정식 MORAI Fast-BEV 설정은 `stride=1`을 채택한다. 기존 stride=2 overfit 설정은
비교 재현용으로 명시적으로 고정한다. anchor 종류 증가는 head 비용을 더 늘리므로
첫 1,050-frame baseline 결과를 보기 전에는 적용하지 않는다.

### 5.3 Obstacle 크기 다양성 후속 과제

현재 `obstacle`은 상자, 배럴, 표지판, 콘, 자전거·스쿠터형 물체처럼 크기와 종횡비가
서로 다른 asset을 하나의 class와 `[1.0, 1.0, 1.0] m` anchor로 학습한다. train split의
1,521개 obstacle GT 크기를 확인한 결과 중앙값은 약 `0.50x0.60x1.03 m`, 90 percentile은
약 `1.65x1.92x3.01 m`, 전체 범위는 `0.13x0.11x0.03 m`부터
`1.88x3.21x6.44 m`까지다. 이 다양성은 높은 center AP에 비해 obstacle mASE와
IoU AP@0.50이 낮은 원인 후보다.

첫 production baseline 전에는 class 정의를 바꾸지 않고 현재 설정을 유지한다. 이후
다음 순서로 별도 ablation을 수행한다.

1. 약 `0.13x0.11x0.03 m`인 퇴화 GT 43개의 asset 이름을 역추적한다.
2. 대회 OD 대상이 아닌 바닥형·잘못된 asset이면 이름 기반으로 dataset 변환에서 제외한다.
3. 정상 train obstacle만 log-space clustering하여 다중 anchor 후보를 만든다.
4. 단일 anchor와 다중 anchor를 동일 split·seed·epoch로 비교한다.
5. obstacle mASE/IoU 개선과 vehicle·pedestrian 성능, FPS, VRAM 변화를 함께 판단한다.

현재 train 분포에서 얻은 초기 3-anchor 후보는 다음과 같으며 확정값은 아니다.

```text
small/low:  0.45 x 0.45 x 0.76 m
tall/thin:  0.37 x 0.53 x 3.22 m
bulk/wide:  0.84 x 1.42 x 0.99 m
```

출력 class는 우선 `obstacle` 하나로 유지하고 내부 anchor template만 늘리는 방향을
우선한다. 의미 subclass 분리는 다중 anchor로도 해결되지 않을 때만 검토한다. 평가에서는
center AP/mATE와 mASE/IoU AP를 함께 보고 위치 검출과 크기 회귀 문제를 분리한다.

학습 전 검증 순서는 다음과 같다.

1. Camera 3D corner를 이미지에 재투영하여 저장 BBox pixel과 비교
2. 네 Camera box를 Ego BEV에 그려 서로 같은 객체가 겹치는지 확인
3. LiDAR point cloud와 변환된 3D box overlay 확인
4. 10~20 frame으로 dataset loader smoke test
5. 10~20 frame에 의도적으로 overfit하여 loss와 예측 box 확인
6. 1,050 train frame 전체 fine-tuning
7. validation으로 checkpoint와 threshold 결정
8. test 222 frame은 마지막 한 번만 평가

baseline 보고서에는 최소한 다음을 기록한다.

- 전체와 class별 AP·Recall
- 거리 구간별 AP·Recall
- camera-visible 객체 기준 누락률
- 3D translation·scale·orientation error
- inference latency와 GPU memory
- 대표 true positive, false positive, false negative 시각화

## 6. StreamPETR 전환 전에 고칠 수집 메타데이터

기존 1,500 frame은 Fast-BEV 1F baseline에 사용한다. StreamPETR temporal 학습에는
센서 시점과 정확히 맞는 Ego pose가 필요하므로 metadata를 보강한 뒤 새로운 데이터셋
버전으로 다시 수집한다.

새 캡처 스키마는 frame마다 다음을 저장해야 한다.

- Camera/LiDAR 실제 sensor timestamp
- `/Ego_topic`의 timestamp가 있는 position `x, y, z`
- Ego orientation quaternion과 roll, pitch, yaw
- Ego linear velocity와 angular velocity
- 필요 시 acceleration
- `ego_to_global`, `global_to_ego` 4×4 행렬
- `scene_id`, `frame_index`, `prev_exists`
- Camera별 timestamp가 다를 경우 각 시점으로 보간한 Ego pose
- 객체 `UNIQUEID`, pose, velocity
- `calibration_id`

구현은 `/Ego_topic`과 필요 시 `/Object_topic`을 시간순 buffer에 보관하고,
`SaveSensorData`가 만든 MORAI timestamp와 가장 가까운 메시지를 선택하거나 앞뒤 pose를
보간하는 방식으로 한다. trigger 순간의 최신 위치만 저장하는 방식은 temporal GT로
사용하지 않는다.

StreamPETR용 데이터는 다음 조건을 만족해야 한다.

- scene 안에서 frame 시간이 단조 증가할 것
- 실제 `delta_t`가 저장될 것
- 첫 frame은 `prev_exists=false`, 이후 frame은 `true`
- scene 경계에서 temporal memory를 초기화할 수 있을 것
- 이전 Ego 좌표에서 현재 Ego 좌표로의 상대 변환을 계산할 수 있을 것
- 동적 객체를 포함한다면 객체 velocity와 ID 연속성이 검증될 것

## 7. 비교 실험의 해석

Fast-BEV와 StreamPETR의 성능 차이를 temporal modeling 효과로 해석하려면 class,
평가 좌표 범위, calibration, image preprocessing, train/validation/test 환경을 가능한
한 동일하게 유지한다. StreamPETR 데이터가 완전히 새로 수집되므로 최종 비교에서는
다음 두 결과를 분리한다.

1. 현재 1,500-frame Fast-BEV quick baseline: 파이프라인과 fine-tuning 가능성 확인
2. 새 temporal dataset에서의 single-frame baseline 대 StreamPETR: temporal 효과의
   공정한 정량 비교

첫 Fast-BEV 결과와 나중 StreamPETR 결과를 서로 다른 데이터셋에서 바로 비교해
temporal 향상치라고 결론내리지 않는다. 새 temporal dataset에서도 동일 split으로
single-frame 모델을 한 번 평가한 뒤 StreamPETR과 비교한다.

## 8. 단계별 완료 조건

### Stage A: Fast-BEV dataset v1

- 정상 250 scene, 1,500 frame 확정
- scene 단위 175/38/37 split 고정
- calibration 재투영 검사 통과
- Fast-BEV info PKL 생성

### Stage B: Fast-BEV quick baseline

- 소량 overfit 성공
- 전체 train 완료
- validation/test와 정성 시각화 보고서 생성

### Stage C: Temporal metadata capture v2

- timestamp-matched full Ego pose 저장
- 4-camera/LiDAR/GT와 frame metadata 동기화 검증
- 짧은 3-scene smoke test에서 상대 Ego transform 검증

### Stage D: StreamPETR dataset과 비교

- temporal dataset 신규 수집
- 같은 데이터로 1F baseline 재평가
- StreamPETR temporal 모델 학습·평가
- 시간 정보 사용에 따른 성능 향상과 비용 보고

## 9. 2026-08-11 Stage A 및 4-camera port 결과

계획 1~5번 가운데 데이터 준비와 dependency-light loader smoke까지 실제로 수행했다.
대용량 원본 report는 변환 스크립트가 로컬에서만 읽으며 콘솔이나 Git에는 원문을
복제하지 않는다.

| 항목 | 결과 |
|---|---:|
| 원본 완료 scene | 286 |
| 엄격한 파일·GT 검사 통과 후보 | 255 |
| 제외 scene | 31 |
| 최종 선택 | 250 scene / 1,500 frame |
| Train | 175 scene / 1,050 frame |
| Validation | 38 scene / 228 frame |
| Test | 37 scene / 222 frame |
| Vehicle box | 3,299 |
| Pedestrian box | 5,302 |
| Obstacle box | 2,323 |

31개 제외 scene은 주로 0-byte LiDAR Instance label처럼 완료 로그만으로는 잡히지 않았던
GT 불완전 scene이다. 250 scene은 `scene_type`과 topology 조합의 원본 분포를
비례 보존하도록 고정 seed `260811`로 선택했다. split도 동일한 두 항목을 고려하되
scene 단위를 유지했다.

Calibration 검증 결과는 다음과 같다.

- 저장 BBox3D corner 재투영 오차: mean `0.00056 px`, p95 `0.00188 px`,
  p99 `0.00442 px`
- 같은 객체의 Camera box 중심을 LiDAR로 변환한 오차: mean `0.00031 m`,
  p95 `0.00162 m`, max `0.00409 m`
- 극단적인 near-plane text rounding 값은 max로 보고하되 robust pass 판정에서는
  p99 재투영 오차와 Camera/LiDAR 3D 중심 오차를 함께 사용한다.

최종 외부 산출물은 다음 경로에 생성했다.

```text
/home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1
```

재생성 명령은 다음과 같다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/build_fastbev_sunny13_1500_dataset.py \
  --output-root \
  /home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1
```

20-frame 경량 smoke는 각 split에서 통과했다.

```bash
cd ~/AjouNice2026

for split in train val test; do
  python3 dataset/morai_od/scripts/smoke_fastbev_morai_infos.py \
    /home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1 \
    --split "$split" \
    --samples 20
done
```

MORAI dataset·pipeline·config는 다음 integration layer로 분리했다.

```text
dataset/morai_od/fastbev_plugin/
├── morai_dataset.py
├── pipelines.py
└── configs/fastbev_m0_morai_4cam_s384x704.py
```

고정 camera 순서는 `front, front_left, front_right, back`이다. 서로 다른 원본
해상도는 너비 기준 resize 후 `384x704` center crop을 사용해 aspect ratio를
왜곡하지 않으며 Fast-BEV의 augmentation이 projection matrix를 함께 갱신한다.

`fastbev_rtx50`은 Python 3.11, PyTorch 2.7.1+cu128 및 RTX 5060 Ti(sm_120)를
사용한다. MMCV 1.4.0 CUDA ops와 Fast-BEV의 mmdet3d ops를 이 조합에 맞춰 빌드했다.
MORAI native loader는 1,050개 train sample과 4-camera image tensor를 정상 생성했고,
모델 build·CUDA 배치도 통과했다.

구형 Fast-BEV의 실제 학습 경로에는 다음 호환 수정을 적용했다.

- 6-camera 하드코딩을 `num_cameras` 모델 인자로 변경하고 MORAI 설정은 4로 고정
- upstream 4-timestep BEV fuse 1,024 channel을 single-frame 256 channel로 변경
- `torch.meshgrid(..., indexing='ij')`를 명시
- anchor 생성 device가 예측 tensor의 device를 따르도록 변경
- PyTorch 내부 구형 API를 호출하는 `AdamW2` 대신 표준 `AdamW` 사용
- 최신 torchvision에서 사라진 `model_urls` 대신 ResNet-18 weight URL 직접 지정
- 존재하지 않는 upstream `load_from` 경로 제거
- nuScenes DB를 요구하는 evaluator 대신 MORAI LiDAR-box 3D IoU AP/Recall 사용

CPU에서 dataset/model/head 구성, finite loss, backward, AdamW step까지 통과했다. 전체
4-camera backprojection을 포함한 CUDA train smoke는 다음 명령으로 확인한다.

```bash
cd ~/AjouNice2026
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50

python3 dataset/morai_od/scripts/smoke_fastbev_native_train.py
```

이 smoke가 통과한 뒤 10~20 frame overfit으로 넘어간다. 단일 train step 통과만으로
학습 품질이나 calibration 정확도가 검증된 것은 아니다.

### 20-frame overfit

`fastbev_m0_morai_4cam_overfit20.py`는 train 1,050장에 `load_interval=50`을
적용해 시간축으로 퍼진 21장을 사용한다. 랜덤 geometric/image augmentation과 FP16을
끄고, tiny run 전체를 덮던 1,000-iteration warm-up도 제거했다.

먼저 3 epoch로 runner와 checkpoint 저장을 확인한다.

```bash
cd ~/AjouNice2026
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50

export MMDET3D="$PWD/third_party/fastbev"
export PYTHONPATH="$PWD/third_party/fastbev:$PWD/dataset/morai_od${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR=/tmp/matplotlib-fastbev
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

mkdir -p artifacts/fastbev_runs/overfit20_smoke3

python3 third_party/fastbev/tools/train.py \
  dataset/morai_od/fastbev_plugin/configs/fastbev_m0_morai_4cam_overfit20.py \
  --work-dir artifacts/fastbev_runs/overfit20_smoke3 \
  --gpus 1 \
  --seed 0 \
  --no-validate \
  --cfg-options runner.max_epochs=3 total_epochs=3 \
  2>&1 | tee artifacts/fastbev_runs/overfit20_smoke3/console.log
```

3 epoch가 끝나면 별도 work directory에서 30 epoch overfit을 실행한다.

```bash
mkdir -p artifacts/fastbev_runs/overfit20_v1

python3 third_party/fastbev/tools/train.py \
  dataset/morai_od/fastbev_plugin/configs/fastbev_m0_morai_4cam_overfit20.py \
  --work-dir artifacts/fastbev_runs/overfit20_v1 \
  --gpus 1 \
  --seed 0 \
  --no-validate \
  2>&1 | tee artifacts/fastbev_runs/overfit20_v1/console.log
```

`positive_bag_loss`, `negative_bag_loss`, 전체 `loss`가 초반보다 명확하게 감소하고
`epoch_30.pth`가 생성되는지 확인한다. overfit 확인 전에는 1,050장 정식 학습으로
넘어가지 않는다.

30 epoch 이후 동일한 21장에 대한 memorization AP/Recall과 처리량을 측정한다.
여기서 한 frame은 동기화된 4-camera image set 하나다. `inference`는 disk image
load/decode를 제외하되 H2D/scatter·backprojection·head·NMS를 포함하고,
`end-to-end`는 dataloader까지 포함한다.

```bash
python3 dataset/morai_od/scripts/evaluate_fastbev_morai.py
```

처음 5 frame은 CUDA warm-up에서 제외하며, 출력에는 inference/end-to-end FPS,
mean/p50/p95 latency, peak CUDA memory 및 class별 3D AP/Recall이 포함된다.

2026-08-12 RTX 5060 Ti overfit20 v1 결과는 다음과 같다.

| 항목 | 결과 |
|---|---:|
| model inference | 32.434 FPS / 30.832 ms |
| inference p95 | 40.347 ms |
| disk decode 포함 end-to-end | 9.667 FPS / 103.442 ms |
| peak CUDA memory | 394.7 MiB |
| vehicle AP@0.25 / AP@0.50 | 0.8804 / 0.5493 |
| pedestrian AP@0.25 / AP@0.50 | 0.0269 / 0.0004 |
| obstacle AP@0.25 / AP@0.50 | 0.0445 / 0.0000 |

`inference FPS`가 실제 ROS/online 모델 처리 성능에 가까운 값이다. end-to-end 측정은
4개 PNG를 `workers=0`으로 직렬 disk decode하는 진단 경로라서 배포 카메라 입력의
상한으로 해석하지 않는다. 차량 성능으로 projection·좌표계·전체 학습 경로는 정상임을
확인했지만 작은 객체의 정밀 localization은 30 epoch에서 아직 memorization되지 않았다.
정식 1,050장 학습 전 동일 21장을 100 epoch까지 이어 학습해 epoch 부족과 구조 문제를
구분한다.

## BEV 격자 해상도와 ROI 후보

현재 production 설정은 `[-50, 50] m`의 XY 범위를 `200x200` cell로 표현하므로
native BEV voxel은 `0.5x0.5 m`이다. `neck_3d.stride=1`을 사용해 detection head도
이 0.5 m grid를 유지한다. 이는 Fast-BEV 구조의 하한이 아니라 현재 속도와 작은 객체
성능 사이에서 선택한 baseline이다. box center는 anchor에 대한 연속 offset으로
회귀되므로 최종 좌표가 0.5 m 단위로 양자화되는 것은 아니다.

추후 ablation 후보는 다음과 같다.

| 후보 | XY grid | 같은 100x100 m 범위의 cell 수 | 현재 대비 XY cell 수 |
|---|---:|---:|---:|
| baseline | 0.5 m | 200x200 | 1.00x |
| 원본 Fast-BEV 고해상도 계열 | 0.4 m | 250x250 | 1.56x |
| small-object 실험 | 0.25 m | 400x400 | 4.00x |

0.4 m를 먼저 시험하고 pedestrian/obstacle 개선이 부족할 때만 0.25 m를 시험한다.
각 후보는 center AP, IoU AP, FP16 FPS/p95 및 VRAM으로 비교한다.

연산량을 제한하면서 더 촘촘한 격자를 얻기 위한 비대칭 ROI 가능성도 열어 둔다.
초기 후보는 Ego 기준 전방 40 m, 후방 15 m, 좌우 각 25 m이다. 대회에서 전방 인지가
가장 중요하고 후방은 주로 끼어들기 간격 확인에 쓰인다는 운용 특성을 반영한 절충안이다.
다만 현재 anchor range, voxel projection, object range filter, NMS 및 평가 거리 기준이
대칭 `[-50, 50] m`를 전제로 연결되어 있으므로 즉시 production에 적용하지 않는다.
별도 config에서 비대칭 좌표 범위의 일관성, 30 m 전방 객체 보존, 경계 객체 누락 및
속도를 검증한 뒤 채택한다.

## MORAI detection 평가 규약

`fastbev_plugin/morai_detection_eval.py`는 nuScenes detection evaluator의 matching과
101-point recall interpolation을 MORAI ego-frame box에 적용한다. 별도 nuScenes DB는
필요하지 않다.

- 주 지표: 클래스별 center-distance AP@0.5/1/2/4 m 및 그 평균 mAP
- 운용 threshold 참고: 각 거리 기준 best-F1의 precision, recall, score threshold
- TP 진단: 2 m 매칭의 mATE, mASE, mAOE
- 보조 지표: 기존 3D IoU AP@0.25/0.50
- 거리 진단: 0-10, 10-20, 20-30, 30-50 m center AP
- 성능: FP16 inference FPS, mean/p50/p95, end-to-end FPS, peak CUDA memory

현재 GT velocity와 attribute는 신뢰할 수 없으므로 mAVE, mAAE 및 공식 NDS는 계산하지
않는다. `obstacle`은 방향성이 없는 물체와 방향성이 있는 물체가 섞인 통합 class이므로
mAOE는 참고값으로만 해석한다. 학습 중 validation은 center AP와 IoU AP만 계산하고,
최종 checkpoint에서 TP·거리 구간·FPS를 모두 측정한다.

```bash
python3 dataset/morai_od/scripts/evaluate_fastbev_morai.py \
  --config <config.py> \
  --checkpoint <checkpoint.pth> \
  --fp16 \
  --cudnn-benchmark \
  --metrics center,tp,iou \
  --output-json <run-directory>/metrics.json
```

`metrics.json`에는 평가 규약 버전, config/checkpoint 경로, frame 수, runtime 및 모든
scalar metric이 저장된다. 이 MORAI 평가기는 Fast-BEV 서브모듈 밖에 있으므로 향후
StreamPETR 예측도 같은 box 형식으로 변환해 동일 기준으로 비교한다.

## Fast-BEV train/validation/test 실행 규약

고정 split은 scene 단위 `train 1,050 / validation 228 / test 222 frame`이다. train만
optimizer update와 augmentation에 사용하고, validation은 매 epoch 모델 선택에만
사용한다. test는 validation으로 최고 checkpoint를 확정한 뒤 한 번만 공개한다. test
결과를 보고 epoch나 hyperparameter를 다시 선택하면 test leakage가 되므로, 그런 변경이
필요하면 다시 validation으로 결정하고 test는 최종 확인에만 사용한다.

최고 checkpoint 선택 기준은 `center_mAP`이다. 실제 값은 `center/mAP`와 같지만 MMCV가
`save_best` 문자열을 checkpoint 파일명에 포함하므로 `/`가 없는 안전한 alias를 사용한다.
이 metric은 vehicle, pedestrian, obstacle을 동일 가중치로 취급하고 각 class의
AP@0.5/1/2/4 m를 다시 평균한다. 따라서 많이 등장하는 vehicle이 작은 class를 압도하지
않는다.

매 epoch validation은 다음 경량 지표만 계산한다.

- `center_mAP`: best checkpoint 선택값
- 클래스별 center AP@0.5/1/2/4 m
- 클래스별 center mAP

best-F1 operating point, TP error, IoU AP, 거리 구간 및 FPS는 학습 중에는 계산하지 않고
최종 test에서만 계산한다. 최근 일반 checkpoint는 3개만 보존하고 validation best
checkpoint는 별도로 보존한다.

학습은 Fast-BEV 원본 `tools/train.py`를 얇은 launcher로 실행한다.

```bash
cd ~/AjouNice2026

dataset/morai_od/scripts/train_fastbev_morai.sh \
  full1500_stride1_fp16_v1
```

중단된 동일 run을 재개할 때는 원본 train 진입점을 사용한다.

```bash
cd ~/AjouNice2026
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50

export MMDET3D="$PWD/third_party/fastbev"
export PYTHONPATH="$PWD/third_party/fastbev:$PWD/dataset/morai_od${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR=/tmp/matplotlib-fastbev
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python3 third_party/fastbev/tools/train.py \
  dataset/morai_od/fastbev_plugin/configs/fastbev_m0_morai_4cam_s384x704.py \
  --work-dir artifacts/fastbev_runs/full1500_stride1_fp16_v1 \
  --resume-from artifacts/fastbev_runs/full1500_stride1_fp16_v1/latest.pth \
  --gpus 1 \
  --seed 0 \
  2>&1 | tee -a artifacts/fastbev_runs/full1500_stride1_fp16_v1/console.log
```

학습 완료 후 validation best checkpoint를 held-out test 222장에 한 번 평가한다.

```bash
dataset/morai_od/scripts/evaluate_fastbev_morai_test.sh \
  artifacts/fastbev_runs/full1500_stride1_fp16_v1
```

최종 산출물은 `best_center_mAP_epoch_*.pth`, `test_metrics.json`,
`test_evaluation.log`이다. 모델 비교 표에는 test center mAP, 클래스별 AP, TP error,
IoU AP, 거리 구간 결과와 FP16 runtime을 함께 기록한다.

## W&B 학습 모니터링

Fast-BEV 원본 학습 루프를 수정하지 않고 MMCV 1.4의 `WandbLoggerHook`를 사용한다.
기존 text/TensorBoard 로그는 그대로 유지하며 W&B에는 train loss, learning rate,
gradient norm, epoch/iteration, 매 epoch validation `center_mAP`와 클래스별 AP가
기록된다. W&B 클라이언트가 GPU·CPU·메모리 같은 시스템 지표도 별도로 수집한다.

기본 run 식별자는 다음과 같다.

- entity: `AjouNice2026`
- project: `morai-fastbev`
- group: `sunny13-1500-single-frame`
- run name 및 run id: `train_fastbev_morai.sh`의 첫 번째 인자
- tags: `fastbev,morai,4cam,single-frame,s384x704,fp16`
- best checkpoint 기준: `center_mAP`

checkpoint는 파일당 약 180 MB이므로 W&B artifact로 자동 업로드하지 않는다. config와
metric은 W&B에 남고, checkpoint와 전체 MMCV config는 로컬 work directory에 남는다.
최종 best checkpoint만 필요할 때 별도 artifact로 올리는 방식을 사용한다.

처음 한 번만 환경을 준비하고 로그인한다. API key는 코드, `.env`, 문서 또는 Git에
저장하지 않는다.

```bash
cd ~/AjouNice2026

dataset/morai_od/scripts/setup_fastbev_wandb.sh

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50
wandb login
```

이후 평소 학습 명령만 실행하면 W&B가 자동 활성화된다.

```bash
cd ~/AjouNice2026

dataset/morai_od/scripts/train_fastbev_morai.sh \
  full1500_stride1_fp16_v1
```

실험별 project/group/tag를 바꾸려면 실행 전 환경변수를 지정한다.

```bash
export WANDB_PROJECT=morai-fastbev
export WANDB_RUN_GROUP=grid-ablation
export WANDB_TAGS=fastbev,morai,4cam,grid-0.4m,fp16

dataset/morai_od/scripts/train_fastbev_morai.sh \
  full1500_grid04_fp16_v1
```

인터넷 없이 로컬 W&B 기록만 시험할 때는 `WANDB_MODE=offline`, W&B 자체를 제외하고
학습할 때는 `FASTBEV_DISABLE_WANDB=1`을 사용한다. 같은 run name으로 checkpoint 학습을
재개할 때는 `WANDB_RUN_ID`와 `resume=allow`에 의해 같은 W&B run으로 이어진다.
