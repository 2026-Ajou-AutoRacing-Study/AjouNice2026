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

기존 12-epoch PIL baseline은 train sample마다 Ego/LiDAR 좌표계의 좌우축을
`flip_ratio_bev_horizontal=0.5`로 반전했고, image horizontal flip도 50%로
사용했다. 원본 파일을 바꾸는 것이 아니라 학습 중 3D box의 `y`, yaw와 camera
projection을 함께 바꾸는 runtime augmentation이며 validation/test에는 적용하지
않는다. 전후 반전은 `flip_ratio_bev_vertical=0.0`으로 사용하지 않는다.

후속 deployment-matched OpenCV 학습은 K-City 우측통행과 고정된 네 camera role을
유효한 prior로 보존하기 위해 BEV horizontal flip을 10%, image horizontal flip을
20%로 낮춘다. 같은 1,050/228/222 split, seed와 12-epoch schedule을 유지해 기존
PIL/50% baseline과 비교한다. 실제 확률은 W&B run config에 남기며 학습 도중에는
변경하지 않는다.

2026-08-14 결합 실험에서 OpenCV/10%/20% 후보의 test center mAP는 `0.5753`으로
PIL/50%/50% baseline의 `0.6001`보다 0.0248 낮았다. vehicle mean AP는
`0.6874 -> 0.6884`로 유지됐지만 obstacle mean AP는 `0.4483 -> 0.3888`로
낮아졌다. 반면 offline end-to-end FPS는 `10.88 -> 13.56`으로 증가했다. 이 결과는
backend와 두 flip 확률을 동시에 변경했으므로 단일 요인의 인과 비교는 아니다.
PIL 모델을 정확도 baseline, OpenCV 모델을 실시간 배포 후보로 각각 보존하고 다음
ablation에서는 OpenCV를 고정한 채 flip 확률만 비교한다.

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

### 6.1 Temporal augmentation 원칙

Fast-BEV single-frame 학습은 각 sample에서 독립적으로 augmentation을 뽑는다.
StreamPETR처럼 여러 frame을 하나의 학습 sample로 사용하는 모델에서는 한 sequence의
모든 frame에 동일한 geometry parameter를 적용한다.

```text
하나의 sequence augmentation 추첨
  -> t0의 front/left/right/back
  -> t1의 front/left/right/back
  -> ...
  -> tN의 front/left/right/back
```

공유 대상은 BEV flip, global rotation/scale/translation과 image
resize/crop/flip/rotation이다. 각 frame에 독립적인 geometry를 적용하면 정적 물체도
시간축에서 위치·크기·방향이 흔들리는 인공 motion이 생길 수 있다. 모든 frame의 3D
annotation, Ego pose, sensor projection에는 같은 transform을 일관되게 반영한다.

Photometric augmentation은 여섯 개로 제한된 MORAI 실제 환경 조합을 직접 수집할 수
있으므로 현재 보류한다. 추후 필요성이 확인되면 view와 temporal frame 사이에서 같은
밝기·색상 parameter를 공유하는 약한 sequence-level 변환부터 검증한다.

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
  python3 perception/detection3d/fastbev/scripts/smoke_infos.py \
    /home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1 \
    --split "$split" \
    --samples 20
done
```

MORAI dataset·pipeline·config는 다음 integration layer로 분리했다.

```text
perception/detection3d/fastbev/
├── plugin/
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

python3 perception/detection3d/fastbev/scripts/smoke_train.py
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
export PYTHONPATH="$PWD:$PWD/third_party/fastbev${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR=/tmp/matplotlib-fastbev
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

mkdir -p artifacts/detection3d/fastbev/runs/overfit20_smoke3

python3 third_party/fastbev/tools/train.py \
  perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_overfit20.py \
  --work-dir artifacts/detection3d/fastbev/runs/overfit20_smoke3 \
  --gpus 1 \
  --seed 0 \
  --no-validate \
  --cfg-options runner.max_epochs=3 total_epochs=3 \
  2>&1 | tee artifacts/detection3d/fastbev/runs/overfit20_smoke3/console.log
```

3 epoch가 끝나면 별도 work directory에서 30 epoch overfit을 실행한다.

```bash
mkdir -p artifacts/detection3d/fastbev/runs/overfit20_v1

python3 third_party/fastbev/tools/train.py \
  perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_overfit20.py \
  --work-dir artifacts/detection3d/fastbev/runs/overfit20_v1 \
  --gpus 1 \
  --seed 0 \
  --no-validate \
  2>&1 | tee artifacts/detection3d/fastbev/runs/overfit20_v1/console.log
```

`positive_bag_loss`, `negative_bag_loss`, 전체 `loss`가 초반보다 명확하게 감소하고
`epoch_30.pth`가 생성되는지 확인한다. overfit 확인 전에는 1,050장 정식 학습으로
넘어가지 않는다.

30 epoch 이후 동일한 21장에 대한 memorization AP/Recall과 처리량을 측정한다.
여기서 한 frame은 동기화된 4-camera image set 하나다. `inference`는 disk image
load/decode를 제외하되 H2D/scatter·backprojection·head·NMS를 포함하고,
`end-to-end`는 dataloader까지 포함한다.

```bash
python3 perception/detection3d/fastbev/tools/evaluate.py
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

`perception/detection3d/common/evaluation/morai_detection.py`는 nuScenes
detection evaluator의 matching과
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

### 향후 복합 지표: GDQS

공식 NDS를 흉내 내거나 특정 시뮬레이터 이름을 붙이지 않고, 현재 확보한 GT만으로
검출과 3D box 품질을 함께 비교하는 프로젝트 지표의 이름은
`GDQS (Geometry-aware Detection Quality Score)`로 정한다. 정적 객체 중심 데이터라
신뢰할 수 없는 velocity와 attribute 항목을 제외하고, 검출 성능과 3D box의 위치·크기·
방향 정확도를 결합한다. 초기 후보 정의는 다음과 같다.

```text
GDQS =
  0.625 * center_mAP
  0.125 * position_score
  0.125 * scale_score
  0.125 * orientation_score

position_score    = max(0, 1 - mATE / 1.0 m)
scale_score       = max(0, 1 - mASE / 0.5)
orientation_score = max(0, 1 - mAOE / (pi / 2))
```

`GDQS/vehicle`, `GDQS/pedestrian`, `GDQS/obstacle`을 먼저 계산하고 전체 `GDQS`는
세 class 점수의 동일 가중 평균으로 계산한다. vehicle 표본 수가 pedestrian과 obstacle을
압도하지 않게 하기 위함이다. 세 class 모두 orientation score를 포함한다. 차량은 미래
궤적, 보행자는 횡단 방향, obstacle은 바리케이드·자전거·킥보드처럼 길쭉한 물체의 점유
방향을 평가할 가치가 있기 때문이다.

단, 이 지표는 아직 설계안이며 현재 best checkpoint 기준은 `center_mAP`이다. 공식 NDS와
비교하거나 논문 수치에 NDS로 표기하지 않는다. GDQS를 도입하더라도 center AP, mATE,
mASE, mAOE 원본 지표를 항상 함께 보존한다.

구현 전에 다음 항목을 검증·개선한다.

1. MORAI asset별 GT yaw 축과 차량·보행자 heading의 좌표계가 일관적인지 확인한다.
2. 차량과 이동 방향이 정의된 보행자는 directed yaw를 사용한다. 긴 대칭 obstacle은
   필요하면 0도와 180도를 같은 축으로 보는 axial yaw 평가를 asset subtype별로 적용한다.
3. 원형 obstacle처럼 방향 의미가 약한 asset도 우선 공통 mAOE에 포함하되, 해당 asset이
   전체 obstacle 점수를 불안정하게 만드는지 별도 분석한다.
4. 1.0 m, 0.5, 90도의 정규화 한계와 62.5/12.5/12.5/12.5 가중치는 validation에서
   고정한 뒤 test 결과에 맞춰 변경하지 않는다.
5. temporal dataset에서 신뢰 가능한 velocity GT를 확보하면 velocity score를 추가한
   새 버전으로 명시적으로 올린다. 이전 GDQS 버전과 수치를 직접 혼용하지 않는다.
6. 최종 prediction 목적에서는 GDQS만으로 결정하지 않고 yaw flip rate, temporal yaw
   jitter, tracking ID metric, ADE/FDE, miss rate를 함께 평가한다.
7. 향후 class별 중요도를 달리하거나 vehicle yaw 비중을 높인 prediction 전용 점수가
   필요하면 GDQS와 분리해 이름과 용도를 명시한다.

#### 2026-08-12 pedestrian yaw 진단

GDQS는 아래 문제가 해결될 때까지 문서상 후보 지표로만 보존하고 best checkpoint
선택에는 사용하지 않는다. 현재 best 기준은 계속 `center_mAP`이다.

12-epoch baseline의 held-out test에서 pedestrian mAOE는 `1.672 rad (95.8 deg)`였다.
`tools/diagnose_yaw.py`로 동일 checkpoint의 2 m center-matched TP를 다시 조사한 결과는
다음과 같다. 진단 도구의 단순 match 평균은 공식 mAOE의 recall interpolation과 계산법이
달라 숫자가 완전히 같지는 않다.

| class | directed yaw 평균 | 180도 대칭 axial yaw 평균 | 중앙값 |
|---|---:|---:|---:|
| vehicle | 34.4 deg | 11.5 deg | 3.2 deg |
| pedestrian | 95.5 deg | 47.8 deg | 94.7 deg |
| obstacle | 25.8 deg | 11.8 deg | 11.8 deg |

확인된 사실은 다음과 같다.

- 시나리오 pedestrian yaw와 Ego/LiDAR의 상대 yaw가 `_Instance.txt` yaw와 일치한다.
- `_Instance_8Points.txt` corner로 얻은 box 축도 `_Instance.txt` yaw와 일치한다.
- vehicle과 obstacle은 같은 변환·평가 경로에서 낮은 axial yaw 오차를 보인다.
- pedestrian 오차는 고정 `+90/-90/180 deg` 보정으로 개선되지 않는다. 따라서 전 class에
  공통인 좌표계 offset이나 단순 부호 오류가 아니다.
- 0--10 m pedestrian도 평균 약 `97.1 deg`로, 원거리 해상도만의 문제도 아니다.
- 종횡비가 1.25 이하인 pedestrian과 더 길쭉한 pedestrian 모두 약 94--96 deg로,
  정사각형 box만의 문제도 아니다.
- test GT pedestrian yaw의 circular concentration은 `0.109`로 거의 균등하지만 예측은
  `0.338`로 일부 방향에 치우쳤다. 현재 모델이 임의 GT 방향을 충분히 복원하지 못했다.
- 같은 파이프라인의 21-frame overfit은 pedestrian mAOE `0.652 rad (37.4 deg)`까지
  내려갔다. 따라서 yaw 학습 경로가 완전히 끊긴 구현 오류라기보다 full baseline의
  일반화·학습 문제가 주원인이다.

가장 가능성 높은 원인은 다음 조합이다.

1. 생성기가 정적 pedestrian yaw를 `[-180, 180] deg`에서 균등 무작위로 배치한다.
   도로 방향이나 이동 경로 prior가 없으므로 모델은 작은 사람의 시각적 앞뒤만으로
   방향을 회귀해야 한다.
2. pedestrian BEV box는 중앙 종횡비가 약 `1.5`이고 약 23%는 1.25 이하로 방향 단서가
   약하다. 현재 pedestrian anchor도 `0.6 x 0.6 m` 정사각형이다.
3. 384x704로 축소한 camera-only 입력, 1,050 train frame, 12 epoch 및 새 3D detection
   head 조합에서는 보행자 orientation 일반화가 부족하다.
4. 정적 보행자의 asset yaw가 실제 보행 진행 방향을 의미하는지는 아직 별도 검증하지
   않았다. BBox 기하 축은 일관되지만 semantic facing direction과 같은지는 다를 수 있다.

다음 개선 순서는 다음과 같다.

1. simulator에서 동일 pedestrian asset을 yaw 0/90/180/270 deg로 놓고, 사람의 얼굴·
   진행 방향과 `_Instance.txt` yaw 축의 관계를 한 번 확인한다.
2. 동적 횡단 pedestrian은 yaw를 waypoint tangent 및 velocity 방향과 정렬하고 frame별
   velocity GT를 저장한다. prediction에는 static asset yaw보다 velocity heading을
   우선 사용한다.
3. pedestrian anchor를 실제 크기 통계에 맞춘 비정사각형 후보와 비교하고, class별 yaw
   loss/방향 정확도를 별도로 기록한다.
4. yaw 학습 여부를 확인하는 소규모 train/validation ablation을 먼저 수행한 뒤 전체
   학습을 반복한다. center mAP가 비슷할 때 pedestrian mAOE가 실제로 낮아지는지 본다.
5. 인접 pedestrian이 2 m center matching에서 서로 바뀌는 영향을 0.5/1 m matching과
   Hungarian matching으로 추가 비교한다.
6. directed yaw가 신뢰 가능해진 뒤에만 GDQS를 shadow best 기준으로 기록하고,
   `best_center_mAP`와 여러 run에서 비교한 후 주 기준으로 승격한다.

진단 명령은 다음과 같다.

```bash
python3 perception/detection3d/fastbev/tools/diagnose_yaw.py \
  --config perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py \
  --checkpoint <checkpoint.pth> \
  --fp16
```

#### Obstacle yaw 해석과 제한

Obstacle도 pedestrian과 다른 이유로 directed yaw 추정이 어렵다. 현재 test의 공식
obstacle mAOE는 `0.326 rad (18.7 deg)`이고 단순 TP 진단의 directed/axial 평균은 각각
`25.8/11.8 deg`다. 수치만 보면 pedestrian보다 좋지만 그대로 일반화하면 안 된다.

- obstacle 통합 class에는 원형·정사각형에 가까운 물체와 바리케이드·자전거·킥보드·
  표지판처럼 길쭉한 물체가 함께 들어 있다. test GT의 약 27%는 BEV 종횡비 1.25 이하라
  yaw의 시각적·기하학적 의미가 약하다.
- 긴 대칭 물체는 0도와 180도가 같은 점유 축을 만든다. 이 경우 directed yaw는 실제
  planning geometry가 동일한 예측도 180도 오차로 벌한다.
- 반대로 앞뒤가 구분되는 표지판이나 이동 가능한 자전거·킥보드는 directed yaw가 의미
  있을 수 있다. obstacle 전체에 한 가지 yaw 규칙을 적용하면 둘 중 하나를 왜곡한다.
- 현재 test obstacle GT yaw의 circular concentration은 `0.977`로 대부분 한 방향에
  몰려 있다. 예측도 `0.814`로 같은 방향에 치우쳐 있어, 낮은 mAOE 일부는 다양한
  방향을 복원한 성능이 아니라 데이터 방향 편향을 이용한 결과일 수 있다.
- 종횡비 1.25 이하 obstacle의 directed 평균은 `42.8 deg`, 길쭉한 obstacle은
  `20.7 deg`였다. 작은·대칭 물체의 방향이 전체 평균을 불안정하게 만들 수 있다.
- 현재 obstacle anchor는 `1.0 x 1.0 x 1.0 m` 하나다. 실제 obstacle의 큰 크기 다양성과
  방향성을 하나의 정사각형 anchor로 표현하므로 class별 크기·yaw 회귀에 불리하다.

따라서 obstacle yaw도 현재 GDQS에 즉시 반영하지 않는다. 향후 asset catalog의 subtype과
실제 GT box 종횡비를 연결해 다음처럼 평가하는 방안을 검증한다.

1. 앞뒤 의미가 있는 subtype은 directed yaw를 사용한다.
2. 길지만 앞뒤가 대칭인 subtype은 axial yaw를 사용한다.
3. 원형 또는 거의 정사각형이고 방향 의미가 없는 subtype은 orientation score에서
   제외하고 나머지 GDQS 항목을 재정규화한다.
4. 방향별로 균형 잡힌 obstacle validation subset을 만들어, 현재 낮은 mAOE가 방향
   편향 때문인지 실제 orientation 학습 때문인지 구분한다.
5. subtype 또는 크기 군집별 anchor와 class 분리 필요성을 obstacle 크기 다양성 실험과
   함께 검증한다.

요약하면 pedestrian은 현재 모델이 무작위 directed yaw를 거의 학습하지 못한 문제이고,
obstacle은 heterogeneous asset·대칭성·방향 분포 편향 때문에 하나의 mAOE가 의미를
섞는 문제다. 둘 다 정리되기 전까지 GDQS는 보류한다.

```bash
python3 perception/detection3d/fastbev/tools/evaluate.py \
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

매 epoch validation은 다음 진단 지표를 계산한다.

- `center_mAP`: best checkpoint 선택값
- 클래스별 center AP@0.5/1/2/4 m
- 클래스별 `center/{class}_mean_AP` (AP@0.5/1/2/4 m의 평균)
- 클래스별 `tp/{class}_mATE`, `mASE`, `mAOE`
- 클래스별 `operating/{class}_1m_precision`, `recall`, `F1`, `score`
- 클래스별·전체 IoU AP/AR@0.25/0.50
- 0--10 m, 10--20 m, 20--30 m의 클래스별 mean AP와 전체 mAP

거리 구간의 threshold별 상세 AP, GT/prediction 수, 30--50 m 진단 및 FPS는 최종
test에서만 계산한다. 최근 일반 checkpoint는 3개만 보존하고 validation best
checkpoint는 별도로 보존한다. best checkpoint 기준은 계속 `center_mAP`이며 추가
지표는 진단과 ROS confidence threshold 결정에 사용한다.

학습은 Fast-BEV 원본 `tools/train.py`를 얇은 launcher로 실행한다.

```bash
cd ~/AjouNice2026

perception/detection3d/fastbev/scripts/train.sh \
  full1500_stride1_fp16_v1
```

중단된 동일 run을 재개할 때는 원본 train 진입점을 사용한다.

```bash
cd ~/AjouNice2026
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50

export MMDET3D="$PWD/third_party/fastbev"
export PYTHONPATH="$PWD:$PWD/third_party/fastbev${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR=/tmp/matplotlib-fastbev
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python3 third_party/fastbev/tools/train.py \
  perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py \
  --work-dir artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_v1 \
  --resume-from artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_v1/latest.pth \
  --gpus 1 \
  --seed 0 \
  2>&1 | tee -a artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_v1/console.log
```

학습 완료 후 validation best checkpoint를 held-out test 222장에 한 번 평가한다.

```bash
perception/detection3d/fastbev/scripts/evaluate_test.sh \
  artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_v1
```

최종 산출물은 `best_center_mAP_epoch_*.pth`, `test_metrics.json`,
`test_evaluation.log`이다. 모델 비교 표에는 test center mAP, 클래스별 AP, TP error,
IoU AP, 거리 구간 결과와 FP16 runtime을 함께 기록한다.

## Scratch baseline의 클래스별 개선 가능성

Sunny 13시 1,500-frame split에서 ImageNet backbone과 나머지 계층을 scratch로 학습한
12-epoch baseline은 validation best epoch 11에서 `center_mAP=0.6081`, held-out test에서
`center_mAP=0.6001`을 기록했다. validation과 test 차이가 작아 현재 split에 대한
일반화는 안정적이지만, 클래스와 거리에 따라 병목이 다르다.

### Vehicle

- test mean AP는 `0.6874`이며 AP@2 m는 `0.8599`로 객체 존재 검출은 안정적이다.
- AP@0.5 m는 `0.3326`이므로 정밀 중심 위치가 주된 병목이다.
- 10--20 m mean AP는 `0.7174`, 20--30 m는 `0.4324`로 작은 객체보다 원거리 저하가
  완만하다.
- 데이터 추가로 개선될 수 있지만 이미 가장 강한 클래스이므로 절대 상승 폭은 다른
  클래스보다 작을 가능성이 있다.
- sedan/SUV/bus/truck 크기 통계에 맞춘 다중 anchor, 좌표 변환 검증, pretrained
  backbone/FPN이 우선 후보이다.

### Pedestrian

- test mean AP는 `0.6645`이고 0--10 m mean AP는 `0.9306`으로 근거리는 강하다.
- 10--20 m에서 `0.3472`, 20--30 m에서 `0.0686`으로 원거리 small-object 성능이
  급격히 하락한다.
- 10--30 m 표본, 부분 가림, 카메라 경계, 다양한 자세를 늘리면 개선 가능성이 높다.
- NuImages 등 2D pretrained backbone/FPN, 전방 고해상도 활용, small-object
  augmentation과 pedestrian anchor 통계 비교가 유효하다.

### Obstacle

- test mean AP는 `0.4483`, IoU AP@0.50은 `0.0878`로 세 클래스 중 가장 약하다.
- 0--10 m mean AP는 `0.7350`이지만 10--20 m는 `0.2565`, 20--30 m는 `0.0420`이다.
- cone, barrel, sign, cargo box, bicycle/scooter처럼 크기와 형상이 다른 객체를 하나의
  클래스와 대표 anchor로 회귀하는 class 내부 다양성이 핵심 병목이다.
- 데이터 수만 늘리기보다 asset별 크기 분포를 먼저 분석하고 obstacle 다중 anchor,
  class-balanced sampling/loss, 필요 시 학습용 세부 taxonomy 후 추론 시 obstacle 통합을
  비교한다.

### 공통 실험 순서

단순히 현재 12-epoch schedule의 epoch만 늘리면 마지막 learning rate가 거의 0이므로
효과가 제한적이다. 한 번에 한 요소만 바꿔 다음 순서로 validation에서 비교한다.

1. 현재 scratch baseline을 고정한다.
2. 같은 데이터·seed·schedule에서 NuImages pretrained backbone/FPN을 비교한다.
3. 다양한 거리·가림·asset을 포함한 약 3,000 frame 데이터로 비교한다.
4. 학습 곡선이 계속 상승하면 LR schedule과 함께 18--24 epoch를 비교한다.
5. obstacle 크기 통계에 근거한 다중 anchor를 비교한다.

Epoch와 실험 설정 선택은 validation으로 수행하고, 채택한 최종 후보만 test로 보고한다.
이번 test에서 확인한 원거리 약점은 다음 데이터 버전의 설계 근거로 기록하되, 동일 test를
반복적으로 튜닝 기준으로 사용하지 않는다.

## W&B 학습 모니터링

Fast-BEV 원본 학습 루프를 수정하지 않고 MMCV 1.4의 `WandbLoggerHook`를 상속한
로컬 `MoraiWandbLoggerHook`를 사용한다.
기존 text/TensorBoard 로그는 그대로 유지하며 W&B에는 train loss, learning rate,
gradient norm, `epoch`, `global_step`, 매 epoch validation `center_mAP`와 클래스별 AP가
기록된다. train metric은 `global_step`, validation metric은 `epoch`를 기본 축으로
정의한다. MMCV의 기본 `_step=runner.iter`는 epoch-end validation 뒤 다음 train
로그보다 커져 W&B가 일부 값을 버릴 수 있으므로 사용하지 않는다. W&B 내부 `_step`은
로그 이벤트 순서로만 유지하고 차트 축에는 `global_step` 또는 `epoch`를 사용한다.
클래스별 네 거리 threshold 평균은 클래스 평균과 혼동하지 않도록
`vehicle_mean_AP`, `pedestrian_mean_AP`, `obstacle_mean_AP`로 기록한다. W&B
클라이언트가 GPU·CPU·메모리 같은 시스템 지표도 별도로 수집한다. 상세 validation
metric은 자동 패널 생성만 숨기고 history는 모두 보존한다. Workspace에서는 관련
metric 여러 개를 한 패널에 묶어 공용 dashboard를 한 번 구성해 이후 run에 재사용한다.

기본 run 식별자는 다음과 같다.

- entity: `AjouNice2026`
- project: `morai-fastbev`
- group: `sunny13-1500-single-frame`
- run name 및 run id: `fastbev/scripts/train.sh`의 첫 번째 인자
- tags: `fastbev,morai,4cam,single-frame,s384x704,fp16`
- best checkpoint 기준: `center_mAP`

checkpoint는 파일당 약 180 MB이므로 W&B artifact로 자동 업로드하지 않는다. config와
metric은 W&B에 남고, checkpoint와 전체 MMCV config는 로컬 work directory에 남는다.
최종 best checkpoint만 필요할 때 별도 artifact로 올리는 방식을 사용한다.

처음 한 번만 환경을 준비하고 로그인한다. API key는 코드, `.env`, 문서 또는 Git에
저장하지 않는다.

```bash
cd ~/AjouNice2026

perception/detection3d/fastbev/scripts/setup_wandb.sh

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate fastbev_rtx50
wandb login
```

이후 평소 학습 명령만 실행하면 W&B가 자동 활성화된다.

```bash
cd ~/AjouNice2026

perception/detection3d/fastbev/scripts/train.sh \
  full1500_stride1_fp16_v1
```

실험별 project/group/tag를 바꾸려면 실행 전 환경변수를 지정한다.

```bash
export WANDB_PROJECT=morai-fastbev
export WANDB_RUN_GROUP=grid-ablation
export WANDB_TAGS=fastbev,morai,4cam,grid-0.4m,fp16

perception/detection3d/fastbev/scripts/train.sh \
  full1500_grid04_fp16_v1
```

인터넷 없이 로컬 W&B 기록만 시험할 때는 `WANDB_MODE=offline`, W&B 자체를 제외하고
학습할 때는 `FASTBEV_DISABLE_WANDB=1`을 사용한다. 같은 run name으로 checkpoint 학습을
재개할 때는 `WANDB_RUN_ID`와 `resume=allow`에 의해 같은 W&B run으로 이어진다.
