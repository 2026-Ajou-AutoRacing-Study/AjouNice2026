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

4-camera port는 원본 submodule을 직접 수정하지 않고 다음 integration layer로 분리했다.

```text
dataset/morai_od/fastbev_plugin/
├── morai_dataset.py
├── pipelines.py
└── configs/fastbev_m0_morai_4cam_s384x704.py
```

고정 camera 순서는 `front, front_left, front_right, back`이다. 서로 다른 원본
해상도는 너비 기준 resize 후 `384x704` center crop을 사용해 aspect ratio를
왜곡하지 않으며 Fast-BEV의 augmentation이 projection matrix를 함께 갱신한다.

현재 host와 `openpcdet_rtx50`, `openpcdet_team` 환경에는 `mmcv`, `mmdet`,
`mmdet3d`가 설치되어 있지 않다. 따라서 이번 단계에서는 생성 PKL·4-camera 순서·이미지
decode·box shape·회전행렬을 검사하는 dependency-light smoke까지 수행했다. Fast-BEV
공식 loader를 실제 import하는 smoke와 10~20 frame overfit은 Fast-BEV 전용 환경을
구성한 다음 Stage B에서 수행한다. 이 제약 때문에 이번 결과를 학습 성공으로 해석하지
않는다.
