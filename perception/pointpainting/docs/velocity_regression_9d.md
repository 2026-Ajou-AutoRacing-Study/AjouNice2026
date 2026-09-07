# PointPainting 9D velocity regression

## 목적

- 기존 PointPainting box `[x, y, z, dx, dy, dz, yaw]`에 절대 객체 속도
  `[vx, vy]`를 추가한다.
- 출력은 LiDAR 축 기준 9D box이다.
- ROS 표준 `DetectedObjects`에 넣을 때는 box yaw를 사용해 객체 body 축
  twist로 변환한다.
- EKF의 detector velocity fusion은 기존 위치 차분을 완전히 대체하지
  않고, 측정 update 하나로 함께 fusion한다.

## 학습 GT

- 정적 1,500 frame과 동적 1,500 frame을 합친 3,000 frame을 사용한다.
- split은 scene 단위로 고정된 train/val/test `2,100/450/450`이다.
- 정적 객체와 obstacle은 `vx=vy=0`이다.
- 동적 차량·보행자는 `/Object_topic` map position을 물리 simulator time으로
  중앙차분한 velocity를 사용한다. MORAI가 real time보다 느리게 실행될 때 ROS
  header/wall time을 그대로 쓰면 속도가 축소되므로 position/raw-speed로 scene
  simulator time을 복원한다.
- MORAI 메시지의 raw velocity 필드는 월드 XY m/s가 아니므로 직접 라벨로
  사용하지 않는다.
- 같은 frame의 actor state가 빠졌을 때에만 같은 scene의 1.5초 이내 가장
  가까운 state를 사용한다.
- 신뢰 가능한 속도가 없는 동적 box는 0으로 위장하지 않고 제외·보고한다.

생성 결과.

- 총 3,000 frame.
- 실제 속도가 연결된 동적 vehicle 935 box.
- 실제 속도가 연결된 동적 pedestrian 287 box.
- 속도 이력이 없어 제외된 vehicle 2 box.
- 산출물 경로:
  `artifacts/datasets/pointpainting_sunny13_static1500_dynamic1500_velocity9d_v1`.

## 모델 변경

- class, anchor, PointPainting 9-feature 입력은 Epoch65 production과 같다.
- `ResidualCoder`만 7D에서 9D로 확장한다.
- localization code weight는 geometry 7개 `1.0`, velocity 2개 `0.2`이다.
- 기존 Epoch65 `conv_box`의 각 anchor별 geometry 7채널은 그대로 복사한다.
- 새 velocity 2채널만 0으로 초기화한다.
- optimizer state는 제거하고 새 fine-tuning run으로 시작한다.

초기 checkpoint.

`artifacts/detection3d/pointpainting/releases/260820_sunny13_static1500_dynamic1500_velocity9d_init/pointpainting_B0_epoch65_velocity9d_init.pth`.

## 데이터 준비

```bash
cd ~/AjouNice2026

python3 perception/pointpainting/tools/build_pointpainting_velocity_dataset.py \
  --input-root \
  artifacts/datasets/pointpainting_sunny13_static1500_dynamic1500_9feat_opencv_v1 \
  --dynamic-source-root \
  artifacts/datasets/morai_dynamic_batch001_accepted1500 \
  --output-root \
  artifacts/datasets/pointpainting_sunny13_static1500_dynamic1500_velocity9d_v1
```

OpenPCDet patch/config를 작업 복제본에 동기화한다.

```bash
cp perception/pointpainting/openpcdet_patch/pcdet/datasets/custom/custom_dataset.py \
  ~/OpenPCDet/pcdet/datasets/custom/custom_dataset.py

cp perception/pointpainting/openpcdet_patch/tools/cfgs/dataset_configs/\
morai_pointpainting_sunny13_static1500_dynamic1500_velocity9d.yaml \
  ~/OpenPCDet/tools/cfgs/dataset_configs/

cp perception/pointpainting/openpcdet_patch/tools/cfgs/custom_models/\
pointpillar_morai_pointpainting_velocity9d.yaml \
  ~/OpenPCDet/tools/cfgs/custom_models/
```

OpenPCDet info와 GT sampling DB를 만든다.

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50

python perception/pointpainting/tools/prepare_pointpainting_velocity_dataset.py \
  --dataset-root \
  artifacts/datasets/pointpainting_sunny13_static1500_dynamic1500_velocity9d_v1 \
  --dataset-config \
  perception/pointpainting/openpcdet_patch/tools/cfgs/dataset_configs/\
morai_pointpainting_sunny13_static1500_dynamic1500_velocity9d.yaml \
  --workers 8
```

## Smoke 검증

현재 완료된 검증.

- 9D label/info/GT database 생성.
- Epoch65 geometry 채널 127/127 parameter 로딩.
- 증강된 9D GT로 1-batch forward/backward.
- 속도가 0이 아닌 GT가 velocity output channel에 non-zero gradient 전달.
- inference와 OpenPCDet serializer 모두 Nx9 box 출력.
- ROS 변환에서 LiDAR velocity를 object-body twist로 변환.
- velocity를 끈 기존 7D ROS 동작 유지.

재실행 명령.

```bash
cd ~/AjouNice2026
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python perception/pointpainting/tools/smoke_pointpainting_velocity_training.py \
  --cfg-file \
  perception/pointpainting/openpcdet_patch/tools/cfgs/custom_models/\
pointpillar_morai_pointpainting_velocity9d.yaml \
  --data-path \
  artifacts/datasets/pointpainting_sunny13_static1500_dynamic1500_velocity9d_v1 \
  --checkpoint \
  artifacts/detection3d/pointpainting/releases/\
260820_sunny13_static1500_dynamic1500_velocity9d_init/\
pointpainting_B0_epoch65_velocity9d_init.pth
```

## 본 학습

본 학습은 smoke와 분리해 사용자 터미널에서 실행한다. 첫 실험은 기존
Epoch65 geometry를 보존하기 위해 30 epoch, learning rate `0.001`로 시작한다.
이 결과에서 box AP가 유지되고 velocity metric이 계속 개선되면 80 epoch
run을 별도로 수행한다.

```bash
cd ~/OpenPCDet/tools
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

python train.py \
  --cfg_file cfgs/custom_models/pointpillar_morai_pointpainting_velocity9d.yaml \
  --pretrained_model \
  ~/AjouNice2026/artifacts/detection3d/pointpainting/releases/\
260820_sunny13_static1500_dynamic1500_velocity9d_init/\
pointpainting_B0_epoch65_velocity9d_init.pth \
  --epochs 30 \
  --workers 4 \
  --fix_random_seed \
  --extra_tag velocity9d_e30_v1 \
  --set \
  DATA_CONFIG.DATA_PATH \
  /home/luv/AjouNice2026/artifacts/datasets/\
pointpainting_sunny13_static1500_dynamic1500_velocity9d_v1 \
  OPTIMIZATION.LR 0.001
```

## 평가와 ROS 적용

- 기존 center AP, 거리별 AP, mATE, mASE, mAOE를 그대로 평가한다.
- 2m center match에 대해 class별 `mAVE`, speed MAE, longitudinal/lateral
  velocity MAE를 추가한다.
- geometry AP와 velocity error를 함께 보고 checkpoint를 선택한다.
- ROS velocity 발행은 기본 off이다. 학습 전 초기 checkpoint를 production에
  잘못 쓰지 않도록 하기 위함이다.
- 학습 checkpoint 검증 후 `publish_detector_velocity:=true`로 켠다.
- 첫 production 후보에서는 vehicle만 detector velocity를 fusion한다.
  pedestrian과 obstacle은 기존 EKF 위치 차분을 유지한다.

3-sweep temporal 입력과 방위각 Ego-deskew A/B는
`temporal_sweep_alignment_azimuth_deskew_ab.md`에서 별도로 관리한다.
