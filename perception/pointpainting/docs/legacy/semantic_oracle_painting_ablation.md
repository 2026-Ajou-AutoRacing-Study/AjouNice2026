# Online semantic audit와 oracle-painting A/B.

## 목적.

- online-like rosbag에서 Pedestrian·Obstacle 성능 저하가 LRASPP semantic
  painting부터 시작되는지 확인함.
- 같은 `S1g E37` PointPillars checkpoint와 같은 online ROS pipeline을 유지하고
  semantic feature만 바꿔 인과 관계를 분리함.
- 이 기능은 진단 전용이며 production 기본값에서는 비활성화함.

## 실험 조건.

- PointPillars.
  - config: `pointpillar_morai_pointpainting_base2_singleframe9_geom_aug_gtdb.yaml`.
  - checkpoint: `best_center_mAP_epoch_37.pth`.
- LRASPP.
  - checkpoint: `260817_sunny13_1500_9feat_80e/lraspp_4class_best_miou.pth`.
  - OpenCV short-side `384`, ImageNet normalization.
- Oracle는 `/Object_topic` GT BBox와 `/Ego_topic`을 LiDAR frame으로 변환함.
  - GT box 안의 실제 LiDAR return만 변경함.
  - 네 카메라 중 하나에도 투영되지 않은 point는 변경하지 않음.
  - 도로면 제외 하단 여백은 Pedestrian `0.15 m`, Obstacle `0.03 m`임.
- 두 종류의 oracle를 비교함.
  - `one_hot`: target class를 1, 나머지 semantic channel을 0으로 설정함.
  - `swap_background`: Background가 target보다 클 때 두 값만 교환함.
    기존 LRASPP probability 분포를 보존하므로 주 판정 지표로 사용함.
- AP는 center-distance `0.5/1/2/4 m` AP의 평균임.

## Pedestrian semantic audit.

| Bag | GT point 표본 | Ped probability mean / p50 | Ped argmax mean / p50 |
|---|---:|---:|---:|
| Crossing | 96 | 0.539 / 0.503 | 56.4% / 60.0% |
| Distance 5--18 m | 488 | 0.366 / 0.342 | 39.5% / 41.7% |

- 동일 asset `Man1`이므로 asset mismatch는 아님.
- Distance bag에서는 실제 Pedestrian point의 평균 60.1%가 Background argmax임.
- Crossing은 이미 과반 point가 Pedestrian argmax라 Distance보다 semantic 상태가
  좋음.

## Pedestrian oracle-painting 결과.

| Bag | Painting | mAP | P / R / F1 @ score 0.10 | mATE @2m |
|---|---|---:|---:|---:|
| Crossing | Normal | **0.442** | 0.959 / 0.515 / **0.670** | **0.293 m** |
| Crossing | One-hot oracle | 0.331 | 0.950 / 0.417 / 0.579 | 0.303 m |
| Crossing | Swap oracle | 0.365 | 0.953 / 0.449 / 0.610 | 0.298 m |
| Distance | Normal | 0.642 | 1.000 / 0.696 / 0.821 | 0.338 m |
| Distance | One-hot oracle | 0.633 | 1.000 / 0.674 / 0.806 | **0.261 m** |
| Distance | Swap oracle | **0.889** | 1.000 / 0.904 / **0.950** | **0.225 m** |

- Distance에서는 target/background 오류를 고치면 recall이 `69.6% -> 90.5%`로
  상승함. 이 slice에는 명확한 semantic 병목이 있음.
- Crossing에서는 GT BBox 내부의 background-like point까지 바꾸면 recall이
  하락함.
  - LRASPP가 이미 사람 point를 비교적 잘 칠하고 있음.
  - moving actor의 GT box·LiDAR 시각 차이, box 내부 road/background return,
    PointPillars 표현력이 함께 섞였을 수 있음.
  - 따라서 Pedestrian 전체를 segmentation 하나의 문제로 일반화하지 않음.
- One-hot은 PointPillars가 학습한 LRASPP soft-probability 분포 밖의 입력이어서
  최종 판정에는 swap oracle를 우선함.

## Obstacle semantic audit.

| Bag | GT point 표본 | Obstacle probability mean / p50 | Obstacle argmax mean / p50 | Background probability p50 |
|---|---:|---:|---:|---:|
| Obstacle3 | 196 | 0.080 / **0.0031** | 7.3% / **0%** | 0.972 |
| Ped-negative obstacle | 273 | 0.006 / **0.0011** | **0% / 0%** | 0.998 |

- Ped-negative obstacle bag에서는 모든 유효 GT obstacle point가 Background
  argmax임.
- Obstacle3의 14개 size family 모두 Obstacle argmax 중앙값이 `0%`임.
  - 긴 obstacle, 작은 compact obstacle, tall obstacle 중 특정 anchor 하나만의
    문제가 아님.
- 즉 현재 LRASPP가 실제 obstacle을 거의 일관되게 배경으로 painting함.

## Obstacle oracle-painting 결과.

| Bag | Painting | mAP | P / R / F1 @ score 0.10 | Best F1 | mATE @2m |
|---|---|---:|---:|---:|---:|
| Obstacle3 | Normal | 0.157 | 0.112 / 0.406 / 0.175 | 0.338 | 0.704 m |
| Obstacle3 | Swap oracle | **0.309** | 0.142 / 0.556 / **0.226** | **0.484** | **0.609 m** |
| Obstacle scene | Normal | 0.261 | 0.211 / 0.466 / 0.291 | 0.330 | 0.800 m |
| Obstacle scene | Swap oracle | **0.451** | 0.249 / 0.500 / **0.332** | **0.600** | **0.516 m** |

- Obstacle3 mAP는 약 `1.96x`, 두 번째 scene은 약 `1.73x` 상승함.
- 두 bag에서 같은 방향으로 개선되므로 Obstacle semantic painting은 실제
  detector 성능의 주요 병목임.
- Oracle 이후에도 production score `0.10`의 FP가 많고 mAP가 `0.31~0.45`에
  머묾. Segmentation만 고치면 끝나는 문제는 아니며 PointPillars
  classification/NMS/shape supervision도 후속 병목임.

## 재학습 판정.

- LRASPP 재학습 가치는 **높음**.
  - 최우선 target은 Obstacle임.
  - Pedestrian은 distance·scale·화면 위치 취약 slice를 함께 보강함.
  - Crossing은 segmentation 단독 개선 대상으로 간주하지 않음.
- LRASPP checkpoint만 교체해서 production에 넣지는 않음.
  - 현재 PointPillars는 기존 LRASPP soft probability로 생성된 painted point로
    학습됨.
  - 새 LRASPP로 Base2/Base3 painted dataset을 다시 생성하고 PointPillars도
    같은 semantic 분포로 재학습 또는 fine-tuning해야 함.
- 권장 다음 실험.
  1. Clip-safe Train/Val/Test를 유지한 LRASPP 4-class 재학습.
  2. Obstacle class-balanced loss와 obstacle shape·거리별 sampling 적용.
  3. Pedestrian scale·화면 가장자리·다양한 횡단보도 crop 보강.
  4. 새 LRASPP 자체의 GT-point semantic audit 통과 여부 확인.
  5. 새 LRASPP로 painted point 전체를 재생성한 뒤 S1g 조건 PointPillars 재학습.
  6. 동일 네 rosbag normal 평가와 full-route false-positive 평가.

## 재현 구현.

- Semantic audit:
  `perception/pointpainting/tools/audit_rosbag_pedestrian_semantics.py`.
  - 기존 이름은 호환성을 위해 유지하지만 `--target-class pedestrian|obstacle`을
    모두 지원함.
- Oracle online diagnostic:
  - `POINTPAINTING_ORACLE_PAINTING_CLASS=pedestrian|obstacle`.
  - `POINTPAINTING_ORACLE_PAINTING_MODE=one_hot|swap_background`.
  - 두 환경변수 모두 production 실행에서는 사용하지 않음.
