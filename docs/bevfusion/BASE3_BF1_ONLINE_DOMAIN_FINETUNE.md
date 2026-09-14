# Base3 BF1 online-domain fine-tuning

## 목적

`B3-BF0`의 MORAI SaveSensorData validation 성능과 실제 비동기 ROS sensor
rosbag 성능 사이의 차이를 줄이기 위해, camera 입력에 약한 JPEG·blur·노출·noise
augmentation을 적용한 저학습률 fine-tuning을 수행했다.

이 문서는 2026-09-14 기준 학습 결과, checkpoint 선택, rosbag 평가와 실험 과정에서
발견한 비교 조건 오류를 함께 기록한다. 이 실험은 production 확정 기록이 아니다.

## 실험 구성

| 항목 | 값 |
|---|---|
| 이름 | `B3-BF1 online-domain` |
| 초기 checkpoint | `B3-BF0 E6` |
| 추가 epoch | 6 |
| optimizer | AdamW |
| learning rate | `2e-5` |
| camera augmentation | JPEG quality 72--96, 약한 blur, exposure gain/bias, Gaussian noise |
| GT database sampling | 사용하지 않음 |
| 출력 경로 | `artifacts/detection3d/bevfusion/training/base3_bf1_online_domain_e6ft6` |

GT sampling을 사용하지 않은 이유는 native multi-modal BEVFusion에서 LiDAR 객체만
붙이면 같은 위치의 camera pixel과 일치하지 않기 때문이다.

초기 checkpoint로 E6를 선택한 당시 근거는 다음이었다.

1. BF0 offline validation의 최고 center mAP checkpoint가 E6였다.
2. 과거 rosbag 결과에서도 E6가 E10보다 좋아 보였다.

그러나 두 번째 근거에는 아래 `평가 조건 오류`가 있었다. 따라서 이 실험은
정확히는 **BF0 E6 개선 실험**이며, production E10에서 시작한 실험으로 해석하면
안 된다.

## Offline validation

| BF1 epoch | Center mAP | Vehicle AP | Pedestrian AP | Obstacle AP |
|---:|---:|---:|---:|---:|
| 1 | 0.9363 | 0.8637 | 0.9868 | 0.9584 |
| 2 | 0.9383 | 0.8637 | 0.9847 | 0.9667 |
| 3 | 0.9397 | 0.8637 | 0.9871 | 0.9684 |
| **4** | **0.9404** | **0.8638** | **0.9870** | **0.9703** |
| 5 | 0.9396 | 0.8637 | 0.9862 | 0.9687 |
| 6 | 0.9394 | 0.8637 | 0.9860 | 0.9686 |

BF0 E6과 BF1 E4의 box quality는 다음과 같다.

| 모델 | Center mAP | IoU mAP@0.5 | Vehicle mATE | Pedestrian mATE | Obstacle mATE |
|---|---:|---:|---:|---:|---:|
| BF0 E6 | 0.9363 | 0.8934 | 0.0858 m | 0.0434 m | 0.0624 m |
| **BF1 E4** | **0.9404** | **0.8937** | **0.0780 m** | **0.0386 m** | **0.0581 m** |

Offline 기준 BF1 best는 `best_val_center_mAP_epoch_4.pth`다. E6은 Vehicle mATE가
조금 더 낮지만 전체 center mAP와 rosbag 균형은 E4보다 낮다.

## 평가 조건 오류와 재발 방지

과거 BF0 E6 rosbag 결과는 detector 출력 threshold가 사실상 class 공통 `0.01`인
실행이었고, production E10 결과는 Vehicle/Pedestrian/Obstacle
`0.40/0.15/0.10`을 사용했다. 이 둘을 직접 비교해 E6가 online에서도 더 좋다고
판단한 것은 잘못이었다.

이를 확인한 뒤 `run_rosbag_detection_evaluation.sh`를 다음과 같이 수정했다.

- 기본 class threshold를 `0.40/0.15/0.10`으로 설정한다.
- 환경변수로 명시적 override가 가능하다.
- 실제 적용 threshold를 각 `summary.json`의
  `runtime_class_score_thresholds`에 기록한다.

앞으로 두 checkpoint를 직접 비교하려면 다음 조건을 모두 맞춰야 한다.

1. 같은 raw bag과 동일한 평가 구간.
2. 같은 class별 score threshold.
3. 같은 maximum range와 GT center calibration.
4. 같은 playback rate 또는 충분히 동일한 runtime frame coverage.
5. `evaluated_frames`와 class별 GT 수 확인.
6. AP와 함께 production operating-point precision/recall/F1 및 mATE 확인.

디렉터리 이름만 보고 평가 조건이 같다고 간주하지 않는다. 위 조건이 다르면 표에
직접적인 개선률을 쓰지 않는다.

## 동일 threshold rosbag 결과

모든 아래 비교는 Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10` 기준이다.

### BF0 E6과 E10 재확인

공통 핵심 4-bag에서 E10은 E6보다 전부 높았다.

| Rosbag | BF0 E6 AP | BF0 E10 AP | E10 변화 |
|---|---:|---:|---:|
| Obstacle 왕복 | 0.6089 | **0.6221** | +0.0132 |
| Obstacle multi-asset | 0.6006 | **0.6067** | +0.0061 |
| 고주로 합류 | 0.5702 | **0.5833** | +0.0131 |
| 신로터리 | 0.7479 | **0.7725** | +0.0246 |
| **4-bag macro** | 0.6319 | **0.6462** | **+0.0143** |

즉 BF0에서는 offline best는 E6이지만, 확인된 online production 기준은 E10이다.

### BF1 내부 checkpoint 선별

핵심 4-bag macro AP는 E1 `0.6480`, E4 `0.6489`, E6 `0.6416`이었다. Offline
center mAP까지 함께 고려하여 BF1 내부 best를 E4로 정한다.

### Production E10과 BF1 E4

| Rosbag | Class | BF0 E10 AP | BF1 E4 AP | 변화 | BF1 E4 mATE |
|---|---|---:|---:|---:|---:|
| Obstacle 왕복 | Obstacle | 0.6221 | 0.6045 | -0.0176 | 0.587 m |
| Obstacle multi-asset | Obstacle | 0.6067 | **0.6479** | **+0.0412** | 0.366 m |
| Pedestrian 횡단 | Pedestrian | 1.0000 | 1.0000 | 0.0000 | 0.318 m |
| Pedestrian 거리별 | Pedestrian | **1.0000** | 0.9444 | -0.0556 | 0.248 m |
| Vehicle 30 m | Vehicle | 0.6924 | 0.6766 | -0.0158 | 0.381 m |
| 고주로 1 | Vehicle | 0.7667 | 0.7648 | -0.0019 | 0.516 m |
| 고주로 합류 | Vehicle | 0.5833 | 0.5839 | +0.0006 | 0.655 m |
| 신로터리 | Vehicle | 0.7725 | 0.7594 | -0.0131 | 0.455 m |
| **8-bag macro** |  | **0.7555** | 0.7477 | **-0.0078** |  |

coverage가 낮았던 multi-asset과 고주로 1은 각각 0.25x와 0.5x로 재생해 다시
확인했다. multi-asset GT는 746개로 맞았고 AP `0.6479`를 유지했으며, 고주로 1은
coverage 1.0, GT 403개에서 AP `0.7648`이었다.

## 결론

- BF1 내부 보존 checkpoint는 E4다.
- BF1 E4는 초기점 BF0 E6보다 핵심 4-bag macro AP가 `+0.0170` 높다.
- 그러나 production BF0 E10 대비 8-bag macro AP는 `-0.0078`이며 개선이 아니다.
- multi-asset obstacle에는 augmentation 이득이 있었지만 Pedestrian 거리별,
  Vehicle 30 m, 로터리에서는 퇴행했다.
- 따라서 BF1 E4를 production으로 교체하지 않고 실험 checkpoint로만 보존한다.
- BEVFusion production 기준은 계속 `B3-BF0 E10`이다.

후속 실험을 한다면 production E10에서 시작하고 augmentation 강도를 낮춘 BF1b를
구성해야 한다. 다만 JPEG/blur/exposure만으로 실제 센서 비동기와 기하학적 domain
gap을 모두 해결할 수 있다고 가정하지 않는다.

## 결과 경로

- BF1 학습: `artifacts/detection3d/bevfusion/training/base3_bf1_online_domain_e6ft6`
- BF1 rosbag: `artifacts/evaluation/bevfusion/base3_bf1_online_domain_screen_260914`
- 동일 조건 BF0 E6 핵심 재평가:
  `artifacts/evaluation/bevfusion/base3_b0_e6_class_thresholds_040_015_010_260914`
- BF0 E10 production 비교:
  `artifacts/evaluation/bevfusion/base3_b0_e10_class_thresholds_040_015_010_260913`

