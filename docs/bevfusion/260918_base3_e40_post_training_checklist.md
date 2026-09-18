# Base3 BEVFusion Fresh E40 학습 종료 후 체크리스트

## 대상 run

- Run: `bf-sw-fresh-e40-rtx5060ti-260918-005536`
- Output:
  `artifacts/detection3d/bevfusion/team_training/sw/bf-sw-fresh-e40-rtx5060ti-260918-005536`
- 학습 시작 당시 evaluator는 범위 밖 GT를 포함한 구버전이다.
- 학습 loss와 checkpoint는 정상이며 폐기하거나 재학습할 필요가 없다.
- 자동 생성된 `best_val_center_mAP_*.pth`는 corrected 평가 전에는 최종 best로
  확정하지 않는다.

## 1. 학습 완료·건전성 확인

1. `epoch_40.pth`, `last_checkpoint`, `train.log` 존재 확인.
2. E31--E40 loss, LR, NaN/OOM/FATAL, AMP overflow 빈도 확인.
3. 기존 evaluator 기준 E1--E40 center mAP, IoU mAP@0.5, class AP, mATE/mASE/mAOE
   추세를 표로 추출한다.
4. 기존 best E26과 late epoch의 수렴·과적합 여부를 비교한다.

## 2. Corrected-range30 validation

반드시 2026-09-18 range-contract 수정 이후 코드로 새 프로세스를 실행한다.

- 현재 localization worktree commit: `839baa39`.
- 팀 공용 branch: `perception/bevfusion-team-docker`.
- 팀 공용 branch commit: `f18f1917`.
- 평가 범위: `[-30, -30, -3, 30, 30, 2] m`.
- GT와 prediction에 같은 축 정렬 사각 범위를 적용한다.
- 기존 학습 W&B history는 수정하거나 삭제하지 않는다.

E1--E40 전체 checkpoint를 evaluation batch `4`로 순차 평가한다. OOM 또는 metric
회귀가 확인될 때만 `--batch-size`로 낮춘다. 기존 Vehicle AP가 범위 오류 ceiling에
눌려 있어 구버전 지표만으로 후보를 과도하게 줄이면 진짜 best를 놓칠 수 있다.
예상 소요 시간은 checkpoint당 약 3분, 전체 약 2시간이다. 전 결과는 별도 corrected
W&B run 하나에 기존 `val/*` key와 `epoch=1..40` 축으로 기록한다.

Corrected val에서 다음을 저장한다.

- `checkpoint_epoch`.
- center mAP와 class별 mean AP.
- center AP@0.5/1/2/4m.
- IoU AP/recall@0.25/0.30/0.50.
- class별 mATE/mASE/mAOE.
- 0--10/10--20/20--30/30--50m distance slice와 GT count.
- operating-point precision/recall/F1/score.

범위 필터 후 예상 Val GT count는 Vehicle 1,100, Pedestrian 1,005,
Obstacle 2,185다. 이 값과 다르면 evaluation contract를 다시 확인한다.

## 3. W&B corrected run

별도 evaluation run을 만든다.

```text
project: morai-3d-object-detection
group: bevfusion-base3-corrected-range30
job_type: evaluation
run name: bf-sw-fresh-e40-corrected-range30-<timestamp>
x-axis: checkpoint_epoch
```

기존 panel key와 최대한 호환하되 corrected 결과는 `corrected/val/...` namespace 또는
명확한 corrected 전용 run으로 구분한다. 기존 학습 run의 과거 epoch history를
덮어쓰지 않는다.

## 4. Best checkpoint 결정

1차 기준은 corrected `val/center/mAP`다.

동률 또는 `0.001` 이내이면 다음 순서로 판단한다.

1. IoU mAP@0.5.
2. Vehicle AP와 Vehicle mATE.
3. Obstacle AP@0.5m와 20--30m slice.
4. Pedestrian AP@0.5m.
5. late epoch 과적합 여부.

최종 best와 runner-up 두 개를 남긴다. 구버전 자동 best symlink는 보존하고 corrected
best alias를 별도로 만든다.

## 5. Corrected test

- corrected val best.
- corrected val runner-up.
- 비교 기준인 기존 production `B3-BF0 E10`.

위 세 checkpoint를 Base3 frozen test에 같은 evaluator로 평가한다. Val로 best를 고른
뒤 Test는 최종 일반화 확인에만 사용하며 Test 결과로 checkpoint를 다시 고르지 않는다.

PointPainting 비교 기준:

- PP1 Val center mAP `0.9786`, IoU mAP@0.5 `0.9591`.
- PP1 Test center mAP `0.9809`, IoU mAP@0.5 `0.9640`.

BEVFusion E30 corrected 예상치는 center mAP 약 `0.980--0.983`, IoU mAP@0.5
약 `0.957--0.961`이지만 실제 재평가 결과를 우선한다.

## 6. Mission rosbag 평가

Corrected test best와 runner-up을 기존 production과 동일한 조건으로 평가한다.

- Threshold: Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`.
- 동일 raw bag, 동일 평가 구간, 동일 GT alignment, 동일 point/visibility 정책.
- 기본 8 bags:
  - 정적 Obstacle 왕복.
  - Base3 multi-asset Obstacle.
  - Pedestrian 횡단.
  - Pedestrian 거리별.
  - Vehicle 30m.
  - 고주로1.
  - 고주로 합류.
  - 신로터리.

모델별로 detection AP/precision/recall/F1, class별 box quality, tracking 성능,
Map-CV prediction 성능을 기존 leaderboard 양식에 추가한다. Offline corrected 성능만으로
production을 교체하지 않는다.

## 7. 최종 산출물

1. corrected val 전 epoch 표와 W&B link.
2. frozen test 비교표.
3. 8-bag 비교표와 slice 분석.
4. 최종 best/runner-up checkpoint 경로와 SHA-256.
5. production 교체 여부와 근거.
6. 필요하면 최종 best를 inference checkpoint 형식으로 export.

## 이번 작업과 분리할 후속 실험

전방 40--50m 확장, 비대칭 BEV range, 고주로 원거리 Vehicle, 정적 장애물·GPS
음영 구간의 대형 Obstacle add-on은 별도 실험이다. 이번 E40 corrected 평가와 섞어
재학습하지 않는다.
