# UDP 차량 GT: 기본 제원 → geometric BBox 대응 복구

상태: EVIDENCE / evaluator 계약. 2026-10-04.
이 문서는 [직전 전체 평가](261004_TEAMMATE_MANUAL_FULL_PIPELINE_EVALUATION.md)의
차량 size 추정 오류를 수정한 후속 결과이다. 모델·tracker·predictor 실행 결과는 그대로이다.

## 수정한 원인과 계약

UDP `/Object_topic`에는 차량 asset 이름이 없을 수 있다. 기존 이름 미식별 차량
resolver는 기본 제원인 `ObjectStatus.size`를 캡처의 geometric `bbox_size_m`에
직접 비교했다. 서로 다른 크기 정의를 비교하여 일부 GT를 누락하거나 잘못 식별했다.

새 대응표는
[`morai_asset_bbox_offsets_status_size_v2.json`](../../dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json)이다.
기존 75개 asset의 BBox 크기·center offset은 모두 유지하고, 43개 차량에 직접 관측한
`object_status_size_m`을 추가했다. 기준은 이름이 확인되는 controlled Static V2
metadata 1,000개이며, 평가 모델의 출력이나 성능에 맞춰 제원을 추정하지 않았다.
입력 checksum과 관측 수가 JSON에 포함된다.

- 기본 제원끼리 비교한다: 3축 Euclidean 오차 ≤0.03m, 다음 제원 후보와 margin ≥0.02m.
- 일치하는 asset이 하나면 그 geometric size와 pivot→center offset을 사용한다.
- 차종 후보가 여러 개라도 geometric size와 offset의 각 성분 차이가 모두 ≤0.01m이면
  공통 geometry를 사용한다. 이름을 임의로 하나 선택하지 않는다.
- 기본 제원이 같아도 geometric BBox가 다르면 미해결로 남긴다.
- 기존 v1 catalog는 과거 재현용으로 보존한다. 평가 CLI와 두 rosbag 평가 runner의
  catalog 기본 경로는 v2로 변경했다. GT-reference 모드 자체는 자동 변경하지 않았다.
- Pedestrian/Obstacle은 이전 scenario ID 기반 이름 복원을 유지한다.
  sensor 데이터·학습 dataset·production detector/tracker 코드는 이 작업으로 변경하지 않았다.

## 복구 범위와 남은 예외

고주로 7개 bag의 차량 trajectory segment 123개 중 117개(95.12%)를 지원한다.
이전 미해결 14개 중 8개를 복구했고, 기존에 DEFAULT_WAGON으로 잘못 대응한
2019_Toyota_Prius 2개 구간도 바로잡았다. segment는 실제 차량 개수와 같지 않을 수 있다.

남은 6개 구간의 기본 제원은 4.645 × 1.775 × 1.600m로 동일하지만 다음처럼
geometric 크기가 서로 다르다. 따라서 기본 크기만으로는 정확한 보정값을 결정할 수 없다.

| 후보 | geometric BBox L/W/H (m) |
|---|---|
| 2015_Toyota_Prius(HEV) | 4.506 / 2.177 / 1.671 |
| DEFAULT_WAGON | 4.572 / 1.812 / 1.501 |

남은 미해결 GT pivot이 Ego 35m 이내인 프레임은 **프레임 전체를 제외**한다.
미해결 GT만 지워 정상 detector 출력을 FP로 세지 않기 위한 보수적 부분집합이며,
완전한 전체 bag 평가가 아니다. Track/prediction도 동일 원칙을 각 출력 timestamp에 적용한다.
미래 GT 보간 가능성에 따라 FDE coverage는 별도로 달라진다.

E25 detection 후보 4,872 frame 중 4,425개(90.82%)를 사용한다.
직전 v1 부분집합의 4,120개(84.56%)보다 305개를 추가 사용한다.

| bag | 사용 / 후보 detection frame |
|---|---:|
| H01_161315 | 764 / 764 |
| H02_161616 | 724 / 724 |
| H02_161831 | 513 / 589 |
| H02_162108 | 779 / 779 |
| H03_162301 | 512 / 649 |
| H04_162616 | 723 / 723 |
| H05_162834 | 410 / 644 |

## 같은 저장 출력으로 다시 계산한 결과

Center AP는 0.5/1/2/4m 평균이다. 아래는 bag별 지표의 단순 macro 평균이고,
pooled AP나 pooled 오차라고 해석하지 않는다. Threshold 0.40/0.15/0.10,
30m 범위, MR heading -1.347° 1회 보정, 지도 v85, tracker detector-velocity 융합 OFF를 유지했다.
E25와 P7 tracker는 위치 기반이다. `locked`는 미리 정한 evaluation_locked 5개 bag이다.

| 모델 / 고주로 범위 | Center AP | Track F1 | Speed MAE m/s | 2D AVE m/s | FDE@1/2/3s m | ID switch |
|---|---:|---:|---:|---:|---|---:|
| E25 / 7-bag | 85.78% | 96.67% | 0.502 | 0.610 | 0.670 / 1.059 / 1.522 | 0 |
| P7 / 7-bag | 85.84% | 96.60% | 0.499 | 0.614 | 0.650 / 1.040 / 1.511 | 0 |
| E25 / locked 5-bag | 89.62% | 96.30% | 0.520 | 0.622 | 0.625 / 1.046 / 1.530 | 0 |
| P7 / locked 5-bag | 89.93% | 96.18% | 0.514 | 0.619 | 0.611 / 1.024 / 1.513 | 0 |

P7 detector-only의 locked 5-bag longitudinal velocity MAE는 0.780m/s,
speed-norm MAE 0.839m/s, 2D AVE 1.204m/s이다. 위 tracker 속도와 혼동하지 않는다.
이 GT 복구 결과로도 P7이 production E25보다 확실히 좋다고 결론 내리지 않는다.
회전교차로·보행자·장애물 9개 case도 새 catalog로 재계산했으며 기존 부분집합 결과와 동일하다.

## 산출물과 재현

결과 root: `artifacts/evaluation/bevfusion/teammate_manual_261004/`.
기존 `summary.json`과 `_resolved_frames` 결과는 덮어쓰지 않았다.
새 case 결과는 `summary_status_size_v2.json`,
`pipeline/summary_status_size_v2.json`, `pipeline/summary_cv_stable_status_size_v2.json`이다.
집계는 `aggregate_status_size_v2.json`과 `tables_status_size_v2.md`이다.
이 산출물과 원본 bag은 ignored 상태를 유지한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export PYTHONPATH="$PWD/base_ws/src/tracking_prediction_evaluation/src:$PYTHONPATH"
python3 perception/detection3d/bevfusion/scripts/reevaluate_teammate_resolved_frames.py \
  --catalog dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json \
  --suffix _status_size_v2
python3 perception/detection3d/bevfusion/scripts/summarize_teammate_manual_evaluation.py \
  --suffix _status_size_v2
```

이 명령은 이미 저장한 detector/tracker/predictor 출력만 재채점하며 GPU 추론을 다시 하지 않는다.
기존 해당 suffix의 결과가 있으면 건너뛴다. 다른 계약으로 재채점할 때 새 suffix를 사용한다.
검증: evaluator tests 46개 통과. nominal→geometry, 동일 제원·동일 geometry,
동일 제원·상이한 geometry의 세 경우를 단위 검증했다.
