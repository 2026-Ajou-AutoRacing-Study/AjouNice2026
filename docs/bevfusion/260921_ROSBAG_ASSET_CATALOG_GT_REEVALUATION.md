# Rosbag asset-catalog GT 재평가

- 상태: **EVIDENCE / IMPLEMENTED**
- 날짜: 2026-09-21
- 모델 출력: 기존 `B3-BF0-F40 E25` 저장 bag 재사용
- 신규 추론: 없음

## 결론

Base3-Online/Base4 학습 변환기와 rosbag 평가기가 이제 같은
`asset-catalog-bbox-center-v1` GT를 선택할 수 있다. 이 모드는
`/Object_topic.position`을 그대로 BBox 중심으로 간주하지 않고, 75개 asset catalog의
로컬 pivot-to-center offset과 geometric BBox size를 적용한다.

기존 `bbox-center-calibrated-v1`은 과거 수치 재현을 위해 기본값으로 보존했다. 신규 결과를
낼 때만 `--gt-reference asset-catalog-bbox-center-v1`을 명시한다. 이 변경은 detector,
tracker, predictor 출력을 바꾸지 않고 평가 GT만 바꾼다.

## 과거 bag 호환 방식

새 generated bag은 `/Object_topic.name`의 정확한 asset명을 사용한다. 기존 고주로
RandomMode bag은 이 필드에 `A2256W...` MGeo link가 기록되어 있어 다음의 제한된
fallback만 허용한다.

1. Vehicle의 안정적인 `/Object_topic.size`와 같은 class의 catalog BBox 크기를 비교한다.
2. 3-D 크기 L2 오차가 `0.12 m` 이하이고 2순위 후보와 차이가 `0.02 m` 이상일 때만
   asset을 확정한다.
3. 조건을 만족하지 않으면 primary metric에서 제외하고 이유와 segment key를 기록한다.
4. Pedestrian/Obstacle은 비슷한 크기의 asset을 잘못 고를 위험 때문에 size 추정을 하지
   않는다. 통제된 단일 asset bag은 별도의 `--gt-box-override` legacy 경로를 쓴다.

이번 두 고주로 bag의 해석 결과는 다음과 같다.

| Bag | Vehicle trajectory | catalog 해석 | 미해결 | size match p95 |
|---|---:|---:|---:|---:|
| 고주로 합류 | 14 | 14 | 0 | 0.060 m |
| 고주로1 | 19 | 19 | 0 | 0.008 m |

Base4 P24에서 현재 보존된 23개 raw bag도 `23/23` trajectory가 exact asset name으로
해석됐고 size fallback이나 미해결 항목은 없었다.

## E25 detection A/B

평가는 동일한 기존 detection bag과 Vehicle threshold `0.40`을 사용했다.

| Bag | GT | Vehicle mean AP | AP@1m | IoU AP@0.5 | F1@1m |
|---|---|---:|---:|---:|---:|
| 합류 legacy | generic +1.34m, 대형차 1 track 제외 | 0.5700 | 0.5414 | 0.5499 | 0.7650 |
| 합류 catalog | asset별 center/size, 14 track 전부 | **0.6261** | **0.6444** | **0.6395** | **0.8218** |
| 고주로1 legacy | generic +1.34m | **0.7716** | **0.9132** | **0.7951** | **0.9623** |
| 고주로1 catalog | asset별 center/size | 0.7381 | 0.8623 | 0.7489 | 0.9372 |

합류 상승은 종전에 제외된 대형차를 포함하고 실제 asset geometry가 detector와 더 잘
정렬된 결과다. 고주로1 하락은 generic `+1.34 m`가 우연히 더 유리했던 차종을 asset별
실제 기준으로 바꾼 결과다. 따라서 catalog 보정은 점수를 일률적으로 올리는 후처리가
아니며, 학습 GT와 같은 물리 reference에서 성능을 다시 정의한다.

## E25 tracking/Map-CV A/B

동일한 저장 `pipeline_outputs.bag`을 재사용했다.

| Bag / GT | center p50 | speed MAE | 2-D AVE | Top-1 FDE@3s | origin-aligned FDE@3s |
|---|---:|---:|---:|---:|---:|
| 합류 legacy | 0.750 m | 0.882 m/s | 1.223 m/s | 5.619 m | 5.257 m |
| 합류 catalog | **0.692 m** | **0.874 m/s** | **1.212 m/s** | 5.651 m | 5.300 m |
| 고주로1 legacy | **0.470 m** | 0.421 m/s | 0.715 m/s | 3.623 m | 3.719 m |
| 고주로1 catalog | 0.509 m | 0.421 m/s | 0.715 m/s | **3.617 m** | **3.718 m** |

속도와 미래 motion 지표 변화가 작은 이유는 같은 asset의 고정 center offset 대부분이
시간 차분과 origin alignment에서 상쇄되기 때문이다. 회전 시에는 보정된 center
trajectory를 먼저 만든 뒤 위치 차분하므로 `omega x r` 효과가 자연스럽게 포함된다.
합류의 trajectory 수가 `13 -> 14`로 바뀐 비교에는 종전 제외 대형차가 추가된 set 변화도
포함된다.

## 산출물과 재실행

결과 JSON:

```text
artifacts/evaluation/bevfusion/bf_e40fresh_e25_asset_catalog_gt_260921/
├── vehicle_highway_merge/
│   ├── legacy_summary.json
│   ├── summary.json
│   ├── pipeline_legacy_summary.json
│   └── pipeline_asset_catalog_summary.json
└── vehicle_highway1/
    ├── legacy_summary.json
    ├── summary.json
    ├── pipeline_legacy_summary.json
    └── pipeline_asset_catalog_summary.json
```

새 Base4 bag 수집 후에는 신규 추론 전에 먼저 exact asset-name coverage를 확인한다. 저장된
detection/pipeline output이 있으면 같은 명령으로 GT만 재평가할 수 있다.

BEVFusion 평가 wrapper에서 신규 기준을 사용할 때는 다음 환경변수를 지정한다.

```bash
export BEVFUSION_EVAL_GT_REFERENCE=asset-catalog-bbox-center-v1
export BEVFUSION_EVAL_ASSET_BBOX_CATALOG="$PWD/dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json"
```

환경변수를 지정하지 않으면 과거 재현용 `bbox-center-calibrated-v1`을 유지한다.

## 남은 경계

- catalog에 없는 asset, scale이 1이 아닌 asset, 이름도 없고 크기 매칭도 모호한 asset은
  추측하지 않는다.
- MORAI 공식 충돌 형상과 어느 GT가 같은지는 담당자 답변 전까지 확정하지 않는다.
- 현재 구현은 학습 annotation과 평가 reference를 통일한 것이며 공식 대회 채점 GT를
  역으로 보장하는 작업은 아니다.
