# 신호등 CNN crop 후보 1차 평가

종류: EVIDENCE / 2026-10-25. 운영 기준은 [CURRENT](CURRENT.md)다.
운영 `best.pt`, HSV, 정책 YAML, launch, planning은 변경하지 않았다.

## 목적과 결과

기존 HSV의 화살표 누락·잘못된 색 동시검출이 학습 기반 분류로 개선되는지 확인했다.
YOLO가 아닌 **이미지 내 올바른 housing 영역을 알고 있다는 조건**의 시험이다.
R/Y/G/좌화살표 4비트를 독립 예측하여 동시점등을 허용한다.

| 고정된 새 pose 시험 표본 | 기존 HSV | ResNet18 CNN |
|---|---:|---:|
| 판독 가능한 신호, 전체 점등 조합 일치 | 93/120 (77.5%) | 120/120 |
| 좌화살표 양성 recall | 6/12 | 12/12 |
| 좌화살표 FP / 비화살표 신호108개 | 0/108 | 0/108 |
| 램프 없는 배경, 4비트 모두0 | 66/72 | 72/72 |

배경을 포함한 192/192를 신호 인지 정확도로 홍보하지 않는다.
특히 원래 시험 신호132개 중 12개는 화살표 판독이 불명확해 제외했다.
현재 수치는 **관찰 가능120/132**에 한정되며, 제외된 어려운 입력의 성공률을 뜻하지 않는다.
QC는 assistant가 원본 crop을 직접 본 것이며 독립 사람의 GT 인증은 아니다.
각 교차로의 렌더러·자산을 공유하는 작은 시험으로, 완전한 실전 일반화 증거도 아니다.

| 신호 crop만: 환경별 조합 일치 | 기존 HSV | CNN |
|---|---:|---:|
| Sunny11 | 17/20 | 20/20 |
| Sunny13 | 18/20 | 20/20 |
| Sunny15 | 11/20 | 20/20 |
| Foggy11 | 17/20 | 20/20 |
| Foggy13 | 14/20 | 20/20 |
| Foggy15 | 16/20 | 20/20 |

이 소량 결과로 Sunny15가 항상 Foggy보다 어렵다고 결론내리지 않는다.
D-2 G 단독12개는 두 방식 모두 맞췄다. 따라서 팀원의 과거 G→좌회전 보고 원인을
이번 시험으로 재현·해결했다고 주장하지 않는다. D-2 R+Y12개는 HSV6개/CNN12개,
Y 단독12개는 HSV9개/CNN12개가 전체 조합을 맞췄다.
R+Y를 맞게 분류하는 것과 그 상태에서 새 출발을 허용하는 판단 정책은 별개다.

## 데이터와 선택 절차

원본·추론 결과는 `artifacts/evaluation/traffic_light_robustness_261025/` 아래에 보관했다.
전후 환경 API readback과 JPEG 전후 exact native ID/color를 확인한 사진만 후보로 썼다.
요청한 상태를 그대로 GT로 쓰지 않았으며, 실패·RPC 재시도 원본은 따로 남겼다.
지도 투영 ROI는 근사치이므로 별도 시각 QC 후 승인한 실제 ROI를 사용했다.

- 원시 신호402개 → 판독 가능한366개 채택, 불명확36개 제외.
- lamp-free 배경180개는 별도 직접 QC. `native_gt_verified:false` 유지.
- 최종 train252 / val102 / test192 =546개. 신호만은180 /66 /120개다.
- 같은 pose의 환경·상태·재시도는 같은 split에 묶고 JPEG SHA의 split 중복도 거부했다.
- 첫 frozen-feature 결과를 본 옛 test는 개발 자료로 취급했다. 본체 학습 전 A20과
  옛 D2_20을 validation으로 지정했고, 옛 B20/C40은 이번 학습·선택·test에서 제외했다.
- 새 test는 A27/B30/C32/D2_25 및 C18/D2_16 회피 시점이다. 멀리서 화살표를
  읽지 못하는 문제가 QC에서 나와, **학습·예측 전에** 가까운 두 test pose를 추가했다.
  이후 모델 결과를 보고 쉬운 test로 바꾼 것이 아니다. 먼 판독 불가 사진도 보존했다.
- val의 macro F1, 동률이면 BCE로 checkpoint 선택. threshold0.5 고정.
  선택된 E30 가중치로 test를 한 번 평가했다. test로 epoch·threshold를 고르지 않았다.

원래 frozen-feature probe의 test 46/66과 이번120/120은 데이터·QC·split이 다르다.
두 수치를 같은 벤치마크의 개선량처럼 직접 빼지 않는다. 위 HSV 비교는 **이번 동일 ROI**다.

## 모델·학습 계약

로컬 ImageNet ResNet18에서 4비트 head를 새로 만들고 backbone까지 fine-tune했다.
입력은 RGB aspect-preserving letterbox224×128 + ImageNet 정규화다.
FP32, compile/AMP OFF, batch16, AdamW, backbone LR1e-4/head1e-3,
weight decay1e-4, warmup2epoch 뒤 cosine, 총30epoch, seed0을 사용했다.
BatchNorm running stats는 고정하되 affine은 학습했다.
양성 가중치는 train에서만 계산했다. 원문 팀 영상이나 YOLO 박스 score를 라벨로 쓰지 않았다.

Train의 신호 crop은 승인 ROI를 자르지 않는 바깥쪽0~10% 확장만 허용했다.
배경은 주변 램프 유입을 막기 위해 공간 증강 없이 승인 ROI를 고정했다.
밝기·대비0.9~1.1만 사용하고 hue/flip/blur는 끈 상태다. val/test는 원래 ROI 그대로다.
HSV도 같은 원본 ROI를 받아 비교하며 CNN의 확대 입력을 HSV에 몰래 적용하지 않았다.

## 재현·주요 근거

- `full_cnn_protocol_v3_prefit.json`: 학습 전 역할·시험 보완 결정.
- `full_cnn_raw_dataset_v3.json`: raw402개, SHA256
  `ef528354ca044a9db470f31ef210abf5c804cc9b694aeafc239ce0a5bac3170a`.
- `full_cnn_curated_dataset_v3.json` / `.curation.json`: 원본 SHA·정확한 QC ROI·제외 사유.
- `visual_qc_margin12_v2/`, `visual_qc_natural_v3/`, `visual_qc_addendum_v4/`와
  `visual_qc_background_v2/`, `visual_qc_background_v3_addendum/`: 직접 QC 원문.
- `full_resnet18_cnn_v3/{epochs.jsonl,metrics.json,predictions.jsonl,subset_comparison.json}`.
- 후보 `full_resnet18_cnn_v3/best.pt`, SHA256
  `3fcdda8645f376d9958a74d10e0a3304b09b3a3516f8b7ea4bc85852a5e09260`.
- `source_at_fit/`에 실행 당시 학습기·공통 loader를 보존했다. 학습 후 독립 사람
  GT로 오해될 수 있는 주석만 정정했으며 학습 계산·라벨·결과를 바꾸지 않았다.

```bash
R=artifacts/evaluation/traffic_light_robustness_261025
PYTHONDONTWRITEBYTECODE=1 ~/miniconda3/envs/traffic_light_gpu/bin/python \
  scripts/evaluation/train_traffic_light_cnn.py \
  --dataset "$R/full_cnn_curated_dataset_v3.json" \
  --backbone-weights ~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth \
  --output "$R/full_resnet18_cnn_reproduction_NEW" --device cuda:0
```

반드시 새 output 경로를 사용한다. 기존 증거·가중치를 덮어쓰지 않는다.
새 진단 도구의 순수 테스트230개, 인지 context 링크 검사, `git diff --check`가 통과했다.
기존 production 정책 테스트6실패/8통과는 옛 기대값과 현행 YAML의 계약 차이로
별도 기록하며, 이번 도구의 테스트 통과에 합산하거나 조용히 수정하지 않았다.

## 판단과 다음 작업

CNN 본체 fine-tune은 **계속 검증할 가치가 있는 후보**다. 즉시 production 승격은 아니다.
다음은 동일 신호인 C016/C018의 품질 기반 선택, 실제 YOLO box 오차,
작거나 판독 불가한 신호에서의 과신, 연속 JPEG의 phase 전환·출력 지연을 확인한다.
이번 단일 head 강제 상태 사진은 자연 프로그램 선택의 end-to-end GT가 아니다.
CNN 원시 관측과 정책 보정·시간 필터·planner 허용 출력을 단계별로 비교해야 한다.

정지 시험은 종료했다. 초기 위치와 Sunny13을 복원하고 신선한 Ego·gear로
정지/P를 확인한 뒤 **직접 시작한 센서 gateway·parking hold만 종료**했다.
신호 default(-1) 복귀를 요청했고 후속5ID 조회는 모두 유효한 native 상태였다.
사용자가 켠 MORAI·roscore는 종료하지 않았다. 복원 증거는
`live_natural_training_v5/events.jsonl`, `final_*readback.json`에 있다.
