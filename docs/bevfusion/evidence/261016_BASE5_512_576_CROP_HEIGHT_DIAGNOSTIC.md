# Base5 512·576 세로 crop CPU 진단

문서 상태: EVIDENCE. 검증: 2026-10-16 10:32 KST.
현재 모델 선택은 [CURRENT](CURRENT.md)를 따른다. 이 작업은 production·학습 설정·원본·GT를 바꾸지 않았다.

## 결론과 검증 범위

512×1056에서 높이만 576으로 늘리면 일부 근접 차량의 상단 형상이 더 남는다.
이번 근접 표본에서는 새 sensor-positive GT 후보가 늘지 않았고, 두 원거리 32장 코호트에서는
추가로 남은 target 픽셀도 없었다. AP·추론시간·학습시간 비교가 아니며 576을 채택한 결과가 아니다.

현재 취득은 원본 PNG를 저장하고 Q90도 native 크기로 변환하므로 모델 crop 변경만으로 재수집하지 않는다.
센서 자체가 원본 밖에서 보지 못한 부분이나 native 해상도가 부족한 정보는 나중에 복원할 수 없다.

CPU 진단은 다음을 분리했다.

- 기존 완료 Sunny13 5K의 0–5m 관측: Vehicle·Pedestrian·Obstacle·Animal을 각각 집계했다.
- v153c R2 initial 32장과 v153d_fast initial 32장: target NPC만 분석하고 retry는 다시 세지 않았다.
- 현재 online 전처리 소스의 순수 crop·homography 메서드: 모델·torch·ROS를 불러오지 않고 실행했다.
- 576 후보의 실제 모델 forward·체크포인트 호환성·GPU 메모리·backward·주행 성능은 검사하지 않았다.

## 입력과 재현

도구는 [report_base5_crop_height_diagnostic.py](../../dataset/morai_od/scripts/report_base5_crop_height_diagnostic.py)다.
기존 mask 분석의 `geometry`·`mask_statistics`·`summarize`를 재사용하되 과거 profile 목록은 수정하지 않는다.
색상 instance ID·annotation class·2D rectangle을 정확히 대조하며 원본 mask 픽셀 수와 기존 512 cache 값이
다르면 실패한다. 입력·raw mask·annotation·진단 소스의 SHA를 시작과 끝에 확인한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -u \
  dataset/morai_od/scripts/report_base5_crop_height_diagnostic.py \
  --far-campaign artifacts/evaluation/base5_far_vehicle_261016/capture_v153c_r2_far32 \
  --far-campaign artifacts/evaluation/base5_far_vehicle_261016/capture_v153d_fast_far32 \
  --output artifacts/evaluation/bevfusion/base5_crop_height_261016/cpu_recheck

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s dataset/morai_od/tests -p 'test_base5_crop*.py' -v
```

output은 새 경로여야 한다. 기존 근거에 overwrite·강제 resume하지 않으며 raw·source 분석 경로 안에 쓰지 않는다.
재현 실행의 timestamp·소요 시간은 달라질 수 있다. 실측 수치와 입력 SHA로 결과를 비교한다.

| 근거 | 값 |
|---|---|
| 기존 5K 분석 | `artifacts/evaluation/bevfusion/base5_crop_final5k_261010` |
| 동결 원본 ledger SHA | `026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43` |
| 이번 보고서 | `artifacts/evaluation/bevfusion/base5_crop_height_261016/cpu_v2/report.json` |
| 보고서 SHA | `a629d95ca9cb13f30cbd05568c7f92e54253df430229c1b066899488e6408902` |
| 진단 도구 SHA | `92526dd83aeb0aad43507a92d77579883cc2e1e61b55462ef0aca5b93044868b` |
| 실행 소요 | 14.80초, GPU 사용 없음 |
| 입력·소스 해시 | 보고서의 `input_sha256`, 총 2,306개 |

`cpu_v1`은 class별 cache 일치 분모 필드를 추가하기 전의 근거로 보존했다. 현재 재현 기준은 `cpu_v2`다.
해당 report에는 기존 source 분석·ledger·far campaign·raw별 절대 경로와 해시가 들어 있다.
실물 raw가 없는 PC에서는 mask 측정을 재현할 수 없다.

## 현재 전처리와 576의 의미

고정 폭 1056과 resize 0.825는 유지했다. 카메라 원본은 전후방 720×1280, 측방 480×640이다.
카메라별 native 폭을 고려하므로 실제 배율은 전후방 0.825, 측방 1.65다.

| 항목 | 512×1056 | 576×1056 |
|---|---|---|
| 전후방 resize 결과 | 594×1056 | 동일 |
| 측방 resize 결과 | 792×1056 | 동일 |
| 전후방 crop 시작 줄 | 82 | 18 |
| 측방 crop 시작 줄 | 280 | 216 |
| 전후방 원본 세로 영역 | 약 99.4–720 | 약 21.8–720 |
| 측방 원본 세로 영역 | 약 169.7–480 | 약 130.9–480 |
| stride-8 feature 크기 | 64×132 | 72×132 |
| 입력·feature 픽셀 수 | 기준 | +12.5% |

실제 online 메서드로 만든 576 RGB의 아래 512줄은 512 입력과 네 카메라 모두 byte-identical이었다.
homography의 scale은 같고 세로 translation만 64픽셀 달라진다. 따라서 기존에 보이던 원거리 물체가
576에서 더 큰 픽셀로 변하는 것은 아니다. 위쪽 시야 복원과 폭/resize 증가에 따른 세부 밀도 증가는 별개다.

같은 resize에서 전후방 높이는 594이므로 공통 stride32 후보의 최대 높이는 576이다.
640·704는 전후방 crop이 원본 resize 높이를 넘어 현재 구현이 거부한다. padding이나 추가 resize·좌우 crop을
쓰려면 별도 설계·투영 검증이 필요하다. 두 크기를 성능 시험했다는 뜻은 아니다.

## 기존 5K 근접 관측의 실제 mask 결과

0≤Ego 중심 XY 거리<5m인 네 클래스의 카메라 관측 1,046개와 raw 카메라 947개를 다시 읽었다.
1,046개 모두 기존 512 mask cache와 정확히 일치했다. Vehicle 279/279도 재현했다.

아래 보존율은 **원본에서 보이던 foreground 픽셀을 90% 이상 남긴 카메라 관측 비율**이다.
native foreground 16픽셀 이상만 집계하며 객체 전체가 잘림 없이 보인다거나 AP가 해당 비율이라는 뜻이 아니다.

| Vehicle 카메라 | 관측 수 | 512 보존율 | 576 보존율 | 평균 원본 픽셀 보존 512→576 | 일부 복원된 관측 |
|---|---:|---:|---:|---:|---:|
| 전방 | 22 | 77.27% | 100.00% | 96.46→99.25% | 7 |
| 좌전방 | 95 | 89.47% | 89.47% | 95.95→97.29% | 11 |
| 우전방 | 75 | 85.33% | 88.00% | 94.65→96.54% | 11 |
| 후방 | 87 | 91.95% | 100.00% | 98.71→99.75% | 11 |

전방은 22개로 작다. 차량 일부가 좋아졌다는 결과를 사람·낮은 장애물·새 배치 전체에 일반화하지 않는다.
576에서도 측방 crop 손실은 남는다. 원본 경계에 닿는 rectangle도 기록하지만, 경계 접촉만으로
센서 밖의 숨은 면적이나 정확한 native 잘림량을 추정하지 않는다.

| 객체군 | 재측정 관측 | 보존율 분모 | 512→576 결과 |
|---|---:|---:|---|
| Vehicle | 279 | 279 | 위 표와 같이 일부 상단 형상 복원 |
| Obstacle | 395 | 395 | 원본 픽셀 100% 보존 유지, 추가 복원 0 |
| Pedestrian | 337 | 336 | 90% 보존율은 양쪽 100%, 12개 관측에서 극소 픽셀 추가 복원 |
| Animal | 35 | 35 | 원본 픽셀 100% 보존 유지, 추가 복원 0 |

Pedestrian의 native foreground 16픽셀 미만 1개는 실제 측정·cache 대조에는 포함하고 보존율 통계에서는 분리했다.
Animal은 원본 의미를 보존한 집계이며 현재 학습에서 Pedestrian으로 합친 정책을 바꾸지 않았다.

### 고유 component와 sensor-positive gate

모든 카메라를 합치되 component를 중복 세지 않았다. 카메라 대응이 모호한 component 28개는
이 합산 진단에서 제외했으며 원본·라벨에서 삭제하지 않았다.

| 객체군 | 카메라에서 보이는 명확한 고유 component | 그중 LiDAR 0점 | 512→576 sensor-positive | 신규 ROI 내 positive 후보 |
|---|---:|---:|---:|---:|
| Vehicle | 196 | 5 | 196→196 | 0 |
| Obstacle | 393 | 192 | 393→393 | 0 |
| Pedestrian | 269 | 0 | 269→269 | 0 |
| Animal | 30 | 20 | 30→30 | 0 |

이 표본은 이미 512에서 camera gate를 통과했다. gate는 mask≥16픽셀·2D 최소 변≥4픽셀이며,
sensor-positive는 LiDAR≥1점 또는 camera gate다. 이 수치는 crop별 정식 infos 변환 결과가 아니라
동일 native component의 센서 지원 재계산이다. 이미 positive인 객체의 형상 정보가 더 남는 이점은 있을 수 있다.

## 서로 다른 원거리 32장 코호트

두 코호트는 **서로 다른 배치·후보 선택 정책**이다. 27→32의 차이를 576 효과로 해석하지 않는다.
각 캠페인의 initial capture만 사용했으며 실패 뒤 retry를 새 source로 중복 집계하지 않았다.

| 원거리 근거 | source 상태 | target camera 관측 | camera gate 512→576 | LiDAR OR camera 512→576 | 추가 복원 관측 |
|---|---|---:|---:|---:|---:|
| `capture_v153c_r2_far32` | 승인 27 / review 5 | 29 | 24→24 | 27→27 | 0 |
| `capture_v153d_fast_far32` | 승인 32 | 37 | 29→29 | 32→32 | 0 |

LiDAR 0점 target은 각각 8개·6개다. 두 코호트의 target 관측은 512에서 추가 crop 잘림이 없었고
576에서도 동일했다. native camera 자체에서 보이지 않거나 가려진 객체가 위쪽 crop 확장으로 생기지는 않았다.
이 결과는 조사한 64개 source에 한정하며 모든 경사·pitch·날씨의 원거리 차량을 대표하지 않는다.

## CPU 검사 통과와 남은 채택 조건

신규 단위 검사 13개와 기존 crop 후보·infos 검사 11개, 합계 24개를 통과했다.
실제 runtime 소스의 crop 유효성·homography·역변환·RGB 포함 관계를 검사했으며,
fixture는 cache/raw 변조·누락·복합체 ambiguity·불완전 far32·중복 source도 거부한다.

현재 결론은 **수집 설정을 바꿀 필요는 없고 512를 운영 기준으로 유지한다**는 것이다.
576이 근접 차량에 줄 수 있는 이점을 완전히 기각한 것은 아니다. 채택하려면 다음을 별도로 수행한다.

1. 새 후보 config의 `image_size=[576,1056]`·`feature_size=[72,132]`와 실제 모델 forward·메모리를 검사한다.
2. crop-specific visibility/infos를 새 파생본으로 만든다. 현재 `build_base5_crop_infos.py`는 576 profile을 허용하지 않는다.
3. 학습·offline·ROS 경로의 실제 RGB·투영 정합 및 학습 비용·학습된 검출의 정확도/지연을 비교한다.

기존 576×1152 비용 시험은 폭도 바뀐 구 5060 Ti 조건이다. 이번 576×1056의 시간이나
현재 4090·정밀 격자·학습된 E24의 비용으로 재사용하지 않는다. TensorRT 성공이나 70ms 동작도 가정하지 않는다.
