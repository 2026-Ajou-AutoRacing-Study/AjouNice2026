# E25 기존·새 rosbag: 현재 파이프라인으로 맞춘 비교

상태: EVIDENCE. 2026-10-04. Production 모델·threshold·tracker 변경 없음.

이 표가 현재의 **기존 bag 대 새 bag 대응 비교**이다. 이전
[`261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md`](261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md)의
9/18 historical baseline과 현재 결과를 직접 비교한 표를 대체한다. 과거 산출물은 보존한다.

## 맞춘 조건

- 같은 production E25 checkpoint, Vehicle/Pedestrian/Obstacle threshold 0.40/0.15/0.10.
- 기존 6개 bag은 저장된 과거 검출을 가져오는 데 그치지 않고 **현재 코드로 새 E25 추론**을
  실행했다. Class-aware rotated BEV NMS IoU 0.50, 4 camera, single-frame, scan deskew off.
- 현재 설치된 tracker 바이너리, CTRV, XY noise 0.40m, internal prediction age 0.50s.
  Detector velocity fusion·center-motion fusion·실험용 빠른 차량 승격은 OFF.
- 같은 v85 지도, Map-CV 3초/0.1초, 현재 TF. Ego GT로 재생하며 localization 오차는 포함하지 않는다.
- 같은 `morai_asset_bbox_offsets_status_size_v2.json`으로 geometric center/size와 center velocity 구성.
  이름 없는 차량은 기본 제원에 대응시키고, 보행자·장애물은 해당 시나리오 UID/DataID로 복원한다.
- 같은 30m 범위, center AP 0.5/1/2/4m 평균, tracker 2m 매칭, all-motion top-1 FDE.
- 추론은 양쪽 0.5배속, tracker/predictor는 저장된 detection으로 1배속 재생한다.
  Bag별 지표의 비가중 평균이며 pooled AP나 quantile 평균을 주장하지 않는다.
- MR yaw -1.347°를 한 번 적용한다. 오래된 static-Ego 보행자 2-bag은 launcher yaw를 독립
  검증할 수 없어 기존 0° 계약을 유지한다. MR/MRP 또는 ROS/UDP를 근거 없이 혼용하지 않는다.

새 bag 결과는 이미 같은 현재 계약으로 실행한 `teammate_manual_261004`의
`aggregate_status_size_v2.json`을 사용한다. 공통 tracker 설정의 불일치는 없으며,
아래 지도·카탈로그·tracker 바이너리 해시도 일치한다.

| 고정 항목 | SHA256 |
|---|---|
| tracker 실행 파일 | `ef67431d3c12987e5d8fd3e9384332b338db9e86157c2e66bf46d4c157101f5c` |
| 지도 v85 | `5111ae16ba788b059e40e88201b51b0cb4d1d662dcd8456f43cda9705066be58` |
| status-size v2 GT catalog | `3573f37bef35443c47156f59cf8a9be7dab4e39a6aaafb31e0a72b4a9a06ebe7` |

## 미션별 비교

속도 단위 m/s, FDE 단위 m. 새 고주로 전체 7-bag은 development 및 training-candidate를
포함한다. 독립 test만의 결과로 부르지 않는다.

| 미션 | bag | Center AP ↑ | Tracking F1 ↑ | Speed MAE ↓ | 2D AVE ↓ | FDE@1/2/3s ↓ |
|---|---|---:|---:|---:|---:|---|
| 고주로 | 기존 2개 | 75.14% | 94.80% | 0.641 | 0.758 | 1.101 / 2.551 / 4.612 |
| 고주로 | 새 7개 | 85.78% | 96.67% | 0.502 | 0.610 | 0.670 / 1.059 / 1.522 |
| 로터리 | 기존 1개 | 81.75% | 96.04% | 0.365 | 0.680 | 0.811 / 2.112 / 4.254 |
| 로터리 | 새 3개 | 83.01% | 95.89% | 0.397 | 0.607 | 0.822 / 2.023 / 3.923 |
| 보행자 | 기존 2개 | 99.27% | 99.71% | 0.068 | 0.083 | 0.353 / 0.453 / 0.632 |
| 보행자 | 새 3개 | 62.98% | 91.63% | 0.356 | 0.510 | 0.977 / 2.180 / 3.836 |
| scale=1 장애물 | 기존 1개 | 68.85% | 93.96% | ≈0 | ≈0 | 0.341 / 0.344 / 0.345 |
| scale=1 장애물 | 새 3개 | 62.80% | 91.20% | ≈0 | ≈0 | 0.325 / 0.328 / 0.332 |

새 고주로 locked 5-bag만 보면 AP 89.62%, F1 96.30%, speed MAE 0.520m/s,
AVE 0.622m/s, FDE 0.625/1.046/1.530m다.

## 범위·coverage와 해석

- **기준은 통일했지만 주행 분포가 동일한 paired 실험은 아니다.** 같은 E25의 서로 다른
  주행 자료 평가이며 학습/알고리즘 개선율로 해석하지 않는다.
- 미식별 GT pivot 35m 이내 frame 전체를 제외하는 동일한 보수적 정책을 양쪽에 적용했다.
  기존 고주로는 모든 차량 geometry를 복구하여 detection 후보 873/873개를 사용했고,
  새 고주로는 4,425/4,872개(90.82%)를 사용했다. 새 고주로 수치는 전체 프레임 성능이 아니다.
- 기존 로터리 291/291, 보행자 655/655, 장애물 418/418개 detection 후보를 사용했다.
  최초 Ego 보간 실패는 후보 구성 전에 별도 집계한다. Pipeline 최초 frame에서도 소수 Ego
  보간 실패를 제외하며, 이를 GT ambiguity 제외나 검출 누락으로 부르지 않는다.
- 기존 6-bag의 detection message는 LiDAR 2,241개 중 2,241개였다.
  실시간 GPU 부하/latency를 평가한 결과가 아니며, 객체 recall 100%를 뜻하지 않는다.
- 기존 `obstacle_backforth`는 공식 sample의 scaled CargoBox를 포함한다. Scale-specific
  geometric GT를 직접 검증하지 않은 채 scale=1 catalog를 적용하지 않았다. 이번 장애물
  비교는 scale=1 multiasset끼리이며 해당 bag의 원본·historical 결과는 삭제하지 않았다.
- 고주로는 현재 기준에서도 새 bag 쪽이 좋다. 기존 합류의 FDE@3s 5.645m와 순항 3.578m가
  남는다. GT/옛 tracker 차이만으로 새 bag의 좋은 결과를 전부 설명할 수는 없다.
- 로터리는 큰 붕괴 없이 비슷한 수준이다. 새 speed MAE는 조금 커지고 AVE/FDE는 작아져,
  모든 지표가 개선됐다고 표현하지 않는다.
- 보행자는 기존 약 99%가 새 주행에 일반화되지 않는다. 기존 bag은 단일 Man1 중심이며
  20–30m GT가 없었다. 새 원거리 누락과 가까운 center 정밀도 문제는 이전 분석도 참고한다.
- 장애물은 새 AP/F1이 낮지만 matched track의 FDE는 비슷하다. 검출하지 못한 객체에는
  FDE를 계산할 수 없으므로 작은 FDE만 보고 안전하다고 하지 않는다.
- 장애물 속도 ≈0은 UNKNOWN stationary 정책이며 velocity head 성능이 아니다.

3초 prediction coverage의 bag macro:

| 미션 | 기존 | 새 |
|---|---:|---:|
| 고주로 | 91.51% | 91.61% |
| 로터리 | 93.68% | 95.00% |
| 보행자 | 99.35% | 85.18% |
| 장애물 | 90.35% | 84.68% |

## 방향·거리별 누락 추가 감사

평가는 전방 전용이 아니다. Ego 기준 30m 원형 범위로 전후좌우를 포함하며 후방은
제외하지 않았다. 입력은 전방/전좌/전우/후방 4 camera + LiDAR이다.

아래 recall은 저장된 E25 검출을 전체 범위에서 confidence 순으로 1m center 매칭한 뒤,
**매칭된 GT 위치**로 거리/방향을 나누어 계산했다. 구간별로 GT와 예측을 따로 잘라
재매칭하는 기존 distance-bin AP와 구간 경계 처리가 다르다. 따라서 이전 거리별
TP/FN 집계와 일부 수가 다르며 혼합하지 않는다. 합계는 전체 TP/FN과 일치한다.
숫자는 unique 객체 수나 frame 수가 아니라 GT box-observation 수다.

| 범위 | 기존 보행자 GT/TP | 새 보행자 GT/TP | 기존 장애물 GT/TP | 새 장애물 GT/TP |
|---|---:|---:|---:|---:|
| 전체 | 1753 / 1751 | 344 / 268 | 746 / 551 | 2267 / 1630 |
| 전방 반원: ego-local x≥0 | 1753 / 1751 | 230 / 192 | 373 / 284 | 1207 / 831 |
| 후방 반원: ego-local x<0 | 0 / 0 | 114 / 76 | 373 / 267 | 1060 / 799 |
| 0–10m | 366 / 364 | 149 / 148 | 252 / 184 | 749 / 464 |
| 10–20m | 1387 / 1387 | 107 / 105 | 251 / 244 | 743 / 716 |
| 20–30m | 0 / 0 | 88 / 15 | 243 / 123 | 775 / 450 |

- 기존 보행자는 모두 전방 ±45°이고 20m 이내였다. 새 보행자는 원거리·옆·뒤까지 포함한다.
  새 76건 FN 중 73건(96.1%)이 20–30m다. 전방 반원의 FN 38건, 후방 반원의 FN 38건으로,
  후방 제외만으로 문제를 설명하거나 해결할 수 없다. 전방만의 recall도 83.48%다.
- 장애물 AP 하락은 68.85→62.80%, 즉 6.05%p이며 보행자의 36.29%p 하락과 규모가 다르다.
  새 전체 1m recall은 71.90%(기존 73.86%), 전방 68.85%(기존 76.14%),
  후방 75.38%(기존 71.58%)다. 후방 성능이 하락의 주원인이라는 가설은 지지되지 않는다.
- 새 장애물 FN 637건은 0–10m 285건, 10–20m 27건, 20–30m 325건이다.
  중거리는 recall 96.37%지만 근접 61.95%, 원거리 58.06%로 떨어진다.
  얇은 측면·부분 가림 등은 별도 asset/시점 검증이 필요한 원인 후보다.
- 측방도 평가에 남긴다. ±45° 전후방 sector를 제외한 새 장애물 왼쪽은 GT 146/TP 78,
  오른쪽은 GT 39/TP 0이다. 오른쪽의 관측은 적고 반복 observation이므로 전체 우측
  인지 능력이나 우측 카메라 장애로 곧바로 일반화하지 않는다.
- 전방/측방/후방과 거리를 **보조 지표로 분리하되 전체 360도 표는 유지**한다.
  회피 도중 옆에 남아 있는 장애물을 사후에 제외하여 성능을 높이지 않는다.

## 산출물·재현

- 기존 새 실행: `artifacts/evaluation/bevfusion/old_mission_current_contract_261004/`.
- 새 bag: `artifacts/evaluation/bevfusion/teammate_manual_261004/`.
- 두 root 모두 `aggregate_status_size_v2.json`, `tables_status_size_v2.md`, case별
  `summary_status_size_v2.json`, `pipeline/summary_status_size_v2.json`이 표의 근거다.
- 기존 실행 root의 `runtime_contract.json`은 E25·지도·카탈로그·tracker SHA와 제외 이유를 기록한다.
- 기존 보행자 원본은 `/home/luv/pointpainting_online_eval_rosbags_7_260831/`에서 찾았다.
  당시 시나리오와 최초 위치 차이는 횡단 0m, 거리별 최대 0.000405m였다. 이름을 크기만으로
  추측하지 않았으며 시나리오 경로/SHA는 각 `asset_name_map.json`에 보존했다.
- source/install 런치 인자 불일치를 발견하여 평가에서는 설치된 production 런치를 명시했다.
  Tracker 바이너리·production 설정은 변경하지 않았다. 실험 승격 OFF가 동일함을 확인했다.
- GT catalog 및 unresolved-frame 관련 단위 테스트 8개 통과. 6-case 완료 및 모든 표 지표의
  finite 값을 확인했다. 사용한 임시 ROS master와 inference server는 종료했다.

```bash
cd ~/AjouNice2026
python3 perception/detection3d/bevfusion/scripts/run_old_mission_current_contract.py

source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export PYTHONPATH="$PWD/base_ws/src/tracking_prediction_evaluation/src:${PYTHONPATH:-}"
python3 perception/detection3d/bevfusion/scripts/reevaluate_teammate_resolved_frames.py \
  --root artifacts/evaluation/bevfusion/old_mission_current_contract_261004 \
  --catalog dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json \
  --suffix _status_size_v2 --wait-for-count 6
python3 perception/detection3d/bevfusion/scripts/summarize_teammate_manual_evaluation.py \
  --root artifacts/evaluation/bevfusion/old_mission_current_contract_261004 \
  --suffix _status_size_v2
```

재현 명령은 이 PC의 legacy bag/시나리오와 build 결과 경로를 사용한다. 이미 완결된 case는
건너뛰므로 새 runtime 재실험은 별도 `--output` 경로를 사용한다. GPU inference 중에는
다른 학습/production inference와 동시에 실행하지 않는다.
