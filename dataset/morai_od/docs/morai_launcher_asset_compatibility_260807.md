# 대회용·개인용 MORAI asset 호환 정책

## 1. 목적

최종 학습 대상 domain은 대회용 런처지만 자동 Camera·LiDAR·GT 수집은
개인용 런처에서 수행한다. 따라서 자동 생성 asset은 다음 두 조건을 모두
만족해야 한다.

1. 대회용 런처의 사용자 검수 목록에 존재한다.
2. 개인용 런처의 사용자 검수 목록에도 존재한다.

공통 asset도 대회용 JSON actor template를 그대로 복사하지 않는다.
개인용 런처가 직접 저장한 template를 사용하고 위치·회전·link 정보만
생성기가 새로 덮어쓴다.

목록 시나리오에서 사용자가 임의로 둔 원래 위치는 생성 위치로 사용하지
않는다.

## 2. 입력과 산출물

대회용 사용자 검수 목록:

```text
~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/
R_KR_PR_K-city_2025/
├── vehicle_list.json
├── pedestrian_list.json
└── obstacle_list_without_vehicle_and_ped_like_objects.json
```

개인용 사용자 검수 목록:

```text
~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/
R_KR_PR_K-city_2025/
├── vehicle_list_per.json
├── pedestrian_list_per.json
└── obstacle_list_per_without_vehicle_and_ped_like_objects.json
```

재현 가능한 카탈로그와 비교 결과:

```text
configs/morai_asset_catalog_260807.json
configs/morai_asset_catalog_personal_260807.json
configs/morai_asset_launcher_compatibility_260807.json
```

## 3. 비교 결과

| 그룹 | 대회용 | 개인용 | 공통 자동 대상 | 대회용 전용 | 개인용 전용 | 정책 제외 |
|---|---:|---:|---:|---:|---:|---:|
| Vehicle | 43 | 49 | 43 | 0 | 6 | 0 |
| Pedestrian 계열 | 33 | 32 | 31 | 2 | 1 | 0 |
| Obstacle | 33 | 36 | 31 | 1 | 5 | 1 |
| 합계 | 109 | 117 | 105 | 3 | 12 | 1 |

### 3.1 대회용에만 기록된 asset

| 그룹 | DataID | 이름 | 개인용 v1 자동 수집 실적 |
|---|---:|---|---:|
| Pedestrian | 50100055 | police_f_1 | 0/26 성공 |
| Pedestrian | 50200003 | Boy2 | 0/27 성공 |
| Obstacle | 40100062 | Scooter_01 | 0/8 성공 |

2026-08-22에 두 위치와 두 번의 독립 실행으로 다시 검증했다. 각 probe
직전에는 같은 객체 그룹의 정상 control asset만 포함한 장면을 넣었다.
control은 12/12 성공했지만 아래 세 asset probe는 12회 중 0회 생성됐다.
실패 장면에는 control actor만 정확히 남았다. 따라서 ROS 연결 불안정이
아니라 개인용 런처에 해당 asset/template이 없는 실제 호환성 문제로
확정하고 개인용 자동 수집에서 제외한다.

대회용 전용 세 종류는 최종 competition coverage에서 버리지 않는다.
별도 scenario를 만든 뒤 대회용 런처에서 수동 배치·수집하고, dataset
metadata에 `capture_runtime=competition_manual`로 구분한다.

### 3.2 과거 런타임 제외 재검증

과거 geometry trial에서는 `50100005 Woman2`와 `50200004 Girl2`가
나타나지 않아 asset 고유 문제로 판단했다. 하지만 당시에는 여러 asset을
한 장면에 섞었고 MORAI ROS 연결도 불안정해 원인을 분리하지 못했다.

2026-08-22 interleaved control/probe 재검증에서는 두 asset 모두 서로 다른
두 위치에서 2/2 생성됐다. 직전 control도 모두 성공했다. 따라서 기존
실패는 asset 고유 문제가 아니라 장면 또는 ROS/MORAI 상태 문제로
재분류하고 두 asset을 개인용 자동 생성 풀에 복구한다.

| DataID | 이름 | 과거 결과 | 재검증 | 개인용 자동 생성 |
|---:|---|---:|---:|---|
| 50100005 | Woman2 | 0/3 | 2/2 | 복구 |
| 50200004 | Girl2 | 0/1 | 2/2 | 복구 |

같은 방식으로 과거 quarantine 차량 8종도 각각 두 위치에서 2/2 생성됐다.
따라서 차량 asset quarantine은 전부 해제한다. 재현 가능한 scene generator,
분석기와 전체 trial 판정은 다음 파일에 둔다.

| 그룹 | 재검증 asset | probe 결과 | 판정 |
|---|---|---:|---|
| Vehicle | Grandeur, Porter2, DEFAULT_SUV, DEFAULT_BUS | 각 2/2 | 복구 |
| Vehicle | Niro EV, Xcient Mixer, Stinger, DEFAULT_MPV | 각 2/2 | 복구 |
| Pedestrian | Woman2, Girl2 | 각 2/2 | 복구 |
| Pedestrian | police_f_1, Boy2 | 각 0/4 | 개인 MRP 미지원 유지 |
| Obstacle | Scooter_01 | 0/4 | 개인 MRP 미지원 유지 |

competition 전용 세 asset의 4회는 두 위치 검증을 독립 실행으로 한 번 더
반복한 값이다. 각 probe 직전 control은 모두 성공했다.

```text
scripts/generate_morai_asset_revalidation_set.py
scripts/analyze_morai_asset_revalidation.py
configs/morai_asset_revalidation_260822.json
```

### 3.3 Tree 계열 정책 제외

`Tree08`은 사용자가 대회용 목록 작성 과정에서 실수로 포함한 항목이며,
개인용 런처에도 존재한다. 그러나 나무는 이 프로젝트의 OD 대상 actor가
아니라 정적 지도 배경으로 취급한다.

따라서 런처 공통 여부와 관계없이 이름이 `Tree`로 시작하는 모든 asset은
다음 대상에서 완전히 제외한다.

- 자동 시나리오 배치
- 대회용 런처 수동 수집 queue
- asset coverage 목표

현재 정책 제외 목록은 `40200009 Tree08` 한 개다. 이후 카탈로그에 다른
`Tree*` asset이 추가되어도 비교 스크립트가 자동으로 제외한다.

### 3.4 개인용에만 기록된 asset

개인용 전용 asset은 대회 환경에 존재한다는 보장이 없으므로 최종 학습
시나리오에 배치하지 않는다.

```text
Vehicle:
  30100017 2019_hyundai_genesis_g90_4WS
  30200016 DR_KGTorres_EVX_2024
  30400014 SE_CT797F_2009
  30600001 WeGo_ERP42
  30600014 AR_DJMini3pro_2022
  30700043 DR_OMLift_2022_Bus

Pedestrian:
  50300011 Dog10

Obstacle:
  40100064 Sportbike_01
  40200085 Color_Cone
  40200086 Rubber_Cone
  40200160 Rockslide
  40200161 Barbed_Wirefence
```

## 4. 공통 asset의 template 차이

DataID가 같아도 serialized actor 형식이 같지는 않았다.

- 공통 Vehicle 43/43개에서 link/waypoint/activeDistance 관련 차이 발견
- 공통 Pedestrian 31개 중 30개에서 `anim.m_FileID` 차이 발견
- 공통 Obstacle 31개는 pose를 제외한 template 차이 없음

특히 보행자는 대회용 animation file ID를 개인용 런처에 그대로 넣으면
DataID가 존재해도 spawn되지 않을 수 있다. 그러므로 v2 생성기는 공통
asset의 이름·ontology·대회 대상 여부는 대회용 카탈로그에서 가져오고,
실제 actor JSON은 개인용 목록에서 복제한다.

## 5. 자동 생성 정책

```text
대회용 ∩ 개인용
  → 개인용 template 사용
  → 생성기가 pose/link/속도/UNIQUEID 재설정
  → 개인용 자동 캡처

대회용 - 개인용
  → 개인용 자동 생성 제외
  → 대회용 런처 수동 수집 queue

개인용 - 대회용
  → target domain에 없으므로 제외

Tree* asset
  → 런처 공통 여부와 관계없이 OD dataset에서 완전 제외
```

현재 이 정책으로 `sunny13_180_smoke_v2` 180개를 생성했다. v2에는
대회용 전용 세 DataID, Tree 계열 및 개인용 전용 DataID가 하나도 들어
있지 않고,
모든 보행자 `anim.m_FileID`가 개인용 목록과 일치함을 정적으로 검증했다.

이번 단계에서는 객체 간 거리, 실제 collider, 지형 Z, 최대 객체 수,
위험 anchor 같은 기하 정책은 수정하지 않았다. asset/template 문제를
분리해 확인한 다음 별도 단계에서 개선한다.

## 6. 카탈로그 재생성

개인용 목록을 수정했다면 다음 순서로 갱신한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/extract_morai_asset_catalog.py \
  --scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --option-list \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/Version/optionList.byte" \
  --vehicle-file vehicle_list_per.json \
  --pedestrian-file pedestrian_list_per.json \
  --obstacle-file \
  obstacle_list_per_without_vehicle_and_ped_like_objects.json \
  --output \
  dataset/morai_od/configs/morai_asset_catalog_personal_260807.json

python3 \
  dataset/morai_od/scripts/compare_morai_launcher_assets.py
```

그 뒤 시나리오를 다시 생성한다.

```bash
python3 \
  dataset/morai_od/scripts/generate_sunny13_180_smoke_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```
