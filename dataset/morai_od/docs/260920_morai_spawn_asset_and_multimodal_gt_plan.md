# MORAI 랜덤 스폰·Asset 호환성과 BEVFusion 멀티모달 GT 계획

- 문서 상태: **EVIDENCE + PLAN**
- 마지막 갱신: 2026-09-20
- 적용 범위: MORAI `mr`/`mrp` asset, 고주로 Random Spawn, Base3/Base4
  BEVFusion dataset 변환

## 1. 결론

1. 현재 61개 고주로 spawn 관측은 대회용 MR Vehicle 43종을 개별 모델 단위로
   거의 균등 추출한다는 가설과 충돌하지 않는다. Wagon을 승용 계열로 합치면
   관측 family 비율은 카탈로그 구성비와 특히 가깝다.
2. Base3에서 대회용 Vehicle을 임의로 많이 제거한 것은 아니다. MR Vehicle 43종은
   모두 MRP와 공통이며 Base3 생성 deck에도 전부 복구되어 있다.
3. `MIN_GT_POINTS=5`는 LiDAR-only 모델에는 보수적인 quality gate지만, Camera+LiDAR
   BEVFusion에서는 카메라에 보이는 sparse/zero-point GT를 false negative로 만들 수
   있다. 다음 변환 A/B는 LiDAR point 수와 camera visibility를 함께 사용해야 한다.
4. `WoodBox`는 load 실패가 아니라 mesh/LiDAR point와 MORAI GT box의 정합 문제다.
   대회 출제 가능성이 있으면 제외를 유지하기보다 전용 시각화·label A/B 후 제한적으로
   재도입한다.

## 2. MR/MRP asset 카탈로그 근거

사용자가 각 런처에서 Vehicle/Pedestrian/Obstacle 목록 시나리오 세 개씩을 직접
저장한 뒤 추출한 raw catalog는 다음과 같다.

| 런처 | Vehicle | Pedestrian container | Obstacle container | 합계 |
|---|---:|---:|---:|---:|
| `mr` 대회용 | 43 | 33 | 33 | 109 |
| `mrp` 개인용 | 49 | 32 | 36 | 117 |

MORAI container와 학습 ontology는 같지 않다. MR의 Pedestrian container 33종은
Pedestrian 28, Animal 3, Cyclist 1, 사람+동물 복합 1이고, Obstacle container
33종은 일반 Obstacle 19와 RoadFurniture/RoadSurface/Roadside 자산을 포함한다.

두 런처의 raw 교집합은 Vehicle 43, Pedestrian 계열 31, Obstacle 31, 총 105종이다.
공통 `Speed_Bump`를 dataset scope에서 제외해 현재 automatic eligible은 104종이다.
최종 Base3 positive pool은 별도의 ontology·가시성 정책을 거쳐 Vehicle 43,
Pedestrian 22, Obstacle 10종이다.

### 2.1 진짜 MRP 미지원으로 확인한 세 자산

| Class | DataID | Asset | 최초 관측 | control/probe 재검증 | 판정 |
|---|---:|---|---:|---:|---|
| Pedestrian | 50100055 | `police_f_1` | 0/26 | 0/4 | MRP 미지원 유지 |
| Pedestrian | 50200003 | `Boy2` | 0/27 | 0/4 | MRP 미지원 유지 |
| Obstacle | 40100062 | `Scooter_01` | 0/8 | 0/4 | MRP 미지원 유지 |

각 probe 직전에 같은 container의 known-good control scene을 넣었고 control은 모두
성공했다. 따라서 이 세 자산은 일반 ROS/ScenarioLoad 불안정보다 MRP asset/template
부재 가능성이 높다.

### 2.2 2026-09-20 현재 런처 재검증 결과

- Vehicle 7종: Porter2, DEFAULT_SUV, DEFAULT_BUS, Niro EV,
  Xcient Mixer, Stinger, DEFAULT_MPV.
- Pedestrian 2종: `Woman2`, `Girl2`.

각각 두 안전 위치에서 2/2 load에 성공해 자동 생성 pool에 복구했다. 과거 실패는
asset 고유 미지원이 아니라 여러 actor를 섞은 장면 또는 당시 ROS/MORAI 상태 문제로
재분류한다.

`2017_Hyundai_Grandeur(30100001)`는 과거 재검증에서는 2/2 성공했지만, 현재
MRP `R1.260701.H3`에서는 known-good control이 통과하는 동안 두 안전 위치 모두
0/2였고, 별도 5회 재시도에서도 0/5였다. `optionList`와 personal vehicle template에는
여전히 등록되어 있으므로 단순 catalog 누락이 아니라 현재 launcher/template/
ScenarioLoad 조합의 회귀로 분류한다. 자동 생성 pool에서는 일단 quarantine하고,
런처 갱신 때 다시 확인한다.

단, 같은 런처의 sample `RandomMode`에서는 Grandeur가 반복적으로 정상 spawn됐다.
그러므로 이 현상은 runtime model 부재가 아니라 direct static ScenarioLoad template/
직렬화 경로 문제로 취급한다.

근거:

- `/home/luv/morai_cap/260920_asset_revalidation_live_probe`
- `/home/luv/morai_cap/260920_asset_revalidation_grandeur_retry`
- `reports/asset_revalidation_260920_live.json`

현재 대용량 capture가 끝난 뒤에는 기존
`generate_morai_asset_revalidation_set.py`와 `analyze_morai_asset_revalidation.py`를
기준으로 아래만 regression 확인한다.

1. 과거 복구 후보 10종과 미지원 3종.
2. 세 안전 anchor, 자산별 최소 3회.
3. 매 probe 직전 같은 class의 known-good control.
4. load 성공, `/Object_topic`, Camera/LiDAR 가시성, GT box 정합을 별도 판정.
5. `unsupported`, `transient load/ROS`, `geometry`, `sparse visibility`,
   `annotation mismatch`를 서로 섞지 않는다.

### 2.3 MR 복구 결과와 Base4 정책

대회 계열 MR에서 minimal five-interface Ego Network로 다시 검사한 결과 MRP 실패
candidate 네 종은 모두 두 위치에서 2/2 load되고 `/Object_topic` identity도 일치했다.

| Asset | MRP direct-load | MR minimal direct-load |
|---|---:|---:|
| `2017_Hyundai_Grandeur` | 0/2, retry 0/5 | 2/2 |
| `police_f_1` | 0/2 | 2/2 |
| `Boy2` | 0/2 | 2/2 |
| `Scooter_01` | 0/2 | 2/2 |

따라서 Base4를 MR에서 취득할 때는 이들을 catalog 단계에서 제거하지 않는다. 다만
`Scooter_01`처럼 대회 ontology/미션 적합성이 낮거나 sparse한 asset은 load 가능성과
positive 학습 편입을 분리해 판단한다. MRP 자동 수집에서는 여전히 제외한다.

## 3. 고주로 Random Spawn 분포 가설

샘플 시나리오의 고주로 spawn point 세 개는 `spawnVehicleTypeID=-1`이며 RandomMode를
사용한다. 초기 가설은 61개 관측이었으나, 2026-09-20에 80분 48초 동안 482개의
unique birth event를 별도 수집해 갱신했다.

| Family | eligible 모델 수/38 | 482회 관측 비율 |
|---|---:|---:|
| Passenger car | 13 | 33.40% |
| SUV | 11 | 29.88% |
| Electric passenger car | 5 | 14.32% |
| Truck | 4 | 10.37% |
| MPV | 3 | 6.64% |
| Bus/Van | 1 | 3.11% |
| Wagon | 1 | 2.28% |

최신 482회에서는 38개 eligible 모델이 관측됐고 모델 균등 goodness-of-fit은
`p=0.974`, family 구성비 goodness-of-fit은 `p=0.885`였다. 현재 MRP RandomMode는
이 38종을 개별 모델 단위로 거의 균등 추출한다고 보는 것이 타당하다.

단, 38종은 MR 43종 전체가 아니다. MR/MRP 공통 35종과 MRP-only 3종으로 구성되며,
MR 8종은 한 번도 나타나지 않았다. `listVehicleLengthType=[2]` 필터가 유력하지만
정의는 비공개이므로 대회 MR pool은 MORAI 답변 전까지 확정하지 않는다.

현재 Base3-Online Highway Static V2는 MR 43종을 균등 deck으로 사용하며 생성된
2,070 Vehicle의 family 비율은 passenger 30.8%, SUV 25.7%, truck 13.6%, EV 9.4%,
MPV 9.0%, bus/van 6.9%, wagon 4.6%다. 즉 실제 sample spawn과 차이가 나는 주된 부분은
MR 43종 coverage deck과 RandomMode eligible 38종 pool의 차이다. 대회 MR의 정확한
eligible pool은 MORAI 답변 전까지 확정하지 않는다.

### 3.1 RandomMode 실측 계약과 결과

사용자 요청 시점까지 **482 unique spawn birth event**를 기록했다.

- 센서 영상 없이 `/Object_topic` 중심의 작은 rosbag이면 충분하다.
- frame별 occupancy를 세지 않는다. 느리거나 오래 체류하는 차량이 과대표집된다.
- object ID가 사라졌다 다시 나타나면 ID가 같더라도 새 birth event로 센다.
- latest Object topic의 실제 asset명을 우선 사용한다.
- 과거 bag처럼 link ID가 오면 size signature를 fallback으로 사용한다.
- first appearance 시점에는 이미 합류한 뒤여서 source spawn 6/8/10은 추정하지 않는다.

Base4 Vehicle asset 정책은 coverage baseline으로 MR 43종 deck을 유지할 수 있으나,
RandomMode 모사 실험에서는 MORAI가 확인한 competition eligible pool을 별도 사용한다.

실측 분석기는 `scripts/analyze_morai_highway_random_spawn.py`다. 최신 동적 spawn NPC의
`ObjectStatus.name`에는 실제 asset명이 발행됐다. 과거 bag의 link ID에는 통제된 Base3
metadata의 3D size signature를 사용하며, 동일 size ambiguity는 추측하지 않는다.

## 4. 현재 BEVFusion의 sparse GT 계약

Raw SaveSensorData와 rosbag에는 sparse actor도 남는다. 그러나 BEVFusion info 변환기는
offline/online 모두 `--minimum-gt-points` 기본값 5를 사용하고, GT box 내부 point가
5개 미만이면 해당 instance를 PKL에서 제거한다. 모델 pipeline이 학습 중 다시 거르는
것이 아니라 E25와 online fine-tuning이 읽는 train/val/test PKL에 이미 반영된 정책이다.

Base3-Static 30 m 내 원본 audit:

| Class | 전체 GT | 5-point 미만 | 제거율 |
|---|---:|---:|---:|
| Vehicle | 7,247 | 8 | 0.11% |
| Pedestrian | 5,919 | 187 | 3.16% |
| Obstacle | 16,379 | 1,204 | 7.35% |

Online mixed30 pilot:

| Class | 유지 | 5-point 미만 제거 | 제거율 |
|---|---:|---:|---:|
| Vehicle | 76 | 2 | 2.6% |
| Pedestrian | 34 | 32 | 48.5% |
| Obstacle | 100 | 74 | 42.5% |

Online mixed30은 작은 통제 pilot이므로 전체 대회 분포로 일반화하지 않는다. 다만
offline val/test가 `LiDAR point >= 5`인 쉬운 조건부 subset이고, 실제 online의
Pedestrian/Obstacle sparse tail을 충분히 평가하지 않았을 가능성을 보여주는 근거다.

### 4.1 문제

BEVFusion은 Camera+LiDAR 모델이다. 카메라에 선명하게 보이는 객체를 LiDAR point 수만
보고 GT에서 제거하면 입력에는 객체가 남고 정답만 사라져 false-negative supervision이
될 수 있다. 특히 `filter_empty_gt=False`인 frame은 그대로 학습에 남을 수 있다.

반대로 `minimum_gt_points=0`으로 모든 GT를 유지하면 모든 카메라 밖이거나 완전 가려져
어느 센서로도 관측할 수 없는 target까지 학습하게 된다. 따라서 단순 threshold 제거가
아니라 modality-aware visibility가 필요하다.

### 4.2 다음 A/B의 GT 유지 정책

```text
keep_gt = (num_lidar_pts >= 5) OR camera_visible
```

- LiDAR 5점 이상: 기존처럼 유지.
- LiDAR 1--4점 + camera-visible: 유지.
- LiDAR 0점 + camera-visible: 유지.
- LiDAR 0점 + 모든 camera 비가시: positive에서 제외하거나 ignore.
- 원본 GT와 point 수는 수정하지 않고 파생 visibility metadata를 추가한다.

권장 metadata:

- `num_lidar_pts`
- `camera_visible`
- `visible_camera_count`
- `visible_pixel_count`
- `projected_bbox_area`
- `visibility_source = lidar | camera | both`
- `annotation_geometry_suspect`

Base3 Offline은 Camera Instance/BBox2D/Semantic이 있어 재취득 없이 실제 pixel
visibility를 계산할 수 있다. Online rosbag은 raw camera와 3D GT만 있어 projection,
in-frame area, truncation과 다른 actor의 depth ordering으로 근사해야 한다. Online의
완전한 occlusion 판정은 별도 GT/instance topic이 없는 한 제한적이다.

첫 controlled experiment는 같은 raw Base3와 같은 split에서 다음 두 info를 만들고
학습 조건을 고정한다.

1. A: 기존 `LiDAR >= 5`.
2. B: `LiDAR >= 5 OR camera-visible`.

Val도 같은 multimodal eligibility를 사용하되 기존 LiDAR-supported metric을 함께 남긴다.
채택은 offline mAP만으로 하지 않고 Pedestrian/Obstacle rosbag recall·precision과
planner에 들어가는 false positive까지 확인한다.

## 5. WoodBox 전용 계획

`CargoBox(40100001)`는 공식 샘플 장애물이며 Base3 positive다.
`WoodBox(40100002)`는 load되지만 pose/yaw에 따라 실제 mesh/LiDAR point가 원본 MORAI
GT box보다 약 5--10 cm 밖에 있어 최종 Base3 positive pool에서 제외했다. 현 BEVFusion
voxel 7.5 cm와 비교하면 약 0.7--1.3 voxel이라 무시할 수 없는 크기다.

대규모 capture 종료 뒤 다음 전용 audit를 수행한다.

- 거리 5/10/15/20 m.
- yaw 0/30/45/60/90 deg.
- 같은 위치의 CargoBox를 known-good control로 사용.
- BEV 확대 그림에 raw GT, `+3 cm` box, box 밖 mesh point, corrected candidate를 함께 그림.
- camera projection은 보조 근거로 쓰고 signed point-to-box distance를 주 근거로 쓴다.

2026-09-20에 위 grid의 CargoBox/WoodBox 20개 exact-pose pair, 총 40 scene을 생성했다.

- manifest: `manifests/woodbox_alignment_260920.json`
- generator: `scripts/generate_woodbox_alignment_probe.py`
- live loader/marker: `scripts/probe_morai_asset_compatibility_live.py --hold-sec ...`
- marker topic: `/dataset/scenario_marker`

이 세트는 asset만 바꾸고 ego/거리/yaw/장애물 pose를 동일하게 유지하므로 두 asset의
mesh--GT 차이를 직접 비교할 수 있다.

MORAI가 수정하지 않지만 대회 출제가 가능하다면:

1. raw GT는 provenance로 보존한다.
2. WoodBox visibility 판정에만 충분히 검증한 asset-aware support region을 쓴다.
3. raw-GT target과 physical-mesh corrected target을 별도 A/B한다.
4. 첫 재도입은 Obstacle positive의 5--10%로 제한한다.
5. 대회 목표가 bbox 정량평가가 아니라 안전 회피라면 실제 mesh footprint와 planning
   safety inflation 결과를 우선한다.

## 6. MORAI 문의 항목

1. 로터리·고주로 NPC의 초기/목표 속도 범위가 샘플 시나리오와 동일한가.
2. 고주로 RandomMode가 어떤 vehicle model/filter와 확률을 사용하는가. 개별 model
   균등 추출인지 family별 가중치가 있는지도 확인한다.
3. `WoodBox(40100002)`의 mesh/LiDAR point와 제공 GT 3D box 불일치가 알려진 문제인지,
   대회 버전에서도 동일하고 실제 장애물 후보가 될 수 있는지 확인한다.

## 7. 관련 근거

- `morai_asset_launcher_compatibility_260807.md`
- `morai_asset_catalog_260807.md`
- `base3_static_asset_catalog.md`
- `legacy/base3_static_classcentric_10k_design.md`
- `legacy/base3_static_10k_final_capture_audit.md`
- `configs/morai_asset_revalidation_260822.json`
- `configs/morai_asset_launcher_compatibility_260807.json`
- `perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py`
- `perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py`
