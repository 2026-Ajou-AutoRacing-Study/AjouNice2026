# MORAI K-City 객체 자산 카탈로그와 학습 정책

## 1. 목적

MORAI Scenario Edit에서 배치 가능한 자산을 시나리오 파일에 한 번씩 넣어
확인하고, 이를 시나리오 생성기가 사용할 수 있는 카탈로그로 변환했다.

중요한 원칙은 **MORAI 시나리오의 저장 컨테이너와 학습 ontology를
동일하게 보지 않는 것**이다. 예를 들어 `pedestrianList`에는 동물과
자전거 탑승자가 있다. 따라서 목록의 위치만 보고 모두 Pedestrian 또는
Obstacle로 학습시키면 라벨 의미가 섞인다.

기계 판독 결과는 다음 파일에 있다.

- `configs/morai_asset_catalog_260807.json`
- 재생성 스크립트: `scripts/extract_morai_asset_catalog.py`

원본 시나리오 세 파일과 MORAI `optionList.byte`는 설치본에 종속되므로
Git에 복사하지 않는다. 카탈로그에는 각 입력 파일의 SHA-256을 남겨 같은
입력에서 생성됐는지 확인할 수 있게 했다.

## 2. 추출 결과

| MORAI 원본 영역 | 고유 DataID | 1차 분류 |
|---|---:|---|
| `vehicleList` | 43 | Vehicle 43 |
| `pedestrianList` + `aiPedstrianList` | 33 | Pedestrian 28, Animal 3, Cyclist 1, 사람+동물 복합 1 |
| curated `objectList` | 33 | Obstacle 19, RoadFurniture 10, RoadSurfaceObject 2, RoadsideFurniture 2 |
| 합계 | 109 | 모델 표시명 미확인 0 |

curated `objectList`에는 `40200162` `concrete-barrier`가 두 번 있었다.
생성기는 DataID 기준으로 중복 제거하고 `source_occurrences: 2`를
기록한다.

- `40200162`: `concrete-barrier`

## 3. 사용자가 수행한 목록 정리 판단

### Object 영역의 차량·트럭 제거

적절한 결정이다. 차량처럼 보이는 자산을 `objectList`와 `vehicleList` 양쪽
에서 뽑으면 다음 문제가 생긴다.

- 같은 외형이 Vehicle과 Obstacle이라는 서로 다른 정답으로 중복될 수 있다.
- 동적 차량 경로와 방향 제약을 적용하기 어렵다.
- 현재 보행자 단일 클래스에서는 모두 배경이지만, 향후 BEV 다중 클래스
  학습에서 Vehicle 정답이 누락되거나 잘못 분류될 수 있다.

차량·버스·트럭·SUV·MPV는 `vehicleList`의 43종만 Vehicle 후보로 삼는다.

### 사람 결합형 Object 제거

사용자는 두 번째 curated 시나리오
`obstacle_list_without_vehicle_and_ped_like_objects.json`에서 다음
8개 DataID를 자동 생성 pool에서 제거했다.

| DataID | 이름 | 제거 이유 |
|---:|---|---|
| 40200011 | `BabyStroller1` | 유모차를 미는 사람 결합형 |
| 40200012 | `BabyStroller2` | 유모차를 미는 사람 결합형 |
| 40200013 | `BabyStroller3` | 유모차를 미는 사람 결합형 |
| 40200017 | `SportBike` | 탑승자 결합형 |
| 40200018 | `ElectricScooter1` | 탑승자 결합형 |
| 40200019 | `ElectricScooter2` | 탑승자 결합형 |
| 40200020 | `Scooter1` | 탑승자 결합형 |
| 40200021 | `Scooter2` | 탑승자 결합형 |

적절한 결정이다. 결합형 자산은 사람 pixel과 3D box가 분리되는지 모르는
상태에서 Pedestrian positive/negative 어느 쪽으로 사용해도 라벨 충돌을
일으킬 수 있다. 현재 파일럿은 사람이 없는 독립 자산만 hard negative로
사용하고, rider class가 필요해지는 향후 별도 데이터셋에서 다시 검토한다.

사람이 없는 `Bicycle_city`, `Bicycle_mountain`, `Scooter_01`,
`Scooter_02`는 유지한다. 이 네 자산은 hard-negative 전용으로 강조하지
않고 일반 Obstacle로 현실적인 위치에 배치한다. 향후 다중 클래스
ontology에서는 TwoWheeler 별도 class 후보로 재매핑할 수 있다.

## 4. 학습·생성 정책

### 4.1 Vehicle

- 43종 모두 시나리오 생성 허용
- 승용차, SUV, 버스·밴, 트럭, 왜건, 전기 승용차, MPV로 subtype 유지
- 현재 Pedestrian PointPainting에서는 배경 문맥
- 향후 BEVFusion/다중 클래스 PointPillars에서는 Vehicle positive
- 차로 또는 주차 영역에만 배치하고 차량 yaw를 도로 접선에 맞춤

### 4.2 Pedestrian 컨테이너

| 자산 유형 | 현재 정책 | 향후 정책 |
|---|---|---|
| 일반 성인·아동·작업자·경찰 25종 | Pedestrian positive | Pedestrian |
| NCAP dummy 3종 | 제한된 비율의 positive | 외형 편향 확인 후 유지 |
| `Bicycle_Female1` | 격리 | Cyclist/TwoWheelerRider |
| Dog 2종, Deer 1종 | Pedestrian hard negative 또는 별도 제외 | Animal |
| `Ped_Pet1` | 격리 | 사람과 동물 GT 구조 확인 후 결정 |

`aiPedstrianList`의 `police1`은 일반 배치 모델과 행동 필드가 다를 수 있어
DataID는 사용할 수 있지만 동적 시나리오 투입 전 로드·이동 검증을 한다.

### 4.3 Obstacle 컨테이너

| 자산 유형 | 수 | 생성 정책 |
|---|---:|---|
| 상자·배럴·드럼·방호물 | 15 | 일반 obstacle/hard negative로 사용 |
| 속도 표지 | 10 | 도로변의 의미 있는 위치에만 배치 |
| 나무·가로등 | 2 | roadside 영역에만 배치 |
| 과속방지턱·맨홀 | 2 | 지면 높이와 도로 방향에 맞춰 배치 |
| 사람이 없는 자전거·스쿠터 | 4 | 일반 Obstacle로 현실적인 위치에 배치 |

고정 시설물을 도로 중앙에 무작위로 두는 식의 배치는 이미지 다양성은
늘리지만 실제 분포를 망친다. 표지판·가로등·나무·도로 표면 자산에는
MGeo semantic zone 제약을 적용한다.

## 5. 180-run 파일럿 적용

초기 180-run은 다음처럼 보수적으로 사용한다.

- Vehicle: 전체 subtype 사용 가능
- Pedestrian positive: 일반 사람 25종 우선
- NCAP dummy: Pedestrian 장면의 최대 10% 이내
- Animal: hard-negative 장면에 제한적으로 사용
- `Bicycle_Female1`, `Ped_Pet1`: 첫 180-run 자동 sampling에서는 제외
- 사람이 없는 자전거·스쿠터 4종: 일반 Obstacle pool에서 낮은 자연 빈도로 사용
- 사람 결합형 obstacle 8종: 자동 생성 및 180-run에서 제외
- 모든 원본 Semantic/Instance/BBox/3D box는 보존해 나중에 ontology를
  바꿔도 재캡처하지 않게 함

이 정책은 카탈로그의 `generation_policy`와
`label_audit_required` 필드로 표현한다. `pedestrianList`에 남은
`quarantine`은 자산을 삭제한다는 뜻이 아니라, 자동 학습 라벨로 확정하지
않았다는 뜻이다.

## 6. 다음 label audit

대표 장면 하나에 다음 자산을 겹치지 않게 배치해 1~2프레임만 캡처한다.

- `Scooter_01`, `Bicycle_city`의 future TwoWheeler 재매핑 가능성
- `Bicycle_Female1`, `Ped_Pet1`
- Dog, Deer, NCAP dummy

각 자산에 대해 아래를 비교한다.

1. Camera Intensity에 실제 사람 형상이 포함되는지
2. Camera Semantic의 class/color
3. Camera Instance와 BBox2D/BBox3D의 instance 수
4. LiDAR Semantic/Instance의 class
5. LiDAR `Instance.txt`와 `Instance_8Points.txt`의 class·box 수

결과가 확인되면 `pedestrianList`의 `quarantine` 자산을 Pedestrian,
Cyclist/TwoWheeler, Animal 또는 제외 중 하나로 확정하고, 모델별 label
mapping을 별도 설정으로 만든다. 사용자 판단으로 제외한 사람 결합형
obstacle 8종은 이 audit 결과와 무관하게 현재 자동 생성 pool에 넣지 않는다.

## 7. 카탈로그 재생성

저장소 루트에서 실행한다.

```bash
python3 dataset/morai_od/scripts/extract_morai_asset_catalog.py \
  --scenario-dir \
  "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --option-list \
  "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/Version/optionList.byte"
```

원본 시나리오를 수정했다면 생성 후 Git diff에서 count, SHA-256,
DataID별 정책 변화를 검토한다.
