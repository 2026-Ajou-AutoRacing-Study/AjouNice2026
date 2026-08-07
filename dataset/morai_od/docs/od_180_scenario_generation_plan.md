# MORAI OD 180-run 파일럿 시나리오 설계

## 1. 목적과 범위

이 파일럿의 즉시 목적은 현재 `Pedestrian` 단일 클래스 PointPainting
파인튜닝이 실제로 개선되는지 빠르게 확인하는 것이다. 그러나 원본 데이터는
이 모델에만 맞춰 버리지 않고, 이후 차량·보행자·장애물 다중 클래스
PointPillars, BEVFusion 계열의 multi-camera BEV representation 및 다른
카메라 중심 BEV 모델에서도 재사용할 수 있게 수집한다.

180은 최종 데이터셋 규모가 아니다. 장면당 6프레임이면 총 1,080개
multi-sensor frame이고, Camera 4대 기준 RGB image는 4,320장이다. 이는
파이프라인과 첫 파인튜닝 효과를 확인하기에는 유용하지만, 다양한 객체와
환경에서 일반화되는 BEV 모델을 처음부터 학습하기에는 작다.

## 2. 임시 주행·캡처 기준

파일럿 Ego 목표 속도는 **3.6 km/h(1.0 m/s)**로 고정한다.

| 항목 | 값 |
|---|---:|
| 이동 방식 | `rolling-fast` |
| Ego 목표 속도 | 3.6 km/h |
| 장면당 캡처 | 6프레임 |
| 캡처 간격 | 이동거리 0.6 m |
| 마지막 프레임까지 이동거리 | 약 3.6 m |
| `load-settle-sec` | 0.5초 |
| `capture-stable-sec` | 0.3초 |

3.6 km/h를 쓰는 이유는 다음과 같다.

1. 7.2 km/h보다 경로 overshoot와 객체 충돌 위험이 낮다.
2. 0.5 m/s 이하보다 장면 처리량이 낫고, 현재 검증된 제어 범위 안이다.
3. 0.6 m 간격에서 인접 프레임이 완전히 같지는 않으면서도 객체 association이
   가능한 정도의 시점 차이를 만든다.
4. p01~p03 18프레임 `rolling-fast`에서 필수 센서 파일 누락 없이
   통과한 실행 근거가 있다.

이 속도는 대회 주행 속도나 최종 temporal 모델 입력 주기를 뜻하지 않는다.
현재 6프레임은 sparse viewpoint augmentation에 가깝다. StreamPETR처럼
일정한 고주기 temporal sequence를 학습하려면 이후 별도로 10~15 Hz
연속 수집을 설계해야 한다.

## 3. Sensor_6에 따른 설계 범위

기준 파일은 `configs/sensors/Sensor_6.json`이다.

| 센서 | 위치·방향 | 해상도·주기 | 데이터 생성 영향 |
|---|---|---|---|
| Front camera | `(1.9, 0, 1.2)`, yaw 0° | 1280×720, 30 Hz, FOV 90° | 전방 중·원거리와 작은 보행자 |
| Front-left | `(1.15, 0.65, 1.2)`, yaw 70° | 640×480, 30 Hz, FOV 130° | 좌측·교차로·카메라 overlap |
| Front-right | `(1.15, -0.65, 1.2)`, yaw 290° | 640×480, 30 Hz, FOV 130° | 우측·교차로·카메라 overlap |
| Back camera | `(-0.28, 0, 1.15)`, yaw 180° | 1280×720, 30 Hz, FOV 90° | 후방 객체와 360° BEV |
| VLP16 LiDAR | `(0.855, 0, 1.388)` | 15 Hz 설정, 물리 범위 100 m | 낮은 채널 수, 원거리 보행자 sparse point |

현재 OpenPCDet PointPainting 설정의 학습 범위는
`[-30, -30, -3, 30, 30, 2]` m이다. 따라서 주 학습 객체 중심은 Ego
기준 28 m 이내에 둔다. 100 m LiDAR 물리 범위를 그대로 사용하면 현재
모델에서 잘려 나가는 라벨이 늘어난다.

`Sensor_6.json`의 일부 `focalLengthpixel` 값과 해상도·FOV의 단순 pinhole
계산은 일치하지 않을 수 있다. 생성기의 camera visibility 판정에서 해당
필드 하나만 신뢰하지 않는다. MORAI가 저장한 BBox와 실제 팀 calibration을
최종 기준으로 사용한다.

## 4. 180회의 구성

### 4.1 30개 base scenario와 180개 capture run

공식 global path에서 **30개 anchor**를 고르고, 각 anchor에 대표 actor
layout 하나를 생성해 **30개 base scenario**를 만든다. 같은 30개를
대회 환경 6종에서 반복한다.

```text
30개 base scenario × Sunny/Foggy × 11/13/15시 = 180 run
```

사용자 확인 결과 MORAI에서 날씨와 시간을 설정한 뒤 Internal Scenario를
로드해도 해당 환경 상태가 유지된다. 따라서 30개 JSON을 환경별로 복제할
필요가 없다. MORAI UI에서 환경을 한 번 바꾸고 같은 30개를 순차 실행하며,
manifest에 `(base_scenario_id, weather, time)` 180행을 기록한다.

이 설계는 같은 geometry와 actor를 두고 조명·안개 영향만 직접 비교할 수
있어 PointPainting의 segmentation과 미래 BEVFusion의 카메라 강건성
검증에 좋다. 반면 독립 geometry가 30개뿐이라는 한계가 있다. 따라서
**첫 파인튜닝 개선 확인 및 환경 paired test로는 충분하지만, 최종 BEV
학습 데이터셋으로는 충분하지 않다.** 첫 결과가 확인되면 layout seed와
anchor를 늘린다.

### 4.2 날씨·시간 균형

환경은 다음 6개이며 각각 정확히 30 run으로 맞춘다.

| 날씨 | 시간 |
|---|---|
| Sunny | 11시, 13시, 15시 |
| Foggy | 11시, 13시, 15시 |

Foggy는 규정의 `Density=1`, `Distance=0`을 사용한다. 날씨와 시간은
Internal Scenario JSON에 넣지 않고 MORAI UI에서 환경 batch 시작 전에
한 번 설정한다. 첫 scenario를 로드한 뒤에도 설정이 유지되는지 화면과
샘플 캡처로 확인하고, manifest에는 실제 적용 조건을 반드시 기록한다.

각 환경에서 동일한 30개를 한 batch로 실행하되 runner report와
`--continue-on-scene-error`로 실패 scenario만 retry queue에 보낸다.
운영상 중간 checkpoint가 필요하면 `15 + 15`로 나눈다. 환경 변경은 여섯
환경 batch 경계에서만 수행한다.

검증된 p01~p03 `rolling-fast` 결과를 단순 비례하면 180 run의 순수 자동
수집 시간은 약 35분이다. MORAI 환경 변경, network 점검과 실패 장면
재시도를 포함한 파일럿 운영 시간은 약 45~60분으로 잡는다. 각 batch는
`--continue-on-scene-error`를 사용하고 실패 장면은 환경별 retry queue로
보낸다.

### 4.3 train/val/test

frame이나 환경 variant가 아니라 **base scenario/anchor 단위**로 먼저
split한다.

| split | base scenario | 환경 variant/base | 최종 run | frame |
|---|---:|---:|---:|---:|
| train | 20 | 6 | 120 | 720 |
| val | 5 | 6 | 30 | 180 |
| test | 5 | 6 | 30 | 180 |

같은 base scenario의 6개 환경 variant와 각 run의 6프레임을 서로 다른
split에 넣지 않는다. 예를 들어 `base_007`이 train이면 여섯 날씨·시간
버전이 모두 train이다. 이 규칙을 어기면 거의 같은 geometry와 actor가
train과 test에 동시에 들어가 평가가 크게 부풀려진다.

## 5. Anchor 30개의 층화

waypoint index를 일정 간격으로 고르면 점 샘플링 밀도 차이 때문에 실제
거리 간격이 불균일해진다. global path의 누적 arc length를 기준으로
후보를 만들고 MGeo 의미를 이용해 다음 primary topology를 할당한다.

| topology | anchor 수 | 주요 목적 |
|---|---:|---|
| 일반 직선 차로 | 7 | 거리·좌우 위치 기본 분포 |
| 곡선 | 5 | 회전된 box와 camera/LiDAR projection |
| 교차로 접근·이탈 | 5 | 다방향 차량과 가림 |
| 횡단보도 | 5 | 보행자 중심 |
| 합류·분기·회전교차로 | 4 | 차량 방향 다양성 |
| 도로변·주차·시설물 밀집 | 4 | hard negative와 occlusion |

anchor 간 최소 경로 거리는 기본 20 m로 두되, topology가 희귀하면 예외를
metadata에 기록한다. train/val/test anchor는 가능한 한 서로 다른 road와
link group에 배정한다.

## 6. 장면 유형 30개 anchor 기준 비율

전체 30개 base scenario에서 다음 비율을 목표로 한다. 30개로 백분율을
정확히 맞추기 어려우면 ±1개를 허용하며, 모든 split에 pedestrian-positive,
vehicle, hard-negative와 sparse/empty가 들어가게 층화한다.

| 장면 유형 | 목표 비율 | 설명 |
|---|---:|---|
| 보행자 중심 | 25% | 거리·자세·횡단보도·군집 변화 |
| 차량 중심 | 20% | 정차·주행 방향·측후방 차량 |
| 차량+보행자 혼합 | 30% | 실제 BEV 장면과 가림 |
| 장애물·시설물 hard negative | 10% | 보행자 오검출 억제 |
| camera seam·부분 가림 hard case | 10% | multi-camera fusion |
| sparse/empty background | 5% | false-positive 기준 |

현재 PointPainting detector의 label은 `Pedestrian` 하나지만 원본 수집에서
차량과 장애물을 제거하지 않는다. RGB semantic/instance, Camera BBox,
LiDAR semantic/instance 및 전체 3D box를 모두 보존해 이후 다중 클래스
BEV 학습에 사용한다.

## 7. 객체 수와 거리 랜덤성

### 7.1 객체 수

장면별 최대치를 무조건 채우지 않고 categorical distribution으로 뽑는다.

| 종류 | 기본 범위 | 분포 의도 |
|---|---:|---|
| NPC 차량 | 0~8대 | 0대 15%, 1~2대 25%, 3~5대 40%, 6~8대 20% |
| 보행자 | 0~10명 | 0명 20%, 1~2명 25%, 3~6명 40%, 7~10명 15% |
| 장애물 | 0~4개 | 0개 40%, 1~2개 45%, 3~4개 15% |

전체 actor는 기본 14개 이하로 제한한다. 보행자 없는 장면 중 절반 이상은
차량·상자·배럴·가로시설물이 있는 hard negative로 만들고, 완전히 빈
장면은 전체의 약 5%만 둔다.

### 7.2 거리

객체 중심의 Ego 기준 거리 목표치는 다음과 같다.

| 거리 | 목표 비율 | 의미 |
|---|---:|---|
| 3~8 m | 25% | 큰 box, 근거리 truncation |
| 8~15 m | 35% | 주 학습 구간 |
| 15~22 m | 30% | VLP16 보행자 난이도 |
| 22~28 m | 10% | LiDAR sparse·camera 중심 hard case |

3 m 이내는 일반 positive에서 제외하고 별도 near-field 진단 장면으로만
둔다. 28 m를 넘는 객체는 현재 PointPainting 범위 밖 label이 되기 쉬우므로
배경으로만 허용한다.

### 7.3 방위와 카메라 overlap

전방에만 객체를 몰지 않고 360° sector를 균형 있게 사용한다.

- front/back camera 중심 영역
- left/right wide camera 중심 영역
- front↔side 경계인 약 ±45°
- side↔back 경계인 약 ±135°

전체 positive의 약 20%는 두 카메라에 동시에 보이는 overlap/seam에,
약 10%는 image boundary의 partial truncation에 둔다. 다만 core train
positive는 적어도 한 카메라에서 box의 대부분이 보여야 한다.

## 8. Map-aware 제약

### 8.1 Ego 안전 회랑

Ego가 3.6 m 움직이고 IONIQ5 길이가 4.635 m이므로 초기 위치 전방
약 8 m까지 swept footprint가 생긴다. 초기 Ego pose에서 global path를
따라 만든 다음 영역에는 정적 actor footprint가 들어오지 않게 한다.

- 종방향: Ego 뒤 1.5 m부터 경로 전방 8.5 m
- 횡방향: Ego 폭 1.892 m에 좌우 margin 0.75 m 추가
- 곡선: 직사각형 한 개가 아니라 path를 따라 이동한 footprint union 사용

파일럿의 정적 보행자도 이 회랑 안에는 두지 않는다. 향후 동적 횡단 장면은
별도 scenario family로 분리한다.

### 8.2 NPC 차량

- `link_set.json`의 접선 방향에 yaw를 맞춘다.
- drivable lane polygon 또는 검증된 parking space 안에 footprint 전체가
  들어가야 한다.
- 일반 주행 차량의 역방향 yaw는 금지한다.
- 주차·비정상 차량은 별도 subtype과 확률로만 허용한다.
- Ego와 NPC, NPC 상호 간 oriented footprint가 margin을 포함해 겹치지
  않아야 한다.

### 8.3 보행자

- 횡단보도 장면은 `singlecrosswalk_set.json` polygon을 우선 사용한다.
- 대기 보행자는 lane boundary의 보도 측 후보를 사용한다.
- 일반 장면에서 차로 중앙에 정지한 보행자를 무작위로 두지 않는다.
- 서로 다른 사람 model, yaw, 군집 간격과 일부 가림을 sampling한다.

### 8.4 장애물

- 도로변, 주차 공간, 장애물 회피용 차로 등 의미가 있는 후보에 둔다.
- 표지판·배럴·상자처럼 보행자 semantic과 혼동될 수 있는 자산을
  hard-negative에 우선 배정한다.
- `object_set.json`의 기존 고정 시설물과 새 obstacle의 footprint가
  겹치지 않게 한다.

## 9. 충돌·가시성·학습 가능성 검사

생성기는 다음 순서로 rejection sampling한다.

1. anchor와 path tangent 계산
2. semantic zone에서 후보 위치 sampling
3. 실제 asset footprint와 yaw 적용
4. Ego swept corridor 교차 검사
5. actor 상호 oriented polygon + class margin 교차 검사
6. road/crosswalk/parking boundary 포함 검사
7. Ego 거리와 PointPainting range 검사
8. Camera FOV coarse check
9. 시나리오 JSON 저장 및 MORAI load 확인
10. 캡처 후 실제 Camera BBox와 LiDAR point 수로 quality tier 결정

권장 최소 margin 초안은 차량 0.8 m, 보행자 0.5 m, 장애물 0.5 m다.
asset catalog에서 실제 크기를 얻은 뒤 원형 거리 대신 oriented footprint
기준으로 최종 조정한다.

VLP16 원거리 보행자는 정답 box가 있어도 점이 거의 없을 수 있다. 이를
무조건 삭제하지 않고 다음처럼 분리한다.

- `core`: 보행자 내부 LiDAR point 3개 이상이고 한 카메라에서 명확히 보임
- `sparse`: 1~2 point 또는 camera-only에 가까운 hard case
- `invalid`: 모든 카메라 밖이고 LiDAR point도 0인 학습 불가능 label

현재 OpenPCDet GT sampling 설정도 `Pedestrian:3`을 사용하므로 core와
sparse 비율을 metadata에 기록해야 한다.

## 10. BEV 재사용을 위한 보존 항목

각 frame에서 다음 원본을 버리지 않는다.

- Camera 1~4 Intensity, Semantic, Instance, Depth
- Camera별 BBox2D, BBox3D가 생성된 경우 해당 파일
- LiDAR Intensity, Semantic, Instance
- LiDAR `Instance.txt`, `Instance_8Points.txt`
- sensor calibration과 `Sensor_6.json` hash
- scenario stem, layout id, seed, anchor/link, environment id
- actor의 model/DataID, class ontology, instance id, pose, size, yaw
- capture timestamp와 camera/LiDAR timestamp group
- train/val/test split 및 quality tier

미래의 BEVFusion은 차량·보행자·장애물 ontology가 필요하고, camera-only
BEV 모델은 카메라 overlap과 depth/3D box가 중요하다. 현재
`Pedestrian`만 학습한다고 다른 class GT를 폐기하면 다시 캡처해야 한다.

## 11. 객체 카탈로그 상태와 남은 검증

사용자가 MORAI에서 배치 가능한 자산을 한 번씩 넣어 만든 세 시나리오에서
사람 결합형 obstacle을 제외한 109개 고유 DataID와 모델 표시명을 추출했다.

| 영역 | 고유 자산 수 |
|---|---:|
| Vehicle | 43 |
| Pedestrian 컨테이너 | 33 |
| Curated obstacle 컨테이너 | 33 |

기계 판독 카탈로그는
`configs/morai_asset_catalog_260807.json`, 분류 근거와 사용 정책은
`docs/morai_asset_catalog_260807.md`를 따른다.

Vehicle은 자동 생성에 사용할 수 있고 일반 사람 모델 25종은 Pedestrian
positive의 기본 pool로 사용한다. NCAP dummy는 낮은 비율로 제한한다.
Animal은 hard negative로 분리한다. 사람이 없는 자전거·스쿠터 4종은
hard-negative 전용 가중치 없이 일반 Obstacle로 현실적인 위치에 배치한다.
`pedestrianList`의 자전거 탑승자와 사람+동물 복합 모델은
`quarantine`한다.

스쿠터·킥보드 탑승자와 유모차를 미는 사람 등 사람 결합형 obstacle 8종은
사람과 이동체의 라벨 의미가 섞일 수 있으므로 자동 생성 pool에서 제외한다.
독립적인 빈 자전거·스쿠터는 일반 Obstacle 다양성만 담당한다.

남은 핵심 검증은 MORAI 저장 컨테이너가 아니라 실제 캡처 GT를 기준으로
다음 정보를 확정하는 것이다.

- Semantic/Instance 및 Camera/LiDAR 3D box에 기록되는 class와 instance
- `pedestrianList`의 자전거 탑승자·사람+동물 복합 자산의 GT 구조
- 자산별 실제 footprint와 scale 허용 범위
- 정적·동적 지원 여부와 waypoint 필드
- 대회용/개인용 런처 load 호환성

따라서 180-run 생성기는 `enabled` 자산만 자동 sampling하고,
`quarantine` 자산은 학습 split 밖의 소규모 label-audit 장면에서 먼저
검증한다.

## 12. 180-run 이후 확장

첫 PointPainting 개선 여부가 확인되면 다음 순서로 확장한다.

1. 180-run 파일럿: 정적 actor, 1,080 frame
2. 900-run: 30 anchor × 5 layout × 6 환경
3. 약 3,000-run: 약 100 anchor × 5 layout × 6 환경
4. 별도 dynamic family: 움직이는 NPC와 횡단 보행자
5. 일정 주기의 temporal sequence family

정적 파일럿과 동적 데이터를 처음부터 한 manifest에 섞지 않는다. 정적
scene generator와 캡처 자동화의 안정성을 먼저 확보한 뒤 dynamic actor
행동과 충돌 회피를 별도 검증한다.
