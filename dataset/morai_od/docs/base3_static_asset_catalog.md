# Base3-Static 10K 객체 카탈로그

마지막 갱신: 2026-09-15

대상 dataset: `morai_static_classcentric_base3_10k` (`Base3-Static 10K`)

## 목적

이 문서는 Base3-Static 최종 수집에 실제 사용한 MORAI 객체를 팀원이 동일한
ontology와 `DataID`로 시나리오에 재사용할 수 있도록 고정한 목록이다.

최종 positive asset pool은 다음과 같다.

| 학습 class | MORAI scenario container | Asset 수 |
| --- | --- | ---: |
| `Vehicle` | `vehicleList` | 43 |
| `Pedestrian` | `pedestrianList` | 22 |
| `Obstacle` | `objectList` | 10 |
| 합계 |  | 75 |

`DataID`는 모델 종류를 선택하고 `UNIQUEID`는 한 scenario 안의 actor instance를
구분한다. 같은 `DataID`를 여러 번 써도 되지만 `UNIQUEID`는 actor마다 달라야 한다.
MORAI container 이름을 학습 class로 임의 해석하지 않는다. 예를 들어
`pedestrianList`에는 격리 대상 동물·탑승자 asset도 존재하지만 아래 22종만 Base3
Pedestrian positive pool이다.

## Vehicle: 43종

Base3에서는 다음 43종을 균등 deck으로 순환했다. 실제 MGeo 차로 중심에 배치하고,
대부분 도로 접선 방향에 정렬하며 기본 yaw jitter는 `±8 deg`였다.

| DataID | Asset name | Subtype |
| ---: | --- | --- |
| 30300001 | `2016_Hyundai_Universe` | bus/van |
| 30300002 | `2017_hyundai_solati_h350` | bus/van |
| 30300003 | `DEFAULT_BUS` | bus/van |
| 30700005 | `2016_Kia_Soul(EV)` | electric passenger car |
| 30700003 | `2019_Hyundai_Kona(EV)` | electric passenger car |
| 30700004 | `2020_Kia_Niro(EV)` | electric passenger car |
| 30700006 | `2020_Kia_Soul(EV)` | electric passenger car |
| 30800001 | `2019_kia_sedona` | MPV |
| 30800003 | `2020_chrysler_pacifica_hybrid` | MPV |
| 30800002 | `2021_Kia_Carnival` | MPV |
| 30800004 | `DEFAULT_MPV` | MPV |
| 30100005 | `2014_Kia_K7` | passenger car |
| 30100004 | `2015_Kia_K5` | passenger car |
| 30100013 | `2016_Hyundai_Genesis_DH` | passenger car |
| 30100007 | `2016_Hyundai_Ioniq` | passenger car |
| 30100003 | `2016_Hyundai_Sonata` | passenger car |
| 30100001 | `2017_Hyundai_Grandeur` | passenger car |
| 30100011 | `2019_Toyota_Prius` | passenger car |
| 30100012 | `2019_hyundai_genesis_g90` | passenger car |
| 30100014 | `2020_Kia_K5` | passenger car |
| 30100009 | `2020_Kia_Stinger` | passenger car |
| 30100008 | `2020_hyundai_sonata` | passenger car |
| 30100010 | `2021_Hyundai_G80` | passenger car |
| 30100015 | `DEFAULT_SEDAN` | passenger car |
| 30200001 | `2016_Hyundai_Santafe` | SUV |
| 30200003 | `2017_Kia_Niro(HEV)` | SUV |
| 30200002 | `2017_Kia_Sorento` | SUV |
| 30200005 | `2019_Hyundai_Nexo` | SUV |
| 30200006 | `2020_volvo_xc90` | SUV |
| 30200012 | `2021_Hyundai_GV70` | SUV |
| 30200007 | `2021_Hyundai_Genesis_GV80` | SUV |
| 30200010 | `2021_Volkswagen_Golf_GTI` | SUV |
| 30200014 | `2023_Hyundai_Ioniq5` | SUV |
| 30200013 | `2023_Kia_EV6` | SUV |
| 30200011 | `DEFAULT_SUV` | SUV |
| 30400002 | `2012_Hyundai_HD65` | truck |
| 30400001 | `2013_Hyundai_Porter2` | truck |
| 30400003 | `2013_Hyundai_Xcient(P520)` | truck |
| 30400004 | `2013_Hyundai_Xcient(P540)` | truck |
| 30400005 | `2014_Hyundai_Xcient(Mixer)` | truck |
| 30400007 | `DEFAULT_TRUCK` | truck |
| 30500001 | `2015_Toyota_Prius(HEV)` | wagon |
| 30500002 | `DEFAULT_WAGON` | wagon |

### Vehicle 시나리오 작성 주의사항

- Base3 core에서는 상대 Vehicle을 정지 상태로 두고 Ego만 이동시켰다.
- 같은 차로 전방 Vehicle은 정확히 1대, 후방은 0--1대를 기본으로 했다.
- 옆·대각선 Vehicle은 실제 인접 MGeo lane에만 배치했다. 단순 XY lateral offset은
  포장면이나 동일 진행방향을 보장하지 않는다.
- 버스와 트럭은 승용차보다 큰 footprint와 clearance를 사용해야 한다.
- 반대 방향 주정차 appearance는 전체의 약 10%만 사용했다.
- 동적 합류·순항 Vehicle을 만들 때도 `DataID`는 재사용할 수 있지만, Base3의 정적
  학습 분포와 동일한 데이터라고 간주하지 않는다.

## Pedestrian: 22종

NCAP dummy, 동물, 자전거 탑승자 및 사람+동물 복합 asset을 제외하고 실제 person
asset만 사용했다. Batch 008의 walking/running 보강도 동일한 22종을 사용했다.

| DataID | Asset name | Subtype |
| ---: | --- | --- |
| 50200001 | `Boy1` | person |
| 50200002 | `Girl1` | person |
| 50200004 | `Girl2` | person |
| 50100001 | `Man1` | person |
| 50100002 | `Man2` | person |
| 50100003 | `Man3` | person |
| 50100007 | `Man4` | person |
| 50100008 | `Man5` | person |
| 50100009 | `Man6` | person |
| 50100010 | `Man7` | person |
| 50100011 | `Man8` | person |
| 50100004 | `Woman1` | person |
| 50100005 | `Woman2` | person |
| 50100006 | `Woman3` | person |
| 50100012 | `Woman4` | person |
| 50100013 | `Woman5` | person |
| 50100014 | `Woman6` | person |
| 50100015 | `Woman7` | person |
| 50100016 | `Woman8` | person |
| 50100058 | `police_m_2` | person |
| 50100074 | `worker_f_1` | person |
| 50100077 | `worker_m_1` | person |

### Pedestrian 시나리오 작성 주의사항

- 정지 Pedestrian은 route-relative 정방향·역방향·좌횡단·우횡단 yaw를 섞는다.
- 횡단보도에서는 좌→우와 우→좌를 균형 있게 구성한다.
- walking/running actor는 검증된 PedPath와 behavior 구조를 복사하고 `DataID`와
  animation 관련 필드만 바꾼다. catalog actor 전체를 임의 복사하면 MORAI에서
  actor는 보이지만 motion이 시작되지 않을 수 있다.
- 최종 동적 Pedestrian 보강 비율은 walking/running/activation transition
  `135/135/30 clips`, 좌→우/우→좌 `150/150 clips`였다.

## Obstacle: 10종

최종 production pool은 LiDAR point support와 MORAI annotation box 정합성이 반복
검증된 아래 10종이다. `CargoBox`의 상대 sampling weight는 60, 나머지는 각각 40이었다.

| DataID | Asset name | Subtype | 상대 weight |
| ---: | --- | --- | ---: |
| 40100001 | `CargoBox` | box | 60 |
| 40100027 | `PE_Drum` | barrel/drum | 40 |
| 40100005 | `TrafficBarrel` | barrel/drum | 40 |
| 40100004 | `YellowBarrel` | barrel/drum | 40 |
| 40200162 | `concrete-barrier` | barrier | 40 |
| 40100047 | `Concrete_Barricade01` | barrier | 40 |
| 40100052 | `Concrete_Barricade02` | barrier | 40 |
| 40100033 | `Steel_Barricade` | barrier | 40 |
| 40100028 | `PE_Firewall_Orange` | barrier | 40 |
| 40100029 | `PE_Firewall_White` | barrier | 40 |

### Obstacle 시나리오 작성 주의사항

- 같은 차로 전방 obstacle은 정확히 1개를 기본으로 한다. 나머지는 회피 차로 전방,
  좌우 대각선, 근거리 측·후측방 및 도로변 context에 분산한다.
- box와 barrel은 전체 yaw 범위를 사용할 수 있다.
- 긴 barrier는 route 기준 수직 40%, 대각선 35%, 평행 25%를 사용했다.
- 첫 capture 기준 권장 거리 상한은 barrel/drum 16 m, box 20 m, 대형 barrier
  30 m다.
- raw MORAI GT box는 수정하지 않는다. 학습용 point count에만 3 cm tolerance를
  적용하고, 0 point GT와 1--4 point GT는 normal-positive loss에서 제외했다.

## Base3 positive pool에서 제외한 대표 asset

아래 asset은 MORAI에 존재하지만 최종 Base3 positive pool에는 넣지 않는다.

| Asset | 제외 이유 |
| --- | --- |
| `WoodBox` | pose에 따라 mesh point가 MORAI GT box보다 약 5--10 cm 밖에 있어 annotation 계약이 불안정함 |
| `RedBarrel` | 반복 pilot에서 zero/sparse LiDAR point가 다른 barrel보다 자주 발생함 |
| `Road_Sign_Mark` | 감사한 46/46 관측이 0 point였음 |
| `hole-cover` | 97.6%가 0 point였고 5-point 이상 관측이 없었음 |
| `Speed_Bump` | 매우 얇은 고정 도로 인프라이며 대회 obstacle mission 분포와 맞지 않음 |
| `fr-streetlight-2` | 고정 도로시설이며 positive로 쓰면 배경 오검출을 강화할 수 있음 |
| `KR_Sign_SL_30`--`110` | 고정 속도표지판으로, Obstacle positive보다 hard-negative 배경으로 다루는 것이 적절함 |
| `Bollard01`, `Bollard04`, `Lane_Separation_Bar` | 대회 중요도 대비 1--4-point 관측 비율이 과도했음 |
| `Bicycle_city`, `Bicycle_mountain`, `Scooter_02` | 작은 two-wheeler 형상에서 sparse point 비율이 높아 최종 clean baseline에서 제외함 |

`WoodBox`는 현실성이 없어서 영구 제외한 것이 아니다. 별도 label-correction audit로
mesh와 GT box 정합성을 해결하면 다시 도입할 수 있다. 위 제외 asset을 성능 stress
scenario에 쓰는 것은 가능하지만 Base3 in-domain headline 평가와 섞지 않는다.

## 시나리오 생성 시 권장 절차

1. 가능하면 actor JSON을 손으로 새로 만들지 말고 검증된 MORAI launcher scenario의
   actor template 전체를 복사한다.
2. 위 표의 `DataID`로 asset을 선택하고 scenario 내부 `UNIQUEID`를 새로 부여한다.
3. Vehicle은 `vehicleList`, Pedestrian은 `pedestrianList`, Obstacle은 `objectList`에
   넣는다.
4. actor pose를 실제 MGeo lane/crosswalk와 Ego swept corridor 기준으로 검증한다.
5. scenario load 뒤 `/Object_topic`의 기대 actor 수와 이름을 확인한다.
6. Camera BBox/Instance와 LiDAR Instance/8Points GT가 생성되는지 확인한다.
7. 30 m 이내 positive는 실제 LiDAR 5 point 이상인지 감사한다.

Base3 자동 생성 구현은
`dataset/morai_od/scripts/generate_static_classcentric_base3_batch.py`가 기준이다.
전체 MORAI provenance와 ontology 판단은
`dataset/morai_od/docs/morai_asset_catalog_260807.md`, 기계 판독용 원본은
`dataset/morai_od/configs/morai_asset_catalog_personal_260807.json` 및
`dataset/morai_od/configs/morai_asset_launcher_compatibility_260807.json`에 있다.
