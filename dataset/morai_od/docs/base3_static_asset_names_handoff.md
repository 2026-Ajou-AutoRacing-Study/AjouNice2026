# Base3-Static 10K 실제 사용 Asset Name

팀원 시나리오 작성용 최소 목록이다. Base3-Static 10K 최종 positive pool에
실제로 사용한 자산만 포함하며, 제외·격리 자산은 싣지 않는다.
`Asset name`은 대소문자와 기호까지 그대로 사용하고, MORAI scenario에는
같은 행의 `DataID`를 함께 사용한다. `UNIQUEID`는 장면 내 actor마다 새로 부여한다.

| 학습 class | MORAI container | Asset 수 |
| --- | --- | ---: |
| `Vehicle` | `vehicleList` | 43 |
| `Pedestrian` | `pedestrianList` | 22 |
| `Obstacle` | `objectList` | 10 |
| **합계** |  | **75** |

## Vehicle (43)

| DataID | Asset name |
| ---: | --- |
| 30300001 | `2016_Hyundai_Universe` |
| 30300002 | `2017_hyundai_solati_h350` |
| 30300003 | `DEFAULT_BUS` |
| 30700005 | `2016_Kia_Soul(EV)` |
| 30700003 | `2019_Hyundai_Kona(EV)` |
| 30700004 | `2020_Kia_Niro(EV)` |
| 30700006 | `2020_Kia_Soul(EV)` |
| 30800001 | `2019_kia_sedona` |
| 30800003 | `2020_chrysler_pacifica_hybrid` |
| 30800002 | `2021_Kia_Carnival` |
| 30800004 | `DEFAULT_MPV` |
| 30100005 | `2014_Kia_K7` |
| 30100004 | `2015_Kia_K5` |
| 30100013 | `2016_Hyundai_Genesis_DH` |
| 30100007 | `2016_Hyundai_Ioniq` |
| 30100003 | `2016_Hyundai_Sonata` |
| 30100001 | `2017_Hyundai_Grandeur` |
| 30100011 | `2019_Toyota_Prius` |
| 30100012 | `2019_hyundai_genesis_g90` |
| 30100014 | `2020_Kia_K5` |
| 30100009 | `2020_Kia_Stinger` |
| 30100008 | `2020_hyundai_sonata` |
| 30100010 | `2021_Hyundai_G80` |
| 30100015 | `DEFAULT_SEDAN` |
| 30200001 | `2016_Hyundai_Santafe` |
| 30200003 | `2017_Kia_Niro(HEV)` |
| 30200002 | `2017_Kia_Sorento` |
| 30200005 | `2019_Hyundai_Nexo` |
| 30200006 | `2020_volvo_xc90` |
| 30200012 | `2021_Hyundai_GV70` |
| 30200007 | `2021_Hyundai_Genesis_GV80` |
| 30200010 | `2021_Volkswagen_Golf_GTI` |
| 30200014 | `2023_Hyundai_Ioniq5` |
| 30200013 | `2023_Kia_EV6` |
| 30200011 | `DEFAULT_SUV` |
| 30400002 | `2012_Hyundai_HD65` |
| 30400001 | `2013_Hyundai_Porter2` |
| 30400003 | `2013_Hyundai_Xcient(P520)` |
| 30400004 | `2013_Hyundai_Xcient(P540)` |
| 30400005 | `2014_Hyundai_Xcient(Mixer)` |
| 30400007 | `DEFAULT_TRUCK` |
| 30500001 | `2015_Toyota_Prius(HEV)` |
| 30500002 | `DEFAULT_WAGON` |

## Pedestrian (22)

| DataID | Asset name |
| ---: | --- |
| 50200001 | `Boy1` |
| 50200002 | `Girl1` |
| 50200004 | `Girl2` |
| 50100001 | `Man1` |
| 50100002 | `Man2` |
| 50100003 | `Man3` |
| 50100007 | `Man4` |
| 50100008 | `Man5` |
| 50100009 | `Man6` |
| 50100010 | `Man7` |
| 50100011 | `Man8` |
| 50100004 | `Woman1` |
| 50100005 | `Woman2` |
| 50100006 | `Woman3` |
| 50100012 | `Woman4` |
| 50100013 | `Woman5` |
| 50100014 | `Woman6` |
| 50100015 | `Woman7` |
| 50100016 | `Woman8` |
| 50100058 | `police_m_2` |
| 50100074 | `worker_f_1` |
| 50100077 | `worker_m_1` |

## Obstacle (10)

| DataID | Asset name |
| ---: | --- |
| 40100001 | `CargoBox` |
| 40100027 | `PE_Drum` |
| 40100005 | `TrafficBarrel` |
| 40100004 | `YellowBarrel` |
| 40200162 | `concrete-barrier` |
| 40100047 | `Concrete_Barricade01` |
| 40100052 | `Concrete_Barricade02` |
| 40100033 | `Steel_Barricade` |
| 40100028 | `PE_Firewall_Orange` |
| 40100029 | `PE_Firewall_White` |

## 근거

- 상세 정책과 subtype: `dataset/morai_od/docs/base3_static_asset_catalog.md`
- 최종 capture 감사: `dataset/morai_od/docs/base3_static_10k_final_capture_audit.md`
- 생성기: `dataset/morai_od/scripts/generate_static_classcentric_base3_batch.py`
- Batch 008 walking/running Pedestrian도 위와 동일한 22종을 사용했다.
