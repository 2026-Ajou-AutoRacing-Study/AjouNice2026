# K-City Lanelet2 v45 교차로 semantic 업데이트 보고서

## 결과

- 원본: `kcity_map/lanelet_map/lanelet2_map_v44.osm` (변경하지 않음)
- 결과: `lanelet2_map_v45.osm` (`kcity_map/lanelet_map/lanelet2_map_v45.osm`에도 동일 inode로 제공)
- 범위: v44 대회 경로가 통과하는 **7개 교차로**. 각 교차로에서 ego 경로 한 개만이 아니라 GIS `A2_LINK.LinkType=1`로 연결된 **교차로 내부의 모든 차량 movement lanelet**을 대상으로 했다.
- 변경: `turn_direction` 57개, `intersection_area` polygon 7개. `right_of_way`는 0개 추가.
- 기존 node/way/relation ID, 기존 lanelet 경계·centerline·relation member topology, stop line 및 traffic light geometry는 변경하지 않았다.

## 근거와 판정 방법

1. 대회 경로는 `kcity_map/competition_route/v44/drive_course_v44_lanelets.csv`의 59개 행(고유 lanelet 58개)을 사용했다. v43 또는 이미지에서 경로를 추정하지 않았다.
2. 국가 GIS 원천의 `gis_map/A2_LINK.shp` 중심선을 v44 lanelet centerline에 대조했다. 대상 차선은 평균/최대 거리 0 m로 대응했다. `LinkType=1` inner link를, 해당 route 접근 차선의 종료 node에서 연결된 component 전체로 확장했다.
3. 진행 방향은 곡률이나 진입 직후 접선만으로 정하지 않았다. 각 inner link의 **직전 접근 A2_LINK와 교차로 이탈 A2_LINK**의 방위를 EPSG:32652에서 비교했다. 각도 `+30° 이상=left`, `-30° 이하=right`, 그 사이=straight`이며, lanelet 연결 topology와 함께 확인했다.
4. `intersection_area`는 접근 차선 버퍼나 convex hull가 아니라 각 component에 속한 inner lanelet 경계 polygon의 유효한 합집합 exterior를 사용했다. 따라서 충돌 가능한 내부 주행면을 모두 포함하면서 접근 차선 밖으로 확장하지 않는다.
5. 물리 제어 근거는 GIS `B2_SURFACELINEMARK.shp`(정지선), `C1_TRAFFICLIGHT.shp`(신호등), `B1_SAFETYSIGN.shp` 및 기존 v44 traffic-light regulatory element를 교차 검토했다. GIS에 신호 phase/우선권 표가 없으므로 임의의 `right_of_way`를 만들지 않았다.

Autoware의 Intersection 모듈은 `turn_direction`이 있는 lanelet을 대상으로 하고, Blind Spot 모듈은 `left`/`right` movement를 대상으로 한다. `intersection_area`는 BVP 로더가 읽는 직접 polygon way (`type=intersection_area`)와 lanelet의 `intersection_area=<way ID>` 참조로 작성했다. [Autoware Intersection 문서](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_intersection_module/)와 저장소의 BVP 검증 문서를 기준으로 했다.

## 대회 주행 경로의 원래 문제와 해결 확인

**원래 문제는 route topology가 아니었다.** v44는 대회 route의 7개 실제 교차로 내부 lanelet 모두에서 `turn_direction=else`, `intersection_area=None`이었다. 따라서 BVP map loader의 Intersection 후보와 Blind Spot 후보가 map semantic만으로는 0개였고, 교차로 conflict 영역도 제공되지 않았다. 반면 Lanelet2 routing graph 자체는 v44에서도 유효했다.

v45에서는 아래 7개만 route의 실제 교차로 내부 movement이며, 각각 진입 lanelet과 이탈 lanelet 사이에 정확히 위치한다. 접근 lanelet에 의미 태그를 붙이는 오류가 없도록 별도 확인했다.

| 내부 lanelet | route 접근 → 내부 → 이탈 | v44 | v45 | 효과 |
|---:|---|---|---|---|
| 200392 | 200889 → 200392 → 200178 | semantic 없음 | `left`, area 104012 | Intersection + Blind Spot |
| 200337 | 200178 → 200337 → 200287 | semantic 없음 | `straight`, area 104013 | Intersection |
| 200348 | 200287 → 200348 → 200346 | semantic 없음 | `straight`, area 104014 | Intersection |
| 200291 | 200346 → 200291 → 200292 | semantic 없음 | `straight`, area 104015 | Intersection |
| 200950 | 200841 → 200950 → 200310 | semantic 없음 | `straight`, area 104016 | Intersection |
| 200261 | 200258 → 200261 → 200315 | semantic 없음 | `right`, area 104017 | Intersection + Blind Spot |
| 200445 | 200315 → 200445 → 200311 | semantic 없음 | `left`, area 104018 | Intersection + Blind Spot |

반증 관점 재검증 결과는 다음과 같다.

- v44 route semantic 후보: 0개. v45 route semantic 후보: 정확히 위 7개.
- 위 7개 바로 앞 접근 lanelet `200889, 200178, 200287, 200346, 200841, 200258, 200315`에는 v45에서도 `turn_direction` 및 `intersection_area`가 없다.
- 57개 전체 movement의 GIS entry→exit 방향 분류가 v45 태그와 57/57 일치했다.
- 7개 area ID 모두 존재하며 해당 route 내부 lanelet을 포함한다.
- C++ Lanelet2 vehicle routing graph는 v44/v45 모두 validity error 0이고, route의 모든 direct successor가 유지됐다.

## v44 감사 결과

| 항목 | v44 결과 | v45 조치 |
|---|---:|---|
| road lanelet | 1,191 | geometry/topology 유지 |
| `turn_direction` | 0 | 57개 추가 |
| `intersection_area` | 0 | 7개 추가 |
| `right_of_way` regulatory element | 0 | 0개 유지(근거 부족) |
| stop line way | 190 | 유지 |
| traffic light way | 124 | 유지 |
| traffic-light regulatory element | 68 | 유지 |
| 존재하지 않는 member ref | 0 | 0 |
| XML | 유효 | 유효 |

## 교차로별 제어·영역 결과

| 구역 (GIS route stop node) | 신호 구분 | 접근 제어 lanelet | stop line / traffic-light regulatory element | traffic light way | intersection_area | 내부 movement 수 | 검증 |
|---|---|---:|---|---|---:|---:|---|
| J01 `A1256W000736` | 무신호 | 200889 | 103552 / 없음 | 없음 | 104012 | 12 | polygon 유효, 참조 12 |
| J02 `A1256W000749` | 무신호 | 200178 | 103551 / 없음 | 없음 | 104013 | 6 | polygon 유효, 참조 6 |
| J03 `A1256W000682` | 신호 | 200287 | 103957 / 201351 | 103891, 103912, 103928 | 104014 | 6 | polygon 유효, 참조 6 |
| J04 `A1256W000807` | 신호 | 200346 | 103971 / 201365 | 103905 | 104015 | 3 | polygon 유효, 참조 3 |
| J05 `A1256W000208` | 신호 | 200841 | 103978 / 201290 | 103830, 103832, 103833, 103913 | 104016 | 8 | polygon 유효, 참조 8 |
| J06 `A1256W000757` | 신호 | 200258 | 103955 / 201397 | 103937 | 104017 | 8 | polygon 유효, 참조 8 |
| J07 `A1256W000742` | 신호 | 200315 | 103960 / 201326 | 103866 | 104018 | 14 | polygon 유효, 참조 14 |

신호 제어는 기존 regulatory element의 `ref_line`과 `refers` member를 재검사했다. 모든 traffic-light regulatory element가 유효한 stop line 및 traffic-light way를 참조하며, route 접근 lanelet에 연결돼 있었다. 반대편 lanelet에 잘못 연결될 수 있는 항목은 발견하지 못했고, 기존 관계를 변경하지 않았다.

## 추가한 `turn_direction` 및 area 참조

신뢰도는 GIS A2_LINK ↔ v44 centerline 일치, immediate predecessor/successor topology, 방향 벡터가 모두 일치한 경우의 `높음`이다. 모든 행의 기존 값은 `없음`, `right_of_way`는 `미생성`, stop line/traffic light는 위 표의 해당 접근 제어에만 존재한다.

| 구역 | lanelet ID | 새 turn_direction | intersection_area ID | 판단 근거 | 신뢰도 | 검증 |
|---|---:|---|---:|---|---|---|
| J01 | 200393 | left | 104012 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J01 | 201070 | right | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200443 | right | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200434 | left | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200444 | right | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200392 | left | 104012 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J01 | 200413 | left | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200414 | right | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200171 | straight | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200160 | straight | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200177 | straight | 104012 | 동 | 높음 | BVP 검출 |
| J01 | 200220 | straight | 104012 | 동 | 높음 | BVP 검출 |
| J02 | 200394 | right | 104013 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J02 | 200391 | right | 104013 | 동 | 높음 | BVP 검출 |
| J02 | 200446 | left | 104013 | 동 | 높음 | BVP 검출 |
| J02 | 200417 | left | 104013 | 동 | 높음 | BVP 검출 |
| J02 | 200290 | straight | 104013 | 동 | 높음 | BVP 검출 |
| J02 | 200337 | straight | 104013 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J03 | 201073 | left | 104014 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J03 | 200385 | left | 104014 | 동 | 높음 | BVP 검출 |
| J03 | 201072 | right | 104014 | 동 | 높음 | BVP 검출 |
| J03 | 200345 | straight | 104014 | 동 | 높음 | BVP 검출 |
| J03 | 200348 | straight | 104014 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J03 | 200964 | right | 104014 | 동 | 높음 | BVP 검출 |
| J04 | 200426 | right | 104015 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J04 | 200291 | straight | 104015 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J04 | 200327 | right | 104015 | 동 | 높음 | BVP 검출 |
| J05 | 200383 | left | 104016 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J05 | 200433 | left | 104016 | 동 | 높음 | BVP 검출 |
| J05 | 200309 | straight | 104016 | 동 | 높음 | BVP 검출 |
| J05 | 200344 | left | 104016 | 동 | 높음 | BVP 검출 |
| J05 | 200950 | straight | 104016 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J05 | 200983 | straight | 104016 | 동 | 높음 | BVP 검출 |
| J05 | 200515 | straight | 104016 | 동 | 높음 | BVP 검출 |
| J05 | 200516 | left | 104016 | 동 | 높음 | BVP 검출 |
| J06 | 200442 | left | 104017 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J06 | 200439 | straight | 104017 | 동 | 높음 | BVP 검출 |
| J06 | 200441 | left | 104017 | 동 | 높음 | BVP 검출 |
| J06 | 200429 | straight | 104017 | 동 | 높음 | BVP 검출 |
| J06 | 200409 | right | 104017 | 동 | 높음 | BVP 검출 |
| J06 | 200318 | left | 104017 | 동 | 높음 | BVP 검출 |
| J06 | 200261 | right | 104017 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J06 | 200169 | straight | 104017 | 동 | 높음 | BVP 검출 |
| J07 | 200468 | right | 104018 | A2_LINK entry→exit heading | 높음 | BVP 검출 |
| J07 | 200386 | left | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 201149 | right | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200398 | right | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200432 | left | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200445 | left | 104018 | 동; 대회 경로 통과 | 높음 | BVP 검출 |
| J07 | 200268 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200270 | right | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200272 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200275 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200278 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200312 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 200313 | straight | 104018 | 동 | 높음 | BVP 검출 |
| J07 | 201174 | left | 104018 | 동 | 높음 | BVP 검출 |

Roundabout lanelet은 Autoware Intersection 범위가 아니므로 `turn_direction`/`intersection_area`를 추가하지 않았다.

## 검증 결과

| 검사 | 결과 |
|---|---|
| XML 문법 (`xmllint --noout`) | 통과 |
| 모든 way/relation member ref 존재 | 통과, 누락 0 |
| area 폐합·자기교차·면적 | 7/7 통과; 각 polygon이 할당된 inner lanelet을 포함 |
| v44 대비 기존 node/way geometry | 동일 |
| v44 대비 기존 relation member topology | 동일 |
| 대회 route 회귀 | 59행/고유 58 lanelet 모두 존재; 58개 following set 동일; 모든 CSV successor 연결 유지 |
| BVP Python map loader | 로드 성공: road 1,191, intersection area 7, direction lanelet 57 |
| BVP Intersection 대상 | 57개 (area 참조 57/57) |
| BVP Blind Spot 대상 | 좌·우회전 35개 검출 |
| Lanelet2 C++ routing graph validity | Autoware Humble Docker에서 `lanelet2_validate --map_file <map> --lat 37.24 --lon 126.77 --filter routing_graph.*` 실행. v44와 v45 모두 `0 issues found`. |
| Lanelet2 C++ map load / route 회귀 | 동일 Docker의 Lanelet2 Python binding으로 vehicle routing graph 생성. v44/v45 모두 graph validity error 0, map lanelet 1,280, 대회 route의 모든 direct successor 통과. |

## ID 변경 목록과 미확정 사항

- 수정 lanelet relation ID: 위 57개 표의 모든 ID.
- 새 node ID: `131078`–`131446` (369개, area exterior vertex 전용).
- 새 way ID: `104012`–`104018` (각각 J01–J07 `type=intersection_area`).
- 새 relation ID: 없음.
- `right_of_way`: 미생성. 신호 phase, priority road/yield 방향을 판정할 수 있는 GIS 속성 또는 교차로 제어도면이 부족하다. 필요한 추가 자료는 신호 현시도와 교차로별 우선권/양보 규칙이다.
- RViz 전용 launch/config는 코드 수정 금지 원칙 때문에 새로 추가하지 않았다. 기존 lanelet map visualizer로 v45를 열어 `turn_direction`, stop line, traffic light, area way를 함께 표시하는 최종 육안 점검을 권장한다.

결론적으로, v45는 대회 경로 교차로 7개에 대해 Autoware Intersection semantic을 활성화할 수 있고, 이 중 좌·우회전 35개는 Blind Spot 대상이다. `right_of_way` 기반 우선권 판단은 원천 자료가 확보될 때까지 의도적으로 비활성 상태다.

## Autoware Planning Simulator 통합 재검증 (2026-07-24)

Docker `ghcr.io/autowarefoundation/autoware:universe-cuda-humble`에서 실제 `planning_simulator.launch.xml`을 사용했다. v45는 map loader, Mission Planner route, BVP, trajectory publisher까지는 정상 동작했지만 **현재의 기본 Blind Spot runtime 설정으로는 autonomous 주행 가능 조건을 만족하지 못했다.** 따라서 아래 결과는 성공으로 과장하지 않는다.

| 비교 맵 / 조건 | map load | 대회 route·trajectory | trajectory rate | autonomous 가능 | 판정 |
|---|---|---|---:|---|---|
| v44 baseline | 통과 | 통과 | 10.02 Hz, OK | 별도 control-gate 문제로 완주은 미실행 | semantic 없는 기준선 |
| v45 전체 (57 movements) | 통과 | 통과; `blind_spot` 기동 로그 | 2.21 Hz, Warn | 불가 | 실패 |
| v45 route 7개만 semantic | 통과 | 통과 | 1.98 Hz, Warn | 불가 | 실패 |
| v45 route 직진 4개만 semantic | 통과 | 통과 | 9.95–10.00 Hz, OK | control stack 준비 가능 | 통과 |
| v45 전체 + Blind Spot attention 30 m (검증 전용) | 통과 | 통과 | 1.92 Hz, Warn | 불가 | 실패 |

### 실제 확인된 내용

- v45는 C++ Lanelet2 map loader에서 정상 publish됐다.
- Mission Planner의 live route에는 route 순서대로 `200392, 200337, 200348, 200291, 200950, 200261, 200445`가 포함됐고 `/planning/trajectory`도 publish됐다.
- 좌·우회전 semantic이 포함되면 BVP `blind_spot` scene module이 실제 기동했다.
- 직진 Intersection semantic만 남긴 비교 맵은 10 Hz를 유지했다. 따라서 57개 전체 area 수나 C++ routing graph가 병목이라는 가설은 배제됐다.
- 좌·우회전 3개만으로도 2 Hz 부근으로 저하됐고, `blind_spot.backward_attention_length`를 100 m에서 30 m로 낮춘 검증 전용 설정도 개선하지 못했다.
- 낮은 trajectory rate 때문에 `/api/operation_mode/change_to_autonomous`는 `The target mode is not available`을 반환했고, 진단에는 `control_command_gate: timeout_builtin` 및 trajectory `WarnRate`가 나타났다. 이로 인해 odometry 진행/goal 도달과 red/green 신호 감속 시험은 **유효하게 수행할 수 없었다**.

### 배포 판단

v45를 현재 기본 Blind Spot 활성화 구성으로 바로 대회 autonomous 주행에 투입하면 안 된다. 다음 중 하나를 먼저 수행해야 한다.

1. 현재 Autoware 버전의 Blind Spot scene module profiling 및 runtime/RTC 설정을 수정하여 좌·우회전 lanelet에서 5 Hz 이상 trajectory를 보장한다.
2. 해당 모듈을 안전 검토 하에 비활성화한 configuration으로는 Intersection/route 회귀만 별도로 수행한다. 이는 Blind Spot 안전 검증을 대체하지 않는다.
3. 수정 후 동일한 대회 route에 대해 autonomous odometry·goal 도달, 신호 red/green factor·정지·재출발, 교차 객체/VRU 시나리오까지 다시 실행한다.

## Hz 저하 원인 재분석 (2026-07-24, 위 Blind Spot 결론 정정)

위 표의 `blind_spot 기동`은 관찰 사실이지만, 병목 원인이라는 결론은 **정정한다**. 동일 Docker/Planning Simulator에서 route를 다시 주입하고 BVP의 자체 processing-time topic을 측정했다.

| 조건 | `/planning/trajectory` | BVP 전체 | Intersection | Blind Spot | 결과 |
|---|---:|---:|---:|---:|---|
| v45, 대회 route의 교차로 활성 구간 | 2.16–2.22 Hz | 428.8–449.6 ms | 402.0–421.1 ms | 1.29–1.54 ms | Intersection이 지연의 대부분 |
| `attention_area_length=50 m` 검증 전용 | 2.08–2.11 Hz | - | 442.8–534.1 ms | - | 개선 없음 |
| `path_interpolation_ds=1.0 m` 검증 전용 | 2.18–2.19 Hz | - | 394.6–437.6 ms | - | 개선 없음 |
| 직진 4개만 semantic | 9.95–10.00 Hz | 정상 범위 | - | - | 정상 |
| 직진 4개 semantic + `200392: turn_direction=left` 한 개만 추가 | 1.77–1.91 Hz | - | 활성화됨 | - | 좌회전 태그 하나로 재현 |

- 마지막 최소 재현 맵의 `200392`에는 `intersection_area` 태그를 넣지 않았다. 따라서 새 `intersection_area` polygon/노드/way가 원인이 아니다.
- 런타임 파라미터 `intersection.occlusion.enable=false`, `intersection.common.use_intersection_area=false`를 확인했다. Occlusion 및 v45 area polygon 해석도 원인이 아니다.
- 활성 상태의 debug state에는 Intersection module이 교차로를 처리 중임이 나타났고, 상세 처리시간 트리는 빌드 설정상 `SceneModuleManagerInterface::modifyPathVelocity` 한 단계만 노출돼 내부 함수명까지는 분해되지 않았다.
- 즉 올바른 좌·우회전 `turn_direction`이 Intersection scene module을 활성화하고, 이 Autoware 빌드/기존 K-City topology 조합에서 해당 모듈의 충돌 차선·경로 처리 비용이 약 0.4–0.5초가 되는 것이 직접 원인이다. GIS에서 추가한 area나 Blind Spot은 직접 병목이 아니다.

따라서 v45 semantic 자체를 삭제하거나 잘못된 방향으로 바꾸는 것은 해결책이 아니다. Intersection module의 세부 CPU profiler(심볼 포함 빌드 또는 ptrace 허용 환경)로 충돌 차선/경로 탐색의 내부 연산을 확인하고, 안전 요구사항을 만족하는 module parameter 또는 Autoware 구현 수정안을 별도로 검증해야 한다.
