# Base3 Dynamic-Ped Add-on v1 최종 capture 감사

마지막 갱신: 2026-08-29.

## 상태: Base3에서 제외된 superseded pilot

이 capture는 `12 Hz × 8 frames` temporal sequence로 만들어졌으나, 최종 Base3의 목표는
동적 보행자도 다른 정적 family와 동일한 `3 m 간격 × 3 independent single-frame`으로
수집하는 것이다. 따라서 아래 600 frames는 Base3 학습·Val·Test 구성에서 전부 제외한다.
대체 데이터는 `Base3-Static Batch 008`, 300 clips/900 frames로 새로 수집한다.

## 원래 목적과 capture 결과

Base3-Static의 정지 Pedestrian과 실제 걷기·달리기 animation 사이 domain gap을 줄이기
위해 시도했던 별도 add-on이다. 최종 dataset source가 아니라 시행착오 감사 기록으로만 유지한다.

| 항목 | 결과 |
| --- | ---: |
| 유효 clip | 75 |
| 유효 frame | 600 |
| Train / Val / Test clip | 53 / 11 / 11 |
| 최초 전체 capture | 74 성공 / 1 실패 |
| replacement | 1 성공 |
| 최종 artifact error | 0 |
| 원본 용량 | 약 2.84 GiB |

최종 선택 root는 다음 두 개다.

- Primary 74 clips:
  `/home/luv/morai_cap/260829_161242_base3_dynamic_ped_addon_v1_final75`.
- `b3dpv1_dpd010` replacement 1 clip:
  `/home/luv/morai_cap/260829_162959_base3_dynamic_ped_addon_v1_replacement_dpd010`.

최초 실패는 MORAI control-mode service가 한 차례 timeout된 직후 pedestrian readiness가
풀리지 않은 통신·runtime 상태 문제였다. 같은 scene은 즉시 재시도에서 정상 동작했고
8/8 frame을 모두 저장했다.

## 구현 중 발견하고 수정한 문제

1. 기존 dynamic pedestrian 생성기는 공식 sample의 `Man1`만 사용했다.
   - personal launcher에서 live 검증된 실제 사람 asset 22종을 round-robin deck으로 연결했다.
   - 각 asset은 3--4 clips에 등장한다.
2. catalog actor 전체를 복사하면 MORAI가 actor를 표시하지만 PedPath motion을 실행하지 않았다.
   - visual identity인 `DataID`와 animation fields만 교체한다.
   - 공식 sample의 검증된 `UNIQUEID`, `waypointEventList`와 behavior 구조는 유지한다.
3. sparse global path 근처 평행도로 Ego가 잘못 snap되는 문제가 있었다.
   - MGeo directed-link route를 재구성해 crosswalk Ego route로 사용한다.
   - 최종 Ego route distance 최대값은 0.248 m다.
4. stationary Ego에서는 SaveSensorData 지연 중 simulator physical time을 신뢰성 있게 복원할
   수 없었다.
   - 원래 stationary/very-low-speed 25% 계획을 very-low-speed moving Ego 25%로 수정했다.
   - 이를 통해 모든 clip에서 실제 물리 진행량 기반 12 Hz sequence 계약을 유지한다.

## 생성 분포

| Crosswalk | Clip | 역할 |
| --- | ---: | --- |
| `CW000000` | 19 | 공식 sample 직접 통과 |
| `CW000002` | 19 | 다른 train 직접 통과 |
| `CW000013` | 11 | location-disjoint validation |
| `CW000015` | 11 | location-disjoint test |
| `CW000001` | 8 | route 인접 측면 관측 |
| `CW000014` | 7 | route 인접 측면 관측 |

- 보행 방향 forward/reverse는 38/37 clips다.
- 일반 대회 주행 Ego 56, 저속 이동 Ego 19 clips다.
- Walking 34, running 34, activation-start transition 7 clips다.
- Pedestrian asset은 Boy/Girl/Man/Woman/worker/police를 포함한 22종이다.

## Capture와 motion 품질

| 지표 | p05 | p50 | p95 |
| --- | ---: | ---: | ---: |
| 실제 sequence rate | 11.27 Hz | 11.63 Hz | 11.97 Hz |
| 8-frame sequence span | 0.585 s | 0.602 s | 0.621 s |
| Pedestrian 실제 speed | 1.10 m/s | 1.87 m/s | 2.75 m/s |
| Clip 내 Pedestrian 이동 | 0.72 m | 1.11 m | 1.63 m |
| Ego state age | 1.62 ms | 5.58 ms | 10.89 ms |
| Object state age | 1.51 ms | 5.81 ms | 25.97 ms |

- Pedestrian actor는 600/600 frames에 존재한다.
- Camera BBox2D는 600/600 frames에서 네 카메라 중 최소 한 곳에 존재한다.
- 모든 clip이 8개 frame과 Camera 4대/LiDAR artifact completeness를 통과했다.

## LiDAR evidence

| 구분 | Frame | 비율 |
| --- | ---: | ---: |
| 5 points 이상 normal positive | 588 | 98.00% |
| 1--4 points sparse positive | 11 | 1.83% |
| 0 point | 1 | 0.17% |

0-point frame은 `b3dpv1_dpd027_f05`다. 학습 packaging에서는 Base3와 동일하게 0-point
GT를 제거하고 1--4-point GT를 positive loss에서 제외한다.

## 가림 profile에 대한 결론

생성 설정의 `walking_partial_candidate`, `running_partial_candidate`와
`activation_transition`은 실제 visibility audit 후보를 표시한다. 자연 crosswalk 구조만으로
가림을 유도했으나 최종 camera BBox와 instance audit에서는 안정적인 partial/strong
occlusion quota가 형성되지 않았다. 따라서 이 V1을 clear dynamic-ped baseline으로 해석한다.

정차 차량을 시야선에 강제로 삽입하면 가림 비율은 만들 수 있지만 pedestrian-only mission의
객체 구성을 바꾸고 collision·shortcut 가능성을 추가한다. 첫 Base3+V1 학습과 실제
Pedestrian rosbag 평가에서 가림 recall이 병목으로 확인될 때만 별도
`Dynamic-Ped Hard-Occlusion Add-on`을 생성한다.

## 폐기된 학습 사용 계약

- Base3 single-frame fine-tuning에서는 이 capture를 사용하지 않는다.
- Base4-Temporal도 별도 목적과 sensor cadence로 다시 설계하므로 이 capture를 production
  source로 승격하지 않는다.
- `derived_velocity_mps`는 진단값이며 학습 GT가 아니다.
- 학습 GT는 이후 builder가 확정한 `velocity_target_ego_mps`만 노출한다.
- split은 frame이 아니라 crosswalk/location과 clip group을 기준으로 유지한다.

생성·capture selection의 기계 판독 결과는
`dataset/morai_od/reports/base3_dynamic_ped_addon_v1_capture_audit.json`에 있다.
