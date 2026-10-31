# Base3-Static 10K 최종 capture 통합 감사.

마지막 갱신: 2026-08-29.

## 최종 상태.

Batch 001--007 primary는 3,334 clips 중 3,322 clips가 최초 capture에서 완료됐다.
실패 12 clips를 family·split·공간 quota에 맞춰 보충하고, post-hoc 정적 actor motion
audit에서 실제 이동한 TrafficBarrel clip 하나를 제외·재보충했다.

| 항목 | 최종 결과 |
| --- | ---: |
| 유효 clip | 3,334 |
| 유효 frame | 10,002 |
| Train / Val / Test clip | 2,334 / 500 / 500 |
| 최초 primary 성공률 | 3,322/3,334, 99.64% |
| 최종 artifact error | 0 |
| 원본 capture 용량 | 45.11 GiB |

최종 유효 구성은 다음과 같다.

- Batch 001--007 primary 완료본 3,322 clips.
- Replacement v1 완료본 10 clips 중 실제 이동한 `b3sfinal10prodb003p_dso106`은 제외.
- Replacement v2 안전 대체본 2 clips.
- Replacement v3 안전 대체본 1 clip.

따라서 `3,322 + 9 + 2 + 1 = 3,334 clips`, clip당 3 frames로 정확히
10,002 frames다.

## Family와 split.

| Family | Clip | 전체 비율 |
| --- | ---: | ---: |
| `stopped_vehicle` | 1,334 | 40.01% |
| `static_obstacle` | 1,000 | 29.99% |
| `standing_pedestrian` | 500 | 15.00% |
| `sparse_background` | 500 | 15.00% |

| Split | Clip | 전체 비율 |
| --- | ---: | ---: |
| Train | 2,334 | 70.01% |
| Val | 500 | 15.00% |
| Test | 500 | 15.00% |

최초 실패는 Train 10, Val 2, Test 0이었다. Replacement도 같은 family와 split을
유지해 최종 quota가 원 설계와 정확히 일치한다.

## 공간분포.

| 공간 stratum | Clip |
| --- | ---: |
| `route_general` | 1,618 |
| 기존 안정 고주로 `highway_three_lane` | 351 |
| `roundabout` | 341 |
| 일반 Pedestrian 위치 `unspecified` | 175 |
| `crosswalk` | 325 |
| `highway_post_taper_two_usable` | 124 |
| `highway_exit_split` | 85 |
| `route_general_two_lane_right` | 120 |
| `highway_four_to_three_taper` | 75 |
| `highway_merge_lane` | 42 |
| `route_general_two_lane_left` | 60 |
| `highway_ramp_approach` | 18 |

Corrective Batch 005--007은 ramp 18, merge 42, exit 85, taper 75, post-taper 124,
일반 2차로 우측 120, 좌측 60 clips를 설계값 그대로 채웠다.

고주로 세부 variant는 다음을 포함한다.

- Ramp approach 18.
- Merge-lane Ego 24, merge-mainline Ego 18.
- Exit mainline lane2/3/4가 18/22/33, exit branch Ego 12.
- Taper lane2/3/4가 18/27/30.
- Post-taper left/right가 54/70.

로터리는 접근 91, 진입·합류 104, 순환 119, 이탈 27 clips다. 이탈을 낮게 두고
접근·합류·순환에 집중한다는 설계가 유지됐다.

## Runtime과 artifact 품질.

- Batch 001--007 전체 post-hoc artifact validator error 0.
- Capture Mode의 `/clock` 부재 warning은 정보성 경고이며 sensor artifact 누락이 아니다.
- Scene duration p50/p95는 10.06/10.97초다.
- SaveSensorData frame 간 실제 trigger lag p50은 1.31초다.
- 따라서 clip의 3 frames는 독립 single-frame 관측이며 12 Hz temporal sweep이 아니다.
- Ego/Object state age p50은 각각 11.3/11.6 ms, p95는 28.5/31.7 ms다.

최초 primary 실패 12건은 scenario object 일부 미로딩 4, collision 3, static actor
spawn 위치 이탈 3, pre-capture 이동 2건이다. Replacement v1에서 희귀 scene 5개를
재시도했으나 ramp rear truck 미로딩과 current-lane Steel_Barricade 충돌은 결정적으로
재현됐다. 안전 대체본은 각각 문제 actor만 제거하고 family·split·공간 quota와 나머지
geometry를 유지했다.

## LiDAR GT evidence.

30 m 이내 MORAI LiDAR GT box 29,545 observations를 실제 XYZI point로 감사했다.
Box tolerance 0.03 m, 정상 positive 기준은 5 points다.

| Class | GT box | 5+ points | 1--4 points | 0 point |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 7,247 | 7,239, 99.89% | 1 | 7 |
| Pedestrian | 5,919 | 5,732, 96.84% | 157 | 30 |
| Obstacle | 16,379 | 15,175, 92.65% | 851 | 353 |
| 전체 | 29,545 | 28,146, 95.26% | 1,009 | 390 |

학습 packaging에서 0-point GT는 제거하고 1--4-point GT는 positive loss에서 제외한다.
Sparse tail은 주로 Steel_Barricade, CargoBox, PE Firewall과 작은 Pedestrian asset에
집중된다. Vehicle geometry는 사실상 전부 정상 positive다.

## 객체 다양성과 방향.

- Vehicle asset 43종.
- Pedestrian asset 22종.
- Obstacle asset 10종.
- 정확한 Asset name, DataID, subtype과 시나리오 작성 규칙은
  `base3_static_asset_catalog.md`를 따른다.

`/Object_topic` actor-frame을 Ego 기준 방위로 나누면 다음과 같다. 이는 30 m LiDAR
positive만이 아니라 scene의 전체 actor observation이므로 학습 GT 분포와 완전히 같지는
않지만 방향 편향 audit에 사용한다.

| Class | Front | Left | Right | Rear |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 5,464 | 379 | 221 | 2,216 |
| Pedestrian | 4,536 | 479 | 709 | 195 |
| Obstacle | 12,142 | 1,256 | 1,344 | 1,647 |

Vehicle right observation은 2.67%로 여전히 낮다. 다만 대회 우측통행과 현재 mission에서
우측 Vehicle 중요도가 낮으므로 Base3 수집을 다시 열지 않는다. Mission rosbag에서 실제
우측 Vehicle recall 문제가 나타날 때 targeted add-on으로 처리한다.

## Post-hoc motion audit에서 얻은 교훈.

원래 pre-capture stability gate에서 한 번 실패했던 TrafficBarrel scene은 replacement
재시도에서 gate를 통과했지만 실제 3 frame 동안 약 2.4 m 이동했다. Frame metadata의
위치 차분 audit이 이를 찾아 최종 dataset에서 제외했다.

따라서 `derived_velocity_mps`를 팀원 공유용 학습 GT로 노출할 필요는 없지만, 연속 위치로
static/dynamic consistency를 다시 계산하는 post-hoc audit 자체는 유지한다. Base4에서는
정확한 timestamp로 builder가 필요 시 계산하고, GT label과 diagnostics namespace를
명확히 분리한다.

## Batch 008: 움직이는 Pedestrian single-frame 보강.

초기 75 clips/600 frames add-on은 `12 Hz × 8 frames`로 잘못 설계되어 Base3에서 제외했다.
Base3는 temporal dataset이 아니며 움직이는 Pedestrian도 Batch 001--007과 동일한 capture
계약으로 수집한다.

- Batch 008은 300 clips × 3 frames = 900 frames다.
- `rolling-grouped`, Ego route displacement 3 m마다 한 장을 저장한다.
- 세 frame은 temporal sweep으로 묶지 않고 독립 single-frame sample로 사용한다.
- Train/Val/Test는 crosswalk-location 단위로 210/45/45 clips다.
- 걷기/뛰기/activation transition은 135/135/30 clips다.
- 좌→우/우→좌는 150/150 clips다.
- 22개 검증 Pedestrian asset은 primary 생성 시 각각 13--14 clips였고, 실제 replacement
  반영 후 최종 12--15 clips 범위다.
- 공식 sample 횡단보도 25%, 다른 train 횡단보도 45%, location-disjoint Val/Test 30%다.
- Velocity raw/derived 값은 audit diagnostics일 뿐 Base3 학습 target으로 노출하지 않는다.

Batch 008 포함 최종 이름도 별도 Dynamic add-on이 아닌 `Base3-Static 10K`로 유지한다.
최종 크기는 3,634 clips/10,902 frames다.

### Batch 008 실제 capture 결과.

- Primary는 300개를 시도해 294 clips/882 frames가 유효했다.
- 동일 scene retry로 2 clips/6 frames를 복구했다.
- 잔여 분포를 보존한 targeted replacement로 4 clips/12 frames를 추가했다.
- 최종 합계는 정확히 300 clips/900 frames이며 output validator artifact error는 0이다.
- 최종 위치 quota는 `75/60/45/45/38/37`, Train/Val/Test는 `210/45/45`다.
- Walking/running/activation transition은 `135/135/30`, 방향은 `150/150`으로 생성
  목표와 정확히 일치한다.
- 일반 대회 주행/저속 접근 Ego profile은 replacement 영향으로 `224/76`이다.
- 실제 frame 간 Ego 이동거리는 최소 3.001 m, 중앙값 3.103 m, p95 3.264 m,
  최대 3.419 m다.
- 실제 Pedestrian speed는 최소 1.115 m/s, 중앙값 1.859 m/s, 최대 2.772 m/s다.
- LiDAR evidence는 5-point 이상 868 frames(96.44%), 1--4 points 31 frames(3.44%),
  0 point 1 frame(0.11%)이다. 후자의 두 구간은 기존 label-quality mask를 적용한다.

최종 capture root는 다음 세 개를 함께 사용한다.

- `batch008_dynamic_ped`: 294 clips/882 frames.
- `batch008_dynamic_ped_replacement_v1`: 2 clips/6 frames.
- `batch008_dynamic_ped_replacement_v2`: 4 clips/12 frames.

모두 `/home/luv/morai_cap/260829_034057_base3_static_10k` 아래에 있다. 최초
`active_at_load` 시행착오 run은 `batch008_dynamic_ped_rejected_active_at_load`로 격리했고
학습 index에서 제외한다.

고주로 spawn 가속 Vehicle은 Batch 008보다 Base4-Temporal Vehicle 구성에 넣는다. 차량은
정적·동적 외형 차이가 작고 Base3에 ramp/merge geometry가 이미 있다. Spawn 0→목표속도는
velocity 학습에는 유용하지만 실제 vehicle temporal 분포를 지배하면 안 되므로 Base4의
Vehicle clips 중 10--15% tail case로 제한한다. 나머지는 이미 목표속도로 주행 중인 합류,
본선 순항·추월, 로터리 순환 차량으로 구성한다.

## 2026-10-18 보관 형식 변경 안내

위 내용은 당시 수집·채택 결과이며 수정하지 않았다. 이후 사용자 승인으로 이 루트의
RGB PNG 44,212장을 같은 해상도의 Q90 JPEG로 변환하고 전수 검증 후 RGB PNG만 삭제했다.
LiDAR·native GT·Semantic/Instance/Depth·메타데이터는 보존했다. 이제 재학습용이 아니라
분석용으로 보관하므로 옛 PNG 학습 경로를 그대로 실행하지 않는다.
분석 색인은 `artifacts/datasets/base3_static_10k_jpeg90_analysis_261018/manifest.json`이며,
최종 성공 기준은 같은 폴더의 `completion.json`이다. [정리·보존 근거](../261017_base5_unit_release_evidence.md#approved-storage-cleanup-261018)를 참고한다.
