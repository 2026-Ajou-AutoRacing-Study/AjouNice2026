# MORAI Asset 재검증·WoodBox 실측 결과

## 현재 판정

- **내부적으로 확정:** 개인용 MRP의 거리/yaw grid에서 WoodBox mesh/LiDAR return과
  raw size-derived GT 경계의 pose-dependent 차이가 반복 재현됐다.
- **대회 MR에서도 다각도 재현:** 정상 센서 주기에서 좌·우 oblique, yaw
  `0/30/60/90 deg`의 CargoBox/WoodBox 4개 exact-pose pair를 다시 기록했고,
  WoodBox의 pose-dependent GT 경계 밖 return을 재현했다.
- **아직 확정하지 않음:** 대회 MR의 공식 WoodBox GT가 잘못됐다는 최종 원인 판정,
  고정 box offset 값, MR/MRP asset·annotation 버전 차이. MORAI 담당자 답변이 필요하다.
- **현재 dataset 정책:** WoodBox는 positive pool에 넣지 않고 scene 전체를 명시적으로
  제외한다. 임의의 `+3/+5/+10 cm` 확대 GT는 학습 label로 채택하지 않는다.
- **외부 답변 대기:** WoodBox의 실제 대회 출제 가능성과 공식 3D GT 사용 계약.

따라서 이 문서의 결론은 "MR/MRP 양쪽에서 현상은 확인됐지만 공식 원인과 correction은
미확정"이다.
MORAI 문의는 닫지 않는다.

- 상태: **LIVE EVIDENCE**
- 실측일: 2026-09-20
- 런처: 개인용 MRP `R1.260701.H3`

## 1. Asset 호환성 재검증

known-good control과 candidate probe를 교대로 로드했다. control 실패를 candidate
미지원으로 오판하지 않도록 `/Object_topic`의 class별 actor 수를 직접 확인했다.

| 결과 | Asset |
|---|---|
| 두 위치 모두 성공 | Porter2, DEFAULT_SUV, DEFAULT_BUS, Niro EV, Xcient Mixer, Stinger, DEFAULT_MPV, Woman2, Girl2 |
| 반복 실패 유지 | `police_f_1`, `Boy2`, `Scooter_01` |
| 현재 런처에서 새 회귀 | `2017_Hyundai_Grandeur` |

Grandeur는 과거 2/2 성공 기록이 있지만 이번에는 control이 성공하는 동안 두 위치
0/2, 별도 재시도 0/5였다. personal option/template에 DataID가 존재하므로 catalog
누락보다는 현재 launcher/template/ScenarioLoad 조합의 회귀로 본다.

같은 날 sample scenario의 `RandomMode` 장기 관측 초반 21 birth 중 Grandeur가 2회
정상 spawn되어 `/Object_topic`에 실제 이름과 크기로 발행됐다. 따라서 runtime model
asset 자체가 없는 것은 아니며, 문제 범위는 direct static ScenarioLoad template 또는
직접 배치 직렬화 경로로 더 좁혀진다.

근거:

- `/home/luv/morai_cap/260920_asset_revalidation_live_probe`
- `/home/luv/morai_cap/260920_asset_revalidation_grandeur_retry`
- `reports/asset_revalidation_260920_live.json`

### 1.1 대회 계열 MR minimal profile 재검증

위 MRP 실패는 전역 asset 미지원이 아니었다. MR `S4.251001.MolitComp03`에서
minimal five-interface Ego Network를 사용해 네 candidate와 같은 class control을 두
위치에서 교차 로드한 결과 총 16/16 scene이 첫 시도에 성공했다.

| Candidate | MRP | MR | `/Object_topic` identity |
|---|---:|---:|---|
| `2017_Hyundai_Grandeur` | 0/2, 별도 0/5 | 2/2 | 일치 |
| `police_f_1` | 0/2 | 2/2 | 일치 |
| `Boy2` | 0/2 | 2/2 | 일치 |
| `Scooter_01` | 0/2 | 2/2 | 일치 |

따라서 네 asset은 **MR 생성 후보로 복구**하되, positive 학습 pool 편입은 LiDAR/camera
visibility와 GT box 정합을 따로 확인한다. 이 결과는 launcher/template 호환성 판정이며
검출 난이도 판정이 아니다.

- runtime result: `/home/luv/morai_cap/260920_asset_revalidation_mr_minimal`
- runtime manifest: `manifests/asset_revalidation_260920_mr_minimal_runtime.json`
- network profile: `configs/network/ego_network_mr_minimal_hybrid.json`

## 2. WoodBox/CargoBox exact-pose A/B

이 절의 40-scene 결과는 개인용 **MRP `R1.260701.H3`** 결과다. 이후 대회 계열 MR에서
수행한 동일 조건 검증은 2.1절에 별도로 기록한다.

거리 `5/10/15/20 m`, 상대 yaw `0/30/45/60/90 deg`에 대해 CargoBox와 WoodBox를
ego와 장애물 pose가 정확히 같은 쌍으로 배치했다. 총 40 scene은 모두 1회에
정상 load되었고 LiDAR와 4-camera를 기록했다.

- bag: `rosbags/datasets/woodbox_alignment/260920_woodbox_cargobox_alignment.bag`
- manifest: `manifests/woodbox_alignment_260920.json`
- live result: `/home/luv/morai_cap/260920_woodbox_alignment_live`
- audit: `artifacts/datasets/woodbox_alignment_260920/audit_centered/woodbox_alignment_audit.json`
- figures: `artifacts/datasets/woodbox_alignment_260920/audit_centered/wba*_bev.png`

MORAI actor position을 ground pivot으로 해석하고 `size.z / 2`만큼 box center를 올린
원시 size-derived GT를 기준으로 셌다. learned asset correction은 적용하지 않았다.

| Asset | Raw GT 내부 points | +3 cm 내부 points | 증가율 | Raw GT 밖이지만 +10 cm 이내 비율 |
|---|---:|---:|---:|---:|
| CargoBox | 173 | 188 | +8.7% | 30.0% |
| WoodBox | 786 | 1,429 | +81.8% | 54.8% |

단, 5 m에서는 VLP-16 근거리/차체 가림 효과가 커서 CargoBox가 0 point이고 WoodBox는
큰 형상 일부가 잡혔다. 문의 근거로는 10--20 m를 우선 사용한다. 10 m 다섯 yaw에서
`+3 cm`가 회수한 비율은 CargoBox `+7.6%`, WoodBox `+25.0%`였다. 15--20 m에서는
point가 줄어 차이가 작아지지만, 그림상 WoodBox 표면 return이 pose에 따라 원시 GT
경계 바로 밖에 연속적으로 놓이는 현상이 재현된다.

이 수치는 box 주변 배경점까지 완전히 instance 분리한 값은 아니다. 따라서 MORAI에
보낼 때는 단일 숫자만 단정적으로 제시하지 않고, exact-pose Cargo control과 같은
거리/yaw의 WoodBox 확대 그림을 함께 제시한다. 핵심 결론은 `load 실패`가 아니라
`pose-dependent mesh/LiDAR--GT boundary mismatch`가 재현됐다는 것이다.

추천 그림 쌍:

- 10 m / 45 deg: `wba015_bev.png`(CargoBox), `wba016_bev.png`(WoodBox)
- 10 m / 30 deg: `wba013_bev.png`(CargoBox), `wba014_bev.png`(WoodBox)
- 15 m / 45 deg: `wba025_bev.png`(CargoBox), `wba026_bev.png`(WoodBox)

### 2.1 대회 계열 MR 재검증

MR `25.S4.MolitComp03`에서 10 m, yaw `0/30/45/60/90 deg`의 동일 pose pair 10개를
다시 기록했으며 전 scene이 정상 load되었다.

- bag: `rosbags/datasets/woodbox_alignment_mr/260920_164959_mr_cargobox_woodbox_10m_yaw_grid.bag`
- live result: `/home/luv/morai_cap/260920_165008_woodbox_mr_10m_yaw_grid`
- audit: `artifacts/datasets/woodbox_alignment_260920/mr_10m_yaw_grid_final/woodbox_alignment_audit.json`

이 stationary MR bag은 LiDAR 약 `1.77 Hz`, camera 약 `3.60 Hz`, Ego 약 `7.16 Hz`인
심각한 simulator slowdown 상태였다. 이어 수행한 좌·우 oblique rolling 8-scene
A/B에서도 Ego가 실제 `10.7--11.1 m` 이동하고 yaw `0/30/60/90 deg`가 적용됐지만,
주행 중 LiDAR `3.09--4.30 Hz`, Ego/Object `12.39--17.72 Hz`로 저하됐다.

따라서 두 MR run의 point audit는 **무효**이며, 정상 주기 재취득 전에는 MRP 문제가
MR에서 재현됐는지 또는 해소됐는지 결론 내리지 않는다.

- rolling bag: `rosbags/datasets/woodbox_alignment_mr/260920_mr_cargobox_woodbox_rolling_oblique.bag`
- rolling result: `/home/luv/morai_cap/260920_170020_woodbox_mr_rolling_oblique`
- rolling audit: `artifacts/datasets/woodbox_alignment_260920/mr_rolling_oblique/woodbox_alignment_audit.json`

이후 minimal profile로 10 m/yaw 45도 pair를 정상 주기에서 다시 기록했다.

| Asset | Raw GT | +3 cm | +5 cm | +10 cm |
|---|---:|---:|---:|---:|
| CargoBox | 19 | 22 | 27 | 33 |
| WoodBox | 73 | 80 | 86 | 92 |

capture window LiDAR는 `11.90/12.63 Hz`, camera는 약 `29.6--29.9 Hz`였으므로 이 pair는
유효하다. 다만 asset별 1 pose뿐이라 MR WoodBox offset을 새로 확정하기에는 부족하다.
현재 변환기는 offset catalog에 없는 WoodBox를 fail-fast하며, smoke dataset에서는
`--drop-scene-asset WoodBox`로 scene 전체를 명시적으로 제외했다.

- bag: `rosbags/datasets/base4_temporal/mr_minimal_hybrid_pilot/260920_202943_mr_minimal_hybrid_2scene_0.bag`
- audit: `artifacts/datasets/woodbox_alignment_260920/mr_minimal_hybrid_2scene/woodbox_alignment_audit.json`

### 2.2 대회 MR 정상 주기 다각도 재검증

MR minimal five-interface profile로 다음 네 geometry의 CargoBox/WoodBox exact-pose pair,
총 8 scene을 다시 기록했다. Ego는 `5 km/h`로 약 `3.0--3.2 m` 이동했고 scene마다
3개 trigger를 남겼다.

| 종/횡 위치 | 상대 yaw | CargoBox raw/+10 cm | WoodBox raw/+10 cm |
|---|---:|---:|---:|
| `+14/+3.0 m` | 0° | `12/30` | `12/93` |
| `+14/-3.0 m` | 30° | `6/7` | `34/43` |
| `+18/+3.5 m` | 60° | `7/8` | `16/25` |
| `+18/-3.5 m` | 90° | `4/7` | `0/28` |

전체 합산은 다음과 같다.

| Asset | Raw GT | +3 cm | +5 cm | +10 cm | raw 밖, +10 cm 안 비율 |
|---|---:|---:|---:|---:|---:|
| CargoBox | 29 | 44 | 48 | 52 | 44.2% |
| WoodBox | 62 | 79 | 120 | 189 | 67.2% |

특히 WoodBox yaw 90° scene은 raw GT 내부 point가 `0`이지만 `+5 cm`에서 `24`,
`+10 cm`에서 `28` point가 잡혔다. 이 결과는 WoodBox의 pose-dependent
mesh/LiDAR--size-derived GT 경계 차이가 개인용 MRP만의 현상이 아니라 대회 계열 MR에서도
재현됨을 뒷받침한다. 다만 audit는 semantic instance segmentation이 아니라 box 주변
support를 세므로, 위 숫자를 곧바로 정답 offset으로 사용하지 않는다.

모든 capture window는 정상 센서 조건을 통과했다.

- LiDAR: `11.08--12.83 Hz`.
- camera: `29.38--30.12 Hz`.
- Ego/Object: 약 `45.75--48.36 Hz`.
- camera--LiDAR nearest age 100 ms 초과: `0`.
- marker/audit: `8/8 completed`, error `0`, PASS.

근거:

```text
rosbags/datasets/woodbox_alignment_mr/normalhz_oblique_grid_final/
artifacts/datasets/woodbox_alignment_260921/mr_normalhz_oblique_grid/
/home/luv/morai_cap/260921_012213_woodbox_mr_oblique_normalhz_final/
```

## 3. 고주로 Random Spawn 장기 관측

분석기는 `scripts/analyze_morai_highway_random_spawn.py`로 구현했다.

- frame별 vehicle occupancy가 아니라 새 ID 출현/reappearance인 birth event를 센다.
- first appearance가 고주로의 lateral x corridor 안에 있는 경우만 센다.
- 최신 Object topic이 실제 asset명을 주면 그것을 사용한다.
- 과거 bag처럼 link ID가 들어오면 통제된 Base3의 3D size signature로 대응한다.
- Prius/DEFAULT_WAGON처럼 size가 같은 경우는 임의로 고르지 않고 ambiguous로 남긴다.

사용자 요청에 따라 1,000회 이전에 종료했으며 최종 유효 표본은 충분한 `482 birth`다.

- bag 시간: 4,848.4초, 약 80분 48초.
- 평균 birth rate: 5.97/min.
- 관측 모델: 38종.
- 모델별 횟수 범위: 6--18회, 균등일 때 기대값 12.68회.
- 38종 개별 모델 균등 goodness-of-fit: chi-square 22.25, df 37, `p=0.974`.
- 38종 구성비로부터 기대한 family 분포와 관측 분포: chi-square 2.35, df 6,
  `p=0.885`.

따라서 현재 MRP sample RandomMode는 **관측된 eligible 38개 모델을 개별 모델 단위로
거의 균등 추출한다**는 해석이 가장 타당하다. p-value는 균등임을 증명하는 값은 아니지만,
482회 관측에서 균등 가설을 기각할 근거가 전혀 없고 편향 패턴도 보이지 않는다.

| Family | 모델 수/38 | 기대 비율 | 관측 birth | 관측 비율 |
|---|---:|---:|---:|---:|
| Passenger car | 13 | 34.21% | 161 | 33.40% |
| SUV | 11 | 28.95% | 144 | 29.88% |
| Electric passenger car | 5 | 13.16% | 69 | 14.32% |
| Truck | 4 | 10.53% | 50 | 10.37% |
| MPV | 3 | 7.89% | 32 | 6.64% |
| Bus/Van | 1 | 2.63% | 15 | 3.11% |
| Wagon | 1 | 2.63% | 11 | 2.28% |

### 3.1 MR 43종 전체와의 차이

관측 38종은 MR 43종 전체가 아니다.

- MR/MRP 공통 모델 중 관측: 35종.
- MRP-only 관측: `2019_hyundai_genesis_g90_4WS`, `DR_KGTorres_EVX_2024`,
  `DR_OMLift_2022_Bus` 3종.
- MR에 있지만 미관측: `2014_Hyundai_Xcient(Mixer)`, `2016_Hyundai_Universe`,
  `DEFAULT_BUS`, `DEFAULT_MPV`, `DEFAULT_SEDAN`, `DEFAULT_SUV`, `DEFAULT_TRUCK`,
  `DEFAULT_WAGON` 8종.

sample spawn point의 `listVehicleLengthType=[2]`가 RandomMode 내부 eligible pool을
거르는 것으로 보이나 정확한 필터 정의는 MORAI 확인이 필요하다. 따라서 대회 MR에서도
동일하게 35종만 쓰인다고 단정하지 않는다. 현재 Base3의 MR 43종 균등 deck은 coverage
측면에서는 안전하지만, 실제 sample spawn 분포의 정확한 모사는 아니다.

세 spawn point의 map 위치는 서로 다르지만 `/Object_topic`에 차량이 처음 보일 때는 이미
하류 합류 뒤이고 동시에 사실상 한 대만 보였다. 따라서 source spawn ID 6/8/10은
추측하지 않고 482회 모두 `downstream_unassigned`로 남겼다.

최종 산출물:

- `artifacts/datasets/morai_spawn_stats/260920_highway_random_spawn_1k/spawn_statistics.md`
- `artifacts/datasets/morai_spawn_stats/260920_highway_random_spawn_1k/spawn_statistics.json`
- `rosbags/datasets/morai_spawn_stats/260920_highway_random_spawn_1k.bag`

## 4. MORAI 문의 문안

최신 전달 문안과 첨부 근거는 `260920_questions_for_morai.md`를 기준으로 한다.
