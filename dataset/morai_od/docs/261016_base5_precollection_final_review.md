# Base5 새 Sunny13 5K 본수집 전 최종 검토

문서 상태: EVIDENCE. 검증일: 2026-10-16.
현재 실행 여부와 release는 [Base5 CURRENT](BASE5_V2_CURRENT.md)를 우선한다.
이 문서는 완료된 기존 5K를 교체하거나 재학습한 기록이 아니다. 새 Sunny13 계획의 추가 검증과 결정을 기록한다.

## 이번 작업에서 결정한 것

근거리 장애물 보강·얇은 자산의 거리 상한·원거리 보조 차량 할당은 앞선 v151–v153 작업을 유지했다.
이번에는 원거리 초기 거리의 편향을 수정하고, 기존 지면 준비의 명시적 재사용을 구현했다.
새 native 40장과 실제 변환·학습 입력 경로를 추가로 검사했다. 5K 본수집과 모델 학습은 실행하지 않았다.

| 검토 항목 | 결과와 선택 |
|---|---|
| 0–20m·20–30m 장애물 | 앞선 paired native 검증을 유지한다. 이번에 근거리 quota를 다시 변경하지 않았다 |
| 60–99m 보조 차량 | 기존 160개 할당의 초기 거리를 구간 안에 분산했다. 자산·primary·Ego·split·역할은 유지한다 |
| 음영 이동 지면 | 같은 source·sensor·준비 근거가 일치한 두 장면만 등록해 재사용한다. 전체 24장으로 일반화하지 않는다 |
| 사거리 이동·복합체 | 새 걷기·달리기·유모차·반려동물 동반 장면을 native 캡처하고 component 변환을 확인한다 |
| 멀고 정보가 적은 비차량 | 거리나 LiDAR 점 수만으로 일괄 삭제하지 않는다. 센서 지원이 있는 어려운 사례를 유지한다 |
| 이미지 높이 | 현재 512×1056을 유지한다. 576은 근접 차량 상단을 일부 복원하지만 원거리 픽셀 밀도는 늘리지 않는다 |
| 원본과 과거 데이터 | 기존 Sunny13 5K·실패 raw·과거 release·ledger를 보존한다. pilot을 기존 학습본에 합산하지 않는다 |

새 근거리 계획의 물리 장애물 19종은 같은 train 4,255장 계획 기준으로 0–20m 1,576→2,235개,
20–30m 625→697개, 전방 0–10m 392→719개다. 이는 배치 계획 수다.
Base3의 실제 retained GT/1,000장과는 분모가 달라 같은 수치처럼 비교하지 않는다.
[이전 실측](261015_base5_front_far_compound_experiments.md)에서 같은 승인 ID 43장의 근거리 GT는
64→98개, LiDAR 20점 이상은 39→55개였다. 새 보강이 E25보다 좋은 AP를 낸다는 뜻은 아니다.

## 원거리 초기 거리 편향 수정

v153d_fast는 허용 후보 중 가시성 proxy가 좋은 쪽을 선택해 각 구간의 가까운 끝으로 몰리는 경향이 있었다.
전방 97–99m 일곱 계획의 초기 거리가 97.02–97.44m에 모였고, 80–90m의 중앙값도 약 80.63m였다.
원거리 보강을 선언해 놓고 실제로는 가까운 경계만 반복하지 않도록 선택 순서를 보완했다.

[새 준비 도구](../scripts/prepare_base5_far_vehicle_spread.py)는 방향·허용 거리 창 안에 사전 목표 거리를
분산하고, source ID의 결정적 순서로 배정한다. 정해진 난수 후보 안에서 가능한 목표를 고른다.
기존 도로·지역·3D 완전 가림·보존 검사는 완화하지 않았다. 가능한 배치가 부족하면 목표와의 차이도 보고한다.
복구 때는 원래 허용 창을 유지하므로 최종 승인 GT가 균등 분포라고 보장하지 않는다.

| 계획 cohort | 이전 초기 거리 min / p50 / max | 새 초기 거리 min / p50 / max |
|---|---|---|
| 전방 60–80m | 60.04 / 62.10 / 79.96 | 60.39 / 69.89 / 79.50 |
| 전방 80–90m | 80.11 / 80.63 / 89.58 | 80.13 / 84.78 / 89.61 |
| 전방 97–99m | 97.02 / 97.16 / 97.44 | 97.09 / 98.16 / 98.66 |
| 후방 97–99m | 97.00 / 97.05 / 97.41 | 97.08 / 98.03 / 98.90 |

최종 geometry source는 `artifacts/evaluation/base5_precollection_261016/candidate_v154d_spread/manifest.json`이다.
5,000개 ID·모든 frozen spec·160개 보조 계획을 보존했고, 142개 scenario payload를 바꿨다.
나머지 4,840개 entry와 payload는 그대로다. 160개 중 18개는 metadata 증거만 갱신됐다.
중간 v154/v154b/v154c 출력은 진단 이력으로 보존하며 최종 실행 source로 사용하지 않는다.

### 새 native 원거리 32장

사전에 등록한 4개 거리 구간 × 전후방 × 같은 차로/인접 차로 × 2개를 모두 취득했다.
19개 차종을 포함하며 32/32가 첫 시도에 승인됐다. 캡처 누적시간은 178.09초다.

| 실제 target 지원 | 수 |
|---|---:|
| LiDAR 0점·camera gate 통과 | 8 |
| LiDAR 1–4점 | 13 |
| LiDAR 5점 이상 | 11 |
| camera 512 gate 통과 | 29 |
| 현재 모델 XYZ ROI 안에 중심 유지 | 32 |

실제 LiDAR 기준 전방 최대 97.94m, 후방 최대 99.39m였다.
전방 97m 이상 두 장면은 LiDAR 0점이지만 camera gate를 통과했다.
ego 중심의 계획 거리와 LiDAR 중심의 실측 거리를 혼동하지 않는다.
가시성·annotation·정지 Z 안정성은 확인했지만 모든 target의 도로 mesh 접촉을 인증한 것은 아니다.
이 결과는 학습된 detector의 recall·AP·tracking 성능이 아니다.

근거는 `native_report_v154.json`과 `qualification_v154_far32/registration.json`이다.
보고서 생성기는 실제 raw·native annotation·정식 parent 재구성 결과를 다시 읽고 source와 runtime을 대조한다.

## 이동·복합체와 기존 지면 준비 재사용

새 smoke 8장은 CargoBox, 중거리 Concrete_Barricade01, BabyStroller2, Ped_Pet1,
사거리 Woman5 걷기·Man2 달리기, 음영 Dog1 걷기·Dog2 달리기로 사전 등록했다.
8/8이 첫 시도에 승인됐고 누적시간은 72.17초다. 실제 이동·가시성·충돌 관측·종료 Park 검사는 유지했다.

음영 두 장면은 [지면 재사용 계약](261016_base5_shadow_ground_reuse.md)에 따라 과거 준비 결과를 등록했다.
source·sensor·준비 경로·과거 승인 ledger·충돌·Park·proof 파일의 해시를 모두 확인한다.
이전 calibration 캡처 2+3회는 반복하지 않고, 준비된 경로에서 **새 native 프레임 두 장**을 취득했다.
과거 프레임을 새 데이터로 세지 않는다. 준비 근거가 달라지면 오류로 중단하며 자동 외삽하지 않는다.

전체 실제 이동은 기존처럼 사거리 154장·음영 24장, 합계 178장이다.
현재 음영 24장 중 exact reuse를 등록한 것은 두 장면뿐이다.
별도의 두 장면은 명시적인 same-quota 대체 방식으로만 재사용 가능하며 이번 release에는 적용하지 않았다.
나머지 경로는 기존 독립 terrain/reference 준비를 따른다. 전체 24장 재촬영은 이번에 하지 않았다.

복합체의 카메라 mask는 parent/component 사이에서 모호하거나 공유될 수 있다.
그 경우에도 부모 mask를 각 component의 독립 증거로 새로 만들어 붙이지 않는다.
원래 component 의미·BBox·parent 관계와 native RPY를 보존하고, 현재 3클래스 매핑은 파생 infos에서만 적용한다.
반려동물 동반 객체와 유모차의 모호성은 별도 집계하며 일반 보행자의 성능으로 대신 인증하지 않는다.

## 실제 JPEG·GT·학습 입력 검사

[새 변환 검사](../scripts/qualify_base5_training_inputs.py)는 승인 ledger에서 원본을 읽어
native Q90 JPEG 전달본과 upright 3클래스 infos를 만들고 예상 retained component ID를 대조한다.
full RPY 점 포함 수를 다시 세고 upright 박스가 원래 여덟 모서리를 감싸는지 확인한다.
원본 파일 SHA·inode·mtime와 ledger는 전후에 동일해야 한다.

선행 고정 179장에서는 native component 623개 중 유효 GT 601개가 동일하게 보존됐고 추가·소실 ID는 없었다.
raw/source payload 중복 두 건은 분리해 177개의 고유 payload/source cell로 보고한다.
실제 MMDetection3D CPU train/val pipeline 358회도 통과했다. GPU forward·학습 실험은 아니다.
한 고정 증강 draw에서 GT 두 개가 ROI 밖으로 이동한 것은 정상 범위 필터이며 변환 소실로 세지 않았다.

점 수 이름도 구분한다. infos의 `num_lidar_pts`는 native full OBB 점 수이고,
지원 분석 `objects.json`의 `native_points`는 XY ROI, `points`는 XY+Z ROI 이후 값이다.
기존 5K의 49개 차이는 이 분모 차이이며 새 GT 오류라고 결론내리지 않았다.
기존 5K와 선행 179장에서 native≥1점이던 retained GT가 모델 XYZ ROI에서는 0점이면서
camera 512 근거도 없는 사례는 확인되지 않았다.

새 40장도 별도로 검사했다. 원거리 32장은 native/retained GT 모두 78개이며 target 32개가 전부 남았다.
혼합 8장은 native 16개 중 14개가 retained GT다. 제외된 두 개는 Ped_Pet1의 동물 성분으로,
LiDAR 0점이고 camera 512 대응도 확정하지 못한 경우다. 사람 성분은 유지됐다.
이는 기존 지원 정책의 정상 제외이며, 새로운 성분 소실·추가 오류는 없었다.
BabyStroller2의 두 사람 성분도 카메라 대응 모호성을 남긴 채 보존했다.
실제 CPU train/val pipeline 80회가 통과했고 한 증강 draw의 원거리 차량 한 개만 정상 ROI 제외됐다.
기존 179장과 새 40장 근거를 합쳐 현재 runtime에서 모두 새로 취득했다고 표현하지 않는다.

### 희소한 원거리 비차량

완료 train의 50m 이상 비차량 retained GT는 92개로 전체 9,350 GT의 약 0.98%다.
이 가운데 camera 정보가 충분한 경우 29개, camera gate를 만족하는 희소 사례 50개,
LiDAR 지원은 있지만 camera 대응을 확정하지 못한 경우 13개로 나눠 볼 수 있다.
센서 정보가 적다는 이유로 모두 잘못된 라벨은 아니다.

따라서 거리 40m 같은 일괄 cut, LiDAR 20점 필수 조건, camera-only GT 삭제는 적용하지 않았다.
작고 얇은 물리 primary의 기존 20/30/40m 거리 상한과 근거리 보강은 유지한다.
특히 음영 바깥 접근에서 안쪽 객체를 미리 보는 분포를 일괄 거리 제한으로 없애지 않는다.
새 loss ignore/가중치 정책은 이번 한 번의 수집 준비에 추가하지 않는다.

## crop·모델과 남은 검증의 경계

[512/576 CPU 진단](../../../docs/bevfusion/261016_BASE5_512_576_CROP_HEIGHT_DIAGNOSTIC.md)에서
0–5m의 카메라 관측 1,046개를 재검사했다. 576은 근접 차량 일부의 상단을 복원했지만
새 sensor-positive GT는 없었고, 별도의 원거리 두 코호트 64개 target에서는 추가 픽셀이 없었다.
현재 512×1056·5cm/20cm·depth 25cm 계약은 바꾸지 않는다.
원본 해상도를 보존하므로 나중에 576용 infos와 runtime을 따로 검증할 수 있으며 재수집할 필요는 없다.

E24 가중치·production threshold·추론 compile OFF는 그대로다.
다음 학습은 별도의 compile OFF 사전검사·GPU 자격 검증 후 시작한다.
새 학습의 근거리 장애물 및 80–100m 차량 AP, tracker 안정성, 실제 24GB 풀스택 지연은 후속 검증이다.
native 수집 승인을 정확도 향상이나 대회 환경 인증으로 해석하지 않는다.

## 실행·재개와 동결 근거

이번 검증 root는 `artifacts/evaluation/base5_precollection_261016`이다.
`registered_v154_shadow2/manifest.json`은 최종 geometry source에 두 exact 지면 registry 설정만 더한다.
새 release 생성기는 전체 5K 감사, source/runtime SHA, 현재 40장 raw, 변환 근거,
완료 replay와 CPU 회귀 결과를 다시 대조한 뒤에만 동결한다.

원거리 수집은 네 장 승인 뒤 실제로 일시정지하고 같은 output에서 나머지 28장을 재개했다.
기존 네 승인 row·raw·event는 보존됐고 시간은 누적됐다.
완료한 원거리·smoke output은 각각 다시 resume해 추가 캡처 0회,
원본·선택 ledger·이벤트·누적시간이 동일한 것을 확인했다.

최종 CPU 회귀는 1,279개 통과했다. 호스트에 Torch가 없는 별도 Base3 segmentation 검사 한 개만
명시적으로 deselect했다. 선택된 테스트의 skip·failure·error는 0개다. 이전 회귀 결과도
덮어쓰지 않고 보존했다. 최종 명령은 다음과 같다.

```bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 -m pytest -q dataset/morai_od/tests \
  --deselect=dataset/morai_od/tests/test_prepare_base3_lraspp_4class.py::test_segmentation_metrics_include_foreground_quality \
  --junitxml=artifacts/evaluation/base5_precollection_261016/cpu_tests_ready.xml
```

이는 로컬 실행 기록이며 JUnit 파일 자체가 독립적인 실행 인증 장치는 아니다.
신규 최종화 회귀는 source ID만 같은 다른 payload·metadata, stale raw inventory,
다른 source의 변환 보고서, 소실된 far target, 전부 skipped인 JUnit의 잘못된 재사용을 거부한다.

| 현재 근거 | 경로 | SHA256 |
|---|---|---|
| 최종 geometry | 검증 root의 `candidate_v154d_spread/manifest.json` | `ab8b425cc6dc41c317eae3e87181fec5b8ddd67e3f90da4a6423c2261a07448e` |
| 지면 registry 포함 실행 명세 | 검증 root의 `registered_v154_shadow2/manifest.json` | `6773f23d4e95bb9964b192a7646c50a7e29d01821fb5d742c168b75adfff5fc2` |
| 새 native 원거리 | 검증 root의 `native_report_v154.json` | `06d6b7b746de046742d80394fd8f7bece6d7e8c7cdf55c41b334867b3261f296` |
| 최종 CPU 회귀 | 검증 root의 `cpu_tests_ready.xml` | `1c7f2b2c36c7ee03328e00d66375ae018025f441c5cdf07f77dfdfecdfe139e1` |
| 새 40장 변환 | `artifacts/evaluation/base5_training_input_qualification_261016/current40_v1/report.json` | `a4d5e92e8dfb7c6c06e99bebea60508556519b21969ec9943bab77aff15e6dba` |
| 새 40장 CPU loader | 같은 prefix의 `current40_cpu_loader_v1.json` | `37e991b60bc2ad6b7535aa65db3e304853f5e55456ed18967b1cff44a6bdd2fa` |

선행 179장 변환은 같은 qualification prefix의 `v5/report.json`, CPU loader는 `cpu_loader_v1.json`이다.
원거리 부분 보존은 검증 root의 `far_partial_before.json`·`far_partial_after.json`,
완료 무촬영 재개는 `far_terminal_resume.json`·`smoke_terminal_resume.json`을 사용했다.

현재 release는 **Sunny13만** 대상으로 한다. 다른 다섯 환경의 `candidate_v150c/future_suite`는
이전 정책의 offline 준비본이며 이번 v154 보강이 자동 적용된 최신 25K가 아니다.
해당 환경은 최신 정책으로 독립 배치를 다시 생성하고 환경별 smoke 후 별도 동결한다.

본수집 예상은 기존 v152의 그룹별 비용 추정 7.95시간/30.43GiB를 참고한 운영 예산 **8–12시간/45GiB**다.
새 원거리 32장의 속도를 전체 5K에 단순 곱하지 않는다. 음영의 미준비 경로와 실패 보충은 추가 비용이 든다.
launcher는 잔여 장수에 비례한 저장 예산 외에 10GiB 여유를 요구한다. 기존 원본은 자동 삭제하지 않는다.

### 동결 결과와 실제 시작 명령

최종 release는 검증 root의 `release_v154_sunny13_final/release.json`이다.
SHA256은 `f34ea718d4f6386f9cbed772bb4ce6696b836265809eee869c7a2fd5fc2cc4b1`이며,
`bulk_ready=true`, `completion_live_qualified=true`, `bulk_started=false`다.
runtime 파일 85개와 입력·검증 파일 11,923개의 SHA를 pin했다.
과거 finalizer 연동 실패 때 생성된 재구성 보고서도 보존하고 새 final 경로로 동결했다.
native 재촬영이나 과거 QA 판정 변경으로 통과시킨 것이 아니다.

네트워크는 `configs/network/ego_network_mr_base5_savesensordata_collision_local.json`을 Load→Connect한다.
Sunny13·Capture Mode·대회용 센서와 카메라 1–4/LiDAR 5의 Instance/GT 저장을 유지한다.
기존 `rc`·guarded bridge를 중복 실행하지 않는다. 사용자 승인 전에는 아래의 촬영 명령을 실행하지 않는다.

```bash
cd ~/AjouNice2026
source base_ws/install_aw149/setup.bash
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release artifacts/evaluation/base5_precollection_261016/release_v154_sunny13_final/release.json \
  --environment sunny13 --confirmed-environment sunny13 --run --complete
```

새 output은 `~/morai_cap/base5_precollection_261016/collection_5k_v154_sunny13`이다.
`--run`을 뺀 같은 launcher로 촬영 없는 prepare-only만 검사했다.
상태는 `prepared_waiting_for_resume`, requested 5,000 / finished 0 / approved 0이고
새 output의 native audit와 service event log는 각각 0개였다.
동일 촬영 명령은 안전하게 정지된 실행의 resume도 처리하고, 종료 뒤 유한 보충을 연결한다.
`--complete`가 모든 실패를 무조건 승인하거나 무한 재시도한다는 뜻은 아니다.
통신/Park 불확실·무수신 CollisionData·source/hash 변경은 계속 중단한다.
