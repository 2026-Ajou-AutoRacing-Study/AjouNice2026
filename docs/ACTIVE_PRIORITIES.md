# AjouNice2026 활성 최우선 계획.

마지막 갱신: 2026-09-18.

## 관리 규칙.

- 이 문서는 모듈이나 과거 실험에 종속되지 않는 프로젝트 공용 작업 큐다.
- 다음 작업에서 반드시 다시 확인할 살아 있는 항목만 유지한다.
- 완료됐거나 더 이상 최우선이 아닌 항목은 이 문서에서 제거하고 해당 실험·설계
  문서에 결과를 남긴다.
- 세부 근거와 과거 시행착오는 이 문서에 복제하지 않고 관련 문서로 연결한다.

## 현재 최우선. 대회 측위 종·횡 안정화와 제어 채터링 제거.

- 현장점검에서 fusion localization의 steering-rate absolute p95가 약 `1.59 rad/s`,
  GT localization은 약 `0.34 rad/s`였다. 이는 시각화가 아니라 제어 안정성 P0다.
- 정상 구간 횡 채터링은 약 `14.8 / 18.5 cm`로 양자화된 GNSS update를 Global EKF가
  강하게 따라가고, controller가 Global state를 직접 소비하는 구조가 주원인임을
  same-bag 분석으로 확인했다.
- 로컬 full-stack bag의 `velocity.x` 21.9% 과적분은 production 속도 bias가 아니다.
  현장 4개 bag의 위치 이동거리/속도 적분 비율은 `0.9998~1.0000`이며, 로컬 한-PC
  MORAI 실시간율 저하와 receipt-time의 시간축 artifact로 판정한다.
- 첫 다음 실험은 BEVFusion 대신 `od_gt`를 사용한 GT-object full stack으로 MORAI
  실시간율을 정상화하고, 이동거리/속도 적분 비율 `0.98~1.02`를 통과하는 test bed를
  확보하는 것이다. 그 후 `0.05 / 0.10 m`와 quantization-aware GNSS update를 A/B한다.
- 최종 pose LPF나 controller gain 변경으로 먼저 가리지 않는다. 동일 bag에서
  quantization-aware update, GPS 음영 복귀 정책, global/local control state 분리,
  controller tuning 순으로 진행한다.
- MR/대회 production alias를 `localization`=`competition`, MRP alias를
  `localization_mrp`=`ego`로 분리했으며 heading correction `-1.347 deg`를 유지한다.
- 대회 GNSS와 IMU에는 인위적인 랜덤 노이즈가 없다는 운영진 답변을 확인했다.
  양자화와 GPS 음영은 별도 문제로 계속 처리한다.
- 최신 문제 정의와 검증 계약:
  [`260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).

## Localization 안정화 직후 재개할 항목.

- Prediction horizon을 기본 5초로 확장하되 로터리에서는 3~4초로 줄이는
  map-zone-aware 정책을 검토한다.
- 고주로 추월을 방해하는 다중 후보 궤적 문제는 단순 `top_k=1` 전환이 아니라,
  tracker history와 lanelet 연결성·차선 변경 가능성을 결합한 temporal lane
  association으로 실제 진행 차선의 Top-1 정확도를 먼저 높인다.
- 위 Prediction/Map-CV 변경은 현재 Localization covariance·jitter 실험과 섞지 않고,
  Localization 안정화 및 동일 rosbag 회귀 검증이 끝난 뒤 별도 단계에서 수행한다.

## P0. Base3-Static 10K 수집·정제·학습.

1. Batch 001~007 production capture를 완료한다.
   - Batch 001~007 primary는 `3,322/3,334 clips`, `9,966 frames`를 완료했다.
   - 실패 quota replacement와 post-hoc motion 불량 재보충을 끝냈다.
   - 최종 유효 구성은 `3,334 clips / 10,002 frames`다.
   - Batch 008에서 moving Pedestrian 300 clips/900 independent single frames를 추가해
     `3,634 clips / 10,902 frames`로 완성한다.
   - 2026-08-30에 Batch 008 primary 294 clips, replacement v1 2 clips,
     replacement v2 4 clips를 완료했다. 최종 300 clips/900 frames이며 artifact error 0이다.
   - 폐기된 12 Hz 8-frame Pedestrian add-on은 Base3 index에 포함하지 않는다.
2. 최종 정제·패키징 전에 `validate_dynamic_capture_output.py`를 수정한다.
   - 이 validator는 실시간 capture gate가 아니라 수집 완료 후 실행하는 read-only
     post-hoc 감사 도구다. 현재 수집은 runner 내부 artifact gate로 계속 진행한다.
   - `timed-grouped`의 `sensor_files` variant와 `rolling-grouped`의 `new_files`,
     `observed_artifacts`, `required_artifacts`, `missing_artifacts` variant를 모두
     지원한다.
   - Capture-mode metadata 차이와 실제 artifact 누락을 분리해 보고한다.
   - 수정 후 이미 수집된 Batch 001~007 전체에 소급 적용해 artifact completeness를
     다시 검증한다.
   - 실제 불량 clip만 제외하고 reserve scene으로 보충한 뒤 packaging을 진행한다.
   - 2026-08-29 schema 호환 수정과 Batch 001--004 소급 검증을 완료했다.
     - 1,898 clips / 5,694 frames artifact error 0.
   - Batch 005--007과 replacement까지 같은 검증을 전체에 다시 실행했다.
     - Batch 001--007 primary artifact error 0.
     - 최종 replacement v2/v3 artifact error 0.
   - `audit_static_classcentric_base3_campaign.py`의 exact replacement quota key로 현재
     Batch 001--004 실패 8개와 향후 실패 scene을 보충한다.
3. Batch 005--007 생성 전에 corrective topology live gate를 통과한다.
   - 30-clip offline pilot과 120-clip offline audit을 통과했다.
   - MORAI live 30/30 clips, 90/90 frames와 120/120 clips, 360/360 frames를 완료했다.
   - 두 live pilot 모두 제한 차로 0, 경로 이탈 0, 최종 실패 0, artifact error 0이다.
   - 120-clip LiDAR audit은 Vehicle 272/272, Pedestrian 200/210, Obstacle 538/573이
     5-point 이상 정상 positive였다.
   - corrective topology live gate는 완료됐다. 다음 단계에서 exact quota로 최종
     Batch 005--007을 생성하되, 기존 Batch 001--004는 다시 생성하지 않는다.
   - 2026-08-29에 Batch 005--007 primary와 reserve source를 생성하고 전체 validator와
     topology audit을 통과했다. 다음 실행은 기존 session에 Batch 005--007만 이어서
     capture하는 것이다.
4. LiDAR GT quality policy를 적용해 학습 dataset을 패키징한다.
   - 0-point GT는 제거한다.
   - 1~4-point sparse GT는 positive loss에서 제외한다.
   - 5 points 이상을 정상 positive로 사용한다.
   - 최종 30 m LiDAR audit은 29,545 GT box 중 28,146개, `95.26%`가 5 points
     이상이다. Vehicle/Pedestrian/Obstacle 정상 positive 비율은 각각
     `99.89% / 96.84% / 92.65%`다.
5. Base3 semantic model과 PointPainting controlled baseline을 학습한다.
   - 진행 중인 Base3 LRASPP natural/ENet/384 baseline은 설정을 바꾸지 않고 끝까지
     학습한다.
   - checkpoint 선택은 foreground mIoU를 주 지표로 하되 class별 precision/recall과
     LiDAR projected-point precision/recall/IoU를 반드시 함께 본다.
   - PointPainting 관점에서는 semantic recall 단독을 최적화하지 않는다. 과대 painting된
     배경 LiDAR point가 Pedestrian/Obstacle feature를 받아 3D false positive와 box
     geometry 오차를 만들 수 있기 때문이다.
   - 다음 segmentation 개선 실험은 한 번에 한 요인씩 수행한다.
     1. 현재 weighted CE baseline과 `weighted CE + 0.3 Dice`를 비교한다.
     2. full image를 유지하면서 object-aware crop/scale 및 mild photometric augmentation을
        일부 sample에만 적용한다.
     3. short-side `384 / 448 / 512`의 accuracy--latency를 비교한다.
     4. 위 실험 후에도 projected-point 및 mission rosbag 병목이 남을 때만 SegFormer-B0
        같은 경량 대체 architecture를 검토한다.
   - 해상도와 architecture는 4-camera 기준 전처리, segmentation 순수 추론, 전체
     PointPainting compute의 mean/p50/p95/p99와 peak VRAM을 측정한 뒤 결정한다.
   - 각 후보는 offline pixel metric만으로 채택하지 않는다. projected-point semantic
     audit과 mission rosbag semantic audit을 먼저 통과시키고, 새 checkpoint로 Base3를
     다시 painting하여 PointPainting을 재학습한 뒤 최종 3D AP와 false positive를 비교한다.
   - Base3 LRASPP natural/ENet/384 E30 학습과 frozen test 평가를 완료했다.
     - Test foreground mIoU `0.7821`; Pedestrian/Vehicle/Obstacle IoU는
       `0.6978 / 0.8999 / 0.7485`다.
     - 기존 mission rosbag 대비 GT-point target argmax가 Vehicle, Pedestrian,
       Obstacle에서 모두 크게 개선됐다.
     - 15 Hz 상당 OpenCV 4-worker runtime은 전체 compute p50/p95
       `43.46 / 48.31 ms`, drop/skip 0이다.
     - 다음 P0 작업은 이 checkpoint로 Base3를 재painting하고 PointPainting controlled
       baseline을 학습하는 것이다.
6. Obstacle loss 지배 여부를 반드시 확인한다.
   - Batch 001 실제 LiDAR 감사에서 Obstacle GT box는 전체 관측의 `55.5%`다.
   - Class별 detection/regression loss, offline validation, mission rosbag의
     Vehicle/Pedestrian/Obstacle 성능을 함께 비교한다.
   - Obstacle이 Vehicle/Pedestrian 학습을 실제로 방해한다고 확인될 때만
     class-balanced loss, per-class sampling 또는 loss weight ablation을 수행한다.
   - Box 비율만으로 진행 중인 수집 구성을 변경하지 않는다.
7. Base3 PointPainting controlled baseline을 완료했다.
   - `B3S-PP0 E39`, canonical frozen Base3 Test center mAP `0.9744`임.
   - 신로터리/고주로1 Vehicle center mAP는 `0.715 / 0.765`임.
   - 고주로 Tracking F1/MOTA는 `0.951 / 0.900`으로 안정적임.
   - 로터리 Tracking vector AVE `1.086 m/s`와 2/3초 minFDE-K p50
     `1.172 / 1.660 m`가 production 승격을 막는 주 병목임.
   - 횡단 Pedestrian은 precision `1.000`, recall `0.399`로 누락 중심 병목임.
   - CargoBox center mAP는 `0.407`로 개선됐지만 mASE `0.939`이므로 정확한
     geometry가 해결된 것으로 판단하지 않는다.
   - `B3S-PP1-GTDB-V20P10O4 E34` 학습과 frozen/online/downstream 평가를 완료함.
     - Val/Test center mAP는 `0.9786/0.9809`임.
     - PP0 대비 신로터리 Vehicle mAP는 `0.715 -> 0.749`, 횡단 Pedestrian은
       `0.322 -> 0.634`, 거리별 Pedestrian은 `0.778 -> 1.000`으로 개선됨.
     - 로터리 Tracking AVE mean은 `1.086 -> 0.809 m/s`, CV-stable 3초
       minFDE-K p50/p90은 `1.660/4.564 -> 1.243/3.402 m`로 개선됨.
     - 고주로 합류 F1, CargoBox, 고주로 Map-CV는 개선되지 않음.
     - PP1은 Base3 계열 보존 checkpoint로 채택하되 전체 production은 S1g
       score `0.40`을 유지함.
   - 다음 P0 실행은 PP1 class별 score calibration과 고주로 합류/CargoBox 원인
     분리이며 재학습은 그 결과 뒤에 결정함.
   - 이후에도 횡단 누락과 로터리 downstream 병목이 남으면 Base3 GT-velocity
     oracle과 횡단 Pedestrian oracle-painting으로 원인을 분리함.
   - 그 전에는 S1g 기반 production을 유지한다.

세부 설계와 생성기 근거:
[`base3_static_classcentric_10k_design.md`](../dataset/morai_od/docs/base3_static_classcentric_10k_design.md).

## P1. Offline validation과 mission rosbag 사이의 domain gap 축소.

- Base3 학습 결과를 동일 checkpoint의 offline validation과 로터리·고주로·보행자·
  장애물 mission rosbag에서 비교한다.
- Class별 AP·recall·precision·mATE·mASE·mAOE와 slice별 false positive를 확인한다.
- Semantic painting, 3D detector geometry, SaveSensorData/online sensor domain 중
  성능 저하가 시작되는 단계를 분리한다.
- Mission rosbag Vehicle geometry가 production 기준에 도달하기 전에는 velocity
  head를 tracker production 입력으로 확정하지 않는다.
- Base3 corrected 평가는 완료했다. Offline best는 `B3-BF0-F40 E26`이다.
  초기 PointPainting 통제 실험인 `Vehicle 30 m` bag은 production 평균에서
  제외했다. 실제 경로·미션 7-bag detection macro AP는 E10/E25/E26
  `0.7645/0.7620/0.7625`로 E10이 근소하게 높다. 실제 경로 Vehicle 3-bag에서
  E25는 tracking AVE `0.7887 -> 0.7252 m/s`, 3초 FDE
  `4.7168 -> 4.4912 m`로 개선되지만 center p50은 `0.5403 -> 0.5457 m`로
  약간 나빠진다. 상세 결과:
  [`260918_base3_e40_production_evaluation_results.md`](bevfusion/260918_base3_e40_production_evaluation_results.md).
- 최종 목표를 per-frame OD AP보다 tracker/Map-CV의 안정적인 속도와 미래경로로
  두므로 production 표준은 `B3-BF0-F40 E25`로 정한다. E10은 rollback baseline,
  E26은 offline best로 보존한다.
- 다음 재학습은 단순 epoch 연장이 아니라 기존 시나리오 생성기를 production
  UDP/ROS 입력 경로와 결합한 `Base3-Online` pilot을 우선한다.
  - 2026-09-19 mixed30 pilot은 30/30 scene, 90 frame 변환, train/val/test
    BEVFusion Docker dataloader까지 통과했다.
  - 다음 단계는 동일 계약의 1,000--2,000 independent-frame 수집과 E25 controlled
    fine-tuning이며, 동적 actor/velocity/deskew는 첫 실험에 섞지 않는다.
  - mixed30의 5-point GT 유지율은 Vehicle 97.4%, Pedestrian 51.5%, Obstacle 57.5%라서
    본 수집량은 scene 수가 아닌 필터 후 class별 positive 수로 산정한다.
  - 첫 확장은 고주로 Vehicle-positive 전용 334 scene으로 수행했다.
    Pedestrian/Obstacle/empty는 섞지 않는다. 2026-09-19 수집은 334/334 scene,
    실패 0으로 완료했다. LiDAR timestamp 기준 Ego/Object pose interpolation과
    5-point positive gate를 적용한 최종 train/val/test는 692/147/144, 총 983 frame,
    Vehicle GT 1,814개다. portable root는
    `/datasets/base3_online_highway_motion_v2`이며 팀 Docker dataloader까지 통과했다.
    다음 비교는 이 데이터만 쓰는 fresh E40과 production E25-init E12 fine-tuning이다.
  - 여러 scene을 rosbag에 연속 기록하되 scene marker로 유효 구간을 명시한다.
  - LiDAR timestamp 기준 production camera synchronization과 Ego/Object 보간을 쓴다.
  - 첫 실험은 30--50 scene 수집 검증 후 1,000--2,000 independent frame controlled
    fine-tuning이며 해상도·GT sampling·temporal 변경을 동시에 섞지 않는다.
  - 상세 계획:
    [`production_online_rosbag_dataset_plan.md`](../dataset/morai_od/docs/production_online_rosbag_dataset_plan.md).
  - 2026-09-19 clean 2-scene bag의 marker/timestamp 감사와 BEVFusion 변환기 oracle
    검증을 완료했다. `/Object_topic.position`이 BBox center가 아닌 asset pivot임을
    확인하고 Base3 75개 asset의 pivot-to-center catalog를 복원했다. 6개 online frame은
    LiDAR/camera overlay, MMDetection3D info validator, 실제 Docker dataloader를 모두
    통과했다. Vehicle/Pedestrian/Obstacle/empty 30-scene mixed pilot도 30/30 scene,
    90 frame과 Docker dataloader까지 통과했다. 본 확장은 그 후속인 고주로
    Vehicle-positive 334-scene campaign이다.

과거 Base2 근거:
[`base2_temporal3_velocity_d4_readiness.md`](../perception/pointpainting/docs/base2_temporal3_velocity_d4_readiness.md),
[`semantic_oracle_painting_ablation.md`](../perception/pointpainting/docs/semantic_oracle_painting_ablation.md).

## P2. Vehicle velocity supervision과 temporal modeling 재개 조건.

- P0와 P1에서 single-frame detection geometry를 먼저 안정화한다.
- 이후 한 번에 한 요인만 바꾸는 velocity ablation을 수행한다.
  - V0: velocity channel weight `0.2` baseline.
  - V1: Vehicle velocity channel weight `0.5`.
  - V2: `0.2`와 class-balanced/moving-balanced velocity loss.
  - V3: V1과 V2가 각각 유효할 때만 결합.
- 판단 지표는 Vehicle AVE, speed MAE, 종·횡방향 MAE와 detection geometry 손상,
  rosbag tracking AVE 및 Map-CV 1/2/3초 FDE다.
- 3-sweep production 후보는 학습과 online 추론에서 동일한 causal sweep 정합 계약을
  만족해야 한다.
- Base3-Static 학습 후 BBox-only tracker의 velocity와 Map-CV prediction을 먼저 평가한다.
  이 결과가 부족할 때만 Base4-Temporal을 시작한다.
- Base4는 Base3 spatial checkpoint로 초기화하되 첫 baseline은 Base4-only full
  fine-tuning으로 수행한다. Base3 single-frame을 연속 3-sweep loader에 직접 섞지 않는다.
- Base4 수집기·label builder 구현을 시작할 때 velocity metadata 계약을 다시 검토한다.
  - `velocity_target_ego_mps`만 학습 GT로 노출한다.
  - Base3 capture archive의 `derived_velocity_mps`는 학습 GT가 아니다.
  - Base4에서는 정확한 timestamp로 derived 값을 필요 시 재계산하고, 새 pilot에서 실제
    오류 검출 효용이 확인될 때만 진단 namespace에 유지한다.
  - 팀원 공유 package에서 진단값이 GT로 오인되지 않도록 schema와 README를 확인한다.
- Static 성능 망각이 확인될 때는 우선 Base4의 static·empty sequence 비중과 learning rate를
  조정한다. 그래도 필요할 때만 shared spatial encoder를 쓰는 별도 single-frame auxiliary
  branch·dual-loader를 구현한다.

### Base3 Dynamic-Ped Add-on v1 완료.

- 75 clips/600 frames capture와 replacement를 완료했다.
- Base3 single-frame fine-tuning에서는 clip당 2--3 frames만 사용한다.
- Pedestrian mission rosbag에서 Base3 단독과 Base3+V1을 A/B한다.
- 자연 구조물만으로 strong occlusion quota는 확보되지 않았다. 실제 rosbag에서 가림
  recall이 병목일 때만 별도 Hard-Occlusion Add-on을 최우선 후보로 올린다.

상세 로드맵:
[`base3_static_to_base4_temporal_roadmap.md`](../perception/pointpainting/docs/base3_static_to_base4_temporal_roadmap.md).

## P3. 학습 관측성과 효율 표준화.

- Epoch 종료 시 W&B에 공통 x축 `epoch`으로 시작·마지막·평균 learning rate를 즉시
  기록한다.
- Iteration 단위 TensorBoard learning rate는 상세 진단용으로 유지한다.
- PointPainting batch-size GPU microbenchmark를 완료했다.
  - 동일 single-frame 9-feature proxy에서 BS `1/2/4/6/8/12/16/24`를 측정했다.
  - BS4는 `39.99 samples/s`, CUDA peak reserved `2.72 GiB`였다.
  - BS8/12/16은 `37.10/38.05/37.61 samples/s`로 더 느렸고 BS24는 OOM이었다.
  - BS1은 `40.32 samples/s`로 0.8% 빠를 뿐이며 optimizer noise와 기존 실험 조건
    차이가 커 첫 Base3 controlled baseline에서는 사용하지 않는다.
  - `B3S-PP0`은 BS4, Adam OneCycle, LR `0.003`을 사용한다. Base3 painting 완료 후
    BS4 peak memory와 throughput만 짧게 재확인한다.
  - 상세 결과는
    [`pointpainting_batch_size_benchmark.md`](../perception/pointpainting/docs/pointpainting_batch_size_benchmark.md)에 기록한다.
