# AjouNice2026 활성 최우선 계획.

마지막 갱신: 2026-08-29.

## 관리 규칙.

- 이 문서는 모듈이나 과거 실험에 종속되지 않는 프로젝트 공용 작업 큐다.
- 다음 작업에서 반드시 다시 확인할 살아 있는 항목만 유지한다.
- 완료됐거나 더 이상 최우선이 아닌 항목은 이 문서에서 제거하고 해당 실험·설계
  문서에 결과를 남긴다.
- 세부 근거와 과거 시행착오는 이 문서에 복제하지 않고 관련 문서로 연결한다.

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
6. Obstacle loss 지배 여부를 반드시 확인한다.
   - Batch 001 실제 LiDAR 감사에서 Obstacle GT box는 전체 관측의 `55.5%`다.
   - Class별 detection/regression loss, offline validation, mission rosbag의
     Vehicle/Pedestrian/Obstacle 성능을 함께 비교한다.
   - Obstacle이 Vehicle/Pedestrian 학습을 실제로 방해한다고 확인될 때만
     class-balanced loss, per-class sampling 또는 loss weight ablation을 수행한다.
   - Box 비율만으로 진행 중인 수집 구성을 변경하지 않는다.

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
- 다음 본 학습 전에 batch size `4/8/12`와 learning-rate scaling을 짧은 smoke로
  비교한다.
  - Samples/sec, epoch 시간, peak VRAM, loss 안정성, epoch validation을 함께 본다.
  - VRAM 사용량만으로 production batch size를 결정하지 않는다.
