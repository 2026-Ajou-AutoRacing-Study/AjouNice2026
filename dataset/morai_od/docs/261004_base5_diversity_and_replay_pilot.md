# Base5 소량 보강과 Base3 replay 학습 준비

상태: EVIDENCE / PREPARED. 2026-10-04, dataset/base5. Production E25와 threshold는 변경하지 않았다.

후속 복구 객체 addon을 포함한 최신 학습 준비본은 [727/124 reviewed replay](261004_base5_restored_replay_results.md)이다. 아래688/115 경로는 변경 전 이력으로 보존한다.

## 실제 수집 결과

- `generate_base5_reinforcement_smoke.py --variant diversity`: 48 scene. 근접 장애물 24 + 고주로 정적 차량 24. Ego 초기속도 0, scale 1, WoodBox 제외.
- 근접: CargoBox/PE_Drum/Concrete_Barricade01, 전방·좌우·후방, yaw 0/45/90/135°. 기존 anchor와 종방향 15m 이동한 anchor를 시험했다. 이동한 anchor는 도로 높이/정지 안정성 추가 검토가 필요하다.
- 원거리: Ioniq/K5, MGeo legal highway route의 arc 80/120m 두 Ego 위치, 전후방 pivot arc 25/40/55m. **요청 거리를 geometric-center 거리로 부르지 않는다.** 변환된 차량 72 GT의 최대 |center.x|=55.523m, ±60m 이탈 0이다.
- 43/48 scene 완료, 129 frame 변환, 미식별 asset 0. 5 scene(b5dv16/17/18/19/24)은 Ego drift 0.273–0.289m로 0.25m gate에서 실패했다. 실패 scene을 채택하거나 gate를 완화하지 않았다.
- 기본 5-point 기준으로 obstacle 42 + vehicle 72 GT 유지, sparse obstacle 15개 제외. 129 frame 중 일부는 빈 GT이며 replay 준비 시 빈 addon frame을 제외한다.
- 약 1.7GiB raw bag. Raw audit는 **FAIL**: 카메라별 678 candidate 중 1개가 100ms age 초과(max 약108ms). 실제 선택 129 frame의 camera age max 27.858ms로 strict gate 통과. raw 전체 PASS로 보고하지 않는다.
- 정지 장면 보강이다. 동적 차량 속도·옆차로 추월·실주행 다양성 확보 결과가 아니다. 동일 layout의 인접 scene val은 최종 일반화 시험이 아니다.

## Sparse camera GT: 기본값은 5-point 유지

`build_base3_online_bevfusion_infos.py --sparse-camera-review <JSON>`을 추가했다.

- sample_idx + instance_id를 직접 검토한 목록에 등록해야 한다. asset 전체 예외가 아니다.
- 중심거리 10m 이내, baseline 256×704의 isotropic resize/bottom crop 이후 projected rectangle 양축 4px 이상이어야 한다.
- projection만으로 가림 여부를 알 수 없으므로 이미지 확인이 필수다. 다른 scene에 목록을 자동 확장하지 않는다.
- 이전 b5rf02의 front_left 3장을 직접 확인했다. CargoBox가 가려지지 않고 보이며 LiDAR GT point는 0이다. 이 3개만 `configs/base5_sparse_camera_review_261004.json`에 등록했고, 실제 재변환 audit에서 exception 3개를 확인했다.
- validator도 동일 review JSON을 요구한다. 전역 `minimum-gt-points=0`으로 우회하지 않았다.
- 신규 diversity sparse GT는 아직 직접 검토하지 않았으므로 기본 5-point 기준으로 제외했다. 원본 bag을 보존하여 후속 검토/재변환할 수 있다.

## 학습 입력 준비

`prepare_base5_replay_pilot.py`는 Base3 600 train/100 val을 고정 seed로 선택하고, ±30m 내 addon만 합친다. 선택 sensor 파일을 hardlink로 묶고 container에서는 read-only로 mount한다. 원본을 편집하지 않으며 다른 filesystem에서는 hardlink가 실패할 수 있다.

- Train: Base3 600 + diversity 63 + reviewed smoke 25 = **688**.
- Val: Base3 100 + diversity 3 + reviewed smoke 12 = **115**. Test 0.
- validator: 803 frame, 680 scene, class GT obstacle 993/vehicle 557/pedestrian 428, 파일·scene split·box 구조 및 sparse whitelist 검증 PASS.
- source별 metainfo를 replay_manifest.json에 보존한다. MR -1.347° 적용 rosbag과 동기 캡처 Base3는 각각 이미 LiDAR geometric-center 기준으로 변환된 입력이다. heading을 다시 적용하지 않는다.
- Far-only 40/55m frame은 이 ±30m 혼합 학습에 포함되지 않는다. Base3의 >30m 라벨 부재를 무시하고 확대 ROI로 학습하지 않는다.
- Recipe: `perception/detection3d/bevfusion/configs/team/recipes/base5_nearfield_replay_e2_pilot.yaml`. E25-init, spatial-only, E2, LR 2e-5, warmup 25 optimizer update. 총 약344 update인데 기본 warmup500을 그대로 쓰는 문제를 피했다.
- Dry-run 성공. **GPU 학습/새 checkpoint rosbag 재평가는 아직 실행하지 않았다.** 현재 MR은 켜져 있다. GPU training과 MR 수집을 동시에 실행하지 않는다.
- CPU Docker에서 실제 train pipeline으로 688/688 frame 로드 PASS. 무작위 augmentation/ROI filtering 뒤 GT 0인 frame은 103개(Base3 97/addon 6), 남은 GT 1663개였다. 이 숫자는 detector 정확도나 누락률이 아니며 실행마다 augmentation으로 달라질 수 있다.

## 경로와 실행

```text
rosbags/datasets/base5/261004_diversity_pilot/       # 성공 snapshot 43 bag
/home/luv/morai_cap/261004_base5_diversity_pilot/    # summary/failures/metadata
artifacts/datasets/base5_diversity_pilot_261004/
  scenarios/manifest.json / runtime.json
  parsed/                   # 129 frame, minimum_gt_points=5
  raw_audit.json
  replay_dataloader_train.json
artifacts/datasets/base5_reinforcement_smoke_261004/parsed_reviewed/
artifacts/datasets/base5_nearfield_replay_pilot_261004/
  morai_base3_infos_{train,val,test}.pkl
  replay_manifest.json / base3/ / addon0/ / addon1/
```

MR을 종료한 뒤 학습한다. ROS core/receiver를 일부러 죽일 필요는 없다. E25 production 가중치를 덮어쓰지 않는다.

```bash
ajou
export BASE5_REPLAY_ROOT="$AJOUNICE_ROOT/artifacts/datasets/base5_nearfield_replay_pilot_261004"
bash perception/detection3d/bevfusion/scripts/run_team_training.sh \
  perception/detection3d/bevfusion/configs/team/recipes/base5_nearfield_replay_e2_pilot.yaml
```

수집 시작/모니터 명령은 [smoke 런북](261004_base5_reinforcement_smoke_runbook.md)을 따른다. diversity 출력/runtime/capture directory를 이번 경로로 바꾸고 생성 시 `--variant diversity`를 붙인다. 기존 출력이나 원본 bag을 덮어쓰지 않는다.

## 다음 판단

1. 실제 E2 학습 wiring/loss와 nearfield held-out rosbag 유지율을 확인한다. 작은 val AP만으로 production 교체하지 않는다.
2. 같은 혼합 데이터에서 해상도만 320×896로 바꾸는 독립 실험을 비교한다. 먼저 baseline 입력을 검증한다.
3. Far ROI는 별도 dataset/recipe로 검증하며, raw/full-cloud에서 >30m 객체 라벨이 빠진 replay를 섞지 않는다.
4. 이번 failed anchor는 재수집 전에 지형·Ego settle/정지 조건을 고친다. dynamic/high-speed 수집으로 무작정 확대하지 않는다.
